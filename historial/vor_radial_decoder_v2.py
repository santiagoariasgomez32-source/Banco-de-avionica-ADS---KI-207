#!/usr/bin/env python3
"""
vor_radial_decoder_v2.py

Igual que vor_radial_decoder.py, pero SIN ningún bloque de Python
personalizado dentro del flowgraph (para evitar el bug conocido y sin
solución oficial de GNU Radio con bloques Python — ver issue #6283 en
el repositorio de GNU Radio). En vez de escribir al serial desde
dentro del flowgraph, el radial calculado (en grados) se manda por
UDP a un proceso aparte: vor_udp_bridge.py, que es el que realmente
calcula la desviación de aguja y escribe al ESP32.

Esta parte SOLO calcula el radial y lo manda por UDP. También sigue
escuchando al panel para poder retonalizar el SDR (eso sí necesita
vivir en este proceso, porque solo aquí existe el objeto del SDR).
"""

import argparse
import threading
import time

import numpy as np

from gnuradio import gr, blocks, filter as gr_filter, analog, network
import osmosdr


class VorTopBlock(gr.top_block):
    def __init__(self, device_args, freq_hz, udp_port):
        gr.top_block.__init__(self, "VOR Radial Decoder v2 (sin bloques Python)")

        self.samp_rate_rf = 1_000_000
        self.samp_rate_env = 100_000
        self.samp_rate_30hz = 1_000

        self.src = osmosdr.source(args=f"numchan=1 {device_args}")
        self.src.set_sample_rate(self.samp_rate_rf)
        self.src.set_center_freq(freq_hz)
        self.src.set_gain_mode(True)

        taps_decim = gr_filter.firdes.low_pass(1.0, self.samp_rate_rf, 15_000, 3_000)
        self.decim = gr_filter.fir_filter_ccf(
            self.samp_rate_rf // self.samp_rate_env, taps_decim
        )
        self.envolvente = blocks.complex_to_mag(1)

        # --- Camino VAR ---
        taps_var_bp = gr_filter.firdes.band_pass(1.0, self.samp_rate_env, 25, 35, 10)
        self.var_bp = gr_filter.fir_filter_fff(1, taps_var_bp)
        taps_var_decim = gr_filter.firdes.low_pass(1.0, self.samp_rate_env, 45, 20)
        self.var_decim = gr_filter.fir_filter_fff(
            self.samp_rate_env // self.samp_rate_30hz, taps_var_decim
        )
        self.var_hilbert = gr_filter.hilbert_fc(65)

        # --- Camino REF ---
        taps_ref_bp = gr_filter.firdes.band_pass(1.0, self.samp_rate_env, 9480, 10440, 200)
        self.ref_bp = gr_filter.fir_filter_fff(1, taps_ref_bp)
        self.ref_hilbert_fm = gr_filter.hilbert_fc(65)
        quad_gain = self.samp_rate_env / (2 * np.pi * 480)
        self.quad_demod = analog.quadrature_demod_cf(quad_gain)
        taps_ref_lp = gr_filter.firdes.low_pass(1.0, self.samp_rate_env, 45, 20)
        self.ref_lp = gr_filter.fir_filter_fff(1, taps_ref_lp)
        self.ref_decim = gr_filter.fir_filter_fff(
            self.samp_rate_env // self.samp_rate_30hz, taps_ref_lp
        )
        self.ref_hilbert = gr_filter.hilbert_fc(65)

        # --- Comparador de fase ---
        self.mult_conj = blocks.multiply_conjugate_cc(1)
        self.c2arg = blocks.complex_to_arg(1)
        self.rad2deg = blocks.multiply_const_ff(180.0 / np.pi)

        # Reduce la tasa antes de mandar por UDP (~20 valores/seg es de
        # sobra para que la aguja se vea fluida, y aligera la red/serial)
        self.keep = blocks.keep_one_in_n(gr.sizeof_float, self.samp_rate_30hz // 20)

        # Bloque NATIVO (no Python) — evita el bug de los bloques
        # personalizados. Manda cada valor de radial (en grados) por UDP.
        self.udp = network.udp_sink(gr.sizeof_float, 1, "127.0.0.1", udp_port, 0, 1472, True)

        # --- Conexiones ---
        self.connect(self.src, self.decim, self.envolvente)
        self.connect(self.envolvente, self.var_bp, self.var_decim, self.var_hilbert)
        self.connect(
            self.envolvente, self.ref_bp, self.ref_hilbert_fm, self.quad_demod, self.ref_lp
        )
        self.connect(self.ref_lp, self.ref_decim, self.ref_hilbert)
        self.connect(self.var_hilbert, (self.mult_conj, 0))
        self.connect(self.ref_hilbert, (self.mult_conj, 1))
        self.connect(self.mult_conj, self.c2arg, self.rad2deg, self.keep, self.udp)

    def set_freq(self, freq_hz):
        self.src.set_center_freq(freq_hz)


class PanelListener(threading.Thread):
    """Escucha el panel SOLO para comandos de frecuencia (FREQ). El
    curso seleccionado (COURSE) ya no se maneja aquí — lo escucha
    directamente vor_udp_bridge.py, porque el cálculo de desviación
    también vive ahí ahora."""

    def __init__(self, panel_port, tb, baudrate=115200):
        super().__init__(daemon=True)
        self.tb = tb
        self.running = True
        try:
            self.ser = serial_module().Serial(panel_port, baudrate, timeout=0.2)
        except Exception as e:
            print(f"[PanelListener] No se pudo abrir {panel_port}: {e}")
            self.ser = None

    def run(self):
        if self.ser is None:
            return
        while self.running:
            try:
                linea = self.ser.readline().decode(errors="ignore").strip()
            except Exception:
                time.sleep(0.5)
                continue
            if not linea:
                continue
            partes = linea.split(",")
            if partes[0] == "FREQ" and len(partes) >= 4 and partes[2] == "ACT":
                try:
                    freq_hz = float(partes[3]) * 1000.0
                    self.tb.set_freq(freq_hz)
                    print(f"[PanelListener] Retonalizado a {freq_hz/1e6:.3f} MHz")
                except ValueError:
                    pass

    def stop(self):
        self.running = False


def serial_module():
    import serial
    return serial


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--device-args", default="")
    ap.add_argument("--freq", type=float, default=113_700_000.0)
    ap.add_argument("--udp-port", type=int, default=6006)
    ap.add_argument("--panel-port", default=None, help="opcional, si no lo pasas no retoniza")
    args = ap.parse_args()

    tb = VorTopBlock(args.device_args, args.freq, args.udp_port)
    tb.start()
    print(f"Corriendo. Mandando radial por UDP a 127.0.0.1:{args.udp_port}")

    panel = None
    if args.panel_port:
        panel = PanelListener(args.panel_port, tb)
        panel.start()

    print("Ctrl+C para detener.")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        if panel:
            panel.stop()
        tb.stop()
        tb.wait()


if __name__ == "__main__":
    main()
