#!/usr/bin/env python3
"""
vor_radial_decoder.py

Flowgraph de referencia (GNU Radio 3.10+) para:
  1. Recibir la señal VOR desde un RTL-SDR.
  2. Separar las dos señales de 30 Hz (VAR directa y REF en subportadora de 9960 Hz).
  3. Comparar su fase para obtener el radial (bearing magnético).
  4. Calcular la desviación de aguja (CDI) y la bandera TO/FROM respecto
     al curso seleccionado en el panel.
  5. Enviar esos datos por puerto serial al ESP32 que maneja el KI-207.
  6. Escuchar en otro puerto serial los comandos del panel (frecuencia y curso).

IMPORTANTE — léelo antes de correrlo:
  Este archivo es un PUNTO DE PARTIDA, no una versión ya validada. Los
  nombres de bloques y su firma exacta pueden variar levemente según tu
  versión de GNU Radio (3.8 / 3.9 / 3.10). Antes de confiar en él:
    - Arma esta misma cadena en GRC (GNU Radio Companion) bloque por
      bloque, usando sinks de frecuencia (FFT) y osciloscopio para
      verificar visualmente cada etapa (envolvente, VAR, REF, fase).
    - Ajusta los cortes de los filtros y el número de taps de forma
      empírica observando esos sinks — los valores aquí son puntos de
      partida razonables, no constantes definitivas.
    - Ten en cuenta que necesitarás GNU Radio, gr-osmosdr y pyserial
      instalados (pip install pyserial; gnuradio y gr-osmosdr normalmente
      se instalan por el gestor de paquetes del sistema, no por pip).

Requiere: gnuradio (>=3.10), gr-osmosdr, numpy, pyserial
"""

import argparse
import threading
import time

import numpy as np
import serial

from gnuradio import gr, blocks, filter as gr_filter, analog
import osmosdr


# --------------------------------------------------------------------------
# Bloque de salida: recibe el radial en grados, calcula desviación/TO-FROM
# respecto al curso seleccionado, y lo manda por serial al ESP32 del KI-207.
# --------------------------------------------------------------------------
class VorSerialSink(gr.sync_block):
    def __init__(self, serial_port, baudrate=115200, full_scale_deg=10.0):
        gr.sync_block.__init__(
            self,
            name="vor_serial_sink",
            in_sig=[np.float32],   # radial en grados (0-360)
            out_sig=None,
        )
        self.full_scale_deg = full_scale_deg
        self.course = 0.0           # curso seleccionado, lo actualiza el panel
        self.lock = threading.Lock()
        self.ser = None
        try:
            self.ser = serial.Serial(serial_port, baudrate, timeout=0)
        except serial.SerialException as e:
            print(f"[VorSerialSink] No se pudo abrir {serial_port}: {e}")

    def set_course(self, course_deg):
        with self.lock:
            self.course = course_deg % 360.0

    @staticmethod
    def _angle_diff(a, b):
        """Diferencia angular normalizada a [-180, 180)."""
        d = (a - b + 180.0) % 360.0 - 180.0
        return d

    def work(self, input_items, output_items):
        radial = float(input_items[0][-1]) % 360.0

        with self.lock:
            course = self.course

        diff = self._angle_diff(radial, course)
        deflexion = max(-1.0, min(1.0, diff / self.full_scale_deg))
        to_from = "FROM" if abs(diff) <= 90.0 else "TO"

        if self.ser is not None:
            linea = f"DEV,{deflexion:.3f},FLAG,{to_from},RAD,{radial:.1f}\n"
            try:
                self.ser.write(linea.encode())
            except serial.SerialException:
                pass

        return len(input_items[0])


# --------------------------------------------------------------------------
# Flowgraph principal
# --------------------------------------------------------------------------
class VorTopBlock(gr.top_block):
    def __init__(self, device_args, freq_hz, ki207_port):
        gr.top_block.__init__(self, "VOR Radial Decoder")

        # --- Parámetros de muestreo ---
        self.samp_rate_rf = 1_000_000     # tasa de captura del SDR
        self.samp_rate_env = 100_000      # tasa tras decimar a la envolvente
        self.samp_rate_30hz = 1_000       # tasa final para el comparador de fase

        # --- Fuente SDR ---
        self.src = osmosdr.source(args=f"numchan=1 {device_args}")
        self.src.set_sample_rate(self.samp_rate_rf)
        self.src.set_center_freq(freq_hz)
        self.src.set_gain_mode(True)  # AGC; ajusta a mano si el ident sale ruidoso

        # --- Decimar a la banda de interés y sacar la envolvente AM ---
        taps_decim = gr_filter.firdes.low_pass(
            1.0, self.samp_rate_rf, 15_000, 3_000
        )
        self.decim = gr_filter.fir_filter_ccf(
            self.samp_rate_rf // self.samp_rate_env, taps_decim
        )
        self.envolvente = blocks.complex_to_mag(1)

        # --- Camino VAR: filtrar 30 Hz directo y bajar la tasa ---
        taps_var_bp = gr_filter.firdes.band_pass(
            1.0, self.samp_rate_env, 25, 35, 10
        )
        self.var_bp = gr_filter.fir_filter_fff(1, taps_var_bp)
        taps_var_decim = gr_filter.firdes.low_pass(
            1.0, self.samp_rate_env, 45, 20
        )
        self.var_decim = gr_filter.fir_filter_fff(
            self.samp_rate_env // self.samp_rate_30hz, taps_var_decim
        )
        self.var_hilbert = gr_filter.hilbert_fc(65)

        # --- Camino REF: aislar subportadora 9960 Hz, demodular FM ---
        taps_ref_bp = gr_filter.firdes.band_pass(
            1.0, self.samp_rate_env, 9480, 10440, 200
        )
        self.ref_bp = gr_filter.fir_filter_fff(1, taps_ref_bp)
        self.ref_hilbert_fm = gr_filter.hilbert_fc(65)
        quad_gain = self.samp_rate_env / (2 * np.pi * 480)
        self.quad_demod = analog.quadrature_demod_cf(quad_gain)
        taps_ref_lp = gr_filter.firdes.low_pass(
            1.0, self.samp_rate_env, 45, 20
        )
        self.ref_lp = gr_filter.fir_filter_fff(1, taps_ref_lp)
        self.ref_decim = gr_filter.fir_filter_fff(
            self.samp_rate_env // self.samp_rate_30hz, taps_ref_lp
        )
        self.ref_hilbert = gr_filter.hilbert_fc(65)

        # --- Comparador de fase ---
        self.mult_conj = blocks.multiply_conjugate_cc(1)
        self.c2arg = blocks.complex_to_arg(1)
        self.rad2deg = blocks.multiply_const_ff(180.0 / np.pi)

        # --- Salida serial hacia el ESP32 del KI-207 ---
        self.serial_sink = VorSerialSink(ki207_port)

        # --- Conexiones ---
        self.connect(self.src, self.decim, self.envolvente)

        self.connect(self.envolvente, self.var_bp, self.var_decim, self.var_hilbert)

        self.connect(
            self.envolvente, self.ref_bp, self.ref_hilbert_fm, self.quad_demod, self.ref_lp
        )
        self.connect(self.ref_lp, self.ref_decim, self.ref_hilbert)

        self.connect(self.var_hilbert, (self.mult_conj, 0))
        self.connect(self.ref_hilbert, (self.mult_conj, 1))
        self.connect(self.mult_conj, self.c2arg, self.rad2deg, self.serial_sink)

    def set_freq(self, freq_hz):
        self.src.set_center_freq(freq_hz)


# --------------------------------------------------------------------------
# Hilo que escucha los comandos del panel (frecuencia y curso seleccionado)
# --------------------------------------------------------------------------
class PanelListener(threading.Thread):
    def __init__(self, panel_port, tb, sink, baudrate=115200):
        super().__init__(daemon=True)
        self.tb = tb
        self.sink = sink
        self.running = True
        try:
            self.ser = serial.Serial(panel_port, baudrate, timeout=0.2)
        except serial.SerialException as e:
            print(f"[PanelListener] No se pudo abrir {panel_port}: {e}")
            self.ser = None

    def run(self):
        if self.ser is None:
            return
        while self.running:
            try:
                linea = self.ser.readline().decode(errors="ignore").strip()
            except serial.SerialException:
                time.sleep(0.5)
                continue
            if not linea:
                continue

            partes = linea.split(",")
            if partes[0] == "FREQ" and len(partes) >= 4 and partes[2] == "ACT":
                # FREQ,VOR1,ACT,113700   (frecuencia en kHz)
                try:
                    freq_hz = float(partes[3]) * 1000.0
                    self.tb.set_freq(freq_hz)
                    print(f"[PanelListener] Retonalizado a {freq_hz/1e6:.3f} MHz")
                except ValueError:
                    pass
            elif partes[0] == "COURSE" and len(partes) >= 2:
                # COURSE,270
                try:
                    self.sink.set_course(float(partes[1]))
                except ValueError:
                    pass

    def stop(self):
        self.running = False


def main():
    ap = argparse.ArgumentParser(description="Decodificador de radial VOR")
    ap.add_argument("--device-args", default="", help='ej: rtl=0')
    ap.add_argument("--freq", type=float, default=113_700_000.0, help="Hz")
    ap.add_argument("--ki207-port", required=True, help="ej: /dev/ttyUSB0")
    ap.add_argument("--panel-port", required=True, help="ej: /dev/ttyUSB1")
    args = ap.parse_args()

    tb = VorTopBlock(args.device_args, args.freq, args.ki207_port)
    panel = PanelListener(args.panel_port, tb, tb.serial_sink)

    tb.start()
    panel.start()
    print("Flowgraph corriendo. Ctrl+C para detener.")

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Deteniendo...")
    finally:
        panel.stop()
        tb.stop()
        tb.wait()


if __name__ == "__main__":
    main()
