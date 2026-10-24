#!/usr/bin/env python3
"""
vor_radial_test_probe.py — Prueba SOLO el cálculo del radial, sin
necesitar el puente, el serial, ni el ESP32/KI-207 para nada.

Usa blocks.probe_signal_f(), un bloque 100% nativo de GNU Radio que
guarda el último valor calculado. Tu propio script lee ese valor
periódicamente (probe.level()) desde el hilo principal — no desde un
callback de GNU Radio, así que no corre ningún riesgo del bug de los
bloques personalizados de Python.

Uso:
    python vor_radial_test_probe.py --freq 113900000
"""

import argparse
import time

import numpy as np

from gnuradio import gr, blocks, filter as gr_filter, analog
import osmosdr


class VorProbeTB(gr.top_block):
    def __init__(self, device_args, freq_hz, use_agc=False, rf_gain=49.6, if_gain=40, bb_gain=20, hilbert_taps=191):
        gr.top_block.__init__(self, "VOR Radial - Solo Prueba")

        self.samp_rate_rf = 1_000_000
        self.samp_rate_env = 100_000
        self.samp_rate_30hz = 1_000

        self.src = osmosdr.source(args=f"numchan=1 {device_args}")
        self.src.set_sample_rate(self.samp_rate_rf)
        self.src.set_center_freq(freq_hz)

        if use_agc:
            self.src.set_gain_mode(True)
            print("[AGC automático activado]")
        else:
            self.src.set_gain_mode(False)
            # Las 3 etapas por separado, igual que en el flowgraph que
            # ya funciona para recibir el identificador con claridad.
            self.src.set_gain(rf_gain, "RF")
            self.src.set_gain(if_gain, "IF")
            self.src.set_gain(bb_gain, "BB")
            print(f"[Ganancia manual: RF={rf_gain} IF={if_gain} BB={bb_gain}, AGC desactivado]")

        taps_decim = gr_filter.firdes.low_pass(1.0, self.samp_rate_rf, 15_000, 3_000)
        self.decim = gr_filter.fir_filter_ccf(
            self.samp_rate_rf // self.samp_rate_env, taps_decim
        )
        self.envolvente = blocks.complex_to_mag(1)

        taps_var_bp = gr_filter.firdes.band_pass(1.0, self.samp_rate_env, 25, 35, 10)
        self.var_bp = gr_filter.fir_filter_fff(1, taps_var_bp)
        taps_var_decim = gr_filter.firdes.low_pass(1.0, self.samp_rate_env, 45, 20)
        self.var_decim = gr_filter.fir_filter_fff(
            self.samp_rate_env // self.samp_rate_30hz, taps_var_decim
        )
        self.var_hilbert = gr_filter.hilbert_fc(hilbert_taps)

        taps_ref_bp = gr_filter.firdes.band_pass(1.0, self.samp_rate_env, 9480, 10440, 100)
        self.ref_bp = gr_filter.fir_filter_fff(1, taps_ref_bp)
        self.ref_hilbert_fm = gr_filter.hilbert_fc(hilbert_taps)
        quad_gain = self.samp_rate_env / (2 * np.pi * 480)
        self.quad_demod = analog.quadrature_demod_cf(quad_gain)
        taps_ref_lp = gr_filter.firdes.low_pass(1.0, self.samp_rate_env, 45, 20)
        self.ref_lp = gr_filter.fir_filter_fff(1, taps_ref_lp)
        self.ref_decim = gr_filter.fir_filter_fff(
            self.samp_rate_env // self.samp_rate_30hz, taps_ref_lp
        )
        self.ref_hilbert = gr_filter.hilbert_fc(hilbert_taps)

        self.mult_conj = blocks.multiply_conjugate_cc(1)
        self.c2arg = blocks.complex_to_arg(1)
        self.rad2deg = blocks.multiply_const_ff(180.0 / np.pi)

        # --- Aquí está la parte nueva: el probe nativo ---
        self.probe = blocks.probe_signal_f()

        self.connect(self.src, self.decim, self.envolvente)
        self.connect(self.envolvente, self.var_bp, self.var_decim, self.var_hilbert)
        self.connect(
            self.envolvente, self.ref_bp, self.ref_hilbert_fm, self.quad_demod, self.ref_lp
        )
        self.connect(self.ref_lp, self.ref_decim, self.ref_hilbert)
        self.connect(self.var_hilbert, (self.mult_conj, 0))
        self.connect(self.ref_hilbert, (self.mult_conj, 1))
        self.connect(self.mult_conj, self.c2arg, self.rad2deg, self.probe)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--device-args", default="")
    ap.add_argument("--freq", type=float, required=True)
    ap.add_argument("--agc", action="store_true", help="usar AGC automático en vez de ganancia manual")
    ap.add_argument("--rf-gain", type=float, default=49.6)
    ap.add_argument("--if-gain", type=float, default=40.0)
    ap.add_argument("--bb-gain", type=float, default=20.0)
    ap.add_argument("--avg-seconds", type=float, default=1.0, help="segundos a promediar antes de mostrar cada lectura")
    args = ap.parse_args()

    tb = VorProbeTB(
        args.device_args, args.freq,
        use_agc=args.agc, rf_gain=args.rf_gain, if_gain=args.if_gain, bb_gain=args.bb_gain,
    )
    tb.start()
    print(f"Corriendo. Promediando el radial cada {args.avg_seconds:.1f}s. Ctrl+C para detener.\n")

    n_lecturas = max(1, int(args.avg_seconds / 0.05))

    try:
        while True:
            # Promedio circular: se promedian el seno y el coseno por
            # separado, y se recalcula el ángulo al final. Promediar
            # el ángulo crudo directamente fallaría cerca de 0°/360°.
            suma_sin = 0.0
            suma_cos = 0.0
            for _ in range(n_lecturas):
                angulo_rad = np.radians(tb.probe.level())
                suma_sin += np.sin(angulo_rad)
                suma_cos += np.cos(angulo_rad)
                time.sleep(0.05)  # 20 lecturas en 1 segundo

            radial_promedio = np.degrees(np.arctan2(suma_sin, suma_cos)) % 360.0

            # Qué tan "de acuerdo" están las 20 lecturas entre sí (1.0 =
            # perfectamente estables, cercano a 0 = puro ruido disperso)
            consistencia = np.sqrt(suma_sin**2 + suma_cos**2) / n_lecturas

            print(f"Radial promedio: {radial_promedio:6.1f}°   (consistencia: {consistencia:.2f})")
    except KeyboardInterrupt:
        pass
    finally:
        tb.stop()
        tb.wait()


if __name__ == "__main__":
    main()
