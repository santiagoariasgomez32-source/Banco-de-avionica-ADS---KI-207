#!/usr/bin/env python3
"""
vor_radial_pll_probe.py — Decodifica el radial VOR usando lazos de
enganche de fase (PLL), igual que lo hace un receptor VOR real, en vez
de medir la fase de un instante aislado (que es frágil ante el ruido).

Cómo funciona un receptor real (y lo que replicamos aquí):
  1. Se extrae la envolvente AM (igual que antes).
  2. La señal REF (en la subportadora de 9960 Hz) se baja a banda base
     mezclándola con un oscilador local a -9960 Hz, y luego un PLL
     hace el demodulado FM seguido de la señal, filtrando el ruido
     con su ancho de banda de lazo (en vez de medir instante a
     instante como quadrature_demod).
  3. Tanto REF como VAR pasan por un segundo PLL (pll_refout_cc) que
     se "engancha" a su tono de 30 Hz y genera una versión limpia,
     de amplitud constante, siguiendo esa fase de forma continua.
  4. La diferencia de fase entre esas dos señales limpias, ya
     enganchadas, es el radial — mucho más resistente al ruido que
     una medición instantánea.

Solo para pruebas: no manda nada por serial ni UDP, solo imprime el
radial usando un probe nativo (sin riesgo del bug de bloques Python).

Uso:
    python vor_radial_pll_probe.py --freq 113900000
"""

import argparse
import time

import numpy as np

from gnuradio import gr, blocks, filter as gr_filter, analog
import osmosdr


class VorPllTB(gr.top_block):
    def __init__(self, device_args, freq_hz, rf_gain=49.6, if_gain=40, bb_gain=20,
                 var_loop_bw_hz=3.0, ref_loop_bw_hz=300.0):
        gr.top_block.__init__(self, "VOR Radial con PLL")

        self.samp_rate_rf = 1_000_000
        self.samp_rate_env = 100_000

        # --- Fuente SDR, ganancias manuales máximas (ya confirmadas) ---
        self.src = osmosdr.source(args=f"numchan=1 {device_args}")
        self.src.set_sample_rate(self.samp_rate_rf)
        self.src.set_center_freq(freq_hz)
        self.src.set_gain_mode(False)
        self.src.set_gain(rf_gain, "RF")
        self.src.set_gain(if_gain, "IF")
        self.src.set_gain(bb_gain, "BB")

        taps_decim = gr_filter.firdes.low_pass(1.0, self.samp_rate_rf, 15_000, 3_000)
        self.decim = gr_filter.fir_filter_ccf(
            self.samp_rate_rf // self.samp_rate_env, taps_decim
        )
        self.envolvente = blocks.complex_to_mag(1)

        # ================== CAMINO VAR ==================
        taps_var_bp = gr_filter.firdes.band_pass(1.0, self.samp_rate_env, 20, 40, 15)
        self.var_bp = gr_filter.fir_filter_fff(1, taps_var_bp)
        self.var_hilbert = gr_filter.hilbert_fc(129)

        # Oscilador local COMPARTIDO de -30 Hz: baja tanto VAR como
        # REF a "casi cero" antes de sus PLLs angostos. Como el mismo
        # oscilador se usa para ambos caminos, la diferencia de fase
        # entre VAR y REF no se altera (se cancela en la comparación
        # final) — solo les damos a los PLLs un punto de partida
        # dentro de su rango de captura real.
        self.lo_30hz = analog.sig_source_c(self.samp_rate_env, analog.GR_COS_WAVE, -30.0, 1.0, 0.0)
        self.var_mixer = blocks.multiply_cc(1)

        # Ahora el PLL solo necesita corregir un pequeño error
        # residual cerca de 0 Hz, no alcanzar 30 Hz desde cero.
        var_loop_bw = 2 * np.pi * var_loop_bw_hz / self.samp_rate_env
        var_max_freq = 2 * np.pi * 8 / self.samp_rate_env
        var_min_freq = -var_max_freq
        self.var_pll = analog.pll_refout_cc(var_loop_bw, var_max_freq, var_min_freq)

        # ================== CAMINO REF ==================
        taps_ref_bp = gr_filter.firdes.band_pass(1.0, self.samp_rate_env, 9480, 10440, 150)
        self.ref_bp = gr_filter.fir_filter_fff(1, taps_ref_bp)
        self.ref_hilbert = gr_filter.hilbert_fc(129)

        # Oscilador local para bajar la subportadora de 9960 Hz a banda base
        self.lo = analog.sig_source_c(self.samp_rate_env, analog.GR_COS_WAVE, -9960.0, 1.0, 0.0)
        self.mixer = blocks.multiply_cc(1)

        # PLL que demodula FM (sigue la frecuencia instantánea, ya
        # centrada cerca de 0 Hz tras bajar la subportadora)
        ref_loop_bw = 2 * np.pi * ref_loop_bw_hz / self.samp_rate_env
        ref_max_freq = 2 * np.pi * 700 / self.samp_rate_env
        ref_min_freq = -ref_max_freq
        self.ref_pll_fm = analog.pll_freqdet_cf(ref_loop_bw, ref_max_freq, ref_min_freq)

        # Limpiar, bajar a banda base con el MISMO oscilador de -30 Hz,
        # y volver a enganchar con un PLL angosto igual que en VAR.
        taps_ref_bp2 = gr_filter.firdes.band_pass(1.0, self.samp_rate_env, 20, 40, 15)
        self.ref_bp2 = gr_filter.fir_filter_fff(1, taps_ref_bp2)
        self.ref_hilbert2 = gr_filter.hilbert_fc(129)
        self.ref_mixer = blocks.multiply_cc(1)
        self.ref_pll = analog.pll_refout_cc(var_loop_bw, var_max_freq, var_min_freq)

        # ================== COMPARADOR DE FASE ==================
        self.mult_conj = blocks.multiply_conjugate_cc(1)
        self.c2arg = blocks.complex_to_arg(1)
        self.rad2deg = blocks.multiply_const_ff(180.0 / np.pi)
        self.probe = blocks.probe_signal_f()

        # --- Conexiones ---
        self.connect(self.src, self.decim, self.envolvente)

        # VAR: filtrar -> bajar a banda base con el LO de -30Hz -> PLL angosto
        self.connect(self.envolvente, self.var_bp, self.var_hilbert)
        self.connect(self.var_hilbert, (self.var_mixer, 0))
        self.connect(self.lo_30hz, (self.var_mixer, 1))
        self.connect(self.var_mixer, self.var_pll)

        # REF: filtrar -> bajar la subportadora a banda base -> PLL de FM ->
        # limpiar -> bajar a banda base con el MISMO LO de -30Hz -> PLL angosto
        self.connect(self.envolvente, self.ref_bp, self.ref_hilbert)
        self.connect(self.ref_hilbert, (self.mixer, 0))
        self.connect(self.lo, (self.mixer, 1))
        self.connect(self.mixer, self.ref_pll_fm)
        self.connect(self.ref_pll_fm, self.ref_bp2, self.ref_hilbert2)
        self.connect(self.ref_hilbert2, (self.ref_mixer, 0))
        self.connect(self.lo_30hz, (self.ref_mixer, 1))
        self.connect(self.ref_mixer, self.ref_pll)

        # Comparador
        self.connect(self.var_pll, (self.mult_conj, 0))
        self.connect(self.ref_pll, (self.mult_conj, 1))
        self.connect(self.mult_conj, self.c2arg, self.rad2deg, self.probe)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--device-args", default="")
    ap.add_argument("--freq", type=float, required=True)
    ap.add_argument("--rf-gain", type=float, default=49.6)
    ap.add_argument("--if-gain", type=float, default=40.0)
    ap.add_argument("--bb-gain", type=float, default=20.0)
    ap.add_argument("--var-loop-bw", type=float, default=3.0, help="ancho de banda del lazo VAR/REF final, en Hz")
    ap.add_argument("--ref-loop-bw", type=float, default=300.0, help="ancho de banda del lazo de FM (REF), en Hz")
    ap.add_argument("--avg-seconds", type=float, default=1.0)
    ap.add_argument("--offset", type=float, default=228.6,
                     help="constante de calibración en grados, calculada con tus coordenadas GPS reales vs. el VOR BOG")
    args = ap.parse_args()

    tb = VorPllTB(
        args.device_args, args.freq,
        rf_gain=args.rf_gain, if_gain=args.if_gain, bb_gain=args.bb_gain,
        var_loop_bw_hz=args.var_loop_bw, ref_loop_bw_hz=args.ref_loop_bw,
    )
    tb.start()
    print("Corriendo con PLLs. Dale unos segundos para que 'enganchen' antes de confiar en el número.")
    print(f"Promediando cada {args.avg_seconds:.1f}s. Ctrl+C para detener.\n")

    n_lecturas = max(1, int(args.avg_seconds / 0.05))

    try:
        while True:
            suma_sin = 0.0
            suma_cos = 0.0
            for _ in range(n_lecturas):
                angulo_rad = np.radians(tb.probe.level())
                suma_sin += np.sin(angulo_rad)
                suma_cos += np.cos(angulo_rad)
                time.sleep(0.05)

            radial_crudo = np.degrees(np.arctan2(suma_sin, suma_cos)) % 360.0
            radial_calibrado = (radial_crudo - args.offset) % 360.0
            consistencia = np.sqrt(suma_sin**2 + suma_cos**2) / n_lecturas
            print(f"Radial: {radial_calibrado:6.1f}°   (crudo: {radial_crudo:6.1f}°, consistencia: {consistencia:.2f})")
    except KeyboardInterrupt:
        pass
    finally:
        tb.stop()
        tb.wait()


if __name__ == "__main__":
    main()
