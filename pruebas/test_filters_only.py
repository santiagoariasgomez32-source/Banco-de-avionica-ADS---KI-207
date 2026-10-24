#!/usr/bin/env python3
"""
test_filters_only.py — Aísla el problema, segunda parte.

Igual que test_sdr_only.py, pero ahora SÍ incluye la cadena de filtros
(decimar + envolvente + keep_one_in_n), aunque termina en un null_sink
en vez del bloque que escribe al puerto serial.

Si este script funciona bien (corre 5 segundos sin cerrarse), el
problema está específicamente en el bloque que escribe al serial.
Si este también se cierra solo, el problema está en la cadena de
filtros (probablemente el fir_filter_ccf con muchos taps).
"""

import argparse
import time

from gnuradio import gr, blocks, filter as gr_filter
import osmosdr


class FiltersOnlyTB(gr.top_block):
    def __init__(self, device_args, freq_hz):
        gr.top_block.__init__(self, "Filters Only Test")

        samp_rate = 1_000_000
        env_rate = 100_000

        self.src = osmosdr.source(args=f"numchan=1 {device_args}")
        self.src.set_sample_rate(samp_rate)
        self.src.set_center_freq(freq_hz)
        self.src.set_gain_mode(True)

        print("Diseñando filtro de decimación (puede tardar unos segundos)...")
        taps = gr_filter.firdes.low_pass(1.0, samp_rate, 15_000, 3_000)
        print(f"Filtro diseñado con {len(taps)} taps.")
        decim = gr_filter.fir_filter_ccf(samp_rate // env_rate, taps)

        envolvente = blocks.complex_to_mag(1)
        keep = blocks.keep_one_in_n(gr.sizeof_float, env_rate // 10)
        sink = blocks.null_sink(gr.sizeof_float)

        self.connect(self.src, decim, envolvente, keep, sink)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--device-args", default="")
    ap.add_argument("--freq", type=float, required=True)
    args = ap.parse_args()

    print("Construyendo flowgraph...")
    tb = FiltersOnlyTB(args.device_args, args.freq)

    print("Iniciando (tb.start())...")
    tb.start()

    print("Si ves este mensaje y el programa sigue corriendo, los filtros SÍ funcionan.")
    for i in range(5):
        time.sleep(1)
        print(f"  ...{i+1} segundo(s) sin caerse")

    print("Deteniendo limpiamente.")
    tb.stop()
    tb.wait()
    print("Listo, terminó sin errores.")


if __name__ == "__main__":
    main()
