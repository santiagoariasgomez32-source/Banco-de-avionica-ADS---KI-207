#!/usr/bin/env python3
"""
test_sdr_only.py — Aísla el problema: abre el SDR y NO hace nada más.

Si este script también se cierra solo sin error, el problema es del
driver/hardware del RTL-SDR en Windows, no de los bloques de filtrado
que agregamos. Si este SÍ funciona (corre 5 segundos sin cerrarse),
el problema está en la cadena de filtros del otro script.
"""

import argparse
import time

from gnuradio import gr, blocks
import osmosdr


class SdrOnlyTB(gr.top_block):
    def __init__(self, device_args, freq_hz):
        gr.top_block.__init__(self, "SDR Only Test")

        self.src = osmosdr.source(args=f"numchan=1 {device_args}")
        self.src.set_sample_rate(1_000_000)
        self.src.set_center_freq(freq_hz)
        self.src.set_gain_mode(True)

        self.sink = blocks.null_sink(gr.sizeof_gr_complex)

        self.connect(self.src, self.sink)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--device-args", default="")
    ap.add_argument("--freq", type=float, required=True)
    args = ap.parse_args()

    print("Construyendo flowgraph...")
    tb = SdrOnlyTB(args.device_args, args.freq)

    print("Iniciando (tb.start())...")
    tb.start()

    print("Si ves este mensaje y el programa sigue corriendo, el SDR SÍ arrancó bien.")
    print("Esperando 5 segundos...")
    for i in range(5):
        time.sleep(1)
        print(f"  ...{i+1} segundo(s) sin caerse")

    print("Deteniendo limpiamente.")
    tb.stop()
    tb.wait()
    print("Listo, terminó sin errores.")


if __name__ == "__main__":
    main()
