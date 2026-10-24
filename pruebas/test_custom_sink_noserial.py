#!/usr/bin/env python3
"""
test_custom_sink_noserial.py — Prueba un bloque personalizado que NO
toca el puerto serial para nada, solo cuenta muestras y las imprime.

Si esto funciona bien, el problema es específicamente el puerto
serial dentro del bloque. Si esto TAMBIÉN se cierra solo, el problema
es cualquier bloque personalizado de Python en este entorno de GNU
Radio (algo más profundo de la instalación).
"""

import argparse
import time

import numpy as np

from gnuradio import gr, blocks


class CounterSink(gr.sync_block):
    def __init__(self):
        gr.sync_block.__init__(
            self, name="counter_sink", in_sig=[np.float32], out_sig=None
        )
        self.total = 0

    def work(self, input_items, output_items):
        self.total += len(input_items[0])
        if self.total % 100 == 0:
            print(f"  (bloque) muestras acumuladas: {self.total}")
        return len(input_items[0])


class DummyTB(gr.top_block):
    def __init__(self):
        gr.top_block.__init__(self, "Dummy Counter Test")

        datos_falsos = list(np.linspace(0, 1, 1000, dtype=np.float32))
        src = blocks.vector_source_f(datos_falsos, repeat=True)
        throttle = blocks.throttle(gr.sizeof_float, 100)
        keep = blocks.keep_one_in_n(gr.sizeof_float, 10)
        sink = CounterSink()

        self.connect(src, throttle, keep, sink)


def main():
    print("Construyendo flowgraph de prueba (sin serial, sin SDR)...")
    tb = DummyTB()

    print("Iniciando (tb.start())...")
    tb.start()

    print("Si ves este mensaje y sigue corriendo, el bloque personalizado SÍ funciona.")
    for i in range(8):
        time.sleep(1)
        print(f"  ...{i+1} segundo(s) sin caerse")

    print("Deteniendo limpiamente.")
    tb.stop()
    tb.wait()
    print("Listo, terminó sin errores.")


if __name__ == "__main__":
    main()
