#!/usr/bin/env python3
"""
test_custom_sink_dummy.py — Prueba el bloque SerialLevelSink DENTRO de
un flowgraph de GNU Radio, pero sin ningún SDR real — los datos vienen
de un generador falso (vector_source). Esto aísla si el problema es
del bloque personalizado en sí (su interacción con el scheduler de
GNU Radio) o algo específico de mezclarlo con el hardware SDR.
"""

import argparse
import time

import numpy as np
import serial

from gnuradio import gr, blocks


class SerialLevelSink(gr.sync_block):
    def __init__(self, port, baudrate=115200):
        gr.sync_block.__init__(
            self, name="serial_level_sink", in_sig=[np.float32], out_sig=None
        )
        self.ser = None
        try:
            self.ser = serial.Serial(port, baudrate, timeout=0)
            print(f"[OK] Puerto serial {port} abierto")
        except serial.SerialException as e:
            print(f"[ERROR] No se pudo abrir {port}: {e}")

    def work(self, input_items, output_items):
        nivel = float(np.mean(np.abs(input_items[0])))
        if self.ser is not None:
            try:
                self.ser.write(f"LEVEL,{nivel:.5f}\n".encode())
            except serial.SerialException:
                pass
        return len(input_items[0])


class DummyTB(gr.top_block):
    def __init__(self, port):
        gr.top_block.__init__(self, "Dummy Custom Sink Test")

        # Genera una onda falsa en vez de leer del SDR
        datos_falsos = list(np.linspace(0, 1, 1000, dtype=np.float32))
        src = blocks.vector_source_f(datos_falsos, repeat=True)
        throttle = blocks.throttle(gr.sizeof_float, 100)  # ~100 muestras/seg
        keep = blocks.keep_one_in_n(gr.sizeof_float, 10)  # ~10 mensajes/seg
        sink = SerialLevelSink(port)

        self.connect(src, throttle, keep, sink)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--esp32-port", required=True)
    args = ap.parse_args()

    print("Construyendo flowgraph de prueba (sin SDR)...")
    tb = DummyTB(args.esp32_port)

    print("Iniciando (tb.start())...")
    tb.start()

    print("Si ves este mensaje y sigue corriendo, el bloque personalizado SÍ funciona en GNU Radio.")
    for i in range(8):
        time.sleep(1)
        print(f"  ...{i+1} segundo(s) sin caerse")

    print("Deteniendo limpiamente.")
    tb.stop()
    tb.wait()
    print("Listo, terminó sin errores.")


if __name__ == "__main__":
    main()
