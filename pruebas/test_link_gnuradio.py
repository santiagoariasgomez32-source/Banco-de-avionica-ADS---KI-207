#!/usr/bin/env python3
"""
test_link_gnuradio.py — Prueba mínima de conectividad de HOY.

Objetivo: confirmar que un dato real capturado por la antena llega vivo
hasta el ESP32. NO calcula el radial todavía (eso ya lo tienes en
vor_radial_decoder.py) — solo mide el nivel de la envolvente AM y lo
manda por serial, para probar el enlace de punta a punta.

Uso:
    python3 test_link_gnuradio.py --freq 113700000 --esp32-port /dev/ttyUSB0

    --freq: frecuencia en Hz de una estación VOR o COM que ya sabes que
            recibes bien (la misma que usaste cuando lograste el ident).
    --esp32-port: el puerto donde aparece tu ESP32 (Administrador de
            dispositivos en Windows -> COMx, o /dev/ttyUSB0 / /dev/ttyACM0
            en Linux). Es el MISMO puerto que usas para programarlo, así
            que cierra el Monitor Serial del IDE de Arduino antes de
            correr este script — el puerto solo lo puede usar un
            programa a la vez.
"""

import argparse
import time

import numpy as np
import serial

from gnuradio import gr, blocks, filter as gr_filter
import osmosdr


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


class TestLinkTB(gr.top_block):
    def __init__(self, device_args, freq_hz, esp32_port):
        gr.top_block.__init__(self, "Test Link")
        samp_rate = 1_000_000
        env_rate = 100_000

        self.src = osmosdr.source(args=f"numchan=1 {device_args}")
        self.src.set_sample_rate(samp_rate)
        self.src.set_center_freq(freq_hz)
        self.src.set_gain_mode(True)

        taps = gr_filter.firdes.low_pass(1.0, samp_rate, 15_000, 3_000)
        decim = gr_filter.fir_filter_ccf(samp_rate // env_rate, taps)
        envolvente = blocks.complex_to_mag(1)

        # Reduce a ~10 mensajes por segundo antes de mandar por serial
        keep = blocks.keep_one_in_n(gr.sizeof_float, env_rate // 10)
        sink = SerialLevelSink(esp32_port)

        self.connect(self.src, decim, envolvente, keep, sink)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--device-args", default="")
    ap.add_argument(
        "--freq", type=float, required=True,
        help="Hz, ej: 113700000 para un VOR o 124300000 para COM",
    )
    ap.add_argument("--esp32-port", required=True, help="ej: /dev/ttyUSB0 o COM5")
    args = ap.parse_args()

    tb = TestLinkTB(args.device_args, args.freq, args.esp32_port)
    tb.start()
    print("Corriendo. Ctrl+C para detener. Observa el LED del ESP32.")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        tb.stop()
        tb.wait()


if __name__ == "__main__":
    main()
