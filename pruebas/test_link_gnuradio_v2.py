#!/usr/bin/env python3
"""
test_link_gnuradio_v2.py — Igual que la primera versión, pero SIN
ningún bloque de Python personalizado (para evitar el bug conocido de
GNU Radio + Python 3.12 con bloques Python). En vez de escribir al
serial directamente desde el flowgraph, manda los datos por UDP a
localhost usando un bloque 100% nativo (blocks.udp_sink).

Corre esto, y en OTRA ventana de terminal corre
udp_to_serial_bridge.py, que recibe esos datos por UDP y sí los manda
al puerto serial del ESP32 (ese script no importa gnuradio para nada,
así que no le afecta el bug).
"""

import argparse
import time

from gnuradio import gr, blocks, network, filter as gr_filter
import osmosdr


class TestLinkTB(gr.top_block):
    def __init__(self, device_args, freq_hz, udp_port):
        gr.top_block.__init__(self, "Test Link v2 (UDP)")
        samp_rate = 1_000_000
        env_rate = 100_000

        self.src = osmosdr.source(args=f"numchan=1 {device_args}")
        self.src.set_sample_rate(samp_rate)
        self.src.set_center_freq(freq_hz)
        self.src.set_gain_mode(True)

        taps = gr_filter.firdes.low_pass(1.0, samp_rate, 15_000, 3_000)
        decim = gr_filter.fir_filter_ccf(samp_rate // env_rate, taps)
        envolvente = blocks.complex_to_mag(1)

        keep = blocks.keep_one_in_n(gr.sizeof_float, env_rate // 10)

        # Bloque NATIVO (no Python) que manda cada float por UDP a localhost.
        # Desde GNU Radio 3.9/3.10 este bloque vive en el módulo "network",
        # no en "blocks" (antes era blocks.udp_sink). Esta versión del
        # binding exige los 3 últimos parámetros explícitos (sin default):
        # header_type=0 (sin encabezado), payloadsize=1472, send_eof=True.
        udp = network.udp_sink(gr.sizeof_float, 1, "127.0.0.1", udp_port, 0, 1472, True)

        self.connect(self.src, decim, envolvente, keep, udp)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--device-args", default="")
    ap.add_argument("--freq", type=float, required=True)
    ap.add_argument("--udp-port", type=int, default=5005)
    args = ap.parse_args()

    tb = TestLinkTB(args.device_args, args.freq, args.udp_port)
    tb.start()
    print(f"Corriendo. Mandando datos por UDP a 127.0.0.1:{args.udp_port}")
    print("Ctrl+C para detener.")
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
