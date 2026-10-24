#!/usr/bin/env python3
"""
ver_radial.py - Muestra en consola el radial que decodifica el decodificador VOR,
sin necesitar el panel ni el KI-207 (solo la antena con el RTL-SDR).

Escucha por UDP (127.0.0.1:6006) los valores de radial (grados, float32, ~20 por
segundo) que manda vor_radial_decoder_pll.py y cada segundo imprime:
  - el radial promedio (promedio circular, para no fallar cerca de 0/360)
  - la consistencia (0 a 1; cerca de 1 = la fase es muy estable)

Uso:  python ver_radial.py
      python ver_radial.py --port 6006 --ventana 1.0
"""
import argparse
import math
import socket
import struct
import time


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=6006)
    ap.add_argument("--ventana", type=float, default=1.0, help="segundos por cada linea")
    ap.add_argument("--min-consistency", type=float, default=0.6)
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", args.port))
    sock.settimeout(0.5)
    print(f"Escuchando radial en UDP 127.0.0.1:{args.port}. Ctrl+C para salir.")
    print("Los PLL tardan unos segundos en enganchar. Cierra el puente si esta abierto.\n")

    t0 = time.time()
    valores = []
    total = 0
    try:
        while True:
            try:
                data, _ = sock.recvfrom(4096)
                n = len(data) // 4
                valores.extend(struct.unpack("<%df" % n, data[: n * 4]))
                total += n
            except socket.timeout:
                if time.time() - t0 > 3 and not valores:
                    print("  sin datos del decodificador (esta corriendo vor_radial_decoder_pll.py?)")
                    t0 = time.time()
            if valores and time.time() - t0 >= args.ventana:
                s = sum(math.sin(math.radians(v)) for v in valores)
                c = sum(math.cos(math.radians(v)) for v in valores)
                r = math.hypot(s, c) / len(valores)
                rad = math.degrees(math.atan2(s, c)) % 360.0
                estado = "OK" if r >= args.min_consistency else "senal debil / inestable"
                print(f"  radial = {rad:6.1f} deg   consistencia = {r:.2f}   [{estado}]")
                valores = []
                t0 = time.time()
    except KeyboardInterrupt:
        print("\nDeteniendo...")
    finally:
        sock.close()


if __name__ == "__main__":
    main()
