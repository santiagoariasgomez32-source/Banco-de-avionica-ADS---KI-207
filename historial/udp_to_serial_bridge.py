#!/usr/bin/env python3
"""
udp_to_serial_bridge.py — Puente independiente: NO importa gnuradio
para nada (por eso no le afecta el bug de Python 3.12). Escucha datos
que llegan por UDP (mandados por test_link_gnuradio_v2.py) y los
reenvía al ESP32 por puerto serial.

Uso:
    python udp_to_serial_bridge.py --esp32-port COM3 --udp-port 5005
"""

import argparse
import socket
import struct
import time

import serial


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--esp32-port", required=True)
    ap.add_argument("--udp-port", type=int, default=5005)
    ap.add_argument("--baudrate", type=int, default=115200)
    args = ap.parse_args()

    print(f"Abriendo puerto serial {args.esp32_port}...")
    ser = serial.Serial(args.esp32_port, args.baudrate, timeout=0)
    print("Puerto serial abierto.")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", args.udp_port))
    print(f"Escuchando UDP en 127.0.0.1:{args.udp_port}...")
    print("Esperando datos del flowgraph de GNU Radio. Ctrl+C para detener.")

    contador = 0
    contador_ventana = 0
    inicio_ventana = time.time()
    try:
        while True:
            data, _ = sock.recvfrom(4096)
            n_floats = len(data) // 4
            if n_floats == 0:
                continue
            valores = struct.unpack(f"<{n_floats}f", data)

            for nivel in valores:
                linea = f"LEVEL,{nivel:.5f}\n"
                ser.write(linea.encode())
                contador += 1
                contador_ventana += 1

            # Mostrar cualquier respuesta que el ESP32 haya mandado
            # (sus mensajes ACK,<total_recibido_por_el>) para comparar
            # contra lo que nosotros le mandamos.
            if ser.in_waiting > 0:
                respuesta = ser.read(ser.in_waiting).decode(errors="ignore")
                for linea_resp in respuesta.strip().splitlines():
                    if linea_resp.startswith("ACK,"):
                        print(f"    [ESP32 confirma haber recibido: {linea_resp}]")

            ahora = time.time()
            if ahora - inicio_ventana >= 2.0:
                tasa = contador_ventana / (ahora - inicio_ventana)
                print(f"  Total: {contador} | Tasa real: {tasa:.1f} valores/seg")
                contador_ventana = 0
                inicio_ventana = ahora
    except KeyboardInterrupt:
        print("Deteniendo...")
    finally:
        ser.close()
        sock.close()


if __name__ == "__main__":
    main()
