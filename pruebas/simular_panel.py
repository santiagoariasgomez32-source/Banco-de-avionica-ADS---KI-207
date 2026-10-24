#!/usr/bin/env python3
"""
simular_panel.py - Hace de "PC" para probar el panel SIN antena ni GNU Radio.

Se conecta al ESP32 del panel por serial, lee el curso y la frecuencia que
elijas con los encoders y le devuelve cada segundo un radial simulado
(oscila alrededor de 109 grados), con la desviacion del CDI, la bandera
TO/FROM y la consistencia, igual que lo haria vor_udp_bridge.py.

Uso:
    python simular_panel.py COM3
    python simular_panel.py COM3 --ident          (tambien manda el ident "BOG")
    python simular_panel.py COM3 --sin-senal      (manda FLAG=OFF para ver el estado sin senal)
    python simular_panel.py COM4 --curso 109      (probar el ESP32 del KI-207 sin panel: la aguja oscila)

IMPORTANTE: cierra el Monitor Serie del Arduino antes (ocupa el puerto).
Ctrl+C para salir.
"""

import argparse
import math
import time

import serial


def angle_diff(a, b):
    return (a - b + 180.0) % 360.0 - 180.0


def calcular_cdi(radial, curso, full_scale_deg=10.0):
    # misma logica que vor_udp_bridge.py
    diff = angle_diff(radial, curso)
    if abs(diff) <= 90.0:
        dev, flag = -diff / full_scale_deg, "FROM"
    else:
        dev, flag = angle_diff(radial, curso + 180.0) / full_scale_deg, "TO"
    return max(-1.0, min(1.0, dev)), flag


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("puerto")
    ap.add_argument("--baudrate", type=int, default=115200)
    ap.add_argument("--ident", action="store_true", help="manda tambien ID,BOG")
    ap.add_argument("--sin-senal", action="store_true", help="manda FLAG=OFF")
    ap.add_argument("--curso", type=float, default=0.0,
                    help="curso fijo para la prueba (p.ej. 109 para que la aguja oscile a ambos lados); "
                         "si hay un panel conectado, lo reemplaza el curso del encoder")
    args = ap.parse_args()

    ser = serial.Serial(args.puerto, args.baudrate, timeout=0.05)
    print(f"Simulando PC en {args.puerto}. Ctrl+C para salir.")

    curso = args.curso % 360.0
    freq = None
    buf = b""
    t0 = time.time()
    t_envio = 0.0
    try:
        while True:
            buf += ser.read(256)
            while b"\n" in buf:
                linea, buf = buf.split(b"\n", 1)
                p = linea.decode(errors="ignore").strip().split(",")
                try:
                    if p[0] == "COURSE" and len(p) == 2:
                        curso = float(p[1])
                        print(f"[Panel] curso = {curso:.0f}")
                    elif p[0] == "FREQ" and len(p) == 4:
                        nueva = int(p[3])
                        if nueva != freq:
                            freq = nueva
                            print(f"[Panel] frecuencia activa = {nueva / 1000:.3f} MHz")
                except ValueError:
                    pass

            ahora = time.time()
            if ahora - t_envio >= 1.0:
                t_envio = ahora
                radial = 109.0 + 15.0 * math.sin((ahora - t0) / 8.0)
                cons = 0.92 - 0.05 * math.sin((ahora - t0) / 3.0)
                if args.sin_senal:
                    msg = f"DEV,0.000,FLAG,OFF,RAD,{radial:.1f},CONS,0.20"
                else:
                    dev, flag = calcular_cdi(radial, curso)
                    msg = f"DEV,{dev:.3f},FLAG,{flag},RAD,{radial:.1f},CONS,{cons:.2f}"
                if args.ident and not args.sin_senal:
                    msg += ",ID,BOG"
                ser.write((msg + "\n").encode())
                print(f"  -> {msg}")
    except KeyboardInterrupt:
        print("Saliendo...")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
