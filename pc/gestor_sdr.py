#!/usr/bin/env python3
"""
gestor_sdr.py - Reparte el UNICO SDR entre el modo VOR y el modo ADS-B.

Reemplaza a "python vor_radial_decoder_pll.py ..." en iniciar_vor.bat. Arranca el
decodificador VOR con los mismos argumentos y se queda escuchando por UDP (puerto 6010)
las ordenes que le reenvia el puente cuando pulsas Enc2 largo en el panel:

    MODE,ADSB          -> cierra el decodificador VOR (libera el SDR) y abre adsb_tcas.py
    MODE,VOR[,<kHz>]   -> cierra adsb_tcas.py y vuelve a abrir el decodificador VOR

Los argumentos que este programa no conoce (--rf-gain, --if-gain, --ident-gain...) se
pasan tal cual al decodificador VOR.
"""
import argparse
import atexit
import os
import socket
import subprocess
import sys
import time

AQUI = os.path.dirname(os.path.abspath(__file__))


class Gestor:
    def __init__(self, args, dec_args):
        self.args = args
        self.dec_args = dec_args
        self.freq_hz = args.freq
        self.modo = None
        self.proc = None
        atexit.register(self.parar_hijo)

    def parar_hijo(self):
        p, self.proc = self.proc, None
        if p and p.poll() is None:
            p.terminate()
            try:
                p.wait(timeout=6)
            except subprocess.TimeoutExpired:
                p.kill()
                p.wait()
            time.sleep(1.5)         # deja que el USB del SDR quede libre

    def lanzar(self, cmd):
        print("  >", " ".join(cmd), flush=True)
        self.proc = subprocess.Popen(cmd, cwd=AQUI)

    def modo_vor(self):
        self.parar_hijo()
        cmd = [sys.executable, "-u", os.path.join(AQUI, "vor_radial_decoder_pll.py"),
               "--freq", str(int(self.freq_hz))] + self.dec_args
        print(f"\n=== MODO VOR ({self.freq_hz / 1e6:.3f} MHz) ===", flush=True)
        self.lanzar(cmd)
        self.modo = "VOR"

    def modo_adsb(self):
        self.parar_hijo()
        a = self.args
        cmd = [sys.executable, "-u", os.path.join(AQUI, "adsb_tcas.py"),
               f"--lat={a.adsb_lat}", f"--lon={a.adsb_lon}", f"--gain={a.adsb_gain}",
               f"--own-alt-ft={a.adsb_alt_ft}", f"--range-nm={a.adsb_range_nm}",
               f"--panel-udp={a.panel_udp_port}"]
        print("\n=== MODO ADS-B (1090 MHz) ===", flush=True)
        self.lanzar(cmd)
        self.modo = "ADSB"

    def orden(self, texto):
        p = texto.strip().split(",")
        if len(p) < 2 or p[0] != "MODE":
            return
        if p[1] == "ADSB" and self.modo != "ADSB":
            self.modo_adsb()
        elif p[1] == "VOR":
            if len(p) >= 3:
                try:
                    self.freq_hz = int(p[2]) * 1000
                except ValueError:
                    pass
            if self.modo != "VOR":
                self.modo_vor()

    def vigilar(self):
        if self.proc and self.proc.poll() is not None:
            print(f"[Gestor] El programa de modo {self.modo} termino (codigo {self.proc.returncode}). "
                  f"Pulsa Enc2 largo en el panel para reintentar.", flush=True)
            self.proc = None


def main():
    ap = argparse.ArgumentParser(allow_abbrev=False)
    ap.add_argument("--gestor-port", type=int, default=6010)
    ap.add_argument("--freq", type=float, default=113.9e6, help="frecuencia VOR inicial en Hz")
    ap.add_argument("--panel-udp-port", type=int, default=6009, help="puerto UDP del puente para el OLED")
    ap.add_argument("--adsb-lat", type=float, required=True)
    ap.add_argument("--adsb-lon", type=float, required=True)
    ap.add_argument("--adsb-gain", default="40")
    ap.add_argument("--adsb-alt-ft", type=int, default=8500)
    ap.add_argument("--adsb-range-nm", type=int, default=40)
    args, dec_args = ap.parse_known_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", args.gestor_port))
    sock.settimeout(0.5)
    print(f"Gestor del SDR: escuchando ordenes de modo en UDP {args.gestor_port}", flush=True)

    g = Gestor(args, dec_args)
    g.modo_vor()
    try:
        while True:
            try:
                data, _ = sock.recvfrom(256)
                g.orden(data.decode(errors="ignore"))
            except socket.timeout:
                g.vigilar()
    except KeyboardInterrupt:
        print("Deteniendo...")
    finally:
        g.parar_hijo()


if __name__ == "__main__":
    main()
