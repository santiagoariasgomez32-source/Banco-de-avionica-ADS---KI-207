#!/usr/bin/env python3
"""
morse_ident_decoder.py - Decodifica el identificador Morse de la estacion VOR.

Recibe por UDP la ENVOLVENTE del tono (salida del Single Pole IIR de
GNU Radio, diezmada a ~1000 muestras/s) y la convierte en letras. El
umbral se adapta solo al nivel de ruido y de tono, une cortes breves
dentro de una raya y descarta pulsos espurios. Con --binary acepta
tambien el 0/1 de un Threshold ya hecho.
No importa gnuradio, asi que no le afecta el bug de bloques Python.

Uso:
    python morse_ident_decoder.py --expect BOG
"""

import argparse
import socket
import struct
import time
from collections import deque

import numpy as np

MORSE = {
    ".-": "A", "-...": "B", "-.-.": "C", "-..": "D", ".": "E", "..-.": "F",
    "--.": "G", "....": "H", "..": "I", ".---": "J", "-.-": "K", ".-..": "L",
    "--": "M", "-.": "N", "---": "O", ".--.": "P", "--.-": "Q", ".-.": "R",
    "...": "S", "-": "T", "..-": "U", "...-": "V", ".--": "W", "-..-": "X",
    "-.--": "Y", "--..": "Z",
    "-----": "0", ".----": "1", "..---": "2", "...--": "3", "....-": "4",
    ".....": "5", "-....": "6", "--...": "7", "---..": "8", "----.": "9",
}


class DecodificadorMorse:
    """Maquina de estados sobre duraciones (ms) de tono y silencio."""

    def __init__(self, dot_ms=150.0, min_pulse_ms=25.0, end_gap_ms=1500.0):
        self.dot = dot_ms
        self.min_pulse = min_pulse_ms
        self.end_gap = end_gap_ms
        self.simbolos = ""      # letra en curso, p.ej. "-..."
        self.letras = []        # letras ya cerradas, p.ej. ["-...", "---"]
        self.n_tonos = 0

    def tono(self, dur_ms):
        if dur_ms < self.min_pulse:
            return
        if dur_ms < 2.0 * self.dot:
            self.simbolos += "."
            self.dot = 0.7 * self.dot + 0.3 * dur_ms
        else:
            self.simbolos += "-"
            self.dot = 0.7 * self.dot + 0.3 * (dur_ms / 3.0)
        self.dot = max(40.0, min(self.dot, 400.0))
        self.n_tonos += 1

    def silencio(self, dur_ms):
        """Devuelve el identificador completo si el silencio cierra el mensaje."""
        if dur_ms >= self.end_gap:
            return self.cerrar()
        if dur_ms >= 2.0 * self.dot and self.simbolos:
            self.letras.append(self.simbolos)
            self.simbolos = ""
        return None

    def cerrar(self):
        if self.simbolos:
            self.letras.append(self.simbolos)
            self.simbolos = ""
        if not self.letras:
            return None
        res = (" ".join(self.letras),
               "".join(MORSE.get(l, "?") for l in self.letras),
               self.n_tonos)
        self.letras = []
        self.n_tonos = 0
        return res


class Nivel:
    """Convierte la envolvente en on/off con umbral adaptativo e histeresis.

    El umbral usa dos referencias y toma la mayor: k veces el RUIDO (mediana
    de los ultimos segundos) y una fraccion del PICO de tono (se recuerda el
    ultimo identificador, hasta 2 min). El segundo evita unir dos puntos
    seguidos cuando el ruido es muy bajo; el primero funciona desde el inicio.
    """

    def __init__(self, rate, ventana_s=15.0, k_on=3.0, k_off=2.0, suavizado_ms=6.0,
                 f_on=0.35, f_off=0.25):
        self.hist = deque(maxlen=int(ventana_s * rate))
        self.suav = deque(maxlen=max(1, int(suavizado_ms * rate / 1000.0)))
        self.k_on, self.k_off = k_on, k_off
        self.f_on, self.f_off = f_on, f_off
        self.pico_ref = 0.0        # ultimo nivel de tono valido (se recuerda entre identificadores)
        self.t_pico = 0
        self.expira = int(120.0 * rate)
        self.warm = int(2.0 * rate)
        self.cada = max(1, int(0.1 * rate))
        self.n = 0
        self.ruido = self.pico = self.ratio = 0.0
        self.thr_on = self.thr_off = float("inf")
        self.on = False

    @property
    def listo(self):
        return self.n >= self.warm

    def feed(self, v):
        self.suav.append(v)
        x = sum(self.suav) / len(self.suav)
        self.hist.append(x)
        self.n += 1
        if self.listo and self.n % self.cada == 0:
            arr = np.fromiter(self.hist, dtype=float)
            self.ruido = float(np.percentile(arr, 50))
            self.pico = float(np.percentile(arr, 98))
            self.ratio = self.pico / max(self.ruido, 1e-12)
            if self.ratio >= 4.0:
                self.pico_ref = self.pico
                self.t_pico = self.n
            elif self.n - self.t_pico > self.expira:
                self.pico_ref = 0.0
            # Umbral = lo MAYOR entre "k veces el ruido" y "fraccion del pico de tono".
            self.thr_on = max(self.k_on * self.ruido, self.f_on * self.pico_ref)
            self.thr_off = max(self.k_off * self.ruido, self.f_off * self.pico_ref)
        if not self.listo:
            self.on = False
        elif self.on:
            self.on = x > self.thr_off
        else:
            self.on = x > self.thr_on
        return self.on


class Analizador:
    """Duraciones de tono/silencio con union de cortes breves (bridge)."""

    def __init__(self, dec, dt_ms, bridge_ms, reportar):
        self.dec = dec
        self.dt = dt_ms
        self.bridge = bridge_ms
        self.reportar = reportar
        self.estado = False
        self.run = 0.0
        self.pending = None
        self.committed = True
        self.closed = True
        self.tonos = 0

    def feed(self, on):
        dt = self.dt
        if self.estado:
            if on:
                self.run += dt
            else:
                self.pending = self.run
                self.estado = False
                self.run = dt
                self.committed = False
                self.closed = False
        else:
            if not on:
                self.run += dt
                if not self.committed and self.run >= self.bridge:
                    self.dec.tono(self.pending)
                    self.tonos += 1
                    self.pending = None
                    self.committed = True
                if self.committed and not self.closed and self.run >= self.dec.end_gap:
                    res = self.dec.silencio(self.run)
                    self.closed = True
                    if res:
                        self.reportar(res)
            else:
                if self.pending is not None:       # corte breve: unir con el tono previo
                    self.run = self.pending + self.run + dt
                    self.pending = None
                else:
                    if not self.closed:
                        res = self.dec.silencio(self.run)
                        if res:
                            self.reportar(res)
                    self.run = dt
                self.estado = True
                self.committed = True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=6008)
    ap.add_argument("--rate", type=float, default=1000.0, help="muestras/s que llegan por UDP")
    ap.add_argument("--expect", default=None, help="identificador esperado, p.ej. BOG")
    ap.add_argument("--dot-ms", type=float, default=100.0, help="duracion inicial estimada de un punto")
    ap.add_argument("--binary", action="store_true", help="la entrada ya es 0/1 (salida de un Threshold)")
    ap.add_argument("--k-on", type=float, default=3.0,
                    help="el tono se enciende cuando la envolvente supera k-on veces el ruido")
    ap.add_argument("--k-off", type=float, default=2.0,
                    help="el tono se apaga cuando baja de k-off veces el ruido")
    ap.add_argument("--f-on", type=float, default=0.35,
                    help="el tono se enciende al superar esta fraccion del pico de tono")
    ap.add_argument("--f-off", type=float, default=0.25,
                    help="el tono se apaga al bajar de esta fraccion del pico de tono")
    ap.add_argument("--min-tonos", type=int, default=3,
                    help="ignorar mensajes con menos tonos que esto (ruido suelto)")
    ap.add_argument("--bridge-ms", type=float, default=45.0,
                    help="cortes de tono mas cortos que esto se unen al tono anterior")
    args = ap.parse_args()

    dt_ms = 1000.0 / args.rate
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", args.port))
    sock.settimeout(0.2)
    modo = "0/1 (Threshold)" if args.binary else "envolvente (umbral adaptativo)"
    print(f"Escuchando UDP 127.0.0.1:{args.port} ({args.rate:.0f} muestras/s) - entrada: {modo}")
    if args.expect:
        print(f"Identificador esperado: {args.expect.upper()}")
    print("Ctrl+C para salir.\n")

    ok = fallos = 0

    def reportar(res):
        nonlocal ok, fallos
        patron, texto, n = res
        if n < args.min_tonos:
            return
        marca = ""
        if args.expect:
            if texto == args.expect.upper():
                ok += 1
                marca = "  OK"
            else:
                fallos += 1
                marca = f"  INCOMPLETO/ERRONEO (esperado {args.expect.upper()})"
        print(f"[{time.strftime('%H:%M:%S')}] {texto:<6} {patron}  ({n} tonos){marca}")
        if args.expect:
            print(f"           aciertos: {ok}   fallos: {fallos}")

    dec = DecodificadorMorse(dot_ms=args.dot_ms, min_pulse_ms=30.0)
    ana = Analizador(dec, dt_ms, args.bridge_ms, reportar)
    nivel = None if args.binary else Nivel(args.rate, k_on=args.k_on, k_off=args.k_off, f_on=args.f_on, f_off=args.f_off)

    t_estado = time.time()
    muestras = unos = tonos_ant = paquetes = 0
    try:
        while True:
            try:
                data, _ = sock.recvfrom(4096)
            except socket.timeout:
                data = b""
            ahora = time.time()
            if ahora - t_estado >= 2.0:
                dt = ahora - t_estado
                if muestras == 0:
                    print(f"  [estado] SIN DATOS por UDP (paquetes totales: {paquetes}). "
                          f"Revisa que GNU Radio este corriendo y el UDP Sink use el puerto {args.port}.")
                elif nivel is None:
                    print(f"  [estado] {muestras / dt:.0f} muestras/s | tono encendido {100.0 * unos / muestras:.0f}%")
                else:
                    det = "hay tono" if nivel.ratio >= 2.5 else "solo ruido"
                    print(f"  [estado] {muestras / dt:.0f} muestras/s | ruido {nivel.ruido:.2e} | "
                          f"pico {nivel.pico:.2e} | ratio {nivel.ratio:.1f} ({det}) | "
                          f"encendido {100.0 * unos / muestras:.0f}% | tonos totales {ana.tonos}")
                t_estado = ahora
                muestras = unos = 0
            if not data:
                continue
            paquetes += 1
            n = len(data) // 4
            for v in struct.unpack(f"<{n}f", data[:n * 4]):
                on = (v > 0.5) if nivel is None else nivel.feed(v)
                muestras += 1
                unos += 1 if on else 0
                ana.feed(on)
    except KeyboardInterrupt:
        print("\nDeteniendo...")
    finally:
        sock.close()


if __name__ == "__main__":
    main()
