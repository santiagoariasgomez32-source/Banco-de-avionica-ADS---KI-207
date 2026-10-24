#!/usr/bin/env python3
"""
ADS-B con RTL-SDR + pantalla estilo TCAS (norte arriba, tu antena en el centro).

IMPORTANTE: ADS-B esta en 1090 MHz. El SDR solo puede abrirlo UN programa a la vez,
asi que NO corre al mismo tiempo que iniciar_vor.bat. Cierra el VOR antes.

Instalacion (una vez):
    pip install "pyModeS<3" pyrtlsdr numpy

Uso:
    python adsb_tcas.py --test --lat 4.70 --lon -74.14          (solo consola: contar mensajes)
    python adsb_tcas.py --lat 4.70 --lon -74.14                  (pantalla TCAS)
    python adsb_tcas.py --lat .. --lon .. --range-nm 40 --gain 40
    python adsb_tcas.py --lat .. --lon .. --demo                 (avion falso, sin SDR)
    python adsb_tcas.py --lat .. --lon .. --dump1090 127.0.0.1:30002   (si usas dump1090)

--lat/--lon = posicion de tu antena (grados decimales). Se usa para decodificar posiciones.
La alerta TA/RA de la pantalla es SOLO una simulacion didactica por distancia/altura
(no es el algoritmo TCAS II real ni sirve para navegacion).
"""
import argparse, math, socket, threading, time, sys
import numpy as np
import pyModeS as pms

NM = 1852.0
ALCANCE = [40]       # NM; lo cambian las teclas +/- de la ventana y lo usa el OLED del panel
OWNALT = [8500]      # ft, altitud de tu antena


def nivel_amenaza(d, dz):
    """0 otro, 1 proximo, 2 TA, 3 RA (simulado, solo por distancia en NM y diferencia de altura en ft)."""
    nivel = 0
    if dz is not None:
        if d < 6 and abs(dz) < 1200: nivel = 2
        elif d < 20 and abs(dz) < 2700: nivel = 1
        if d < 3 and abs(dz) < 800: nivel = 3
    elif d < 20:
        nivel = 1
    return nivel


# ----------------------------------------------------------------- demodulador
def demodular(iq_mag):
    """iq_mag: magnitud a 2 MS/s. Devuelve lista de mensajes hex (DF17/18 con CRC valido)."""
    m = iq_mag.astype(np.float32)
    n = len(m) - 16 - 224
    if n <= 0:
        return []
    a = lambda k: m[k:k + n]
    pulsos = np.minimum.reduce([a(0), a(2), a(7), a(9)])
    huecos = np.maximum.reduce([a(1), a(3), a(4), a(5), a(6), a(8)])
    # el pulso minimo debe superar claramente al hueco maximo y a un piso de ruido
    piso = np.median(m) * 2.5
    cand = np.nonzero((pulsos > huecos * 1.5) & (pulsos > piso))[0]
    out = []
    ultimo = -300
    for i in cand:
        if i - ultimo < 240:
            continue
        d = m[i + 16:i + 16 + 224]
        bits = (d[0::2] > d[1::2]).astype(np.uint8)
        val = int("".join(map(str, bits)), 2)
        hexa = f"{val:028x}"
        df = int(hexa[0], 16) * 2 + (int(hexa[1], 16) >> 3)  # 5 bits altos
        df = int(hexa[:2], 16) >> 3
        if df in (17, 18) and pms.crc(hexa) == 0:
            out.append(hexa)
            ultimo = i
    return out


# ----------------------------------------------------------------- fuentes
class Fuente:
    def __init__(self, args, callback):
        self.args, self.cb = args, callback
        self.mensajes = 0
        self.muestras_ok = 0
        self.max_pico = 0.0
        self.parar = False

    def correr_rtlsdr(self):
        from rtlsdr import RtlSdr
        sdr = RtlSdr()
        sdr.sample_rate = 2.0e6
        sdr.center_freq = 1090e6
        try:
            sdr.gain = float(self.args.gain)
        except Exception:
            sdr.gain = "auto"
        resto = np.zeros(0, dtype=np.float32)
        N = 2 ** 18 * 2
        try:
            while not self.parar:
                x = sdr.read_samples(N)
                mag = np.abs(x).astype(np.float32)
                self.max_pico = max(self.max_pico * 0.9, float(mag.max()))
                buf = np.concatenate([resto, mag])
                for h in demodular(buf):
                    self.mensajes += 1
                    self.cb(h)
                resto = buf[-300:]
        finally:
            sdr.close()

    def correr_dump1090(self):
        host, port = self.args.dump1090.split(":")
        s = socket.create_connection((host, int(port)))
        f = s.makefile("r")
        for linea in f:
            linea = linea.strip()
            if linea.startswith("*") and linea.endswith(";"):
                h = linea[1:-1].lower()
                if len(h) == 28 and pms.crc(h) == 0:
                    self.mensajes += 1
                    self.cb(h)
            if self.parar:
                break

    def correr_demo(self):
        t0 = time.time()
        while not self.parar:
            self.cb("DEMO")
            time.sleep(0.2)


# ----------------------------------------------------------------- estado de aviones
class Trafico:
    def __init__(self, lat, lon):
        self.lat0, self.lon0 = lat, lon
        self.av = {}
        self.lock = threading.Lock()
        self.total = 0

    def mensaje(self, h):
        if h == "DEMO":
            return self._demo()
        try:
            tc = pms.adsb.typecode(h)
            icao = pms.adsb.icao(h)
        except Exception:
            return
        with self.lock:
            a = self.av.setdefault(icao, dict(icao=icao, call="", lat=None, lon=None, alt=None,
                                              gs=None, trk=None, vr=None, visto=0, n=0))
            a["visto"] = time.time(); a["n"] += 1; self.total += 1
            try:
                if 1 <= tc <= 4:
                    a["call"] = pms.adsb.callsign(h).strip("_")
                elif 9 <= tc <= 18 or 20 <= tc <= 22:
                    alt = pms.adsb.altitude(h)
                    if alt is not None:
                        a["alt"] = alt
                    p = pms.adsb.position_with_ref(h, self.lat0, self.lon0)
                    if p and abs(p[0] - self.lat0) < 5 and abs(p[1] - self.lon0) < 5:
                        a["lat"], a["lon"] = p
                elif tc == 19:
                    v = pms.adsb.velocity(h)
                    if v and v[0] is not None:
                        a["gs"], a["trk"], a["vr"] = v[0], v[1], v[2]
            except Exception:
                pass

    def _demo(self):
        t = time.time()
        with self.lock:
            for k, (r0, rumbo, alt, nombre) in enumerate([(30, 40, 12000, "AVA123"), (12, 200, 9000, "LAT456"),
                                                          (5, 90, 8500, "DEMO03")]):
                ang = math.radians(rumbo + t * 2 * (k + 1) % 360)
                r = r0 * NM
                lat = self.lat0 + (r * math.cos(ang)) / 111320.0
                lon = self.lon0 + (r * math.sin(ang)) / (111320.0 * math.cos(math.radians(self.lat0)))
                self.av[f"DEMO{k}"] = dict(icao=f"DEMO{k}", call=nombre, lat=lat, lon=lon, alt=alt,
                                           gs=250, trk=(math.degrees(ang) + 90) % 360, vr=(-500, 0, 800)[k],
                                           visto=t, n=1)
            self.total += 3

    def lista(self, max_edad=60):
        t = time.time()
        with self.lock:
            return [dict(a) for a in self.av.values() if t - a["visto"] < max_edad]


def rel(a, lat0, lon0):
    """(este_NM, norte_NM, distancia_NM) o None."""
    if a["lat"] is None:
        return None
    n = (a["lat"] - lat0) * 111320.0 / NM
    e = (a["lon"] - lon0) * 111320.0 * math.cos(math.radians(lat0)) / NM
    return e, n, math.hypot(e, n)


# ----------------------------------------------------------------- pantalla TCAS
def pantalla(trafico, fuente, args):
    import tkinter as tk
    root = tk.Tk()
    root.title("ADS-B / TCAS (simulado)")
    S = 720
    cv = tk.Canvas(root, width=S, height=S, bg="black", highlightthickness=0)
    cv.pack()
    c = S / 2
    R = S * 0.45
    alcance = ALCANCE
    ownalt = OWNALT

    def tecla(ev):
        opciones = [5, 10, 20, 40, 80, 120, 200]
        i = min(range(len(opciones)), key=lambda k: abs(opciones[k] - alcance[0]))
        if ev.keysym in ("plus", "equal", "Up"):
            alcance[0] = opciones[max(0, i - 1)]
        elif ev.keysym in ("minus", "Down"):
            alcance[0] = opciones[min(len(opciones) - 1, i + 1)]
    root.bind("<Key>", tecla)

    def dibujar():
        cv.delete("all")
        rg = alcance[0]
        for f in (1, 0.5):
            cv.create_oval(c - R * f, c - R * f, c + R * f, c + R * f, outline="#555", dash=(3, 5))
            cv.create_text(c + 4, c - R * f - 8, text=f"{rg * f:g}", fill="#888", anchor="w", font=("Consolas", 10))
        cv.create_line(c, c - R, c, c + R, fill="#333"); cv.create_line(c - R, c, c + R, c, fill="#333")
        cv.create_text(c, 14, text="N", fill="#aaa", font=("Consolas", 12, "bold"))
        # ownship
        cv.create_polygon(c, c - 9, c - 7, c + 8, c, c + 4, c + 7, c + 8, fill="white", outline="white")
        lista = trafico.lista()
        conpos = 0
        peor = 0
        for a in lista:
            r = rel(a, trafico.lat0, trafico.lon0)
            if r is None:
                continue
            conpos += 1
            e, n, d = r
            if d > rg * 1.02:
                continue
            x, y = c + e / rg * R, c - n / rg * R
            dz = None if a["alt"] is None else a["alt"] - ownalt[0]
            nivel = nivel_amenaza(d, dz)
            peor = max(peor, nivel)
            col = ("#00e5ff", "#00e5ff", "#ffb000", "#ff2020")[nivel]
            s = 8
            if nivel == 0:
                cv.create_polygon(x, y - s, x + s, y, x, y + s, x - s, y, outline=col, fill="")
            elif nivel == 1:
                cv.create_polygon(x, y - s, x + s, y, x, y + s, x - s, y, outline=col, fill=col)
            elif nivel == 2:
                cv.create_oval(x - s, y - s, x + s, y + s, outline=col, fill=col)
            else:
                cv.create_rectangle(x - s, y - s, x + s, y + s, outline=col, fill=col)
            if dz is not None:
                txt = f"{'+' if dz >= 0 else '-'}{abs(int(round(dz / 100))):02d}"
                flecha = ""
                if a["vr"] is not None:
                    flecha = "↑" if a["vr"] > 500 else ("↓" if a["vr"] < -500 else "")
                cv.create_text(x + 12, y + (-8 if dz >= 0 else 8), text=txt + flecha, fill=col,
                               anchor="w", font=("Consolas", 11, "bold"))
            if a["call"]:
                cv.create_text(x + 12, y + (8 if dz is None or dz >= 0 else -8), text=a["call"],
                               fill="#aaa", anchor="w", font=("Consolas", 9))
        estado = ("TRAFFIC" if peor == 2 else "RESOLUTION" if peor == 3 else "")
        if estado:
            cv.create_text(c, S - 24, text=estado + " (simulado)", fill="#ffb000" if peor == 2 else "#ff2020",
                           font=("Consolas", 16, "bold"))
        cv.create_text(6, S - 8, anchor="sw", fill="#888", font=("Consolas", 9),
                       text=f"msgs ADS-B: {fuente.mensajes}   aviones: {len(lista)}   con posicion: {conpos}   "
                            f"alcance {rg} NM  (+/- cambia)  alt propia {ownalt[0]} ft")
        root.after(300, dibujar)

    dibujar()
    root.mainloop()


# ----------------------------------------------------------------- hacia el OLED del panel
def linea_panel(trafico):
    """ADS,<vistos>,<rango>,<call>,<dist>,<dz100>[,<dx>,<dy>,<nivel>]... (dx,dy en pixeles, radio 30)."""
    rg = ALCANCE[0]
    lst = trafico.lista()
    con_pos = []
    for a in lst:
        r = rel(a, trafico.lat0, trafico.lon0)
        if r:
            con_pos.append((r[2], r[0], r[1], a))
    con_pos.sort(key=lambda z: z[0])
    if con_pos:
        d0, _, _, a0 = con_pos[0]
        call = (a0["call"] or a0["icao"]).replace(",", "")[:8] or "-"
        dz0 = 999 if a0["alt"] is None else int(max(-98, min(98, round((a0["alt"] - OWNALT[0]) / 100.0))))
        cab = f"ADS,{len(lst)},{rg},{call},{int(round(d0))},{dz0}"
    else:
        cab = f"ADS,{len(lst)},{rg},-,0,999"
    pts = ""
    n = 0
    for d, e, nn, a in con_pos:
        if d > rg or n >= 10:
            continue
        dz = None if a["alt"] is None else a["alt"] - OWNALT[0]
        dx = int(round(e / rg * 30))
        dy = int(round(-nn / rg * 30))
        pts += f",{dx},{dy},{nivel_amenaza(d, dz)}"
        n += 1
    return (cab + pts + "\n").encode()


def hilo_panel(trafico, puerto, parar):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    while not parar():
        try:
            sock.sendto(linea_panel(trafico), ("127.0.0.1", puerto))
        except OSError:
            pass
        time.sleep(0.5)


# ----------------------------------------------------------------- consola de prueba
def consola(trafico, fuente, args):
    t0 = time.time()
    try:
        while True:
            time.sleep(5)
            lst = trafico.lista()
            lejos = 0.0
            for a in lst:
                r = rel(a, trafico.lat0, trafico.lon0)
                if r: lejos = max(lejos, r[2])
            print(f"[{int(time.time() - t0):4d}s] mensajes={fuente.mensajes} aviones={len(lst)} "
                  f"max_alcance={lejos:.0f} NM pico_senal={fuente.max_pico:.2f}")
            for a in sorted(lst, key=lambda z: -z["n"])[:8]:
                r = rel(a, trafico.lat0, trafico.lon0)
                d = f"{r[2]:.0f}NM" if r else "sin pos"
                print(f"        {a['icao']} {a['call'] or '-':8s} alt={a['alt']} {d} msgs={a['n']}")
    except KeyboardInterrupt:
        pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--lat", type=float, required=True)
    ap.add_argument("--lon", type=float, required=True)
    ap.add_argument("--range-nm", type=int, default=40)
    ap.add_argument("--gain", default="40")
    ap.add_argument("--own-alt-ft", type=int, default=8500, help="altitud de tu antena (Bogota ~8500 ft)")
    ap.add_argument("--test", action="store_true", help="solo consola")
    ap.add_argument("--demo", action="store_true")
    ap.add_argument("--dump1090", default="")
    ap.add_argument("--panel-udp", type=int, default=0,
                    help="puerto UDP al que mandar el trafico para el OLED del panel (lo recibe el puente; 6009)")
    args = ap.parse_args()

    ALCANCE[0] = args.range_nm
    OWNALT[0] = args.own_alt_ft
    tr = Trafico(args.lat, args.lon)
    fu = Fuente(args, tr.mensaje)
    destino = fu.correr_demo if args.demo else fu.correr_dump1090 if args.dump1090 else fu.correr_rtlsdr

    def envoltura():
        try:
            destino()
        except Exception as e:
            print("Error en la fuente:", repr(e), file=sys.stderr)
    threading.Thread(target=envoltura, daemon=True).start()
    if args.panel_udp:
        threading.Thread(target=hilo_panel, args=(tr, args.panel_udp, lambda: fu.parar), daemon=True).start()
    if args.test:
        consola(tr, fu, args)
    else:
        pantalla(tr, fu, args)
    fu.parar = True


if __name__ == "__main__":
    main()
