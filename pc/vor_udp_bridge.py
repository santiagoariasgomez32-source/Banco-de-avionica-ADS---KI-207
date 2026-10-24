#!/usr/bin/env python3
"""
vor_udp_bridge.py — Puente entre GNU Radio y el ESP32 del panel.
No importa gnuradio (así no le afecta el bug de bloques Python).

  * Recibe por UDP el radial calibrado (20 valores/s) del decoder.
  * Abre UN solo puerto serial: el del ESP32 del panel (LCD + botones).
      Panel -> PC : FREQ,NAV1,ACT,<kHz>   COURSE,<grados>
      PC -> Panel : DEV,<-1..1>,FLAG,<TO|FROM|OFF>,RAD,<grados>,CONS,<0..1>[,ID,<texto>]
  * Decodifica el identificador Morse (envolvente por UDP 6008, la manda el
    decodificador VOR) con morse_ident_decoder.py y lo manda al panel (ID).
    Debe estar en la misma carpeta. No ejecutes morse_ident_decoder.py aparte
    mientras el puente corre (ocupan el mismo puerto).
  * Reenvia al OLED del panel el espectro que calcula el decodificador (UDP 6009,
    lineas SPC/SPW). No se manda al KI-207.
  * Al cambiar la frecuencia, se la manda al decoder por UDP (puerto de
    control) para que retonalice el SDR.
  * Promedia 1 s con promedio circular; la longitud del vector resultante
    (0..1) es la "consistencia". Si es baja o no llegan datos -> FLAG=OFF.

Uso:
    python vor_udp_bridge.py --panel-port COM3
Opcional: --ki207-port COMx  (segundo ESP32 con el instrumento real)
"""

import argparse
import math
import socket
import struct
import threading
import time

import serial

try:
    import morse_ident_decoder as morse
except ImportError:
    morse = None


def angle_diff(a, b):
    """Diferencia angular normalizada a [-180, 180)."""
    return (a - b + 180.0) % 360.0 - 180.0


def calcular_cdi(radial, curso, full_scale_deg=10.0, invertir=False):
    """Devuelve (deflexion -1..1, 'TO'|'FROM'). Curso = valor del OBS.
    Convención de aguja REAL: + = aguja a la derecha, que significa que la
    línea del curso está a tu derecha (estás a la IZQUIERDA del curso).
    FROM: si estás en un radial mayor que el curso, estás a la derecha del
          curso -> aguja a la izquierda (-).
    TO  : se compara contra el recíproco del radial (curso+180)."""
    diff = angle_diff(radial, curso)
    if abs(diff) <= 90.0:
        dev, flag = -diff / full_scale_deg, "FROM"
    else:
        dev, flag = angle_diff(radial, curso + 180.0) / full_scale_deg, "TO"
    dev = max(-1.0, min(1.0, dev))
    return (-dev if invertir else dev), flag


class Estado:
    def __init__(self):
        self.lock = threading.Lock()
        self.curso = 0.0
        self.freq_khz = None
        self.ident = None          # (texto, instante) del ultimo ident valido
        self.ident_ratio = 0.0     # tono / ruido de la envolvente del ident (mayor = mejor)
        self.ident_pico = 0.0
        self.ident_ruido = 0.0
        self.modo = "VOR"          # "VOR" o "ADSB" (lo decide el panel con Enc2 largo)
        self.vuelta = False        # True al volver a VOR: reinicia el enganche


def lector_panel(ser, estado, ctrl_sock, ctrl_addr, parar, gestor_addr=None):
    buf = b""
    while not parar.is_set():
        try:
            buf += ser.read(ser.in_waiting or 1)
        except (serial.SerialException, OSError):
            time.sleep(0.5)
            continue
        while b"\n" in buf:
            linea, buf = buf.split(b"\n", 1)
            linea = linea.decode(errors="ignore").strip()
            p = linea.split(",")
            try:
                if p[0] == "COURSE" and len(p) == 2:
                    with estado.lock:
                        estado.curso = float(p[1]) % 360.0
                    print(f"[Panel] Curso: {estado.curso:.0f}°")
                elif p[0] == "FREQ" and len(p) == 4 and p[2] == "ACT":
                    khz = int(float(p[3]))
                    if khz != estado.freq_khz:
                        estado.freq_khz = khz
                        ctrl_sock.sendto(f"FREQ,{khz}".encode(), ctrl_addr)
                        print(f"[Panel] Frecuencia: {khz/1000:.3f} MHz -> retonalizando SDR")
                elif p[0] == "MODE" and len(p) == 2 and p[1] in ("VOR", "ADSB"):
                    nuevo = p[1]
                    if nuevo != estado.modo:
                        print(f"[Panel] Modo: {estado.modo} -> {nuevo}")
                        with estado.lock:
                            estado.modo = nuevo
                            estado.ident = None
                            if nuevo == "VOR":
                                estado.vuelta = True
                    # se reenvia siempre (el panel lo repite cada 5 s): asi el gestor se sincroniza
                    # aunque se haya perdido un mensaje o se haya iniciado despues
                    if gestor_addr:
                        msg = f"MODE,{nuevo}"
                        if nuevo == "VOR" and estado.freq_khz:
                            msg += f",{estado.freq_khz}"
                        ctrl_sock.sendto(msg.encode(), gestor_addr)
            except ValueError:
                pass


def hilo_ident(estado, puerto, rate, parar, k_on=3.0, k_off=2.0, f_on=0.35, f_off=0.25,
               bridge_ms=45.0, dot_ms=100.0):
    """Escucha la envolvente del tono (UDP) y decodifica el Morse."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind(("127.0.0.1", puerto))
    except OSError as e:
        print(f"[Ident] No pude abrir UDP {puerto} ({e}). Cierra morse_ident_decoder.py si esta abierto.")
        return
    sock.settimeout(0.3)

    def reportar(res):
        patron, texto, n = res
        valido = n >= 3 and 2 <= len(texto) <= 4 and "?" not in texto
        if valido:
            with estado.lock:
                estado.ident = (texto, time.time())
            print(f"[Ident] {texto}  {patron}  ({n} tonos)")
        else:
            print(f"[Ident] descartado: {texto} {patron} ({n} tonos)")

    dec = morse.DecodificadorMorse(dot_ms=dot_ms, min_pulse_ms=30.0)
    ana = morse.Analizador(dec, 1000.0 / rate, bridge_ms, reportar)
    nivel = morse.Nivel(rate, k_on=k_on, k_off=k_off, f_on=f_on, f_off=f_off)
    while not parar.is_set():
        try:
            data, _ = sock.recvfrom(4096)
        except socket.timeout:
            continue
        except OSError:
            break
        k = len(data) // 4
        for v in struct.unpack(f"<{k}f", data[:k * 4]):
            ana.feed(nivel.feed(v))
        estado.ident_ratio = nivel.ratio
        estado.ident_pico = nivel.pico
        estado.ident_ruido = nivel.ruido
    sock.close()


def hilo_espectro(panel, lock_panel, puerto, parar):
    """Reenvia al panel las lineas de espectro (SPC/SPW) y de trafico ADS-B (ADS) que llegan por UDP."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind(("127.0.0.1", puerto))
    except OSError as e:
        print(f"[Espectro] No pude abrir UDP {puerto} ({e}). El OLED no mostrara espectro.")
        return
    sock.settimeout(0.3)
    while not parar.is_set():
        try:
            data, _ = sock.recvfrom(1024)
        except socket.timeout:
            continue
        except OSError:
            break
        if data[:4] in (b"SPC,", b"SPW,", b"ADS,") and data.endswith(b"\n") and len(data) < 300:
            try:
                with lock_panel:
                    panel.write(data)
            except (serial.SerialException, OSError):
                pass
    sock.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--udp-port", type=int, default=6006)
    ap.add_argument("--control-port", type=int, default=6007)
    ap.add_argument("--panel-port", required=True)
    ap.add_argument("--ki207-port", default=None)
    ap.add_argument("--baudrate", type=int, default=115200)
    ap.add_argument("--full-scale-deg", type=float, default=10.0)
    ap.add_argument("--invert", action="store_true", help="invierte el sentido de la aguja")
    ap.add_argument("--smooth-s", type=float, default=0.0,
                    help="constante de tiempo (s) del suavizado del radial; 0 = sin suavizar (por defecto, "
                         "valor lo mas real posible)")
    ap.add_argument("--log", default=None,
                    help="guarda un CSV con el radial crudo, el suavizado y la consistencia (p.ej. medida1.csv)")
    ap.add_argument("--stats-s", type=float, default=60.0,
                    help="cada cuantos segundos imprimir el resumen de calidad de la senal (0 = nunca)")
    ap.add_argument("--spec-port", type=int, default=6009, help="puerto UDP del que llega el espectro")
    ap.add_argument("--no-spectrum", action="store_true", help="no reenviar el espectro al OLED")
    ap.add_argument("--gestor-port", type=int, default=6010,
                    help="puerto UDP del gestor del SDR (gestor_sdr.py) al que se avisa del cambio VOR/ADS-B")
    ap.add_argument("--ident-port", type=int, default=6008)
    ap.add_argument("--no-ident", action="store_true", help="no decodificar el identificador Morse")
    ap.add_argument("--ident-k-on", type=float, default=3.0,
                    help="el tono se enciende al superar k veces el ruido (mas alto = menos falsos pitidos)")
    ap.add_argument("--ident-k-off", type=float, default=2.0, help="el tono se apaga al bajar de k veces el ruido")
    ap.add_argument("--ident-f-on", type=float, default=0.35, help="encender al superar esta fraccion del pico de tono")
    ap.add_argument("--ident-f-off", type=float, default=0.25, help="apagar al bajar de esta fraccion del pico de tono")
    ap.add_argument("--ident-bridge-ms", type=float, default=45.0,
                    help="cortes de tono mas cortos que esto se unen al tono anterior (sube si parte las rayas)")
    ap.add_argument("--ident-dot-ms", type=float, default=100.0, help="duracion estimada de un punto")
    ap.add_argument("--ident-hold-s", type=float, default=60.0,
                    help="segundos que se mantiene el ident en el panel tras decodificarlo")
    ap.add_argument("--min-consistency", type=float, default=0.6)
    ap.add_argument("--settle-s", type=float, default=6.0,
                    help="segundos con FLAG=OFF tras cambiar frecuencia (PLL enganchando)")
    args = ap.parse_args()

    panel = serial.Serial(args.panel_port, args.baudrate, timeout=0.1)
    ki207 = serial.Serial(args.ki207_port, args.baudrate, timeout=0) if args.ki207_port else None
    print(f"Panel en {args.panel_port}" + (f", KI-207 en {args.ki207_port}" if ki207 else ""))

    estado = Estado()
    parar = threading.Event()
    lock_panel = threading.Lock()      # el panel recibe escrituras del bucle principal y del hilo de espectro
    if not args.no_spectrum:
        threading.Thread(target=hilo_espectro, args=(panel, lock_panel, args.spec_port, parar),
                         daemon=True).start()
        print(f"Espectro para el OLED: escuchando UDP {args.spec_port}")
    ctrl_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    ctrl_addr = ("127.0.0.1", args.control_port)
    gestor_addr = ("127.0.0.1", args.gestor_port)
    threading.Thread(target=lector_panel, args=(panel, estado, ctrl_sock, ctrl_addr, parar, gestor_addr),
                     daemon=True).start()

    if not args.no_ident:
        if morse is None:
            print("[Ident] No encuentro morse_ident_decoder.py en esta carpeta: sin identificador.")
        else:
            threading.Thread(target=hilo_ident, args=(estado, args.ident_port, 1000.0, parar,
                                                      args.ident_k_on, args.ident_k_off, args.ident_f_on,
                                                      args.ident_f_off, args.ident_bridge_ms, args.ident_dot_ms),
                             daemon=True).start()
            print(f"Identificador Morse: escuchando envolvente en UDP {args.ident_port}")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", args.udp_port))
    sock.settimeout(0.25)
    print(f"Escuchando radial en UDP 127.0.0.1:{args.udp_port}. Ctrl+C para salir.")

    ssin = scos = 0.0
    n = 0
    t_ventana = time.time()
    t_cambio = time.time()
    ultima_freq = None
    ultima_linea_ok = None
    ultima_cons = 0.0
    t_ultima_ok = 0.0
    ema = None                      # vector medio suavizado (seno, coseno)
    alfa = 1.0 if args.smooth_s <= 0 else 1.0 - math.exp(-1.0 / args.smooth_s)
    radial_crudo = 0.0
    # estadisticas de calidad de la senal (para comparar antenas/ganancias con numeros)
    st_cons, st_sin, st_cos, st_ok, st_tot = [], 0.0, 0.0, 0, 0
    t_stats = time.time()
    flog = None
    if args.log:
        flog = open(args.log, "a", encoding="utf-8")
        if flog.tell() == 0:
            flog.write("hora,radial_crudo,radial,consistencia,muestras,estado\n")
            flog.flush()

    try:
        while True:
            try:
                data, _ = sock.recvfrom(4096)
                k = len(data) // 4
                for r in struct.unpack(f"<{k}f", data[:k * 4]):
                    if math.isfinite(r):
                        ssin += math.sin(math.radians(r))
                        scos += math.cos(math.radians(r))
                        n += 1
            except socket.timeout:
                pass

            if estado.freq_khz != ultima_freq or estado.vuelta:
                estado.vuelta = False
                ultima_freq = estado.freq_khz
                t_cambio = time.time()
                ultima_linea_ok = None
                with estado.lock:
                    estado.ident = None
                ssin = scos = 0.0
                n = 0
                t_ventana = time.time()

            ahora = time.time()
            if ahora - t_ventana >= 1.0:
                with estado.lock:
                    curso = estado.curso
                if n >= 5:
                    consist = math.hypot(ssin, scos) / n
                    radial = math.degrees(math.atan2(ssin, scos)) % 360.0
                else:
                    consist, radial = 0.0, 0.0
                radial_crudo = radial
                asentando = (ahora - t_cambio) < args.settle_s
                if asentando:
                    ema = None
                    linea = f"DEV,0.000,FLAG,OFF,RAD,{radial:.1f},CONS,{consist:.2f}\n"
                    estado_txt = "OFF (enganchando tras cambio de frecuencia)"
                elif n >= 5 and consist >= args.min_consistency:
                    # suavizado: media exponencial del vector (los segundos poco consistentes pesan menos)
                    vs, vc = ssin / n, scos / n
                    if ema is None:
                        ema = (vs, vc)
                    else:
                        ema = (ema[0] + alfa * (vs - ema[0]), ema[1] + alfa * (vc - ema[1]))
                    radial = math.degrees(math.atan2(ema[0], ema[1])) % 360.0
                    dev, flag = calcular_cdi(radial, curso, args.full_scale_deg, args.invert)
                    linea = f"DEV,{dev:.3f},FLAG,{flag},RAD,{radial:.1f},CONS,{consist:.2f}\n"
                    estado_txt = f"{flag} dev={dev:+.2f}"
                    ultima_linea_ok = (radial, flag)
                    ultima_cons = consist
                    t_ultima_ok = ahora
                elif n < 5 and ultima_linea_ok and (ahora - t_ultima_ok) < 3.0:
                    # Sin datos nuevos este segundo: mantener el ultimo valor bueno
                    # hasta 3 s en vez de apagar la bandera de inmediato.
                    radial = ultima_linea_ok[0]
                    dev, flag = calcular_cdi(radial, curso, args.full_scale_deg, args.invert)
                    linea = f"DEV,{dev:.3f},FLAG,{flag},RAD,{radial:.1f},CONS,{ultima_cons:.2f}\n"
                    estado_txt = f"{flag} dev={dev:+.2f} (manteniendo)"
                else:
                    if ahora - t_ultima_ok > 5.0:
                        ema = None
                    linea = f"DEV,0.000,FLAG,OFF,RAD,{radial:.1f},CONS,{consist:.2f}\n"
                    estado_txt = "OFF (sin señal / baja consistencia)"
                with estado.lock:
                    ident = estado.ident
                if ident and (ahora - ident[1]) < args.ident_hold_s and not asentando:
                    linea = linea.rstrip("\n") + f",ID,{ident[0]}\n"
                    estado_txt += f" | ID {ident[0]}"
                with lock_panel:
                    panel.write(linea.encode()); panel.flush()
                if ki207:
                    ki207.write(linea.encode()); ki207.flush()
                    try:
                        # el ESP32 del KI-207 responde "OK ..." a cada linea: se lee y se descarta
                        # para que el buffer del puerto no se llene
                        if ki207.in_waiting:
                            ki207.read(ki207.in_waiting)
                    except (serial.SerialException, OSError):
                        pass
                if estado.modo == "ADSB":
                    estado_txt = "ADS-B (VOR en pausa)"
                else:
                    print(f"  radial={radial:5.1f}° (crudo {radial_crudo:5.1f}°) cons={consist:.2f} curso={curso:.0f}° -> {estado_txt}")
                if not asentando and estado.modo == "VOR":
                    st_tot += 1
                    if n >= 5:
                        st_cons.append(consist)
                        st_sin += math.sin(math.radians(radial_crudo))
                        st_cos += math.cos(math.radians(radial_crudo))
                    if n >= 5 and consist >= args.min_consistency:
                        st_ok += 1
                if flog:
                    flog.write(f"{time.strftime('%H:%M:%S')},{radial_crudo:.1f},{radial:.1f},{consist:.3f},{n},{estado_txt.split(' ')[0]}\n")
                    flog.flush()
                if args.stats_s > 0 and ahora - t_stats >= args.stats_s and st_tot > 0:
                    if st_cons:
                        m = len(st_cons)
                        R = max(math.hypot(st_sin, st_cos) / m, 1e-9)
                        disp = math.degrees(math.sqrt(max(-2.0 * math.log(min(R, 1.0)), 0.0)))
                        rmed = math.degrees(math.atan2(st_sin, st_cos)) % 360.0
                        print(f"  [calidad {args.stats_s:.0f} s] consistencia media {sum(st_cons)/m:.2f} (min {min(st_cons):.2f}) | "
                              f"radial crudo medio {rmed:.1f}° | dispersion ±{disp:.1f}° | segundos validos {st_ok}/{st_tot}"
                              + ("" if args.no_ident else f" | ident: pico/ruido {estado.ident_ratio:.1f} (ruido {estado.ident_ruido:.1e}, pico {estado.ident_pico:.1e})"))
                    else:
                        print(f"  [calidad {args.stats_s:.0f} s] sin datos del decodificador")
                    st_cons, st_sin, st_cos, st_ok, st_tot = [], 0.0, 0.0, 0, 0
                    t_stats = ahora
                ssin = scos = 0.0
                n = 0
                t_ventana = ahora
    except KeyboardInterrupt:
        print("Deteniendo...")
    finally:
        parar.set()
        panel.close()
        if ki207:
            ki207.close()
        if flog:
            flog.close()
        sock.close()


if __name__ == "__main__":
    main()
