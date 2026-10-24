#!/usr/bin/env python3
"""
vor_radial_decoder_pll.py — La versión "de producción": combina la
cadena de PLL ya validada (consistencia 0.9+ con la estación BOG) con
salida UDP nativa (sin bloques Python personalizados) hacia el puente
vor_udp_bridge.py, y aplica la calibración directamente en el
flowgraph (offset de 270° por defecto, con el sentido de fase invertido, que es la
calibración final del proyecto; se cambia con --offset y --no-invert).

Tambien regenera el identificador Morse (tono de 700 Hz solo cuando hay
ident, sin silbidos) y manda su envolvente por UDP (puerto 6008) a
morse_ident_decoder.py. Cadena portada de vor_test.grc.

Uso:
    python vor_radial_decoder_pll.py --freq 113900000
    python vor_radial_decoder_pll.py --freq 113900000 --no-audio     (sin sonido)
"""

import argparse
import socket
import threading
import time

import numpy as np

from gnuradio import gr, blocks, filter as gr_filter, analog, network, audio
from gnuradio.fft import window
import osmosdr


class VorPllTopBlock(gr.top_block):
    def __init__(self, device_args, freq_hz, udp_port,
                 rf_gain=49.6, if_gain=40, bb_gain=20,
                 var_loop_bw_hz=3.0, ref_loop_bw_hz=300.0, offset_deg=270.0,
                 ident_port=6008, ident_low=0.000067, ident_high=0.0001,
                 ident_hold=3000, ident_on=0.45, ident_off=0.15,
                 volume=0.3, audio_on=True, ident_gain=1.0, debug_rf=False, spectrum=True,
                 invert_phase=True):
        gr.top_block.__init__(self, "VOR Radial PLL - Producción")

        self.samp_rate_rf = 1_200_000   # igual que el GRC (1.2 MS/s; /12 = 100 kHz)
        self.samp_rate_env = 100_000

        self.src = osmosdr.source(args=f"numchan=1 {device_args}")
        self.src.set_sample_rate(self.samp_rate_rf)
        self.src.set_center_freq(freq_hz)
        self.src.set_gain_mode(False)
        # Mismas llamadas que genera GNU Radio Companion (el nombre "RF" no
        # existe en el RTL-SDR y dejaba la ganancia real mucho mas baja).
        self.src.set_gain(rf_gain, 0)
        self.src.set_if_gain(if_gain, 0)
        self.src.set_bb_gain(bb_gain, 0)

        taps_decim = gr_filter.firdes.low_pass(1.0, self.samp_rate_rf, 15_000, 3_000)
        self.decim = gr_filter.fir_filter_ccf(
            self.samp_rate_rf // self.samp_rate_env, taps_decim
        )
        self.envolvente = blocks.complex_to_mag(1)

        # ================== CAMINO VAR ==================
        taps_var_bp = gr_filter.firdes.band_pass(1.0, self.samp_rate_env, 20, 40, 15)
        self.var_bp = gr_filter.fir_filter_fff(1, taps_var_bp)
        self.var_hilbert = gr_filter.hilbert_fc(129)

        self.lo_30hz = analog.sig_source_c(self.samp_rate_env, analog.GR_COS_WAVE, -30.0, 1.0, 0.0)
        self.var_mixer = blocks.multiply_cc(1)

        var_loop_bw = 2 * np.pi * var_loop_bw_hz / self.samp_rate_env
        var_max_freq = 2 * np.pi * 8 / self.samp_rate_env
        var_min_freq = -var_max_freq
        self.var_pll = analog.pll_refout_cc(var_loop_bw, var_max_freq, var_min_freq)

        # ================== CAMINO REF ==================
        taps_ref_bp = gr_filter.firdes.band_pass(1.0, self.samp_rate_env, 9480, 10440, 150)
        self.ref_bp = gr_filter.fir_filter_fff(1, taps_ref_bp)
        self.ref_hilbert = gr_filter.hilbert_fc(129)

        self.lo = analog.sig_source_c(self.samp_rate_env, analog.GR_COS_WAVE, -9960.0, 1.0, 0.0)
        self.mixer = blocks.multiply_cc(1)

        ref_loop_bw = 2 * np.pi * ref_loop_bw_hz / self.samp_rate_env
        ref_max_freq = 2 * np.pi * 700 / self.samp_rate_env
        ref_min_freq = -ref_max_freq
        self.ref_pll_fm = analog.pll_freqdet_cf(ref_loop_bw, ref_max_freq, ref_min_freq)

        taps_ref_bp2 = gr_filter.firdes.band_pass(1.0, self.samp_rate_env, 20, 40, 15)
        self.ref_bp2 = gr_filter.fir_filter_fff(1, taps_ref_bp2)
        self.ref_hilbert2 = gr_filter.hilbert_fc(129)
        self.ref_mixer = blocks.multiply_cc(1)
        self.ref_pll = analog.pll_refout_cc(var_loop_bw, var_max_freq, var_min_freq)

        # ================== COMPARADOR DE FASE + CALIBRACIÓN ==================
        self.mult_conj = blocks.multiply_conjugate_cc(1)
        self.c2arg = blocks.complex_to_arg(1)
        self.rad2deg = blocks.multiply_const_ff(180.0 / np.pi)
        self.calibrar = blocks.add_const_ff(-offset_deg)

        # Tasa de 20 valores/seg confirmada como confiable por UDP. El
        # promedio de 1 segundo para suavizar la aguja ahora se hace
        # del lado del puente (vor_udp_bridge.py), no aquí — un bloque
        # nativo de GNU Radio mandando a una tasa tan baja como 1/seg
        # puede quedarse esperando llenar un paquete UDP indefinidamente.
        self.keep_post = blocks.keep_one_in_n(gr.sizeof_float, self.samp_rate_env // 20)

        # payload de 40 bytes = 10 valores: con 20 valores/s sale un paquete
        # cada 0.5 s. Con 1472 bytes (368 valores) el paquete tardaba ~18 s
        # en llenarse y el puente recibia datos solo a rafagas.
        self.udp = network.udp_sink(gr.sizeof_float, 1, "127.0.0.1", udp_port, 0, 40, True)

        # ================== IDENTIFICADOR MORSE (audio + decodificador) ==================
        # Cadena portada tal cual de tu flowgraph de GNU Radio Companion (vor_test.grc):
        #   envolvente (100 kHz) -> 48 kHz -> DC blocker -> pasabanda 1005-1035 Hz
        #   -> cuadrado -> IIR (envolvente del tono)
        #        |-> Keep 1 in N (1000 muestras/s) -> UDP: morse_ident_decoder.py
        #        '-> Threshold (histeresis) -> promedio movil -> Threshold -> IIR
        #            -> x tono de 700 Hz -> audio
        # El audio se regenera: cuando no hay ident, la compuerta esta cerrada y
        # no suena nada (sin silbidos).
        # Rama identica a la del GRC: SDR 1.2 MS/s -> Freq Xlating FIR (decim 25,
        # pasabajos 6 kHz) -> 48 kHz -> magnitud -> DC blocker -> ...
        taps_xl = gr_filter.firdes.low_pass(1, 1_200_000, 6e3, 1e3)
        self.ident_xl = gr_filter.freq_xlating_fir_filter_ccf(25, taps_xl, 0, 1_200_000)
        self.ident_mag = blocks.complex_to_mag(1)
        # ganancia de amplitud opcional (1.0 = igual que GNU Radio)
        self.ident_gain = blocks.multiply_const_ff(ident_gain)
        self.ident_dc = gr_filter.dc_blocker_ff(32, True)
        taps_ident_bp = gr_filter.firdes.band_pass(
            8.0, 48000, 1005, 1035, 30, window.WIN_BLACKMAN, 6.76)
        self.ident_bp = gr_filter.fir_filter_fff(1, taps_ident_bp)
        self.ident_sq = blocks.multiply_ff(1)
        self.ident_iir1 = gr_filter.single_pole_iir_filter_ff(0.005)

        # rama del decodificador (envolvente analogica por UDP)
        self.ident_keep = blocks.keep_one_in_n(gr.sizeof_float, 48)
        self.ident_udp = network.udp_sink(gr.sizeof_float, 1, "127.0.0.1", ident_port, 0, 200, True)

        # rama de audio (compuerta + tono regenerado)
        self.ident_thr1 = blocks.threshold_ff(ident_low, ident_high, 0)
        self.ident_avg = blocks.moving_average_ff(ident_hold, 1.0 / ident_hold, 4000)
        self.ident_thr2 = blocks.threshold_ff(ident_off, ident_on, 0)
        self.ident_iir2 = gr_filter.single_pole_iir_filter_ff(0.005)
        self.ident_tono = analog.sig_source_f(48000, analog.GR_COS_WAVE, 700, volume, 0)
        self.ident_mix = blocks.multiply_ff(1)
        if audio_on:
            self.ident_audio = audio.sink(48000, "", True)
        else:
            self.ident_audio = blocks.null_sink(gr.sizeof_float)

        # --- Conexiones ---
        self.connect(self.src, self.decim, self.envolvente)

        self.connect(self.src, self.ident_xl, self.ident_mag, self.ident_gain, self.ident_dc, self.ident_bp)
        self.connect(self.ident_bp, (self.ident_sq, 0))
        self.connect(self.ident_bp, (self.ident_sq, 1))
        self.connect(self.ident_sq, self.ident_iir1)
        self.connect(self.ident_iir1, self.ident_keep, self.ident_udp)
        self.connect(self.ident_iir1, self.ident_thr1, self.ident_avg, self.ident_thr2,
                     self.ident_iir2, (self.ident_mix, 0))
        self.connect(self.ident_tono, (self.ident_mix, 1))
        self.connect(self.ident_mix, self.ident_audio)

        # sondas de nivel (solo se leen con --debug-ident)
        self.ident_probe_car = blocks.probe_signal_f()      # nivel de la portadora (magnitud a 48 kHz)
        self.ident_probe_env = blocks.probe_signal_f()
        self.ident_probe_gate = blocks.probe_signal_f()
        self.spectrum = spectrum
        if spectrum:
            # sondas para el espectro del panel: canal (100 kHz, tras el filtro de 15 kHz) y banda ancha (SDR)
            self.sp_vec_c = blocks.stream_to_vector(gr.sizeof_gr_complex, 4096)
            self.sp_probe_c = blocks.probe_signal_vc(4096)
            self.connect(self.decim, self.sp_vec_c, self.sp_probe_c)
            self.sp_vec_w = blocks.stream_to_vector(gr.sizeof_gr_complex, 1024)
            self.sp_probe_w = blocks.probe_signal_vc(1024)
            self.connect(self.src, self.sp_vec_w, self.sp_probe_w)
        if debug_rf:
            # sonda de la entrada del SDR para detectar saturacion (se lee con --debug-rf)
            self.rf_mag = blocks.complex_to_mag(1)
            self.rf_vec = blocks.stream_to_vector(gr.sizeof_float, 4096)
            self.rf_probe = blocks.probe_signal_vf(4096)
            self.connect(self.src, self.rf_mag, self.rf_vec, self.rf_probe)
        self.connect(self.ident_mag, self.ident_probe_car)
        self.connect(self.ident_iir1, self.ident_probe_env)
        self.connect(self.ident_iir2, self.ident_probe_gate)

        self.connect(self.envolvente, self.var_bp, self.var_hilbert)
        self.connect(self.var_hilbert, (self.var_mixer, 0))
        self.connect(self.lo_30hz, (self.var_mixer, 1))
        self.connect(self.var_mixer, self.var_pll)

        self.connect(self.envolvente, self.ref_bp, self.ref_hilbert)
        self.connect(self.ref_hilbert, (self.mixer, 0))
        self.connect(self.lo, (self.mixer, 1))
        self.connect(self.mixer, self.ref_pll_fm)
        self.connect(self.ref_pll_fm, self.ref_bp2, self.ref_hilbert2)
        self.connect(self.ref_hilbert2, (self.ref_mixer, 0))
        self.connect(self.lo_30hz, (self.ref_mixer, 1))
        self.connect(self.ref_mixer, self.ref_pll)

        # Sentido de la resta de fase. Por defecto: arg(var * conj(ref)) = fase_var - fase_ref.
        # En un VOR la fase de la senal variable ATRASA respecto a la referencia en el angulo del
        # radial, es decir radial = fase_ref - fase_var. Con --invert se usa ese sentido
        # (arg(ref * conj(var))), que es el que corresponde si el radial sale "espejado".
        if invert_phase:
            self.connect(self.ref_pll, (self.mult_conj, 0))
            self.connect(self.var_pll, (self.mult_conj, 1))
        else:
            self.connect(self.var_pll, (self.mult_conj, 0))
            self.connect(self.ref_pll, (self.mult_conj, 1))
        self.connect(self.mult_conj, self.c2arg, self.rad2deg, self.calibrar, self.keep_post, self.udp)

    def set_freq(self, freq_hz):
        self.src.set_center_freq(freq_hz)


class EspectroPanel:
    """Calcula el espectro del canal y de la banda y lo manda por UDP al puente (que lo reenvia
    al OLED del panel). Cada linea: SPC,<C/N dB>,<128 caracteres>  /  SPW,0,<128 caracteres>
    con caracter = chr(48 + nivel 0..63) (nunca coincide con ',' ni con el salto de linea)."""

    COLS = 128

    def __init__(self, tb, puerto, db_min=-90.0, db_max=-10.0, debug=False):
        self.tb = tb
        self.addr = ("127.0.0.1", puerto)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.db_min, self.db_max = db_min, db_max
        self.debug = debug
        self.win_c = np.hanning(4096).astype(np.float32)
        self.win_w = np.hanning(1024).astype(np.float32)
        self.acc_c = np.zeros(4096)
        self.acc_w = np.zeros(1024)
        self.n_c = 0
        self.n_w = 0
        self.t_envio = time.time()
        self.t_dbg = time.time()
        self.ultimo_cn = 0.0

    @staticmethod
    def _potencia(vec, ventana):
        x = np.asarray(vec, dtype=np.complex64)
        if x.size != ventana.size:
            return None
        X = np.fft.fftshift(np.fft.fft(x * ventana))
        return (np.abs(X) / ventana.sum()) ** 2          # 1.0 = tono de plena escala

    def _nivel(self, db):
        v = (db - self.db_min) / (self.db_max - self.db_min)
        return np.clip(v, 0.0, 1.0) * 63.0

    def _texto(self, niveles):
        return "".join(chr(48 + int(round(n))) for n in niveles)

    def muestrear(self):
        """Tomar una instantanea de cada sonda (llamar ~20 veces por segundo)."""
        pc = self._potencia(self.tb.sp_probe_c.level(), self.win_c)
        if pc is not None:
            self.acc_c += pc
            self.n_c += 1
        pw = self._potencia(self.tb.sp_probe_w.level(), self.win_w)
        if pw is not None:
            self.acc_w += pw
            self.n_w += 1

    def enviar_si_toca(self, cada_s=0.25):
        ahora = time.time()
        if ahora - self.t_envio < cada_s:
            return
        self.t_envio = ahora
        if self.n_c > 0:
            db = 10.0 * np.log10(self.acc_c / self.n_c + 1e-20)
            # +-640 bins (+-15.6 kHz) -> 128 columnas de 10 bins (se queda el maximo de cada grupo)
            seg = db[2048 - 640:2048 + 640].reshape(self.COLS, 10).max(axis=1)
            # C/N: portadora (maximo en +-120 Hz) contra la mediana del ruido entre 2 y 15 kHz
            portadora = float(db[2048 - 5:2048 + 6].max())
            f = np.abs(np.arange(4096) - 2048) * (100000.0 / 4096)
            ruido = float(np.median(db[(f > 2000.0) & (f < 15000.0)]))
            self.ultimo_cn = portadora - ruido
            msg = f"SPC,{int(round(self.ultimo_cn))},{self._texto(self._nivel(seg))}\n"
            self.sock.sendto(msg.encode("ascii"), self.addr)
            if self.debug and ahora - self.t_dbg >= 3.0:
                print(f"[espectro] portadora {portadora:.1f} dBFS | ruido {ruido:.1f} dBFS | C/N {self.ultimo_cn:.1f} dB")
                self.t_dbg = ahora
        if self.n_w > 0:
            db = 10.0 * np.log10(self.acc_w / self.n_w + 1e-20)
            seg = db.reshape(self.COLS, 8).max(axis=1)
            msg = f"SPW,0,{self._texto(self._nivel(seg))}\n"
            self.sock.sendto(msg.encode("ascii"), self.addr)
        self.acc_c[:] = 0.0
        self.acc_w[:] = 0.0
        self.n_c = self.n_w = 0


class ControlListener(threading.Thread):
    """Escucha por UDP local mensajes de texto "FREQ,<kHz>" que manda el
    puente (vor_udp_bridge.py) y retonaliza el SDR. Así solo UN proceso
    abre el puerto serial del panel (el puente) y no hay conflicto."""

    def __init__(self, tb, port):
        super().__init__(daemon=True)
        self.tb = tb
        self.running = True
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", port))
        self.sock.settimeout(0.5)

    def run(self):
        while self.running:
            try:
                data, _ = self.sock.recvfrom(256)
            except socket.timeout:
                continue
            except OSError:
                break
            partes = data.decode(errors="ignore").strip().split(",")
            if len(partes) >= 2 and partes[0] == "FREQ":
                try:
                    freq_hz = float(partes[1]) * 1000.0
                    if 60e6 <= freq_hz <= 1700e6:
                        self.tb.set_freq(freq_hz)
                        print(f"[Control] Retonalizado a {freq_hz/1e6:.3f} MHz")
                except ValueError:
                    pass

    def stop(self):
        self.running = False
        self.sock.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--device-args", default="")
    ap.add_argument("--freq", type=float, required=True)
    ap.add_argument("--udp-port", type=int, default=6006)
    ap.add_argument("--control-port", type=int, default=6007)
    ap.add_argument("--rf-gain", type=float, default=49.6)
    ap.add_argument("--if-gain", type=float, default=40.0)
    ap.add_argument("--bb-gain", type=float, default=20.0)
    ap.add_argument("--offset", type=float, default=270.0,
                    help="offset de calibracion en grados (depende del sitio de la antena)")
    ap.add_argument("--invert", action=argparse.BooleanOptionalAction, default=True,
                    help="sentido de la resta de fase (radial = fase_ref - fase_var). Activo por defecto; "
                         "--no-invert lo desactiva y obliga a recalibrar --offset")
    ap.add_argument("--var-bw", type=float, default=3.0,
                    help="ancho de banda (Hz) del PLL de 30 Hz variable: menor = radial mas estable pero engancha mas lento")
    ap.add_argument("--ref-bw", type=float, default=300.0, help="ancho de banda (Hz) del PLL de la referencia FM")
    # --- identificador Morse (valores de tu flowgraph de GNU Radio) ---
    ap.add_argument("--volume", type=float, default=0.3, help="volumen del identificador (amplitud del tono de 700 Hz)")
    ap.add_argument("--no-audio", action="store_true", help="no abrir la salida de audio (solo radial)")
    ap.add_argument("--ident-gain", type=float, default=1.0,
                    help="ganancia de amplitud antes del pasabanda del ident (1.0 = igual que GNU Radio; "
                         "si los niveles salen ~10 veces mas bajos, prueba 10)")
    ap.add_argument("--ident-port", type=int, default=6008, help="puerto UDP de la envolvente del ident")
    ap.add_argument("--no-spectrum", action="store_true", help="no calcular el espectro para el OLED del panel")
    ap.add_argument("--spec-port", type=int, default=6009, help="puerto UDP al que se manda el espectro (lo reenvia el puente)")
    ap.add_argument("--spec-min", type=float, default=-90.0, help="nivel minimo del espectro (dBFS)")
    ap.add_argument("--spec-max", type=float, default=-10.0, help="nivel maximo del espectro (dBFS)")
    ap.add_argument("--debug-spec", action="store_true", help="imprime portadora, ruido y C/N del canal cada 3 s")
    ap.add_argument("--debug-rf", action="store_true",
                    help="imprime cada 3 s el nivel de entrada del SDR (pico ~1 = saturado, hay que bajar la ganancia)")
    ap.add_argument("--debug-ident", action="store_true",
                    help="cada 3 s imprime el nivel de la envolvente y cuanto tiempo estuvo abierta la compuerta")
    ap.add_argument("--ident-low", type=float, default=0.000067, help="Threshold 1: umbral bajo")
    ap.add_argument("--ident-high", type=float, default=0.0001, help="Threshold 1: umbral alto")
    ap.add_argument("--ident-hold", type=int, default=3000, help="largo del promedio movil (muestras a 48 kHz)")
    ap.add_argument("--ident-on", type=float, default=0.45, help="Threshold 2: umbral alto")
    ap.add_argument("--ident-off", type=float, default=0.15, help="Threshold 2: umbral bajo")
    args = ap.parse_args()

    tb = VorPllTopBlock(
        args.device_args, args.freq, args.udp_port,
        rf_gain=args.rf_gain, if_gain=args.if_gain, bb_gain=args.bb_gain,
        var_loop_bw_hz=args.var_bw, ref_loop_bw_hz=args.ref_bw,
        offset_deg=args.offset,
        ident_port=args.ident_port, ident_low=args.ident_low, ident_high=args.ident_high,
        ident_hold=args.ident_hold, ident_on=args.ident_on, ident_off=args.ident_off,
        volume=args.volume, audio_on=not args.no_audio, ident_gain=args.ident_gain,
        debug_rf=args.debug_rf, spectrum=not args.no_spectrum, invert_phase=args.invert,
    )
    tb.start()
    print(f"Corriendo. Mandando radial calibrado por UDP a 127.0.0.1:{args.udp_port}")
    print("Dale unos segundos para que los PLL enganchen.")
    print(f"Identificador Morse: audio {'DESACTIVADO' if args.no_audio else 'ACTIVO'} | volumen {args.volume} | "
          f"umbral1 {args.ident_low}/{args.ident_high} | umbral2 {args.ident_off}/{args.ident_on} | "
          f"ganancia {args.ident_gain} | envolvente UDP {args.ident_port}")

    panel = ControlListener(tb, args.control_port)
    panel.start()
    print(f"Escuchando cambios de frecuencia en UDP 127.0.0.1:{args.control_port}")

    print("Ctrl+C para detener.")
    try:
        t_dbg = time.time()
        env_max, env_min, abiertas, muestras, car_sum = 0.0, 1e9, 0, 0, 0.0
        rf_pico, rf_cuad, rf_n, rf_sat = 0.0, 0.0, 0, 0
        t_rf = time.time()
        esp = None if args.no_spectrum else EspectroPanel(
            tb, args.spec_port, args.spec_min, args.spec_max, args.debug_spec)
        while True:
            if esp is None and not args.debug_ident and not args.debug_rf:
                time.sleep(1)
                continue
            time.sleep(0.05)
            if esp is not None:
                try:
                    esp.muestrear()
                    esp.enviar_si_toca()
                except Exception as e:      # el espectro nunca debe tirar el decodificador
                    print(f"[espectro] error: {e}")
                    esp = None
            if args.debug_rf:
                v = tb.rf_probe.level()
                if len(v) > 0:
                    pk = max(v)
                    rf_pico = max(rf_pico, pk)
                    rf_cuad += sum(x * x for x in v) / len(v)
                    rf_n += 1
                    rf_sat += 1 if pk >= 0.95 else 0
                if time.time() - t_rf >= 3.0:
                    rms = (rf_cuad / max(rf_n, 1)) ** 0.5
                    aviso = "  <-- POSIBLE SATURACION, baja la ganancia" if rf_sat > 0 else ""
                    print(f"[rf] pico {rf_pico:.2f} | nivel rms {rms:.3f} | instantes con pico >= 0.95: {rf_sat}/{rf_n}{aviso}")
                    t_rf = time.time()
                    rf_pico, rf_cuad, rf_n, rf_sat = 0.0, 0.0, 0, 0
            if not args.debug_ident:
                continue
            env = tb.ident_probe_env.level()
            car_sum += tb.ident_probe_car.level()
            env_max = max(env_max, env)
            env_min = min(env_min, env)
            abiertas += 1 if tb.ident_probe_gate.level() > 0.5 else 0
            muestras += 1
            if time.time() - t_dbg >= 3.0:
                print(f"[ident] portadora {car_sum / max(muestras, 1):.4f} | "
                      f"envolvente min {env_min:.2e} max {env_max:.2e} | "
                      f"compuerta abierta {100.0 * abiertas / max(muestras, 1):.0f}%")
                t_dbg = time.time()
                env_max, env_min, abiertas, muestras, car_sum = 0.0, 1e9, 0, 0, 0.0
    except KeyboardInterrupt:
        pass
    finally:
        panel.stop()
        tb.stop()
        tb.wait()


if __name__ == "__main__":
    main()
