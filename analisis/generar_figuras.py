"""
generar_figuras.py - Genera las 4 figuras del hallazgo del diente de sierra del radial.

USO (en una terminal, estando en esta carpeta):
    pip install numpy matplotlib
    python generar_figuras.py

ENTRADAS (carpeta datos/):
    prueba1_puente.txt ... prueba5_puente.txt   consola del puente (vor_udp_bridge.py) copiada a un .txt
    dump1_var_30hz.bin, dump1_ref_30hz.bin,
    dump1_var_pll.bin,  dump1_ref_pll.bin       volcados de --dump-30hz dump1 (prueba 4)
SALIDAS (carpeta figuras/):
    fig1_diente_de_sierra.png, fig2_rizado_y_muestreo.png,
    fig3_antes_y_despues.png, fig4_tonos_constantes_y_prueba_corregida.png
Ademas imprime en pantalla las cifras que se citan en el documento.
"""
import os, re
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.ticker import FuncFormatter

# ------------------------- CONFIGURACION -------------------------
AQUI = os.path.dirname(os.path.abspath(__file__))
CARPETA_DATOS = os.path.join(AQUI, 'datos')
CARPETA_FIGURAS = os.path.join(AQUI, 'figuras')
PREFIJO_DUMP = 'dump1'     # el que se uso en --dump-30hz
OFFSET = 270.0             # offset con el que se tomaron las pruebas (antes de recalibrar a 263,9)
FS = 500.0                 # muestras/s de los volcados (100000 // 500 en el decodificador)
PASO = 25                  # 500 muestras/s / 20 Hz: una muestra cada 0,05 s
SALTAR_S = 12              # segundos iniciales que ignora el puente tras sintonizar (--settle-s 12)
PRUEBAS = {                # clave: (archivo, titulo)
    'P1': ('prueba1_puente.txt', 'Prueba 1 (dongle ya caliente, con audio)'),
    'P2': ('prueba2_puente.txt', 'Prueba 2 (arranque en frío, con audio)'),
    'P3': ('prueba3_puente.txt', 'Prueba 3 (sin audio)'),
    'P4': ('prueba4_puente.txt', 'Prueba 4 (sin audio, con volcado de tonos)'),
    'P5': ('prueba5_puente.txt', 'Prueba 5 (con la corrección)'),
}
# ------------------------------------------------------------------

os.makedirs(CARPETA_FIGURAS, exist_ok=True)
plt.rcdefaults()
matplotlib.rcParams['font.family'] = 'DejaVu Sans'

# Linea del puente:  [mm:ss HH:MM:SS] radial=  110.4° (crudo  110.5°) cons=1.00 curso=0° -> TO ...
PAT = re.compile(r'^\[(\d\d):(\d\d) (\d\d:\d\d:\d\d)\] radial=\s*([\d.]+)° \(crudo\s*([\d.]+)°\) cons=([\d.]+)')

def cargar(clave):
    """Lee la consola del puente. Devuelve t (s), radial crudo y radial filtrado (grados)."""
    t, crudo, filtrado = [], [], []
    with open(os.path.join(CARPETA_DATOS, PRUEBAS[clave][0]), encoding='utf-8', errors='replace') as f:
        for linea in f:
            m = PAT.match(linea)
            if m:
                t.append(int(m[1]) * 60 + int(m[2]))
                filtrado.append(float(m[4]))
                crudo.append(float(m[5]))
    return dict(t=np.array(t), r=np.array(crudo), rf=np.array(filtrado))

def validos(d):
    """Solo los segundos con radial real (el puente imprime 0,0 mientras engancha)."""
    return (d['r'] > 0) & (d['t'] >= SALTAR_S)

# Coma decimal en los ejes (APA en espanol)
coma = FuncFormatter(lambda v, p: ('%g' % round(v, 6)).replace('.', ','))
def formato(ax):
    ax.xaxis.set_major_formatter(coma)
    ax.yaxis.set_major_formatter(coma)
    ax.grid(True)

def a_radial(fase_rad):
    """Fase (rad) de REF*conj(VAR) a radial crudo: grados menos el offset, en 0-360."""
    return (np.degrees(fase_rad) - OFFSET) % 360

def media_circular(grados):
    mu = np.degrees(np.angle(np.mean(np.exp(1j * np.radians(grados))))) % 360
    return mu, (grados - mu + 180) % 360 - 180

# ------------------------- Datos del volcado -------------------------
def leer(nombre, tipo):
    return np.fromfile(os.path.join(CARPETA_DATOS, f'{PREFIJO_DUMP}_{nombre}.bin'), dtype=tipo)
var_pll, ref_pll = leer('var_pll', np.complex64), leer('ref_pll', np.complex64)
var_30, ref_30 = leer('var_30hz', np.float32), leer('ref_30hz', np.float32)
n = min(len(var_pll), len(ref_pll), len(var_30), len(ref_30))
t = np.arange(n) / FS
# Misma operacion que hace el decodificador: REF * conj(VAR) (con --invert)
z = (ref_pll[:n] * np.conj(var_pll[:n]))
fase = np.angle(z)

# ------------------------- Figura 1 -------------------------
fig, axs = plt.subplots(4, 1, figsize=(8, 10), sharex=True, sharey=True)
for ax, k in zip(axs, ('P1', 'P2', 'P3', 'P4')):
    d = cargar(k); ok = validos(d)
    ax.plot(d['t'][ok] / 60, d['r'][ok], lw=.8, label='crudo (1 s)')
    ax.plot(d['t'][ok] / 60, d['rf'][ok], lw=.8, label='filtrado por el puente')
    ax.legend(loc='upper right', fontsize=8)
    ax.set_title(PRUEBAS[k][1] + ', duración %s min' % ('%.1f' % (d['t'][-1] / 60)).replace('.', ','))
    ax.set_ylabel('Radial crudo (°)'); ax.set_ylim(98, 128); formato(ax)
axs[-1].set_xlabel('Tiempo desde el arranque del puente (min)')
fig.tight_layout(); fig.savefig(os.path.join(CARPETA_FIGURAS, 'fig1_diente_de_sierra.png'), dpi=100); plt.close()

# ------------------------- Figura 2 -------------------------
# Dos instantes separados medio periodo del diente de sierra (periodo = 1/0,0018 Hz aprox.)
periodo = 1 / 0.0018
fig = plt.figure(figsize=(10, 7.5)); gs = fig.add_gridspec(2, 2)
for j, t0 in enumerate((100.0, 100.0 + periodo / 2)):
    ax = fig.add_subplot(gs[0, j])
    i0 = int(t0 * FS); i0 -= i0 % PASO; i1 = i0 + int(0.3 * FS)
    ax.plot((t[i0:i1] - t[i0]) * 1000, a_radial(fase[i0:i1]), lw=1, label='fase instantánea (500 muestras/s)')
    g = np.arange(i0, i1, PASO)   # lo que tomaba el decodificador original
    ax.plot((t[g] - t[i0]) * 1000, a_radial(fase[g]), 'o', color='tab:red', ms=6, label='muestras tomadas cada 0,05 s (20 Hz)')
    ax.set_title('Segundo %.0f del volcado' % t0); ax.set_xlabel('Tiempo (ms)'); ax.set_ylim(98, 134); formato(ax)
    if j == 0: ax.set_ylabel('Radial instantáneo (°)')
    ax.legend(loc='upper right')
ax = fig.add_subplot(gs[1, :])
fase_desenrollada = np.unwrap(fase)
residuo = fase_desenrollada - np.polyval(np.polyfit(t, fase_desenrollada, 3), t)   # quita la tendencia lenta
amp = np.abs(np.fft.rfft(residuo * np.hanning(n))) * 2 / (n / 2); frec = np.fft.rfftfreq(n, 1 / FS)
ax.plot(frec, np.degrees(amp), lw=.8); ax.set_xlim(0, 250)
ax.set_xlabel('Frecuencia (Hz)'); ax.set_ylabel('Amplitud del rizado (°)')
for fr in (60, 120, 180): ax.axvline(fr, color='gray', ls=':', lw=.8)
ax.set_title('Espectro de la diferencia de fase entre referencia y variable (picos en 60, 120 y 180 Hz)'); formato(ax)
fig.tight_layout(); fig.savefig(os.path.join(CARPETA_FIGURAS, 'fig2_rizado_y_muestreo.png'), dpi=100); plt.close()
for fr in (60, 120, 180):
    s = (frec > fr - 1) & (frec < fr + 1); print('rizado en %d Hz: amplitud %.1f°' % (fr, np.degrees(amp[s].max())))

# ------------------------- Figura 3 -------------------------
# Original: un valor cada 25 muestras (=5000 a 100 kS/s). Corregido: promedio de esas 25.
m = n // PASO
original20 = np.angle(z[::PASO])
corregido20 = np.angle(z[:m * PASO].reshape(m, PASO).mean(axis=1))
def promedio_1s(a):            # lo que hace el puente: media circular de 1 s (20 valores)
    k = 20; mm = len(a) // k
    return a_radial(np.angle(np.exp(1j * a[:mm * k]).reshape(mm, k).mean(axis=1)))
orig1, corr1 = promedio_1s(original20), promedio_1s(corregido20)
d4 = cargar('P4'); ok4 = validos(d4)
fig, axs = plt.subplots(3, 1, figsize=(9, 8.5), sharex=True, sharey=True)
axs[0].plot(d4['t'][ok4] / 60, d4['r'][ok4], lw=.8, label='crudo')
axs[0].plot(d4['t'][ok4] / 60, d4['rf'][ok4], lw=.8, label='filtrado por el puente')
axs[0].legend(loc='upper right', fontsize=8)
axs[0].set_title('Prueba 4, medición en vivo con el decodificador original')
tm = (np.arange(len(orig1)) + .5) / 60
axs[1].plot(tm[SALTAR_S:], orig1[SALTAR_S:], lw=.8); axs[1].set_title('Mismos datos crudos del volcado, un valor cada 0,05 s (método original)')
axs[2].plot(tm[SALTAR_S:], corr1[SALTAR_S:], lw=.8); axs[2].set_title('Mismos datos crudos del volcado, promedio de cada 0,05 s (método corregido)')
for ax in axs: ax.set_ylabel('Radial crudo (°)'); ax.set_ylim(98, 122); formato(ax)
axs[-1].set_xlabel('Tiempo (min)')
fig.tight_layout(); fig.savefig(os.path.join(CARPETA_FIGURAS, 'fig3_antes_y_despues.png'), dpi=100); plt.close()

# ------------------------- Figura 4 -------------------------
def ventanas(x, f0, w=5.0):
    """Demodula a f0 y promedia en ventanas de w s (f0=30 para los tonos reales, 0 para la salida de los PLL)."""
    zz = x[:n] * np.exp(-2j * np.pi * f0 * t); k = int(w * FS); mm = n // k
    return zz[:mm * k].reshape(mm, k).mean(axis=1)
antes = np.angle(ventanas(ref_30, 30.0) * np.conj(ventanas(var_30, 30.0)))
despues = np.angle(ventanas(ref_pll, 0.0) * np.conj(ventanas(var_pll, 0.0)))
tw = (np.arange(len(antes)) + .5) * 5 / 60
fig, axs = plt.subplots(1, 2, figsize=(11, 4.2), sharey=True, gridspec_kw={'width_ratios': [3, 1.4]})
ax = axs[0]
ax.plot(tw[3:], a_radial(antes)[3:], lw=1.2, label='REF − VAR antes de los PLL')
ax.plot(tw[3:], a_radial(despues)[3:], lw=1.2, ls='--', label='REF − VAR a la salida de los PLL')
ax.set_title('Diferencia de fase de los dos tonos de 30 Hz (ventanas de 5 s, prueba 4)')
ax.set_xlabel('Tiempo (min)'); ax.set_ylabel('Radial crudo (°)'); ax.set_ylim(98, 122); ax.legend(loc='upper right'); formato(ax)
d5 = cargar('P5'); ok5 = validos(d5)
ax = axs[1]; ax.plot(d5['t'][ok5] / 60, d5['r'][ok5], lw=1)
ax.set_title('Prueba 5, con la corrección'); ax.set_xlabel('Tiempo (min)'); formato(ax)
fig.tight_layout(); fig.savefig(os.path.join(CARPETA_FIGURAS, 'fig4_tonos_constantes_y_prueba_corregida.png'), dpi=100); plt.close()

# ------------------------- Cifras del documento -------------------------
print('\nCifras (radial crudo, grados):')
for nombre, x in (('volcado, método original', orig1[SALTAR_S:]), ('volcado, método corregido', corr1[SALTAR_S:]),
                  ('prueba 4 en vivo', d4['r'][ok4]), ('prueba 5 en vivo', d5['r'][ok5])):
    mu, dd = media_circular(x)
    print('  %-26s media %.2f  desv.est. %.2f  mín %.1f  máx %.1f  n=%d' % (nombre, mu, dd.std(), x.min(), x.max(), len(x)))
for nombre, a in (('tonos antes de PLL', antes), ('tonos tras PLL', despues)):
    mu, dd = media_circular(a_radial(a)[3:]); print('  %-26s media %.2f  desv.est. %.2f' % (nombre, mu, dd.std()))
print('\nFiguras guardadas en', CARPETA_FIGURAS)
