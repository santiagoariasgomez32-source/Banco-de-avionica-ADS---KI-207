@echo off
REM ============================================================
REM  config_vor.bat - AJUSTES DEL USUARIO
REM  Edite SOLO este archivo. iniciar_vor.bat lo lee al arrancar.
REM
REM  Para ver los puertos serie conectados, en la consola del entorno:
REM      python -m serial.tools.list_ports
REM ============================================================

REM --- Entorno de Python con GNU Radio ---------------------------------
REM Carpeta de instalacion de radioconda y nombre del entorno.
REM Por defecto: C:\Users\<usuario>\radioconda y el entorno "gr310".
REM La ruta NO debe tener espacios.
set RADIOCONDA=%USERPROFILE%\radioconda
set ENTORNO=gr310

REM --- Puertos serie -----------------------------------------------------
REM ESP32 del panel (LCD + OLED + encoders). Cambie COMX por el puerto real, ej. COM5.
set PUERTO_PANEL=COMX

REM ESP32 del instrumento KI-207. Deje vacio (set PUERTO_KI207=) para no usarlo.
set PUERTO_KI207=

REM --- Frecuencia inicial del VOR (Hz) -----------------------------------
set FRECUENCIA_HZ=113900000

REM --- Decodificador VOR -------------------------------------------------
REM --offset : offset de calibracion en grados. DEPENDE DEL SITIO de la antena
REM            (270 fue el valor obtenido en las pruebas del proyecto).
REM            Debe recalcularse si la antena cambia de lugar.
REM --invert : sentido de la resta de fase (activo por defecto; calibracion final).
REM Ganancias: con 49.6/40 el SDR se satura; 40/30 queda limpio.
REM --ident-gain compensa el nivel mas bajo para la compuerta del audio del identificador.
set OPC_SDR=--invert --offset 270 --rf-gain 40 --if-gain 30 --ident-gain 2.5

REM --- Opciones extra del puente (identificador Morse, suavizado) --------
REM Ejemplo: --ident-k-on 4 --ident-bridge-ms 60
set OPC_PUENTE=

REM --- Modo ADS-B ----------------------------------------------------------
REM Posicion del receptor (se usa para distancias y alertas TA/RA simuladas).
REM Por defecto, el centro de Bogota. Cambiela por la posicion real de la antena.
set ADSB_LAT=4.7110
set ADSB_LON=-74.0721
REM Altitud de referencia del receptor en pies y ganancia del SDR en ADS-B.
set ADSB_ALT_FT=8500
set ADSB_GAIN=40
