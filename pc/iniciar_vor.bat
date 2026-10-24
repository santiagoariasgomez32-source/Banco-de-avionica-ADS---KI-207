@echo off
REM ============================================================
REM  iniciar_vor.bat - Lanzador de un clic
REM  Abre 2 ventanas con el entorno de GNU Radio:
REM    1. Puente: ESP32 del panel + ESP32 del KI-207 + UDP.
REM    2. Gestor del SDR: arranca el decodificador VOR (PLL) y, con la
REM       pulsacion larga del encoder 2 en el panel, cambia a ADS-B
REM       (1090 MHz) y viceversa.
REM  Todos los ajustes (puertos, frecuencia, ganancias, offset) estan
REM  en config_vor.bat. Este archivo no se edita.
REM  Cierre el Monitor Serie del Arduino IDE antes de ejecutarlo.
REM  Los scripts de Python deben estar en esta misma carpeta.
REM ============================================================

set CARPETA=%~dp0

if not exist "%CARPETA%config_vor.bat" (
  echo No se encuentra config_vor.bat en %CARPETA%
  pause
  exit /b 1
)
call "%CARPETA%config_vor.bat"

if /i "%PUERTO_PANEL%"=="COMX" (
  echo.
  echo FALTA poner el puerto del panel. Abra config_vor.bat y cambie COMX por el COM real.
  echo Para ver los puertos:  python -m serial.tools.list_ports
  echo.
  pause
  exit /b 1
)

if not exist "%RADIOCONDA%\Scripts\activate.bat" (
  echo.
  echo No se encuentra radioconda en %RADIOCONDA%
  echo Corrija la variable RADIOCONDA en config_vor.bat.
  echo.
  pause
  exit /b 1
)

set OPC_KI=
if defined PUERTO_KI207 set OPC_KI=--ki207-port %PUERTO_KI207%

echo Panel: %PUERTO_PANEL%   KI-207: %PUERTO_KI207%   Frecuencia: %FRECUENCIA_HZ%

start "Puente VOR-Panel" cmd /k "call %RADIOCONDA%\Scripts\activate.bat %ENTORNO% && cd /d %CARPETA% && python vor_udp_bridge.py --panel-port %PUERTO_PANEL% %OPC_KI% %OPC_PUENTE%"

timeout /t 2 /nobreak >nul

start "Gestor SDR (VOR / ADS-B)" cmd /k "call %RADIOCONDA%\Scripts\activate.bat %ENTORNO% && cd /d %CARPETA% && python gestor_sdr.py --adsb-lat=%ADSB_LAT% --adsb-lon=%ADSB_LON% --adsb-gain=%ADSB_GAIN% --adsb-alt-ft=%ADSB_ALT_FT% --freq %FRECUENCIA_HZ% %OPC_SDR%"

echo Listo. Esta ventana se puede cerrar.
pause
