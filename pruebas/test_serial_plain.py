#!/usr/bin/env python3
"""
test_serial_plain.py — Prueba el puerto serial solo, sin GNU Radio.

Si esto falla, el problema es del puerto COM3 en sí (otro programa
tiene el puerto abierto, permisos, cable, driver). Si esto funciona
bien, el problema está en cómo el bloque personalizado de GNU Radio
usa ese mismo puerto.
"""

import time
import serial

PUERTO = "COM3"
BAUDRATE = 115200

print(f"Intentando abrir {PUERTO}...")
try:
    ser = serial.Serial(PUERTO, BAUDRATE, timeout=1)
    print("Puerto abierto correctamente.")
except serial.SerialException as e:
    print(f"ERROR al abrir el puerto: {e}")
    raise SystemExit(1)

print("Escribiendo 10 mensajes de prueba, uno por segundo...")
for i in range(10):
    linea = f"LEVEL,{i * 0.01:.5f}\n"
    ser.write(linea.encode())
    print(f"  Enviado: {linea.strip()}")
    time.sleep(1)

ser.close()
print("Listo, terminó sin errores.")
