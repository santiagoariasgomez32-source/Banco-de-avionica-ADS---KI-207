/*
  test_link_esp32_v2.ino — Igual que la versión anterior, pero ahora
  el ESP32 RESPONDE por el mismo puerto serial cada vez que procesa un
  mensaje válido, diciendo su conteo interno. Así podemos ver desde
  Python (sin pelear por el puerto con el Monitor Serial de Arduino)
  si el ESP32 realmente está recibiendo los 10 mensajes por segundo
  que le estamos mandando, o si está perdiendo la mayoría.
*/

#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif

int contadorMensajes = 0;
long totalRecibidos = 0;
bool estadoLED = false;

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  if (Serial.available()) {
    String linea = Serial.readStringUntil('\n');
    linea.trim();

    if (linea.startsWith("LEVEL,")) {
      totalRecibidos++;
      contadorMensajes++;

      // Responde cada 10 mensajes recibidos, para no saturar el enlace
      if (totalRecibidos % 10 == 0) {
        Serial.print("ACK,");
        Serial.println(totalRecibidos);
      }

      if (contadorMensajes >= 10) {
        contadorMensajes = 0;
        estadoLED = !estadoLED;
        digitalWrite(LED_BUILTIN, estadoLED ? HIGH : LOW);
      }
    }
  }
}
