/*
  test_link_esp32.ino — Prueba mínima de conectividad de HOY.

  Objetivo: confirmar que el dato que manda test_link_gnuradio.py desde
  la PC llega vivo al ESP32. No mueve el KI-207 todavía — solo hace
  parpadear el LED integrado cada vez que llega un lote de mensajes,
  para que puedas confirmarlo con la vista sin necesitar el Monitor
  Serial (que no puedes abrir mientras el script de Python tiene el
  puerto ocupado).

  Cableado: nada especial. Solo el cable USB del ESP32 a la PC — es el
  mismo puerto que usas para programarlo.
*/

#ifndef LED_BUILTIN
#define LED_BUILTIN 2   // ajusta según tu placa si no se define solo
#endif

int contadorMensajes = 0;
bool estadoLED = false;
float ultimoNivel = 0.0;

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
      ultimoNivel = linea.substring(6).toFloat();
      contadorMensajes++;

      // Cada 10 mensajes (~1 vez por segundo, ya que GNU Radio manda
      // ~10 mensajes/seg) alterna el LED. Así ves un parpadeo claro y
      // constante mientras el enlace esté vivo, en vez de un parpadeo
      // demasiado rápido para percibir.
      if (contadorMensajes >= 10) {
        contadorMensajes = 0;
        estadoLED = !estadoLED;
        digitalWrite(LED_BUILTIN, estadoLED ? HIGH : LOW);
      }
    }
  }
}
