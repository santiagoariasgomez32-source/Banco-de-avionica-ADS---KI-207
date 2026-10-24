/*
  ki207_test_echo.ino — Prueba de mañana: confirmar que los datos
  REALES del VOR (ya calibrados) llegan correctamente al ESP32, sin
  necesitar el instrumento KI-207 físico ni los DAC conectados.

  No llama a ninguna función de DAC/I2C — así que funciona con el
  ESP32 solo, sin ningún otro hardware conectado.

  Qué hace:
    1. Parpadea brevemente el LED cada vez que llega un mensaje válido
       (confirmación visual rápida de "algo está llegando").
    2. Responde por el mismo puerto serial repitiendo los valores que
       entendió, para que los veas con precisión total en la terminal
       de la PC (en vez de tener que interpretar parpadeos).
*/

#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif

unsigned long ultimoDato = 0;
const unsigned long TIMEOUT_MS = 1000;

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
}

int dividirEnPartes(String linea, String partes[], int maxPartes) {
  int contador = 0;
  int inicio = 0;
  int coma;
  while (contador < maxPartes) {
    coma = linea.indexOf(',', inicio);
    if (coma == -1) {
      partes[contador++] = linea.substring(inicio);
      break;
    }
    partes[contador++] = linea.substring(inicio, coma);
    inicio = coma + 1;
  }
  return contador;
}

void loop() {
  if (Serial.available()) {
    String linea = Serial.readStringUntil('\n');
    linea.trim();

    if (linea.startsWith("DEV,")) {
      String partes[6];
      int n = dividirEnPartes(linea, partes, 6);

      if (n >= 6) {
        String deflexion = partes[1];
        String estado = partes[3];
        String radial = partes[5];

        // Validar que los campos tengan sentido antes de confiar en
        // ellos. Si llegó un byte corrupto por el camino (puede pasar
        // a alta velocidad por serial), es mejor descartar la línea
        // completa que mostrar un dato roto o a medias.
        bool estadoValido = (estado == "TO" || estado == "FROM");
        bool radialValido = true;
        for (unsigned int i = 0; i < radial.length(); i++) {
          char c = radial.charAt(i);
          if (!isDigit(c) && c != '.' && c != '-') {
            radialValido = false;
            break;
          }
        }
        bool deflexionValida = true;
        for (unsigned int i = 0; i < deflexion.length(); i++) {
          char c = deflexion.charAt(i);
          if (!isDigit(c) && c != '.' && c != '-') {
            deflexionValida = false;
            break;
          }
        }

        if (estadoValido && radialValido && deflexionValida && radial.length() > 0) {
          // Parpadeo corto (100ms) como confirmación visual rápida
          digitalWrite(LED_BUILTIN, HIGH);
          delay(100);
          digitalWrite(LED_BUILTIN, LOW);

          Serial.print("ESP32 recibio -> deflexion=");
          Serial.print(deflexion);
          Serial.print(" flag=");
          Serial.print(estado);
          Serial.print(" radial=");
          Serial.println(radial);

          ultimoDato = millis();
        } else {
          Serial.println("ESP32: linea descartada (datos corruptos)");
        }
      }
    }
  }
}
