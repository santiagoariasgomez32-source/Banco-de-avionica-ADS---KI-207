#include <Wire.h>
#include <Adafruit_MCP4725.h>

// Dos DAC independientes
Adafruit_MCP4725 dac_CDI;
Adafruit_MCP4725 dac_GS;

// CDI
int centro_CDI = 2040;
int desviacion_CDI = 76;

// GS (todavía no tenemos dato real de glideslope en este proyecto,
// se queda centrado por ahora)
int centro_GS = 2054;
int desviacion_GS = 40;

// Control de datos "viejos": si no llega nada nuevo en este tiempo,
// centramos la aguja en vez de dejarla mostrando una lectura falsa.
unsigned long ultimoDato = 0;
const unsigned long TIMEOUT_MS = 1000;

void setup() {
  Serial.begin(115200);

  dac_CDI.begin(0x60); // CDI
  dac_GS.begin(0x61);  // GS

  // Arranca centrado, no en la posición de la última prueba
  dac_CDI.setVoltage(centro_CDI, false);
  dac_GS.setVoltage(centro_GS, false);
}

void moverAgujaCDI(float deflexion) {
  // deflexion esperado entre -1.0 (izquierda) y +1.0 (derecha)
  deflexion = constrain(deflexion, -1.0, 1.0);
  int valor = centro_CDI + (int)round(deflexion * desviacion_CDI);
  valor = constrain(valor, 0, 4095);
  dac_CDI.setVoltage(valor, false);
}

void centrarCDI() {
  dac_CDI.setVoltage(centro_CDI, false);
}

void moverBanderaTOFROM(String estado) {
  // TODO: aquí va tu código que YA tienes funcionando para las
  // banderas TO/FROM (mencionaste que ya funcionan). Reemplaza esto
  // con la llamada real, por ejemplo algo como:
  //   if (estado == "TO") { activarBanderaTO(); }
  //   else if (estado == "FROM") { activarBanderaFROM(); }
  //   else { ocultarBanderas(); } // para el caso "sin señal"
}

// Divide una línea tipo "DEV,0.345,FLAG,FROM,RAD,123.4" en sus partes
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

      if (n >= 4) {
        float deflexion = partes[1].toFloat();
        String estado = partes[3]; // "TO" o "FROM"

        moverAgujaCDI(deflexion);
        moverBanderaTOFROM(estado);

        ultimoDato = millis();
      }
    }
  }

  // Si dejó de llegar dato nuevo por más de TIMEOUT_MS, centrar la
  // aguja en vez de dejarla "congelada" mostrando algo falso.
  if (millis() - ultimoDato > TIMEOUT_MS) {
    centrarCDI();
  }
}
