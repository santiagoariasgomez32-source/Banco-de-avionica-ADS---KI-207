/*
  ki207_bridge.ino
  Lee por Serial (115200) una linea por segundo con el formato:
    DEV,<-1..1>,FLAG,<TO|FROM|OFF>,RAD,<grados>,CONS,<0..1>[,ID,<texto>]
  y mueve CDI, GS (simulado a partir del DEV) y la bandera FROM/TO
  con la calibracion confirmada en las pruebas de calibracion.

  Mapeo:
    DEV=-1  -> CDI/GS en el extremo izquierdo (centro - fondo_escala)
    DEV=0   -> centrado
    DEV=+1  -> extremo derecho (centro + fondo_escala)
    fondo_escala = marcas * paso  (5 marcas x 19 = 95 cuentas = 10 grados)

  GS no tiene dato real todavia: se simula como DEV_CDI * FACTOR_GS.

  Si no llega una linea valida en TIMEOUT_MS, se asume "sin senal":
  CDI y GS vuelven al centro, FROMTO vuelve al centro (no hay circuito
  de bandera de alarma N/F todavia, asi que esto es lo mas cercano a
  "invalido" que el hardware actual puede mostrar).
*/

#include <Wire.h>
#include <Adafruit_MCP4725.h>

Adafruit_MCP4725 dac_CDI;
Adafruit_MCP4725 dac_GS;
Adafruit_MCP4725 dac_FROMTO;

#define TCA_ADDR 0x70
#define CH_CDI    0
#define CH_GS     1
#define CH_FROMTO 2

// ----- Calibracion CDI -----
const int centro_CDI = 2067;
const int paso_CDI = 19;
const int marcas_CDI = 5;
const int fondoEscala_CDI = paso_CDI * marcas_CDI;   // 95 cuentas = 10 grados

// ----- Calibracion GS (simulado) -----
const int centro_GS = 2053;
const int paso_GS = 19;
const int marcas_GS = 5;
const int fondoEscala_GS = paso_GS * marcas_GS;      // 95 cuentas
const float FACTOR_GS = 0.5;   // que tanto del DEV del CDI se refleja en GS (ajustable)

// ----- FROMTO -----
const int centro_FROMTO = 2045;
const int desviacion_FROMTO = 100;
const int disparo_ms = 2;

// ----- Movimiento suave CDI/GS -----
const int paso_vuelo_ms = 25;   // cada cuanto avanza 1 cuenta hacia el objetivo

// ----- Timeout de datos -----
const unsigned long TIMEOUT_MS = 3000;

long posCDI, objCDI;
long posGS, objGS;
long posFROMTO;

String estadoFlagActual = "OFF";
unsigned long ultimaLineaValida = 0;
unsigned long ultimoPasoVuelo = 0;

String bufferRx = "";

void tcaSelect(uint8_t canal) {
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << canal);
  Wire.endTransmission();
}

void escribirDAC(uint8_t canal, Adafruit_MCP4725 &dac, int valor) {
  tcaSelect(canal);
  dac.setVoltage(valor, false);
}

void rampaFROMTO(int desde, int hasta) {
  int paso = (hasta >= desde) ? 1 : -1;
  for (int v = desde; v != hasta + paso; v += paso) {
    escribirDAC(CH_FROMTO, dac_FROMTO, v);
    delay(disparo_ms);
  }
}

void moverFlag(String nuevoEstado) {
  if (nuevoEstado == estadoFlagActual) return;

  int destino;
  if (nuevoEstado == "TO") {
    destino = centro_FROMTO + desviacion_FROMTO;
  } else if (nuevoEstado == "FROM") {
    destino = centro_FROMTO - desviacion_FROMTO;
  } else {
    destino = centro_FROMTO; // OFF / sin señal
  }

  rampaFROMTO(posFROMTO, destino);
  posFROMTO = destino;
  estadoFlagActual = nuevoEstado;
}

bool esNumero(const String &s) {
  if (s.length() == 0) return false;
  for (unsigned int i = 0; i < s.length(); i++) {
    char c = s.charAt(i);
    if (!(isDigit(c) || c == '.' || c == '-' || c == '+')) return false;
  }
  return true;
}

// Parser generico por pares clave,valor separados por coma
void procesarLinea(String linea) {
  linea.trim();
  if (linea.length() == 0) return;

  float dev = 0;
  String flag = "OFF";
  float rad = 0;
  float cons = 0;
  bool devOk = false, flagOk = false;

  int inicio = 0;
  while (inicio < (int)linea.length()) {
    int c1 = linea.indexOf(',', inicio);
    if (c1 < 0) break;
    int c2 = linea.indexOf(',', c1 + 1);
    String clave = linea.substring(inicio, c1);
    String valor = (c2 < 0) ? linea.substring(c1 + 1) : linea.substring(c1 + 1, c2);

    if (clave == "DEV" && esNumero(valor)) {
      dev = valor.toFloat();
      devOk = true;
    } else if (clave == "FLAG") {
      if (valor == "TO" || valor == "FROM" || valor == "OFF") {
        flag = valor;
        flagOk = true;
      }
    } else if (clave == "RAD" && esNumero(valor)) {
      rad = valor.toFloat();
    } else if (clave == "CONS" && esNumero(valor)) {
      cons = valor.toFloat();
    }
    // ID se ignora por ahora

    if (c2 < 0) break;
    inicio = c2 + 1;
  }

  if (!devOk || !flagOk) return;  // linea incompleta o corrupta, se descarta

  // limitar DEV a su rango valido
  if (dev > 1.0) dev = 1.0;
  if (dev < -1.0) dev = -1.0;

  objCDI = centro_CDI + (long)round(dev * fondoEscala_CDI);

  float devGS = dev * FACTOR_GS;
  if (devGS > 1.0) devGS = 1.0;
  if (devGS < -1.0) devGS = -1.0;
  objGS = centro_GS + (long)round(devGS * fondoEscala_GS);

  moverFlag(flag);

  ultimaLineaValida = millis();

  Serial.print("OK dev=");
  Serial.print(dev, 3);
  Serial.print(" flag=");
  Serial.print(flag);
  Serial.print(" rad=");
  Serial.print(rad, 1);
  Serial.print(" cons=");
  Serial.println(cons, 2);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(21, 22);

  Wire.beginTransmission(TCA_ADDR);
  if (Wire.endTransmission() == 0) {
    Serial.println("TCA9548A detectado en 0x70");
  } else {
    Serial.println("ERROR: TCA9548A no responde.");
  }

  tcaSelect(CH_CDI);
  if (!dac_CDI.begin(0x60, &Wire)) Serial.println("ERROR: CDI no responde");

  tcaSelect(CH_GS);
  if (!dac_GS.begin(0x60, &Wire)) Serial.println("ERROR: GS no responde");

  tcaSelect(CH_FROMTO);
  if (!dac_FROMTO.begin(0x60, &Wire)) Serial.println("ERROR: FROMTO no responde");

  posCDI = objCDI = centro_CDI;
  posGS = objGS = centro_GS;
  posFROMTO = centro_FROMTO;

  escribirDAC(CH_CDI, dac_CDI, posCDI);
  escribirDAC(CH_GS, dac_GS, posGS);
  escribirDAC(CH_FROMTO, dac_FROMTO, posFROMTO);

  ultimaLineaValida = millis();

  Serial.println("ki207_bridge listo. Esperando datos...");
}

void loop() {
  // --- leer serial, linea por linea ---
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') {
      procesarLinea(bufferRx);
      bufferRx = "";
    } else if (c != '\r' && bufferRx.length() < 120) {
      bufferRx += c;
    }
  }

  // --- timeout: si no hay datos frescos, ir a posicion neutra ---
  if (millis() - ultimaLineaValida > TIMEOUT_MS) {
    objCDI = centro_CDI;
    objGS = centro_GS;
    if (estadoFlagActual != "OFF") {
      moverFlag("OFF");
    }
  }

  // --- mover CDI y GS suavemente hacia el objetivo ---
  if (millis() - ultimoPasoVuelo >= (unsigned long)paso_vuelo_ms) {
    ultimoPasoVuelo = millis();

    if (posCDI != objCDI) {
      posCDI += (objCDI > posCDI) ? 1 : -1;
      escribirDAC(CH_CDI, dac_CDI, posCDI);
    }
    if (posGS != objGS) {
      posGS += (objGS > posGS) ? 1 : -1;
      escribirDAC(CH_GS, dac_GS, posGS);
    }
  }
}
