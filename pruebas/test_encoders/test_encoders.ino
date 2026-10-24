// test_encoders.ino - Prueba de hardware del panel (ESP32)
// 1) Escanea el bus I2C (LCD y OLED deben aparecer)
// 2) Lee los 2 encoders (giro + pulsador) y los 2 pulsadores sueltos
// Abre el Monitor Serie a 115200.
//
// Conexiones:
//   (todo en el mismo lado del ESP32: el lado de GPIO21/22)
//   Encoder 1: CLK=19  DT=18  SW=5
//   Encoder 2: CLK=17  DT=16  SW=4
//   Pulsador A=23  Pulsador B=15  (el otro lado a GND)
//   I2C: SDA=21  SCL=22 (LCD y OLED en el mismo bus)
//   Encoders: pin "+" a 3V3, GND a GND

#include <Wire.h>

const int PIN_SDA = 21, PIN_SCL = 22;
const int E1_SW = 5, E2_SW = 4, BTN_A = 23, BTN_B = 15;

struct Enc { int a, b; uint8_t last; int8_t acc; };
Enc e1 = {19, 18, 0, 0};
Enc e2 = {17, 16, 0, 0};

// Tabla de decodificacion en cuadratura: indice = (estado_anterior<<2)|estado_nuevo
const int8_t TABLA[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

void iniciarEnc(Enc &e) {
  pinMode(e.a, INPUT_PULLUP);
  pinMode(e.b, INPUT_PULLUP);
  e.last = (digitalRead(e.a) << 1) | digitalRead(e.b);
  e.acc = 0;
}

// Devuelve +1 o -1 por cada "click" (detent) del encoder, 0 si no hubo.
int8_t leerEnc(Enc &e) {
  uint8_t s = (digitalRead(e.a) << 1) | digitalRead(e.b);
  if (s == e.last) return 0;
  e.acc += TABLA[(e.last << 2) | s];
  e.last = s;
  if (e.acc >= 4) { e.acc = 0; return 1; }
  if (e.acc <= -4) { e.acc = 0; return -1; }
  return 0;
}

struct Boton { int pin; const char *nombre; bool prev; unsigned long t; };
Boton botones[4] = {
  {E1_SW, "Encoder 1 (pulsador)", true, 0},
  {E2_SW, "Encoder 2 (pulsador)", true, 0},
  {BTN_A, "Pulsador A", true, 0},
  {BTN_B, "Pulsador B", true, 0},
};

void escanearI2C() {
  Serial.println("--- Escaneo I2C ---");
  int n = 0;
  for (uint8_t dir = 1; dir < 127; dir++) {
    Wire.beginTransmission(dir);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  Dispositivo en 0x%02X\n", dir);
      n++;
    }
  }
  if (n == 0) Serial.println("  Nada encontrado: revisa SDA/SCL y alimentacion.");
  Serial.println("  (LCD suele ser 0x27 o 0x3F; OLED suele ser 0x3C o 0x3D)");
  Serial.println("-------------------");
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Wire.begin(PIN_SDA, PIN_SCL);
  escanearI2C();

  iniciarEnc(e1);
  iniciarEnc(e2);
  for (auto &b : botones) pinMode(b.pin, INPUT_PULLUP);
  Serial.println("Gira los encoders y pulsa los botones.");
  Serial.println("Horario deberia dar +1. Si da -1, intercambia CLK y DT en el cableado.");
}

int cuenta1 = 0, cuenta2 = 0;

void loop() {
  int8_t d1 = leerEnc(e1);
  if (d1) { cuenta1 += d1; Serial.printf("Encoder 1: %+d  (total %d)\n", d1, cuenta1); }
  int8_t d2 = leerEnc(e2);
  if (d2) { cuenta2 += d2; Serial.printf("Encoder 2: %+d  (total %d)\n", d2, cuenta2); }

  for (auto &b : botones) {
    bool v = digitalRead(b.pin);
    if (v != b.prev && millis() - b.t > 30) {
      b.t = millis();
      b.prev = v;
      Serial.printf("%s: %s\n", b.nombre, v ? "suelto" : "PULSADO");
    }
  }
}
