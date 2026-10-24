/*
  panel_prototipo.ino — Mini panel de pruebas
  ESP32 + LCD I2C + 2 pulsadores

  Idea: simula el RMP / panel de control con lo que ya tienes en la
  protoboard. Los pulsadores eligen frecuencia y curso y se los mandan
  a la PC; el LCD muestra lo que la PC devuelve desde la antena
  (radial, desviación del CDI, bandera TO/FROM).

  ---------------------------------------------------------------
  CABLEADO (todo se alimenta por el USB del ESP32, sin fuente externa)
    LCD I2C : GND -> GND | VCC -> 3V3 (o VIN si se ve tenue) | SDA -> GPIO21 | SCL -> GPIO22
    Pulsador A : un lado a GPIO18, el otro lado a GND
    Pulsador B : un lado a GPIO19, el otro lado a GND
    (Usan la resistencia pull-up interna: NO necesitas resistencias externas.)
  ---------------------------------------------------------------
  LIBRERIA: instala "LiquidCrystal I2C" (autor: Frank de Brabander)
            desde Herramientas > Administrar bibliotecas.
  PANTALLA: por defecto 16x2. Si tu LCD es de 20x4 cambia LCD_COLS a 20
            y LCD_ROWS a 4 (ahí se ve más información).
  ---------------------------------------------------------------
  CONTROLES
    Modo FRECUENCIA (el de arranque):
      A corto  : siguiente frecuencia en STANDBY
      A largo  : frecuencia anterior en STANDBY
      B corto  : SWAP (standby <-> activa) y se manda a la PC
      B largo  : pasar al modo CURSO
    Modo CURSO:
      A corto  : curso +10 grados
      A largo  : curso +1 grado
      B corto  : curso -10 grados
      B largo  : volver al modo FRECUENCIA
    ("largo" = mantener presionado ~0.8 s)
  ---------------------------------------------------------------
  PROTOCOLO SERIAL (115200)
    ESP32 -> PC : FREQ,NAV1,ACT,<kHz>   |   COURSE,<grados>
    PC -> ESP32 : DEV,<-1..1>,FLAG,<TO|FROM|OFF>,RAD,<grados>
  Se re-anuncia el estado cada 5 s por si la PC arrancó después.
*/

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ===================== CONFIGURACION =====================
#define LCD_COLS 16
#define LCD_ROWS 2
#define PIN_SDA 21
#define PIN_SCL 22
#define PIN_BTN_A 18
#define PIN_BTN_B 19

struct Preset {
  long khz;
  const char* nombre;
};

// Frecuencias que se pueden elegir con los pulsadores (en kHz).
// Cuando tengas encoders, esto se reemplaza por sintonía libre.
const Preset PRESETS[] = {
  {113900, "BOG"},     // VOR Bogota
  {108600, "SOA"},     // VOR Soacha
  {114700, "ZIP"},     // VOR Zipaquira
  {110700, "ILS13R"},  // Localizador pista 13R (IADO)
  {111300, "ILS13L"},  // Localizador pista 13L (IEDR)
};
const int N_PRESETS = sizeof(PRESETS) / sizeof(PRESETS[0]);

const unsigned long DEBOUNCE_MS = 30;
const unsigned long LARGO_MS = 800;
const unsigned long LINK_TIMEOUT_MS = 3000;
const unsigned long HEARTBEAT_MS = 5000;
const unsigned long REFRESCO_MS = 200;

enum Evento { EV_NINGUNO, EV_CORTO, EV_LARGO };
enum Modo { MODO_FREQ, MODO_CURSO };

struct Boton {
  uint8_t pin;
  bool estado;            // true = presionado (ya filtrado)
  bool ultimaLectura;
  unsigned long tUltimoCambio;
  unsigned long tInicioPresion;
  bool largoEmitido;
};

Boton btnA, btnB;
LiquidCrystal_I2C* lcd = nullptr;

int idxActivo = 0;
int idxStandby = 1;
int curso = 0;
Modo modo = MODO_FREQ;

float devRx = 0.0;
float radialRx = 0.0;
String flagRx = "OFF";
unsigned long ultimoDato = 0;
bool hayDatos = false;

String bufferRx = "";
unsigned long tHeartbeat = 0;
unsigned long tRefresco = 0;

// ===================== BOTONES =====================
void iniciarBoton(Boton &b, uint8_t pin) {
  b.pin = pin;
  b.estado = false;
  b.ultimaLectura = false;
  b.tUltimoCambio = 0;
  b.tInicioPresion = 0;
  b.largoEmitido = false;
  pinMode(pin, INPUT_PULLUP);
}

// Devuelve EV_CORTO al soltar (si no fue largo), EV_LARGO al cumplirse
// el tiempo mantenido, o EV_NINGUNO.
int leerBoton(Boton &b) {
  bool lectura = (digitalRead(b.pin) == LOW);  // LOW = presionado (pull-up)
  unsigned long ahora = millis();
  int evento = EV_NINGUNO;

  if (lectura != b.ultimaLectura) {
    b.tUltimoCambio = ahora;
    b.ultimaLectura = lectura;
  }

  if ((ahora - b.tUltimoCambio) > DEBOUNCE_MS && lectura != b.estado) {
    b.estado = lectura;
    if (b.estado) {
      b.tInicioPresion = ahora;
      b.largoEmitido = false;
    } else if (!b.largoEmitido) {
      evento = EV_CORTO;
    }
  }

  if (b.estado && !b.largoEmitido && (ahora - b.tInicioPresion) > LARGO_MS) {
    b.largoEmitido = true;
    evento = EV_LARGO;
  }
  return evento;
}

// ===================== SERIAL =====================
void enviarFreq() {
  Serial.print("FREQ,NAV1,ACT,");
  Serial.println(PRESETS[idxActivo].khz);
}

void enviarCurso() {
  Serial.print("COURSE,");
  Serial.println(curso);
}

bool esNumero(const String &s) {
  if (s.length() == 0) return false;
  for (unsigned int i = 0; i < s.length(); i++) {
    char c = s.charAt(i);
    if (!(isDigit(c) || c == '.' || c == '-' || c == '+')) return false;
  }
  return true;
}

// Linea esperada: DEV,<dev>,FLAG,<flag>,RAD,<radial>
void procesarLinea(String linea) {
  linea.trim();
  if (!linea.startsWith("DEV,")) return;

  int c1 = linea.indexOf(',');
  int c2 = linea.indexOf(',', c1 + 1);
  int c3 = linea.indexOf(',', c2 + 1);
  int c4 = linea.indexOf(',', c3 + 1);
  int c5 = linea.indexOf(',', c4 + 1);
  if (c1 < 0 || c2 < 0 || c3 < 0 || c4 < 0 || c5 < 0) return;

  String sDev = linea.substring(c1 + 1, c2);
  String sFlag = linea.substring(c3 + 1, c4);
  String sRad = linea.substring(c5 + 1);

  // Si la linea llego corrupta se descarta completa.
  if (sFlag != "TO" && sFlag != "FROM" && sFlag != "OFF") return;
  if (!esNumero(sDev) || !esNumero(sRad)) return;

  devRx = sDev.toFloat();
  radialRx = sRad.toFloat();
  flagRx = sFlag;
  ultimoDato = millis();
  hayDatos = true;
}

// ===================== LOGICA DE CONTROLES =====================
void manejarBotones(int evA, int evB) {
  if (modo == MODO_FREQ) {
    if (evA == EV_CORTO) idxStandby = (idxStandby + 1) % N_PRESETS;
    if (evA == EV_LARGO) idxStandby = (idxStandby + N_PRESETS - 1) % N_PRESETS;
    if (evB == EV_CORTO) {
      int tmp = idxActivo;       // SWAP real: activa <-> standby
      idxActivo = idxStandby;
      idxStandby = tmp;
      enviarFreq();
    }
    if (evB == EV_LARGO) modo = MODO_CURSO;
  } else {
    if (evA == EV_CORTO) { curso = (curso + 10) % 360;  enviarCurso(); }
    if (evA == EV_LARGO) { curso = (curso + 1) % 360;   enviarCurso(); }
    if (evB == EV_CORTO) { curso = (curso + 350) % 360; enviarCurso(); }
    if (evB == EV_LARGO) modo = MODO_FREQ;
  }
}

// ===================== LCD =====================
uint8_t detectarDireccionLCD() {
  const uint8_t candidatas[] = {0x27, 0x3F};
  for (unsigned int i = 0; i < sizeof(candidatas); i++) {
    Wire.beginTransmission(candidatas[i]);
    if (Wire.endTransmission() == 0) return candidatas[i];
  }
  return 0x27;  // por defecto
}

void imprimirLinea(uint8_t fila, const char* texto) {
  char buf[LCD_COLS + 1];
  snprintf(buf, sizeof(buf), "%-*s", LCD_COLS, texto);
  lcd->setCursor(0, fila);
  lcd->print(buf);
}

void dibujar() {
  bool vivo = hayDatos && (millis() - ultimoDato) < LINK_TIMEOUT_MS;

  char radTxt[8], devTxt[8], flagTxt[8];
  if (!vivo) {
    strcpy(radTxt, "---");
    strcpy(devTxt, "NO-PC");
    strcpy(flagTxt, "----");
  } else if (flagRx == "OFF") {
    strcpy(radTxt, "---");
    strcpy(devTxt, " OFF ");
    strcpy(flagTxt, "OFF");
  } else {
    snprintf(radTxt, sizeof(radTxt), "%03d", ((int)round(radialRx)) % 360);
    snprintf(devTxt, sizeof(devTxt), "%+5.2f", devRx);
    snprintf(flagTxt, sizeof(flagTxt), "%s", flagRx.c_str());
  }

  float fAct = PRESETS[idxActivo].khz / 1000.0;
  float fSby = PRESETS[idxStandby].khz / 1000.0;
  char l0[24], l1[24];

#if LCD_ROWS >= 4
  char l2[24], l3[24];
  snprintf(l0, sizeof(l0), "ACT %7.3f %-6s", fAct, PRESETS[idxActivo].nombre);
  snprintf(l1, sizeof(l1), "STB %7.3f %-6s", fSby, PRESETS[idxStandby].nombre);
  snprintf(l2, sizeof(l2), "RAD %s CRS %03d %-4s", radTxt, curso, flagTxt);
  snprintf(l3, sizeof(l3), "DEV %s  MODO:%s", devTxt, modo == MODO_FREQ ? "FRQ" : "CRS");
  imprimirLinea(0, l0);
  imprimirLinea(1, l1);
  imprimirLinea(2, l2);
  imprimirLinea(3, l3);
#else
  // 16x2:  fila 0 = activa + radial + bandera (F/T/O)
  //        fila 1 = standby (modo FRECUENCIA) o curso (modo CURSO) + desviacion
  snprintf(l0, sizeof(l0), "A%6.2f R%s %c", fAct, radTxt, flagTxt[0]);
  if (modo == MODO_FREQ) {
    snprintf(l1, sizeof(l1), "S%6.2f D%s", fSby, devTxt);
  } else {
    snprintf(l1, sizeof(l1), "CRS%03d  D%s", curso, devTxt);
  }
  imprimirLinea(0, l0);
  imprimirLinea(1, l1);
#endif
}

// ===================== SETUP / LOOP =====================
void setup() {
  Serial.begin(115200);
  iniciarBoton(btnA, PIN_BTN_A);
  iniciarBoton(btnB, PIN_BTN_B);

  Wire.begin(PIN_SDA, PIN_SCL);
  uint8_t dir = detectarDireccionLCD();
  lcd = new LiquidCrystal_I2C(dir, LCD_COLS, LCD_ROWS);
  lcd->init();
  lcd->backlight();
  lcd->clear();

  char txt[LCD_COLS + 1];
  imprimirLinea(0, "Panel VOR/COM");
  snprintf(txt, sizeof(txt), "LCD en 0x%02X", dir);
  imprimirLinea(1, txt);
  delay(1200);
  lcd->clear();

  tHeartbeat = millis() - HEARTBEAT_MS;  // anuncia el estado de inmediato
}

void loop() {
  int evA = leerBoton(btnA);
  int evB = leerBoton(btnB);
  manejarBotones(evA, evB);

  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') {
      procesarLinea(bufferRx);
      bufferRx = "";
    } else if (c != '\r' && bufferRx.length() < 80) {
      bufferRx += c;
    }
  }

  unsigned long ahora = millis();
  if (ahora - tHeartbeat >= HEARTBEAT_MS) {
    tHeartbeat = ahora;
    enviarFreq();
    enviarCurso();
  }
  if (ahora - tRefresco >= REFRESCO_MS) {
    tRefresco = ahora;
    dibujar();
  }
}
