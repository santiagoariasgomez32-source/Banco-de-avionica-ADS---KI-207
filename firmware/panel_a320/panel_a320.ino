/*
  panel_a320.ino - Panel estilo RMP del A320 (NAV) + instrumento didactico
  ESP32 (30 pines) + LCD 16x2 I2C + OLED 0.96" 128x64 I2C + 2 encoders + 2 pulsadores

  ---------------------------------------------------------------
  CABLEADO (todo en el lado de GPIO21/22; 3V3 y GND en ese mismo lado)
    LCD  : SDA -> D21 | SCL -> D22 | VCC -> 3V3 (o VIN si se ve tenue) | GND
    OLED : SDA -> D21 | SCL -> D22 | VCC -> 3V3 | GND        (mismo bus que el LCD)
    Encoder 1 (MHz / curso x10): CLK -> D19 | DT -> D18 | SW -> D5   | + -> 3V3 | GND
    Encoder 2 (kHz / curso x1) : CLK -> TX2 (GPIO17) | DT -> RX2 (GPIO16) | SW -> D4 | + -> 3V3 | GND
    Pulsador A : D23 <-> GND
    Pulsador B : D15 <-> GND
  ---------------------------------------------------------------
  LIBRERIAS (Herramientas > Administrar bibliotecas):
    "LiquidCrystal I2C" (Frank de Brabander)   y   "U8g2" (olikraus)
  Si tu OLED fuera SH1106, cambia la linea del constructor U8G2 (abajo).
  ---------------------------------------------------------------
  LCD 16x2 (copia las ventanas del RMP):
      113.900>108.600        ACTIVE > STBY      (en modo curso:  113.900>CRS 109)
      BOG    FROM R109       estacion, bandera, radial recibido

  CONTROLES
    Encoder 1 (giro)  : MHz de la frecuencia STBY   |  en modo CURSO: curso +-10 grados
    Encoder 2 (giro)  : kHz de STBY (pasos de 50)   |  en modo CURSO: curso +-1 grado
    Encoder 1 (pulsar): SWAP  (STBY <-> ACTIVE; la ACTIVE es la que sintoniza el SDR)
    Encoder 2 (pulsar): en modo CURSO = pone el curso igual al radial actual
                        en modo FRECUENCIA = siguiente pagina de la OLED
    Pulsador A corto  : alterna ventana STBY entre FRECUENCIA y CURSO (CRS)
    Pulsador A largo  : siguiente preset (BOG, SOA, ZIP, ILS13R, ILS13L) en STBY
    Pulsador B corto  : siguiente pagina de la OLED (HSI / Radiales / Senal / Espectro canal / Espectro banda)
    ("largo" = mantener ~0.8 s)

  OLED (5 paginas): 1) HSI con barra de desviacion y TO/FROM
                    2) Diagrama de radiales (radial recibido vs curso)
                    3) Calidad de senal + estacion + historial
                    4) Espectro del CANAL (+-15 kHz): portadora, bandas de 30 Hz/9960 Hz, C/N
                    5) Espectro de la BANDA (1.2 MHz): interferencias y saturacion
  ---------------------------------------------------------------
  PROTOCOLO SERIAL (115200)
    ESP32 -> PC : FREQ,NAV1,ACT,<kHz>   |   COURSE,<grados>
    PC -> ESP32 : DEV,<-1..1>,FLAG,<TO|FROM|OFF>,RAD,<grados>[,CONS,<0..1>][,ID,<texto>]
                  (CONS = consistencia del PLL; ID = identificador decodificado, opcional)
                  SPC,<cn_dB>,<128 caracteres>   espectro del canal  (cada caracter = chr(48 + nivel 0..63))
                  SPW,0,<128 caracteres>         espectro de la banda ancha
*/

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <U8g2lib.h>
#include <math.h>

// ===================== CONFIGURACION =====================
#define LCD_COLS 16
#define LCD_ROWS 2
#define PIN_SDA 21
#define PIN_SCL 22
#define PIN_E1_CLK 19
#define PIN_E1_DT 18
#define PIN_E1_SW 5
#define PIN_E2_CLK 17
#define PIN_E2_DT 16
#define PIN_E2_SW 4
#define PIN_BTN_A 23
#define PIN_BTN_B 15

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);
// U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);   // alternativa SH1106

struct Preset { long khz; const char* nombre; };
const Preset PRESETS[] = {
  {113900, "BOG"},     // VOR Bogota
  {108600, "SOA"},     // VOR Soacha
  {114700, "ZIP"},     // VOR Zipaquira
  {110700, "ILS13R"},  // Localizador pista 13R
  {111300, "ILS13L"},  // Localizador pista 13L
};
const int N_PRESETS = sizeof(PRESETS) / sizeof(PRESETS[0]);

const long FREQ_MIN_MHZ = 108;
const long FREQ_MAX_MHZ = 117;      // hasta 117.95
const unsigned long DEBOUNCE_MS = 30;
const unsigned long LARGO_MS = 800;
const unsigned long LINK_TIMEOUT_MS = 3000;
const unsigned long HEARTBEAT_MS = 5000;
const unsigned long REFRESCO_MS = 200;
const unsigned long ENGANCHE_MS = 8000;   // tiempo que el PLL tarda en enganchar tras sintonizar
const int N_PAGINAS = 5;
const int HIST_N = 100;

enum Evento { EV_NINGUNO, EV_CORTO, EV_LARGO };
enum Modo { MODO_FREQ, MODO_CURSO };

// Las estructuras van ANTES de la primera funcion: el Arduino IDE genera los
// prototipos de todas las funciones al inicio y necesita conocer estos tipos.
struct Boton {
  uint8_t pin;
  bool estado;
  bool ultimaLectura;
  unsigned long tUltimoCambio;
  unsigned long tInicioPresion;
  bool largoEmitido;
};
Boton btnA, btnB, btnE1, btnE2;

// Trafico ADS-B para el mini radar (llega del PC ya convertido a pixeles)
struct Aeronave { int8_t dx, dy; uint8_t lvl; };

// ===================== ENCODERS (por interrupcion) =====================
struct Encoder {
  uint8_t a, b;
  volatile uint8_t last;
  volatile int8_t acc;
  volatile int delta;
};
Encoder enc1 = {PIN_E1_CLK, PIN_E1_DT, 0, 0, 0};
Encoder enc2 = {PIN_E2_CLK, PIN_E2_DT, 0, 0, 0};
static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// Decodificacion en cuadratura: indice = (estado_anterior<<2)|estado_nuevo
DRAM_ATTR const int8_t TABLA[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

void IRAM_ATTR isrEnc(void* arg) {
  Encoder* e = (Encoder*)arg;
  uint8_t s = (digitalRead(e->a) << 1) | digitalRead(e->b);
  if (s == e->last) return;
  e->acc += TABLA[(e->last << 2) | s];
  e->last = s;
  if (e->acc >= 4) { e->acc = 0; e->delta++; }
  else if (e->acc <= -4) { e->acc = 0; e->delta--; }
}

void iniciarEncoder(Encoder &e) {
  pinMode(e.a, INPUT_PULLUP);
  pinMode(e.b, INPUT_PULLUP);
  e.last = (digitalRead(e.a) << 1) | digitalRead(e.b);
  e.acc = 0;
  e.delta = 0;
  attachInterruptArg(digitalPinToInterrupt(e.a), isrEnc, &e, CHANGE);
  attachInterruptArg(digitalPinToInterrupt(e.b), isrEnc, &e, CHANGE);
}

int tomarDelta(Encoder &e) {
  portENTER_CRITICAL(&mux);
  int d = e.delta;
  e.delta = 0;
  portEXIT_CRITICAL(&mux);
  return d;
}

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

int leerBoton(Boton &b) {
  bool lectura = (digitalRead(b.pin) == LOW);
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

// ===================== ESTADO =====================
LiquidCrystal_I2C* lcd = nullptr;

long freqActiva = 113900;    // kHz
long freqStandby = 108600;   // kHz
int idxPreset = 0;
int curso = 0;
Modo modo = MODO_FREQ;
int pagina = 0;
bool cursoPendiente = false;
unsigned long tUltCurso = 0;
unsigned long tCambioFreq = 0;

float devRx = 0.0;
float radialRx = 0.0;
float consRx = -1.0;             // -1 = el puente no lo manda
String flagRx = "OFF";
char identRx[8] = "";
unsigned long tIdent = 0;
unsigned long ultimoDato = 0;
bool hayDatos = false;

// espectros recibidos del PC (128 columnas, nivel 0..63)
const int SPEC_N = 128;
uint8_t specC[SPEC_N];
uint8_t specW[SPEC_N];
int cnC = 0;
unsigned long tSpecC = 0;
unsigned long tSpecW = 0;

uint8_t hist[HIST_N];
int histIdx = 0;
unsigned long tHist = 0;

// ----- modo ADS-B (Enc2 pulsacion larga alterna VOR <-> ADS-B) -----
bool modoAdsb = false;
unsigned long tCambioModo = 0;
const int ADS_MAX = 10;
Aeronave acs[ADS_MAX];
int adsN = 0;            // aviones vistos
int adsNRadar = 0;       // aviones dibujados
int adsRng = 40;         // alcance del radar (NM)
char adsCall[10] = "-";
int adsDist = 0;
int adsDz = 999;         // centenas de pies respecto a la antena; 999 = desconocido
unsigned long tAds = 0;

char lineaRx[260];
int nLineaRx = 0;
unsigned long tHeartbeat = 0;
unsigned long tRefresco = 0;

// ===================== UTILIDADES =====================
const char* nombreEstacion(long khz) {
  for (int i = 0; i < N_PRESETS; i++)
    if (PRESETS[i].khz == khz) return PRESETS[i].nombre;
  return "-----";
}

// Localizadores ILS: 108.10-111.95 MHz con decima impar. El resto es VOR.
bool esILS(long khz) {
  return khz >= 108000 && khz < 112000 && ((khz / 100) % 2 == 1);
}

float difAng(float a, float b) {   // a-b normalizado a [-180,180)
  return fmodf(a - b + 540.0f, 360.0f) - 180.0f;
}

bool esNumero(const char* s) {
  if (!s || !*s) return false;
  for (; *s; s++)
    if (!(isDigit(*s) || *s == '.' || *s == '-' || *s == '+')) return false;
  return true;
}

bool enlaceVivo() {
  return hayDatos && (millis() - ultimoDato) < LINK_TIMEOUT_MS;
}

bool senalValida() {
  return enlaceVivo() && flagRx != "OFF";
}

// ===================== SERIAL =====================
void enviarFreq() {
  Serial.print("FREQ,NAV1,ACT,");
  Serial.println(freqActiva);
}

void enviarCurso() {
  Serial.print("COURSE,");
  Serial.println(curso);
}

// Linea de espectro: SPC,<cn>,<128 caracteres>  /  SPW,0,<128 caracteres>
bool procesarEspectro(char* linea) {
  if (linea[0] != 'S' || linea[1] != 'P' || linea[3] != ',') return false;
  bool canal = (linea[2] == 'C');
  if (!canal && linea[2] != 'W') return false;
  char* coma = strchr(linea + 4, ',');
  if (!coma) return true;
  int cn = atoi(linea + 4);
  char* datos = coma + 1;
  if ((int)strlen(datos) < SPEC_N) return true;      // linea incompleta: se descarta
  uint8_t* destino = canal ? specC : specW;
  for (int i = 0; i < SPEC_N; i++) {
    int v = (int)datos[i] - 48;
    destino[i] = (uint8_t)constrain(v, 0, 63);
  }
  if (canal) { cnC = cn; tSpecC = millis(); }
  else tSpecW = millis();
  return true;
}

void enviarModo() {
  Serial.print("MODE,");
  Serial.println(modoAdsb ? "ADSB" : "VOR");
}

// Linea: ADS,<n>,<rango>,<callsign>,<dist>,<dz100>[,<dx>,<dy>,<nivel>]...
bool procesarAds(char* linea) {
  if (strncmp(linea, "ADS,", 4) != 0) return false;
  char* tok[6 + ADS_MAX * 3];
  int n = 0;
  for (char* p = strtok(linea, ","); p && n < 6 + ADS_MAX * 3; p = strtok(NULL, ",")) tok[n++] = p;
  if (n < 6) return true;
  adsN = atoi(tok[1]);
  adsRng = atoi(tok[2]);
  strncpy(adsCall, tok[3], sizeof(adsCall) - 1);
  adsCall[sizeof(adsCall) - 1] = 0;
  adsDist = atoi(tok[4]);
  adsDz = atoi(tok[5]);
  int k = 0;
  for (int i = 6; i + 2 < n && k < ADS_MAX; i += 3, k++) {
    acs[k].dx = (int8_t)constrain(atoi(tok[i]), -31, 31);
    acs[k].dy = (int8_t)constrain(atoi(tok[i + 1]), -31, 31);
    acs[k].lvl = (uint8_t)constrain(atoi(tok[i + 2]), 0, 3);
  }
  adsNRadar = k;
  tAds = millis();
  return true;
}

// Linea: DEV,<dev>,FLAG,<flag>,RAD,<rad>[,CONS,<c>][,ID,<texto>]
void procesarLinea(char* linea) {
  if (procesarEspectro(linea)) return;
  if (procesarAds(linea)) return;
  char* tok[14];
  int n = 0;
  for (char* p = strtok(linea, ","); p && n < 14; p = strtok(NULL, ",")) tok[n++] = p;
  if (n < 6 || strcmp(tok[0], "DEV") != 0) return;
  if (strcmp(tok[2], "FLAG") != 0 || strcmp(tok[4], "RAD") != 0) return;
  const char* f = tok[3];
  if (strcmp(f, "TO") && strcmp(f, "FROM") && strcmp(f, "OFF")) return;
  if (!esNumero(tok[1]) || !esNumero(tok[5])) return;

  devRx = atof(tok[1]);
  radialRx = atof(tok[5]);
  flagRx = f;
  for (int i = 6; i + 1 < n; i += 2) {
    if (!strcmp(tok[i], "CONS") && esNumero(tok[i + 1])) {
      consRx = atof(tok[i + 1]);
    } else if (!strcmp(tok[i], "ID")) {
      strncpy(identRx, tok[i + 1], sizeof(identRx) - 1);
      identRx[sizeof(identRx) - 1] = 0;
      tIdent = millis();
    }
  }
  ultimoDato = millis();
  hayDatos = true;
}

// ===================== LOGICA DE CONTROLES =====================
void hacerSwap() {
  long tmp = freqActiva;
  freqActiva = freqStandby;
  freqStandby = tmp;
  tCambioFreq = millis();
  enviarFreq();
}

void aplicarEncoders() {
  int d1 = tomarDelta(enc1);
  int d2 = tomarDelta(enc2);
  if (modo == MODO_FREQ) {
    if (d1) {
      long mhz = freqStandby / 1000;
      long k = freqStandby % 1000;
      long span = FREQ_MAX_MHZ - FREQ_MIN_MHZ + 1;
      mhz = FREQ_MIN_MHZ + (((mhz - FREQ_MIN_MHZ + d1) % span) + span) % span;
      freqStandby = mhz * 1000 + k;
    }
    if (d2) {
      long mhz = freqStandby / 1000;
      long paso = (freqStandby % 1000) / 50;           // 0..19
      paso = (((paso + d2) % 20) + 20) % 20;
      freqStandby = mhz * 1000 + paso * 50;
    }
  } else {
    if (d1) { curso = (((curso + d1 * 10) % 360) + 360) % 360; cursoPendiente = true; }
    if (d2) { curso = (((curso + d2) % 360) + 360) % 360; cursoPendiente = true; }
  }
}

void manejarBotones() {
  int evA = leerBoton(btnA);
  int evB = leerBoton(btnB);
  int ev1 = leerBoton(btnE1);
  int ev2 = leerBoton(btnE2);

  if (evA == EV_CORTO) modo = (modo == MODO_FREQ) ? MODO_CURSO : MODO_FREQ;
  if (evA == EV_LARGO) {
    idxPreset = (idxPreset + 1) % N_PRESETS;
    freqStandby = PRESETS[idxPreset].khz;
    modo = MODO_FREQ;
  }
  if (evB == EV_CORTO) pagina = (pagina + 1) % N_PAGINAS;

  if (ev2 == EV_LARGO) {          // Enc2 pulsacion larga: alterna VOR <-> ADS-B
    modoAdsb = !modoAdsb;
    tCambioModo = millis();
    tAds = 0;
    adsN = 0; adsNRadar = 0;
    enviarModo();
  }
  if (ev1 == EV_CORTO) hacerSwap();
  if (ev2 == EV_CORTO) {
    if (modo == MODO_CURSO) {
      if (senalValida()) {
        curso = ((int)lroundf(radialRx)) % 360;
        if (curso < 0) curso += 360;
        cursoPendiente = true;
      }
    } else {
      pagina = (pagina + 1) % N_PAGINAS;
    }
  }
}

// ===================== LCD =====================
uint8_t detectarDireccionLCD() {
  const uint8_t candidatas[] = {0x27, 0x3F};
  for (unsigned int i = 0; i < sizeof(candidatas); i++) {
    Wire.beginTransmission(candidatas[i]);
    if (Wire.endTransmission() == 0) return candidatas[i];
  }
  return 0x27;
}

void imprimirLinea(uint8_t fila, const char* texto) {
  char buf[LCD_COLS + 1];
  snprintf(buf, sizeof(buf), "%-*s", LCD_COLS, texto);
  lcd->setCursor(0, fila);
  lcd->print(buf);
}

bool adsFresco() {
  return tAds != 0 && tAds >= tCambioModo && (millis() - tAds) < 3000;
}

void dibujarLCD() {
  if (modoAdsb) {
    char a0[24], a1[24];
    if (adsFresco()) {
      snprintf(a0, sizeof(a0), "ADS-B   AC:%d", adsN);
      if (adsN > 0) snprintf(a1, sizeof(a1), "%s %dNM", adsCall, adsDist);
      else snprintf(a1, sizeof(a1), "sin trafico");
    } else if (millis() - tCambioModo < 9000) {
      snprintf(a0, sizeof(a0), "ADS-B 1090 MHz");
      snprintf(a1, sizeof(a1), "iniciando...");
    } else {
      snprintf(a0, sizeof(a0), "ADS-B 1090 MHz");
      snprintf(a1, sizeof(a1), "sin datos (PC?)");
    }
    imprimirLinea(0, a0);
    imprimirLinea(1, a1);
    return;
  }
  bool vivo = enlaceVivo();
  bool valido = senalValida();
  char sby[12], l0[24], l1[24], radTxt[6];

  if (modo == MODO_FREQ) snprintf(sby, sizeof(sby), "%7.3f", freqStandby / 1000.0);
  else snprintf(sby, sizeof(sby), "CRS %03d", curso);
  snprintf(l0, sizeof(l0), "%7.3f>%s", freqActiva / 1000.0, sby);
  imprimirLinea(0, l0);

  if (!vivo) {
    imprimirLinea(1, "SIN PC (puente?)");
    return;
  }
  if (valido) snprintf(radTxt, sizeof(radTxt), "%03d", ((int)lroundf(radialRx)) % 360);
  else strcpy(radTxt, "---");
  snprintf(l1, sizeof(l1), "%-6s %-4s R%s", nombreEstacion(freqActiva), flagRx.c_str(), radTxt);
  imprimirLinea(1, l1);
}

// ===================== OLED =====================
// Linea desde radio r0 hasta r1 en direccion angDeg (0 = arriba, sentido horario)
void lineaPolar(int cx, int cy, float angDeg, float r0, float r1) {
  float t = angDeg * DEG_TO_RAD;
  float s = sinf(t), c = cosf(t);
  u8g2.drawLine(lroundf(cx + s * r0), lroundf(cy - c * r0), lroundf(cx + s * r1), lroundf(cy - c * r1));
}

void rosa(int cx, int cy, int paso10) {
  u8g2.drawCircle(cx, cy, 30);
  for (int a = 0; a < 360; a += (paso10 ? 10 : 30)) {
    float r1 = (a % 90 == 0) ? 25 : ((a % 30 == 0) ? 27 : 28);
    lineaPolar(cx, cy, a, 30, r1);
  }
  u8g2.setFont(u8g2_font_5x7_tr);
  u8g2.drawStr(cx - 2, cy - 18, "N");
}

void paginaHSI() {
  const int cx = 31, cy = 32;
  bool vivo = enlaceVivo();
  bool valido = senalValida();
  char buf[16];

  rosa(cx, cy, 0);

  float th = curso * DEG_TO_RAD;
  float dx = sinf(th), dy = -cosf(th);       // direccion del curso en pantalla
  float px = cosf(th), py = sinf(th);        // perpendicular (derecha del curso)

  // flechas fijas del curso (cabeza y cola)
  u8g2.drawLine(lroundf(cx + dx * 17), lroundf(cy + dy * 17), lroundf(cx + dx * 22), lroundf(cy + dy * 22));
  u8g2.drawTriangle(lroundf(cx + dx * 27), lroundf(cy + dy * 27),
                    lroundf(cx + dx * 21 + px * 3), lroundf(cy + dy * 21 + py * 3),
                    lroundf(cx + dx * 21 - px * 3), lroundf(cy + dy * 21 - py * 3));
  u8g2.drawLine(lroundf(cx - dx * 17), lroundf(cy - dy * 17), lroundf(cx - dx * 26), lroundf(cy - dy * 26));

  if (valido) {
    // puntos de escala (2 a cada lado)
    for (int k = -2; k <= 2; k++) {
      if (k == 0) continue;
      u8g2.drawDisc(lroundf(cx + px * k * 6.5), lroundf(cy + py * k * 6.5), 1);
    }
    // barra de desviacion
    float d = constrain(devRx, -1.0f, 1.0f) * 13.0f;
    float bx = cx + px * d, by = cy + py * d;
    u8g2.drawLine(lroundf(bx - dx * 15), lroundf(by - dy * 15), lroundf(bx + dx * 15), lroundf(by + dy * 15));
    u8g2.drawLine(lroundf(bx - dx * 15 + px), lroundf(by - dy * 15 + py), lroundf(bx + dx * 15 + px), lroundf(by + dy * 15 + py));
    // triangulo TO/FROM sobre la barra
    float s = (flagRx == "TO") ? 1.0f : -1.0f;
    u8g2.drawTriangle(lroundf(bx + dx * s * 12), lroundf(by + dy * s * 12),
                      lroundf(bx + dx * s * 5 + px * 3), lroundf(by + dy * s * 5 + py * 3),
                      lroundf(bx + dx * s * 5 - px * 3), lroundf(by + dy * s * 5 - py * 3));
    // punto en el borde: tu radial respecto a la estacion
    float tr = radialRx * DEG_TO_RAD;
    u8g2.drawDisc(lroundf(cx + sinf(tr) * 30), lroundf(cy - cosf(tr) * 30), 3);
  } else {
    u8g2.setDrawColor(0);
    u8g2.drawBox(cx - 14, cy - 7, 29, 14);
    u8g2.setDrawColor(1);
    u8g2.drawFrame(cx - 14, cy - 7, 29, 14);
    u8g2.setFont(u8g2_font_6x12_tr);
    u8g2.drawStr(cx - 9, cy + 4, vivo ? "OFF" : "N/PC");
  }

  // columna de texto
  u8g2.setFont(u8g2_font_helvB10_tr);
  u8g2.drawStr(66, 12, nombreEstacion(freqActiva));
  u8g2.setFont(u8g2_font_6x12_tr);
  snprintf(buf, sizeof(buf), "CRS %03d", curso);
  u8g2.drawStr(66, 28, buf);
  if (valido) snprintf(buf, sizeof(buf), "RAD %03d", ((int)lroundf(radialRx)) % 360);
  else snprintf(buf, sizeof(buf), "RAD ---");
  u8g2.drawStr(66, 41, buf);
  u8g2.setFont(u8g2_font_helvB10_tr);
  u8g2.drawStr(66, 58, vivo ? flagRx.c_str() : "NO PC");
}

void paginaRadiales() {
  const int cx = 31, cy = 32;
  bool valido = senalValida();
  char buf[16];

  rosa(cx, cy, 1);
  u8g2.drawCircle(cx, cy, 3);   // la estacion

  // curso seleccionado: linea punteada
  for (int r = 6; r < 27; r += 6) lineaPolar(cx, cy, curso, r, r + 3);

  if (valido) {
    lineaPolar(cx, cy, radialRx, 4, 27);
    lineaPolar(cx + 1, cy, radialRx, 4, 27);
    float tr = radialRx * DEG_TO_RAD;
    u8g2.drawDisc(lroundf(cx + sinf(tr) * 27), lroundf(cy - cosf(tr) * 27), 3);
  }

  u8g2.setFont(u8g2_font_6x10_tr);
  u8g2.drawStr(66, 10, "RADIAL");
  u8g2.setFont(u8g2_font_helvB14_tn);
  if (valido) snprintf(buf, sizeof(buf), "%03d", ((int)lroundf(radialRx)) % 360);
  else snprintf(buf, sizeof(buf), "---");
  u8g2.drawStr(66, 29, buf);
  u8g2.setFont(u8g2_font_6x12_tr);
  snprintf(buf, sizeof(buf), "CURSO %03d", curso);
  u8g2.drawStr(66, 44, buf);
  if (valido) snprintf(buf, sizeof(buf), "DIF %+d", (int)lroundf(difAng(radialRx, curso)));
  else snprintf(buf, sizeof(buf), "DIF ---");
  u8g2.drawStr(66, 58, buf);
}

const char* estadoSenal() {
  if (!enlaceVivo()) return "SIN PC";
  if (senalValida()) return "SENAL OK";
  if (millis() - tCambioFreq < ENGANCHE_MS) return "ENGANCHANDO...";
  return "SIN SENAL";
}

void paginaSenal() {
  char buf[24];
  bool vivo = enlaceVivo();

  // cabecera: estacion + frecuencia + tipo
  u8g2.setFont(u8g2_font_helvB10_tr);
  u8g2.drawStr(0, 11, nombreEstacion(freqActiva));
  u8g2.setFont(u8g2_font_6x12_tr);
  snprintf(buf, sizeof(buf), "%.3f", freqActiva / 1000.0);
  u8g2.drawStr(52, 11, buf);
  u8g2.drawStr(100, 11, esILS(freqActiva) ? "ILS" : "VOR");

  u8g2.drawStr(0, 24, estadoSenal());

  // barra de consistencia con marca en 60 %
  int pct = (vivo && consRx >= 0) ? (int)constrain(consRx * 100.0f, 0.0f, 100.0f) : 0;
  u8g2.drawFrame(0, 28, 100, 9);
  int ancho = (pct > 98) ? 98 : pct;
  if (ancho > 0) u8g2.drawBox(1, 29, ancho, 7);
  u8g2.drawVLine(60, 26, 13);
  if (vivo && consRx >= 0) snprintf(buf, sizeof(buf), "%d%%", pct);
  else snprintf(buf, sizeof(buf), "--");
  u8g2.drawStr(104, 36, buf);

  // historial de consistencia (ultimos ~100 s)
  u8g2.drawHLine(0, 63, HIST_N);
  for (int i = 0; i < HIST_N; i++) {
    int v = hist[(histIdx + i) % HIST_N];
    int h = (v * 22) / 100;
    if (h > 0) u8g2.drawVLine(i, 63 - h, h);
  }
  u8g2.drawVLine(HIST_N + 1, 41, 23);

  // ident (solo si el puente lo manda)
  u8g2.setFont(u8g2_font_5x7_tr);
  u8g2.drawStr(104, 48, "ID");
  u8g2.setFont(u8g2_font_6x12_tr);
  if (identRx[0] && (millis() - tIdent) < 30000) u8g2.drawStr(104, 61, identRx);
  else u8g2.drawStr(104, 61, "---");
}

// Espectro recibido del PC. ancha=false: canal de +-15.6 kHz (portadora al centro, bandas de
// 9960 Hz marcadas); ancha=true: banda completa del SDR (1.2 MHz, portadora al centro).
void paginaEspectro(bool ancha) {
  const uint8_t* sp = ancha ? specW : specC;
  unsigned long t = ancha ? tSpecW : tSpecC;
  bool vivo = (t != 0) && ((millis() - t) < 2500);
  char buf[24];

  u8g2.setFont(u8g2_font_5x7_tr);
  u8g2.drawStr(0, 7, ancha ? "BANDA 1.2MHz" : "CANAL +-15kHz");
  if (vivo && !ancha) {
    snprintf(buf, sizeof(buf), "C/N %ddB", cnC);
    u8g2.drawStr(88, 7, buf);
  } else if (!ancha) {
    u8g2.drawStr(88, 7, "C/N --");
  }

  const int Y0 = 63;       // linea base
  const int ALTO = 52;     // alto maximo de la grafica
  u8g2.drawHLine(0, Y0, SPEC_N);
  if (!vivo) {
    u8g2.setFont(u8g2_font_6x12_tr);
    u8g2.drawStr(12, 34, "SIN ESPECTRO");
    u8g2.setFont(u8g2_font_5x7_tr);
    u8g2.drawStr(12, 46, "revisa el decodificador");
    return;
  }
  for (int x = 0; x < SPEC_N; x++) {
    int h = (sp[x] * ALTO) / 63;
    if (h > 0) u8g2.drawVLine(x, Y0 - h, h);
  }
  // marcas: centro (portadora)
  u8g2.drawVLine(64, 9, 3);
  if (!ancha) {
    u8g2.drawVLine(23, 9, 3);      // banda lateral -9960 Hz
    u8g2.drawVLine(105, 9, 3);     // banda lateral +9960 Hz
    u8g2.drawPixel(60, 9);         // tono del identificador -1020 Hz
    u8g2.drawPixel(68, 9);         // tono del identificador +1020 Hz
  }
}

void romboRadar(int x, int y, bool lleno) {
  if (lleno) {
    u8g2.drawTriangle(x, y - 3, x + 3, y, x - 3, y);
    u8g2.drawTriangle(x, y + 3, x + 3, y, x - 3, y);
  } else {
    u8g2.drawLine(x, y - 3, x + 3, y);
    u8g2.drawLine(x + 3, y, x, y + 3);
    u8g2.drawLine(x, y + 3, x - 3, y);
    u8g2.drawLine(x - 3, y, x, y - 3);
  }
}

// Mini radar tipo TCAS: antena al centro, norte arriba. Niveles: 0 otro, 1 proximo, 2 TA, 3 RA.
void paginaAdsb() {
  const int cx = 32, cy = 32;
  char buf[24];
  u8g2.setFont(u8g2_font_6x12_tr);
  if (!adsFresco()) {
    u8g2.drawStr(4, 14, "ADS-B 1090 MHz");
    if (millis() - tCambioModo < 9000) {
      u8g2.drawStr(4, 34, "CAMBIANDO DE MODO");
      u8g2.drawStr(4, 48, "iniciando SDR...");
    } else {
      u8g2.drawStr(4, 34, "SIN DATOS ADS-B");
      u8g2.setFont(u8g2_font_5x7_tr);
      u8g2.drawStr(4, 48, "revisa el gestor en el PC");
    }
    u8g2.setFont(u8g2_font_5x7_tr);
    u8g2.drawStr(4, 62, "Enc2 largo = volver a VOR");
    return;
  }
  u8g2.drawCircle(cx, cy, 30);
  u8g2.drawCircle(cx, cy, 15);
  u8g2.setFont(u8g2_font_5x7_tr);
  u8g2.drawStr(cx - 2, 9, "N");
  u8g2.drawPixel(cx, cy);                    // tu antena
  u8g2.drawHLine(cx - 2, cy, 5);
  u8g2.drawVLine(cx, cy - 2, 5);
  for (int i = 0; i < adsNRadar; i++) {
    int x = cx + acs[i].dx, y = cy + acs[i].dy;
    uint8_t l = acs[i].lvl;
    if (l == 0) romboRadar(x, y, false);
    else if (l == 1) romboRadar(x, y, true);
    else if (l == 2) u8g2.drawDisc(x, y, 3);
    else u8g2.drawBox(x - 3, y - 3, 7, 7);
  }
  u8g2.setFont(u8g2_font_6x12_tr);
  u8g2.drawStr(68, 10, "ADS-B");
  snprintf(buf, sizeof(buf), "AC %d", adsN);
  u8g2.drawStr(68, 23, buf);
  u8g2.setFont(u8g2_font_5x7_tr);
  snprintf(buf, sizeof(buf), "RNG %dNM", adsRng);
  u8g2.drawStr(68, 33, buf);
  if (adsN > 0) {
    u8g2.setFont(u8g2_font_6x12_tr);
    u8g2.drawStr(68, 46, adsCall);
    if (adsDz == 999) snprintf(buf, sizeof(buf), "%dNM  --", adsDist);
    else snprintf(buf, sizeof(buf), "%dNM %c%02d", adsDist, adsDz >= 0 ? '+' : '-', abs(adsDz));
    u8g2.drawStr(68, 59, buf);
  } else {
    u8g2.setFont(u8g2_font_5x7_tr);
    u8g2.drawStr(68, 50, "sin trafico");
  }
}

void dibujarOLED() {
  u8g2.clearBuffer();
  if (modoAdsb) {
    paginaAdsb();
    u8g2.sendBuffer();
    return;
  }
  if (pagina == 0) paginaHSI();
  else if (pagina == 1) paginaRadiales();
  else if (pagina == 2) paginaSenal();
  else if (pagina == 3) paginaEspectro(false);
  else paginaEspectro(true);
  u8g2.sendBuffer();
}

// ===================== SETUP / LOOP =====================
void setup() {
  Serial.setRxBufferSize(1024);   // las lineas de espectro son largas (~135 bytes)
  Serial.begin(115200);
  iniciarBoton(btnA, PIN_BTN_A);
  iniciarBoton(btnB, PIN_BTN_B);
  iniciarBoton(btnE1, PIN_E1_SW);
  iniciarBoton(btnE2, PIN_E2_SW);
  iniciarEncoder(enc1);
  iniciarEncoder(enc2);
  memset(hist, 0, sizeof(hist));

  Wire.begin(PIN_SDA, PIN_SCL);
  u8g2.begin();
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_helvB10_tr);
  u8g2.drawStr(8, 28, "PANEL NAV/COM");
  u8g2.setFont(u8g2_font_6x12_tr);
  u8g2.drawStr(8, 46, "VOR / ILS");
  u8g2.sendBuffer();

  uint8_t dir = detectarDireccionLCD();
  lcd = new LiquidCrystal_I2C(dir, LCD_COLS, LCD_ROWS);
  lcd->init();
  lcd->backlight();
  lcd->clear();
  imprimirLinea(0, "Panel NAV/COM");
  char txt[LCD_COLS + 1];
  snprintf(txt, sizeof(txt), "LCD en 0x%02X", dir);
  imprimirLinea(1, txt);
  delay(1200);
  lcd->clear();

  tHeartbeat = millis() - HEARTBEAT_MS;   // anuncia el estado de inmediato
  tCambioFreq = millis();
}

void loop() {
  manejarBotones();
  aplicarEncoders();

  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') {
      lineaRx[nLineaRx] = 0;
      if (nLineaRx > 0) procesarLinea(lineaRx);
      nLineaRx = 0;
    } else if (c != '\r' && nLineaRx < (int)sizeof(lineaRx) - 1) {
      lineaRx[nLineaRx++] = c;
    }
  }

  unsigned long ahora = millis();

  // curso: se manda con un pequeno retraso para no inundar el puerto al girar rapido
  if (cursoPendiente && (ahora - tUltCurso) >= 80) {
    tUltCurso = ahora;
    cursoPendiente = false;
    enviarCurso();
  }

  if (ahora - tHeartbeat >= HEARTBEAT_MS) {
    tHeartbeat = ahora;
    enviarFreq();
    enviarCurso();
    enviarModo();
  }

  if (ahora - tHist >= 1000) {
    tHist = ahora;
    uint8_t v = (enlaceVivo() && consRx >= 0) ? (uint8_t)constrain(consRx * 100.0f, 0.0f, 100.0f) : 0;
    hist[histIdx] = v;
    histIdx = (histIdx + 1) % HIST_N;
  }

  if (ahora - tRefresco >= REFRESCO_MS) {
    tRefresco = ahora;
    dibujarLCD();
    dibujarOLED();
  }
}
