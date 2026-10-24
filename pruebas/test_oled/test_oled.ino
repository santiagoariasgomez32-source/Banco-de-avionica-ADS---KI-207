// test_oled.ino - Prueba de la OLED 0.96" 128x64 I2C (ESP32)
// Libreria necesaria: "U8g2" (Arduino IDE -> Herramientas -> Administrar bibliotecas -> buscar U8g2 -> instalar)
// Conexion: SDA=GPIO21  SCL=GPIO22  VCC=3V3  GND=GND
//
// Si la pantalla sigue apagada, prueba la linea alternativa SH1106 de abajo
// (algunas pantallas de 1.3" usan ese chip; las de 0.96" casi siempre son SSD1306).

#include <Wire.h>
#include <U8g2lib.h>

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);
// U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);  // alternativa SH1106

int x = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  // Si el escaneo dio 0x3D en vez de 0x3C, descomenta esta linea:
  // u8g2.setI2CAddress(0x3D * 2);
  u8g2.begin();
  Serial.println("OLED iniciada");
}

void loop() {
  u8g2.clearBuffer();
  u8g2.drawFrame(0, 0, 128, 64);
  u8g2.setFont(u8g2_font_ncenB10_tr);
  u8g2.drawStr(10, 22, "OLED OK");
  u8g2.setFont(u8g2_font_6x10_tr);
  u8g2.drawStr(10, 38, "Panel VOR/ILS");
  u8g2.drawBox(x, 50, 10, 8);   // cuadrito que se mueve
  u8g2.sendBuffer();
  x = (x + 4) % 110;
  delay(80);
}
