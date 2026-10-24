// Prueba de pulsadores: abre el Monitor Serie a 115200.
// Con el boton suelto debe decir 1 (HIGH). Al presionarlo debe decir 0 (LOW).
#define PIN_A 18
#define PIN_B 19
void setup() {
  Serial.begin(115200);
  pinMode(PIN_A, INPUT_PULLUP);
  pinMode(PIN_B, INPUT_PULLUP);
}
void loop() {
  Serial.print("A(GPIO18)=");
  Serial.print(digitalRead(PIN_A));
  Serial.print("   B(GPIO19)=");
  Serial.println(digitalRead(PIN_B));
  delay(200);
}
