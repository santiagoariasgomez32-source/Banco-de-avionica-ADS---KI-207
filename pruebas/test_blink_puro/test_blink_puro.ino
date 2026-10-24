/*
  test_blink_puro.ino — Blink clásico, sin nada de serial ni lógica
  extra. Solo para identificar sin ninguna duda cuál LED físico
  responde al código (digitalWrite), separándolo de cualquier otra
  lucecita de la placa que no dependa del programa (como la de
  alimentación).
*/

#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);
  digitalWrite(LED_BUILTIN, LOW);
  delay(500);
}
