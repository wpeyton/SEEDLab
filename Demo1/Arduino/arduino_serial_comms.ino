#include <Wire.h>
String test = "abcd";

void setup() {
  Serial.begin(115200);
}



void loop() {
  Serial.println("Hello from Arduino!");
  delay(1);
  test = Serial.read();
  Serial.println(test);
}
