#include <Wire.h>
#define SLAVE_ADDRESS 0x04
byte voltOut[2] = {0};
int analogValue  = 0;
// the setup routine runs once when you press reset:
void setup() {
  pinMode(13, OUTPUT);
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  Wire.begin(SLAVE_ADDRESS);
  Wire.onRequest(sendData);
}

void loop() {
  analogValue = analogRead(A0);
  Serial.println(analogValue);
}

void sendData() {
    for (int i = 0; i<2; i++) {
    voltOut[1-i] = (analogValue >> i*8);
  }
  Wire.write(voltOut, 2);
}
