#include <Encoder.h>

#define TARE 6
#define COUNTS_PER_REVOLUTION 80

Encoder myEnc(2, 4);

void setup() {
  pinMode(TARE, INPUT);
  Serial.begin(9600);
  Serial.println("Basic Encoder Test:");
}

long oldPosition  = -999;

void loop() {
  long newPosition = myEnc.read();
  double angle = 2 * PI * newPosition / COUNTS_PER_REVOLUTION;
  if (newPosition != oldPosition) {
    oldPosition = newPosition;
    Serial.println(angle);
  }

  if(!digitalRead(TARE)) {
    myEnc.write(0);
    delay(50);
  }
}
