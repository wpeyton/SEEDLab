#include <Wire.h>

String data = "";
String test = "";

void setup() {
  Serial.begin(9600);

}



void loop() {
  if (Serial.available() > 0) {
    test = dataRead();
  }
  
}

String dataRead(){
      String piData = Serial.readStringUntil('\n');
      int len = piData.length();
      float newAngle = 0;
      float newDist = 0;
      Serial.print("You sent me: ");
      Serial.println(piData);
      if (piData[0] == 'a') {
        int distanceLoc = piData.indexOf('d');
        String angleStr = piData.substring(1, distanceLoc);
        String disStr = piData.substring(distanceLoc+1, len);
        newAngle = angleStr.toFloat();
        Serial.print("Angle:");
        Serial.println(newAngle);
        newDist = disStr.toFloat();
        Serial.print("Distance (m):");
        Serial.println(newDist);
      }    
  return piData;
}
