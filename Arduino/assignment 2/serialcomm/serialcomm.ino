void setup() {
  Serial.begin(9600);
}

void loop() {
  int incomingByte;
  
  if(Serial.available() > 0) {
    // read the incoming byte:
    incomingByte = Serial.read();

    // echo
    Serial.write(incomingByte); 
  }
}
