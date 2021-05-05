#include <Wire.h>

#define SLAVE_ADDRESS 0x04
int number = 0;
int state = 0;
int data[32] = { 0 };
int i = 0;

void setup() {
pinMode(13, OUTPUT);
Serial.begin(9600); // start serial for output
// initialize i2c as slave
Wire.begin(SLAVE_ADDRESS);

// define callbacks for i2c communication
Wire.onReceive(receiveData);
Wire.onRequest(sendData);

Serial.println("Ready!");
}

void loop() {
delay(100);
}

// callback for received data
void receiveData(int byteCount){
i = 0;
while(Wire.available()) {
  data[i] = Wire.read();
  Serial.print("data received: ");
  Serial.print(data[i]);
  Serial.println(' ');
  i++;
}
Serial.println(' ');
}

// callback for sending data
void sendData(){
if (data[0] == 0) {
    data[1] = data[1]+5; 
}else if (data[0] == 1 ) {
  data[1] = data[1]+10;
}
Wire.write(data[1]);
}
