#include <Wire.h>

#define SLAVE_ADDRESS 0x04
int number = 0;
byte data[32] = { 0 };
byte data2[32] = { 0 };
int i = 0;
int j = 0;
int k = 0;
int tempVar;
bool temp = 1; 
void setup() {
pinMode(13, OUTPUT);
Serial.begin(115200); // start serial for output
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
while(Wire.available()) {
  data[i] = Wire.read();
  if (data[i] == 0) {
    data[i] = Wire.read();
  }
    i++;
  }
}


// callback for sending data
void sendData(){
  j=i-2;
  k=0;
  while(k<j){ //reverse array 
        tempVar=data[k];
        data[k]=data[j];
        data[j]=tempVar;
        k++;
        j--;
  }
  Wire.write(data, i);
  i=0;
}
