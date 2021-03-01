/*  Group 10 - Andrew Burton, Trevor Bachand, William Peyton, & Kyra Squier
 *  EENG350B - SEED Lab
 *  Open Loop Step Response Arduino Code
 *  01 March 2021
 *  
 *  Description: 
 *  The following program turns on a motor using the motor driver shield and ouputs
 *  time and positional data to the serial monitor.
 */

#include <Encoder.h> // This is the encoder library, you will need to download it through the library manger

#define COUNTS_PER_REVOLUTION 3200  // Number of counts per revolution on the encoder on the motor

#define M_ENABLE  4     // Motor enable pin -- must be set high to operate motors
#define M1_DIR    7     // Motor 1 direction pin -- set true for cw rotation (i think) and false for ccw
#define M1_PWM    9     // Motor 1 pwm pin -- set value between 0 and 255 (0 to not move, 255 is max speed)

bool M1_Dir_Val = true; // value to store dir of motor -- set true for cw rotation (i think) and false for ccw
int  M1_PWM_Val = 255;  // value to store speed of motor -- set value between 0 and 255 (0 to not move, 255 is max speed)

int startTime = 0;      // Store program start time in millis
int previousTime = 0;   // Store previous loop iteration start time in millis
int currentTime = 0;    // Store current loop iteration start time in millis

long myPosition  = 0;   // Store current position in cts
double myAngle = 0;     // Store current position in rads

Encoder myEnc(2, 5);    // Declare encoder object, with pin A = 2 and pin B = 5

void setup() {
  pinMode(M_ENABLE, OUTPUT);    // Define enable pin as output
  pinMode(M1_DIR, OUTPUT);      // Define direction pin as output
  pinMode(M1_PWM, OUTPUT);      // Define pwm pin as output
  
  digitalWrite(M_ENABLE, HIGH); // IMPORTANT!! -- set enable pin high

  Serial.begin(115200);         // Begin serial
  while (!Serial);              // Wait for serial monitor to open

  Serial.println("Step Response:"); // Print header to serial

  startTime = millis();         // Get program start time
  previousTime = startTime - 5; // Initiate prevtime value
  
  digitalWrite(M1_DIR, M1_Dir_Val); // Set motor direction
  analogWrite(M1_PWM, M1_PWM_Val);  // Set motor speed
}

void loop() {
  currentTime = millis(); // Get current time in millis

  // If more than 5 ms have passed since prev iteration an you are still under 10 rad, then
  if (currentTime - previousTime >= 5 && myAngle <= (long) 9.99 ) {
    previousTime = currentTime; // Reset prevtime
    
    myPosition = myEnc.read(); // Read encoder
    myAngle = 2 * PI * myPosition / COUNTS_PER_REVOLUTION; // Convert cts to rads

    // Print time stamp and angular pos to serial monitor
    Serial.print(currentTime - startTime);
    Serial.print("\t");
    Serial.print(myAngle, 6);
    Serial.print("\r\n");
    
  } else if (myAngle > (long) 9.99){ // Once you hit 10 rad, stop
    digitalWrite(M1_DIR, M1_Dir_Val);
    analogWrite(M1_PWM, 0);
  }
}
