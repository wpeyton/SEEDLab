/*  Group 10 - Andrew Burton, Trevor Bachand, William Peyton, & Kyra Squier
    EENG350B - SEED Lab
    Full Robot Open Loop Step Response Arduino Code
    05 March 2021

    Description:
    The following program turns on the motors to half power and records
    the positional data and outputs results to the serial monitor.
    The bool var isRotating allows the user to select between the
    straight forward step response and the rotation step response.
*/

#include <Encoder.h>                // Encoder Library

#define COUNTS_PER_REVOLUTION 3200  // Number of counts per revolution on the encoder on the motor

#define M_ENABLE  4                 // Motor enable pin -- must be set high to operate motors
#define M1_DIR    7                 // Motor 1 direction pin -- set true for cw rotation and false for ccw
#define M1_PWM    9                 // Motor 1 pwm pin -- set value between 0 and 255 (0 to not move, 255 is max speed)
#define M2_DIR    8                 // Motor 1 direction pin -- set true for cw rotation and false for ccw
#define M2_PWM   10                 // Motor 1 pwm pin -- set value between 0 and 255 (0 to not move, 255 is max speed)

long startTimeForSerial = 0;         // Store previous loop iteration start time in millis
long currentTimeForSerial = 0;       // Store current loop iteration start time in millis

float theta1 = 0;
float theta2 = 0;

Encoder enc1(2, 5);                 // Declare encoder object, with pin A = 2 (IOC) and pin B = 5
Encoder enc2(3, 6);                 // Declare encoder object, with pin A = 3 (IOC) and pin B = 6

bool isRotating = true;

int pos1 = 0;
int pos2 = 0;

void setup() {
  pinMode(M_ENABLE, OUTPUT);        // Define enable pin as output
  pinMode(M1_DIR, OUTPUT);          // Define M1 direction pin as output
  pinMode(M1_PWM, OUTPUT);          // Define M1 pwm pin as output
  pinMode(M2_DIR, OUTPUT);          // Define M2 direction pin as output
  pinMode(M2_PWM, OUTPUT);          // Define M2 pwm pin as output

  digitalWrite(M_ENABLE, HIGH);     // IMPORTANT!! -- set enable pin high

  Serial.begin(115200);             // Begin serial
  while (!Serial);                  // Wait for serial monitor to open

  Serial.println("Full Robot Open Loop Step Response Experiment:"); // Print header to serial
  Serial.println("Time (s)\tTheta1 (rad)\tTheta2 (rad)");

  startTimeForSerial = micros();    // Get program start time

  enc1.write(0);
  enc2.write(0);
  
  digitalWrite(M1_DIR, false);       // Set motor direction
  analogWrite(M1_PWM, 128);         // Set motor speed
  digitalWrite(M2_DIR, !isRotating); // Set motor direction
  analogWrite(M2_PWM, 128);         // Set motor speed

}

void loop() {
  while (currentTimeForSerial - startTimeForSerial <= 3000000) {
    currentTimeForSerial = micros();

    pos1 = enc1.read();
    pos2 = enc2.read();

    theta1 = -2 * PI * pos1 / COUNTS_PER_REVOLUTION;  // Convert counts to radians
    theta2 = 2 * PI * pos2 / COUNTS_PER_REVOLUTION;  // Convert counts to radians
    
    Serial.print(0.000001 * (currentTimeForSerial - startTimeForSerial), 6);
    Serial.print("\t\t");
    Serial.print(theta1, 6);
    Serial.print("\t\t");
    Serial.print(theta2, 6);
    Serial.print("\r\n");
  }

  digitalWrite(M_ENABLE, LOW);  

}
