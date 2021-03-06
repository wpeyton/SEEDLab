/*  Group 10 - Andrew Burton, Trevor Bachand, William Peyton, & Kyra Squier
 *  EENG350B - SEED Lab
 *  Closed Loop Step Response Arduino Code
 *  01 March 2021
 *  
 *  Description: 
 *  The following program uses the designed positional controller and measures 
 *  the closed loop step response and prints it to the serial monitor.
 */
 
#include <Encoder.h>                // Encoder Library

#define COUNTS_PER_REVOLUTION 3200  // Number of counts per revolution on the encoder on the motor

#define M_ENABLE  4                 // Motor enable pin -- must be set high to operate motors
#define M1_DIR    7                 // Motor 1 direction pin -- set true for cw rotation and false for ccw
#define M1_PWM    9                 // Motor 1 pwm pin -- set value between 0 and 255 (0 to not move, 255 is max speed)

bool M1_Dir_Val = true;             // Value to store dir of motor -- set true for cw rotation and false for ccw
int  M1_PWM_Val = 0;                // Value to store speed of motor -- set value between 0 and 255 (0 to not move, 255 is max speed)

long myPosition  = 0;   // Store current position in cts

float theta;                        // Radians
float thetaDesired;                 // Radians
float Va;                           // Volts
int T = 5;                          // Sample time in ms
float totalError;                   // Variable to store accumulated error
float prevTime = 0;                 // Variable to store last time control algorithm was applied (ms)
float Kp = 27;                      // Proportional gain
float Ki = 6;                       // Integral gain
int umax = 12;                      // Max ouput of controller (V)

int startTimeForSerial = 0;      // Store program start time in millis
int previousTimeForSerial = 0;   // Store previous loop iteration start time in millis
int currentTimeForSerial = 0;    // Store current loop iteration start time in millis

Encoder myEnc(2, 5);                // Declare encoder object, with pin A = 2 (IOC) and pin B = 5

// Runs once at first boot up
void setup() {
  pinMode(M_ENABLE, OUTPUT);    // Define enable pin as output
  pinMode(M1_DIR, OUTPUT);      // Define direction pin as output
  pinMode(M1_PWM, OUTPUT);      // Define pwm pin as output
  
  digitalWrite(M_ENABLE, HIGH); // IMPORTANT!! -- set enable pin high

  Serial.begin(115200);         // Begin serial
  while (!Serial);              // Wait for serial monitor to open

  Serial.println("Open Loop Step Response:"); // Print header to serial

  startTimeForSerial = millis();         // Get program start time
  previousTimeForSerial = startTimeForSerial - 5; // Initiate prevtime value
  
  digitalWrite(M1_DIR, M1_Dir_Val); // Set motor direction
  analogWrite(M1_PWM, M1_PWM_Val);  // Set motor speed
  
  thetaDesired = 1.00;
}

// Runs repeatedly as long as power is applied to the arduino
void loop() {
  updateCurrentPos();               // Read the encoder and update the position variables
  PI_Controller();                  // Apply the control algorithm
  setMotors();                      // Set the motor outputs to values determined by control algorithm
  
  currentTimeForSerial = millis();           // Get current time in millis

  // If more than 5 ms have passed since prev iteration an you are still under 5 sec since start, then
  if (currentTimeForSerial - previousTimeForSerial >= 5 && currentTimeForSerial - startTimeForSerial <= 5000 ) {
    previousTimeForSerial = currentTimeForSerial; // Reset previousTimeForSerial

    // Print time stamp and angular pos to serial monitor
    Serial.print(currentTimeForSerial - startTimeForSerial);
    Serial.print("\t");
    Serial.print(theta, 6);
    Serial.print("\r\n");
    
  } else if (currentTimeForSerial - startTimeForSerial > 5000 ){ // Once you hit 5 sec, stop
    digitalWrite(M_ENABLE, LOW);
  }
}

// Read the encoder and update the position variables
void updateCurrentPos() {
  myPosition = myEnc.read();                            // Store the position in counts
  theta = 2 * PI * myPosition / COUNTS_PER_REVOLUTION;  // Convert counts to radians
}

// Apply the control algorithm
void PI_Controller() {
  float currentTime = millis();           // Returns current time value in ms
  int deltaTime = currentTime - prevTime; // Amount of time since previous algorithm application in ms

  if (deltaTime >= T) {                   // If more than 5 ms have passed since last control loop, then: 
    float error = thetaDesired - theta;   // Calculate the current error
    prevTime = currentTime;               // Reset the clock
    
    // Cap the error to +/- 1 rad to prevent excessive values
    if ( error > 1) {
      error = 1;
    } else if (error < -1) {
      error = -1;
    }
    
    // If the total error is less than 0.5 rad or the error and total error have opposite signs
    if (abs(totalError) < 0.5 || (error < 0 && totalError > 0) || (error > 0 && totalError < 0)) {
      totalError += (error * deltaTime * 0.001); // Increment the integral of the error
    }
   
    Va = Kp * error + Ki * totalError;    //PI controller algorithm
    
    // Cap the output of the controls to +/- umax
    if (Va > umax) {
      Va = umax;
    } else if (Va < -umax) {
      Va = -umax;
    }
  }
}

// Set the motor outputs to values determined by control algorithm
void setMotors() {
  // Determine motor direction
  if (Va > 0) {
    M1_Dir_Val = true;
  } else {
    M1_Dir_Val = false;
  }

  // Determine motor speed and scale for pwm signal output
  M1_PWM_Val = map(abs(Va), 0, 12, 0, 255);

  // Set the motor controller inputs accordingly
  digitalWrite(M1_DIR, M1_Dir_Val);
  analogWrite(M1_PWM, M1_PWM_Val);
}
