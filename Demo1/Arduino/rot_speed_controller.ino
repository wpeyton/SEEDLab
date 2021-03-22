/*  Group 10 - Andrew Burton, Trevor Bachand, William Peyton, & Kyra Squier
    EENG350B - SEED Lab
    Rotational Speed Controller Arduino Code
    05 March 2021
    Description:
    The following program runs the rotational speed feedback control algorithm. 
    A speed set point is stored as a variable, and the motor outputs are calculated to 
    get the robot to turn at that rate. 
*/

#include <Encoder.h>                // Encoder Library

#define COUNTS_PER_REVOLUTION 3200  // Number of counts per revolution on the encoder on the motor

#define M_ENABLE  4                 // Motor enable pin -- must be set high to operate motors
#define M1_DIR    7                 // Motor 1 direction pin -- set true for cw rotation and false for ccw
#define M2_DIR    8                 // Motor 2 direction pin -- set true for cw rotation and false for ccw
#define M1_PWM    9                 // Motor 1 pwm pin -- set value between 0 and 255 (0 to not move, 255 is max speed)
#define M2_PWM   10                 // Motor 2 pwm pin -- set value between 0 and 255 (0 to not move, 255 is max speed)

Encoder enc1(2, 5);                 // Declare encoder object, with pin A = 2 (IOC) and pin B = 5
Encoder enc2(3, 6);                 // Declare encoder object, with pin A = 3 (IOC) and pin B = 6

bool M1_Dir_Val = true;             // Value to store dir of motor 1 -- set true for cw rotation and false for ccw
bool M2_Dir_Val = true;             // Value to store dir of motor 2 -- set true for cw rotation and false for ccw
int  M1_PWM_Val = 0;                // Value to store speed of motor 1 -- set value between 0 and 255 (0 to not move, 255 is max speed)
int  M2_PWM_Val = 0;                // Value to store speed of motor 2 -- set value between 0 and 255 (0 to not move, 255 is max speed)

// Vars for all controllers
const float r = 7.5 * 0.01;         // Radius of the wheel
const float d = 25 * 0.01;          // Distance between the wheels

float Va = 0;                       // Control algorithm output, between [-1.0, 1.0], describes linear motion
float delta_Va = 0;                 // Control algorithm output, between [-1.0, 1.0], describes rotational motion

float theta_1_previous = 0;         // Store the previous value to determine the derivative terms
float theta_2_previous = 0;         // Store the previous value to determine the derivative terms

float theta_1 = 0;                  // Store the current position of wheel 1 in radians 
float theta_2 = 0;                  // Store the current position of wheel 2 in radians

float theta_dot_1 = 0;              //  Store the current velocity of wheel 1 in radians per second  
float theta_dot_2 = 0;              //  Store the current velocity of wheel 2 in radians per second

// Vars for rot_speed_controller
float phi_dot_set = 0.0;            // Variable storing desired angular velocity in radians per second

float phi_dot = 0;                  // Current angular velocity
float phi_dot_error = 0;            // Angular velocity error
float phi_dot_total_error = 0;      // Integral of angular velocity error

const float Kp_phi_dot = 0.1;       // Velocity controller proportional gain
const float Ki_phi_dot = 1;         // Velocity controller proportional gain

void setup() {
  // put your setup code here, to run once:
  pinMode(M_ENABLE, OUTPUT);        // Define enable pin as output
  pinMode(M1_DIR, OUTPUT);          // Define direction pin as output
  pinMode(M1_PWM, OUTPUT);          // Define pwm pin as output
  pinMode(M2_DIR, OUTPUT);          // Define direction pin as output
  pinMode(M2_PWM, OUTPUT);          // Define pwm pin as output

  Serial.begin(9600);               // Initialize Serial comms
  while (!Serial);                  // Wait for initialization

  digitalWrite(M_ENABLE, HIGH);     // IMPORTANT!! -- set enable pin high
  digitalWrite(M1_DIR, true); // Set initial motor direction to cw
  analogWrite(M1_PWM, 0);  // Set initial motor speed to 0
  digitalWrite(M2_DIR, true); // Set initial motor direction to cw
  analogWrite(M2_PWM, 0);  // Set initial motor speed to 0

  OCR0A = 0xAF;                     // Set TMR0 interrupt comparison value
  TIMSK0 |= _BV(OCIE0A);            // Enable TMR0 interrupt
}

void loop() {
  // Set motor ouputs to the calculated values
  digitalWrite(M1_DIR, M1_Dir_Val);
  analogWrite(M1_PWM, M1_PWM_Val);
  digitalWrite(M2_DIR, M2_Dir_Val);
  analogWrite(M2_PWM, M2_PWM_Val);

  delay(50); // Wait 50 ms
  
  // Print out debug info to serial monitor
  Serial.print(M1_PWM_Val);
  Serial.print("\t");
  Serial.println(M2_PWM_Val);
}

// TMR0 interrupt service routine
SIGNAL(TIMER0_COMPA_vect) {
  readEncoders();       // Get current position
  phi_dot_controller(); // Calculate deltaVa
  setMotorVals();       // Scale ouput for PWM
}

void readEncoders() {
  theta_1_previous = theta_1;   // Store previous value 
  theta_2_previous = theta_2;   // Store previous value

  theta_1 = -2.0 * PI * enc1.read() / COUNTS_PER_REVOLUTION;  // Store position in radians
  theta_2 = 2.0 * PI * enc2.read() / COUNTS_PER_REVOLUTION;   // Store position in radians

  theta_dot_1 = (theta_1 - theta_1_previous) * 1000;          // Calculate angular velocity
  theta_dot_2 = (theta_2 - theta_2_previous) * 1000;          // Calculate angular velocity
}

// Apply feedback control to calculate motor voltages to rotate as desired
void phi_dot_controller() {
  phi_dot = - r * (theta_dot_1 - theta_dot_2) / d;    // Calculate angular velocity
  phi_dot_error = phi_dot_set - phi_dot;              // Calculate angular velocity error

  // Error bounding
  if (phi_dot_error < -10.0) phi_dot_error = -10.0; 
  if (phi_dot_error > 10.0) phi_dot_error = 10.0;

  // Update the integral term, apply integral clamping
  if (abs(phi_dot_total_error) < 0.5 || (phi_dot_error < 0 && phi_dot_total_error > 0) || (phi_dot_error > 0 && phi_dot_total_error < 0)) {
    phi_dot_total_error += (phi_dot_error * 0.001); // Increment the integral of the error
  }
  
  // Apply feedback control alorithm to calculate deltaVa
  delta_Va = - (Kp_phi_dot * phi_dot_error + Ki_phi_dot * phi_dot_total_error);
}

// Scale the controller outputs to be pwm values
void setMotorVals() {
  // Calculate the pwm value for each motor based on Va, deltaVa
  M1_PWM_Val = (int) (255 * (Va + delta_Va) / 2.0);
  M2_PWM_Val = (int) (255 * (Va - delta_Va) / 2.0);

  // Set motor directions
  if (M1_PWM_Val > 0) M1_Dir_Val = false;
  else M1_Dir_Val = true;

  if (M2_PWM_Val > 0) M2_Dir_Val = true;
  else M2_Dir_Val = false;
  
  // Make PWM output unsigned
  M1_PWM_Val = abs(M1_PWM_Val);
  M2_PWM_Val = abs(M2_PWM_Val);
}
