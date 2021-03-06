#include <Encoder.h>                // Encoder Library

#define COUNTS_PER_REVOLUTION 3200  // Number of counts per revolution on the encoder on the motor

#define NUM_FT_TO_MOVE 1
#define NUM_DEG_TO_TURN -180
#define TRANS_RAMP_UP_TIME 110

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
const float r = 7.5 * 0.01;
const float d = 24.62 * 0.01;

float Va = 0;
float delta_Va = 0;

float theta_1_previous = 0;
float theta_2_previous = 0;

float theta_1 = 0;
float theta_2 = 0;

float theta_dot_1 = 0;
float theta_dot_2 = 0;

// Vars for rot_pos_controller
float phi_set = 0.0;

float phi = 0;
float phi_error = 0;
float phi_error_prev = 0;
float phi_error_dot = 0;
float phi_total_error = 0;

const float Kp_phi = 10;
const float Ki_phi = 10;
const float Kd_phi = 0.25;

// Vars for rot_speed_controller
float phi_dot_set = 0.0;

float phi_dot = 0;
float phi_dot_error = 0;
float phi_dot_total_error = 0;

const float Kp_phi_dot = 0.1;
const float Ki_phi_dot = 1;

// Vars for trans_speed_controller
float rho_dot_set = 0.0;

float rho_dot = 0;
float rho_dot_error = 0;
float rho_dot_total_error = 0;

const float Kp_rho_dot = 1;
const float Ki_rho_dot = 5;

int startTime = 0;
int currentTime = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(M_ENABLE, OUTPUT);        // Define enable pin as output
  pinMode(M1_DIR, OUTPUT);          // Define direction pin as output
  pinMode(M1_PWM, OUTPUT);          // Define pwm pin as output
  pinMode(M2_DIR, OUTPUT);          // Define direction pin as output
  pinMode(M2_PWM, OUTPUT);          // Define pwm pin as output

  Serial.begin(9600);
  while (!Serial);

  digitalWrite(M_ENABLE, HIGH);     // IMPORTANT!! -- set enable pin high
  digitalWrite(M1_DIR, true); // Set initial motor direction to cw
  analogWrite(M1_PWM, 0);  // Set initial motor speed to 0
  digitalWrite(M2_DIR, true); // Set initial motor direction to cw
  analogWrite(M2_PWM, 0);  // Set initial motor speed to 0
  
  OCR0A = 0xAF;
  TIMSK0 |= _BV(OCIE0A);

  startTime = millis();
  phi_set = NUM_DEG_TO_TURN * PI / 180;
}

void loop() {
  // put your main code here, to run repeatedly:
  currentTime = millis();
  
  digitalWrite(M1_DIR, M1_Dir_Val);
  analogWrite(M1_PWM, M1_PWM_Val);
  digitalWrite(M2_DIR, M2_Dir_Val);
  analogWrite(M2_PWM, M2_PWM_Val);

  if(currentTime - startTime >= ((NUM_FT_TO_MOVE * 0.3048 / 0.5) * 1000)+ TRANS_RAMP_UP_TIME + (15 * abs(NUM_DEG_TO_TURN))) {
    rho_dot_set = 0.0;
  } else if(currentTime - startTime >= 15 * abs(NUM_DEG_TO_TURN)) {
    rho_dot_set = 0.5;
  }
  
//  delay(25);
//  Serial.print(phi);
//  Serial.print("\t");
//  Serial.print(phi_dot_set);
//  Serial.print("\t");
//  Serial.print(M1_PWM_Val);
//  Serial.print("\t");
//  Serial.println(M2_PWM_Val);
}

SIGNAL(TIMER0_COMPA_vect) {
  readEncoders();
  phi_controller();
  phi_dot_controller();
  rho_dot_controller();
  setMotorVals();
}

void readEncoders() {
  theta_1_previous = theta_1;
  theta_2_previous = theta_2;

  theta_1 = -2.0 * PI * enc1.read() / COUNTS_PER_REVOLUTION;  // Convert counts to radians
  theta_2 = 2.0 * PI * enc2.read() / COUNTS_PER_REVOLUTION;  // Convert counts to radians

  theta_dot_1 = (theta_1 - theta_1_previous) * 1000;
  theta_dot_2 = (theta_2 - theta_2_previous) * 1000;
}

void phi_controller() {
  phi = - r * (theta_1 - theta_2) / d;
  phi_error_prev = phi_error;
  phi_error = phi_set - phi;

  if (phi_error < -1.0) phi_error = -1.0;
  if (phi_error > 1.0) phi_error = 1.0;

  if (abs(phi_total_error) < 0.5 || (phi_error < 0 && phi_total_error > 0) || (phi_error > 0 && phi_total_error < 0)) {
    phi_total_error += (phi_error * 0.001); // Increment the integral of the error
  }

  phi_error_dot = (phi_error - phi_error_prev) * 1000;
  
  phi_dot_set = Kp_phi * phi_error + Kd_phi * phi_error_dot;
  // Ki_phi * phi_total_error
  
  if (phi_dot_set > 8.0) phi_dot_set = 8.0;
  if (phi_dot_set < -8.0) phi_dot_set = -8.0;

}

void phi_dot_controller() {
  phi_dot = - r * (theta_dot_1 - theta_dot_2) / d;
  phi_dot_error = phi_dot_set - phi_dot;

  if (phi_dot_error < -8.0) phi_dot_error = -8.0;
  if (phi_dot_error > 8.0) phi_dot_error = 8.0;

  if (abs(phi_dot_total_error) < 0.5 || (phi_dot_error < 0 && phi_dot_total_error > 0) || (phi_dot_error > 0 && phi_dot_total_error < 0)) {
    phi_dot_total_error += (phi_dot_error * 0.001); // Increment the integral of the error
  }

  delta_Va = - (Kp_phi_dot * phi_dot_error + Ki_phi_dot * phi_dot_total_error);
}

void rho_dot_controller() {
  rho_dot = r * (theta_dot_1 + theta_dot_2) / 2.0;
  rho_dot_error = rho_dot_set - rho_dot;

  if (rho_dot_error < -1.0) rho_dot_error = -1.0;
  if (rho_dot_error > 1.0) rho_dot_error = 1.0;

  if (abs(rho_dot_total_error) < 0.5 || (rho_dot_error < 0 && rho_dot_total_error > 0) || (rho_dot_error > 0 && rho_dot_total_error < 0)) {
    rho_dot_total_error += (rho_dot_error * 0.001); // Increment the integral of the error
  }

  Va = Kp_rho_dot * rho_dot_error + Ki_rho_dot * rho_dot_total_error;
}

void setMotorVals() {
  M1_PWM_Val = (int) (255 * (Va + delta_Va) / 2.0);
  M2_PWM_Val = (int) (255 * (Va - delta_Va) / 2.0);

  if (M1_PWM_Val > 0) M1_Dir_Val = false;
  else M1_Dir_Val = true;

  if (M2_PWM_Val > 0) M2_Dir_Val = true;
  else M2_Dir_Val = false;

  M1_PWM_Val = abs(M1_PWM_Val);
  M2_PWM_Val = abs(M2_PWM_Val);
}
