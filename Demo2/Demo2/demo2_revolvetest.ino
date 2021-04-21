/*  Group 10 - Andrew Burton, Trevor Bachand, William Peyton, & Kyra Squier
    EENG350B - SEED Lab
    Demo2 Controller Arduino Code
    07 April 2021
    Description:
    The following program runs the full control algorithm.
    It takes in a set point from the Pi over serial, travels to it, and then circles the marker.
*/
#include <Encoder.h>                // Encoder Library
#define COUNTS_PER_REVOLUTION 3200  // Number of counts per revolution on the encoder on the motor

#define M_ENABLE  4                 // Motor enable pin -- must be set high to operate motors
#define M1_DIR    7                 // Motor 1 direction pin -- set true for cw rotation and false for ccw
#define M2_DIR    8                 // Motor 2 direction pin -- set true for cw rotation and false for ccw
#define M1_PWM    9                 // Motor 1 pwm pin -- set value between 0 and 255 (0 to not move, 255 is max speed)
#define M2_PWM   10                 // Motor 2 pwm pin -- set value between 0 and 255 (0 to not move, 255 is max speed)

bool do_revolve = true;
bool found_marker = false;

bool align = true;

bool run_phi_controller = false;
bool run_rho_controller = false;

float marker_phi = 0;
float marker_rho = 0;

float phi_at_revolve_start = 0;

String test = "";

typedef enum {
  SEARCH,
  ALIGN,
  APPROACH,
  REVOLVE,
  STOP
} state_t;

state_t current_state = SEARCH; // changed to test

Encoder enc1(2, 5);                 // Declare encoder object, with pin A = 2 (IOC) and pin B = 5
Encoder enc2(3, 6);                 // Declare encoder object, with pin A = 3 (IOC) and pin B = 6

bool M1_Dir_Val = true;             // Value to store dir of motor 1 -- set true for cw rotation and false for ccw
bool M2_Dir_Val = true;             // Value to store dir of motor 2 -- set true for cw rotation and false for ccw
int  M1_PWM_Val = 0;                // Value to store speed of motor 1 -- set value between 0 and 255 (0 to not move, 255 is max speed)
int  M2_PWM_Val = 0;                // Value to store speed of motor 2 -- set value between 0 and 255 (0 to not move, 255 is max speed)

// Vars for all controllers
const float r = 7.5 * 0.01;         // Radius of the wheel
const float d = 24.62 * 0.01;       // Distance between the wheels

float Va = 0;                       // Control algorithm output, between [-1.0, 1.0], describes linear motion
float delta_Va = 0;                 // Control algorithm output, between [-1.0, 1.0], describes rotational motio

float theta_1_previous = 0;         // Store the previous value to determine the derivative terms
float theta_2_previous = 0;         // Store the previous value to determine the derivative terms

float theta_1 = 0;                  // Store the current position of wheel 1 in radians
float theta_2 = 0;                  // Store the current position of wheel 2 in radians

float theta_dot_1 = 0;              // Store the current velocity of wheel 1 in radians per second
float theta_dot_2 = 0;              // Store the current velocity of wheel 2 in radians per second

// Vars for rot_pos_controller
float phi_set = 0.0;                //  Variable storing desired position in radians

float phi = 0;                      // Current Position
float phi_error = 0;                // Positional error
float phi_error_prev = 0;           // Last positional error
float phi_error_dot = 0;            // Derivative of positional error
float phi_total_error = 0;          // Integral of positional error

const float Kp_phi = 7;            // Positional controller proportional gain
const float Ki_phi = 7;            // Positional controller integral gain
const float Kd_phi = 0.25;          // Positional controller derivative gain

// Vars for rot_speed_controller
float phi_dot_set = 0.0;            // Variable storing desired angular velocity in radians per second

float phi_dot = 0;                  // Current angular velocity
float phi_dot_error = 0;            // Angular velocity error
float phi_dot_total_error = 0;      // Integral of angular velocity error

const float Kp_phi_dot = 0.1;       // Velocity controller proportional gain
const float Ki_phi_dot = 1;         // Velocity controller proportional gain

// Vars for trans_speed_controller
float rho_dot_set = 0.0;            // Desired translational velocity (in m/s)

float rho_dot = 0;                  // Current translational speed (in m/s)
float rho_dot_error = 0;            // Translational speed error (in m/s)
float rho_dot_total_error = 0;      // Integral of translational speed error (in m)

const float Kp_rho_dot = 1;         // Translational speed controller proportional gain
const float Ki_rho_dot = 5;         // Translational speed controller integral gain

int startTime = 0;                  // Store program start time
int currentTime = 0;                // Store current time

float rho_set = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(M_ENABLE, OUTPUT);        // Define enable pin as output
  pinMode(M1_DIR, OUTPUT);          // Define direction pin as output
  pinMode(M1_PWM, OUTPUT);          // Define pwm pin as output
  pinMode(M2_DIR, OUTPUT);          // Define direction pin as output
  pinMode(M2_PWM, OUTPUT);          // Define pwm pin as output

  Serial.begin(115200);               // Initialize Serial comms
  while (!Serial);                  // Wait for initialization

  digitalWrite(M_ENABLE, HIGH);     // IMPORTANT!! -- set enable pin high
  digitalWrite(M1_DIR, true);       // Set initial motor direction to cw
  analogWrite(M1_PWM, 0);           // Set initial motor speed to 0
  digitalWrite(M2_DIR, true);       // Set initial motor direction to cw
  analogWrite(M2_PWM, 0);           // Set initial motor speed to 0

  OCR0A = 0xAF;                     // Set TMR0 interrupt comparison value
  TIMSK0 |= _BV(OCIE0A);            // Enable TMR0 interrupt

  startTime = millis();
}

void loop() {
  // Record current time
  currentTime = millis();
  if (Serial.available() > 0) {
    dataRead(marker_phi, marker_rho); // phi & rho passed by reference into program
    if (current_state != REVOLVE) {
      found_marker = true;
      align = true;
      // phi_dot_set = 0;
    }
  }
  // Set motor ouputs to the calculated values
  digitalWrite(M1_DIR, M1_Dir_Val);
  analogWrite(M1_PWM, M1_PWM_Val);
  digitalWrite(M2_DIR, M2_Dir_Val);
  analogWrite(M2_PWM, M2_PWM_Val);

  switch (current_state) {
    case SEARCH:
      if (found_marker) {
        current_state = ALIGN;
        startTime = millis();
        align = true;
        rho_dot_set = 0;
        phi_dot_set = 0;
        run_rho_controller = false;
        found_marker = false;
        delay(10);
      } else {
        phi_dot_set = 0.3; // adjusted down from 5.0
        rho_dot_set = 0;
      }
      break;
    case ALIGN:
      if (align && found_marker) {
        rho_dot_set = 0;
        phi_dot_set = 0;
        run_phi_controller = true;
        phi_set = (marker_phi / 2) + phi;
        align  = false;
        delay(50);
      }
      else if (found_marker && abs(phi_error) <= 0.02) {
        current_state = APPROACH;
        delay(50);
        rho_set = marker_rho;
        startTime = millis();
      }
      else if(!found_marker) {
        if (currentTime - startTime > 1000) {
          phi_dot_set = -0.1;
        }
      }
      break;
    case APPROACH:
      run_rho_controller = true;

            // if (abs(marker_rho) <= 0.65) {
            if (currentTime - startTime >= rho_set * 1825) {
              phi_dot_set = 0;
              rho_dot_set = 0;
              startTime = millis();
              if (do_revolve) {
                current_state = REVOLVE;
                phi_at_revolve_start = phi;
                delay(15);
                found_marker = false;
              } else
                current_state = STOP;
            } else if (found_marker) {
              //phi_set = phi + marker_phi;
              rho_dot_set = 0.5;
              found_marker = false; //added
            }

      break;

    case REVOLVE:

      delay(250);
      if (currentTime - startTime >= 9450) {
        current_state = STOP;
      } else if (currentTime - startTime >= 1400) {
        run_phi_controller = false;
        phi_dot_set = 0.8; //0.8
        rho_dot_set = 0.3; //0.3
      } else if (currentTime - startTime >= 1350) {
        run_phi_controller = false;
        phi_dot_set = 0.8; //0.8
        rho_dot_set = 0.8; //0.8
      } else if (currentTime - startTime >= 1300) {
        run_phi_controller = false;
        phi_dot_set = 0;
        rho_dot_set = 0;
      } else {
        run_phi_controller = true;
        phi_set = phi_at_revolve_start - (PI / 2);
        rho_dot_set = 0.0;
      }
      break;
    case STOP:
      Serial.println("Stop");
      digitalWrite(M_ENABLE, LOW);
      phi_set = phi;
      phi_dot_set = 0;
      rho_dot_set = 0;
      break;
  }

}

// TMR0 interrupt service routine
SIGNAL(TIMER0_COMPA_vect) {
  readEncoders();       // Get current position
  if (run_phi_controller) phi_controller();    // Calculate necessary angular speed
  phi_dot_controller(); // Calculate deltaVa
  if (run_rho_controller) rho_dot_controller(); // Calcualte Va
  setMotorVals();       // Scale ouput for PWM
}

// Record current position and velocuty of the wheels
void readEncoders() {
  theta_1_previous = theta_1;   // Store previous value
  theta_2_previous = theta_2;   // Store previous value

  theta_1 = -2.0 * PI * enc1.read() / COUNTS_PER_REVOLUTION;  // Store position in radians
  theta_2 = 2.0 * PI * enc2.read() / COUNTS_PER_REVOLUTION;   // Store position in radians

  theta_dot_1 = (theta_1 - theta_1_previous) * 1000;          // Calculate angular velocity
  theta_dot_2 = (theta_2 - theta_2_previous) * 1000;          // Calculate angular velocity

  phi = - r * (theta_1 - theta_2) / d;      // Calculate current angle
  phi_dot = - r * (theta_dot_1 - theta_dot_2) / d;    // Calculate angular velocity
  rho_dot = r * (theta_dot_1 + theta_dot_2) / 2.0;  // Calculate linear velocity
}

// Apply feedback control to calculate necessary angular velocity
void phi_controller() {
  phi = - r * (theta_1 - theta_2) / d;      // Calculate current angle
  phi_error_prev = phi_error;               // Store prev error
  phi_error = phi_set - phi;                // Calculate current error

  // Error bounding
  if (phi_error < -1.0) phi_error = -1.0;
  if (phi_error > 1.0) phi_error = 1.0;

  // Calculate integral of the error, apply integral clamping
  if (abs(phi_total_error) < 0.5 || (phi_error < 0 && phi_total_error > 0) || (phi_error > 0 && phi_total_error < 0)) {
    phi_total_error += (phi_error * 0.001); // Increment the integral of the error
  }

  // Calculate derivative of the error
  phi_error_dot = (phi_error - phi_error_prev) * 1000;

  // Apply PID control algorithm to determine angular velocity
  phi_dot_set = Kp_phi * phi_error + Kd_phi * phi_error_dot;

  // Output bounding
  if (phi_dot_set > 8.0) phi_dot_set = 8.0;
  if (phi_dot_set < -8.0) phi_dot_set = -8.0;

}

// Apply feedback control to calculate motor voltages to rotate as desired
void phi_dot_controller() {
  phi_dot = - r * (theta_dot_1 - theta_dot_2) / d;    // Calculate angular velocity
  phi_dot_error = phi_dot_set - phi_dot;              // Calculate angular velocity error

  // Error bounding
  if (phi_dot_error < -8.0) phi_dot_error = -8.0;
  if (phi_dot_error > 8.0) phi_dot_error = 8.0;

  // Update the integral term, apply integral clamping
  if (abs(phi_dot_total_error) < 0.5 || (phi_dot_error < 0 && phi_dot_total_error > 0) || (phi_dot_error > 0 && phi_dot_total_error < 0)) {
    phi_dot_total_error += (phi_dot_error * 0.001); // Increment the integral of the error
  }

  // Apply feedback control alorithm to calculate deltaVa
  delta_Va = - (Kp_phi_dot * phi_dot_error + Ki_phi_dot * phi_dot_total_error);
}

// Apply feedback control to calculate motor voltages to translate as desired
void rho_dot_controller() {
  rho_dot = r * (theta_dot_1 + theta_dot_2) / 2.0;  // Calculate linear velocity
  rho_dot_error = rho_dot_set - rho_dot;            // Calculate linear velocity error

  // Error bounding
  if (rho_dot_error < -1.0) rho_dot_error = -1.0;
  if (rho_dot_error > 1.0) rho_dot_error = 1.0;

  // Update the integral term, apply integral clamping
  if (abs(rho_dot_total_error) < 0.5 || (rho_dot_error < 0 && rho_dot_total_error > 0) || (rho_dot_error > 0 && rho_dot_total_error < 0)) {
    rho_dot_total_error += (rho_dot_error * 0.001); // Increment the integral of the error
  }

  // Apply feedback control alorithm to calculate Va
  Va = Kp_rho_dot * rho_dot_error + Ki_rho_dot * rho_dot_total_error;
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


void dataRead(float &targetPhi, float &targetRho) { //pass by reference marker_phi & phi_set
  String piData = Serial.readStringUntil('\n');
  int len = piData.length();
  if (piData[0] == 'a') {
    int distanceLoc = piData.indexOf('d');
    String angleStr = piData.substring(1, distanceLoc);
    String disStr = piData.substring(distanceLoc + 1, len);
    targetPhi = angleStr.toFloat();
    targetRho = disStr.toFloat();
  }
}
