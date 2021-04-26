/*  Group 10 - Andrew Burton, Trevor Bachand, William Peyton, & Kyra Squier
    EENG350B - SEED Lab
    Final Demo Arduino Code
    03 May 2021
    Description:
    The following program runs the full control algorithm and runs through the state machine for the final demo run.
*/

#include <Encoder.h>                // Encoder Library

#define COUNTS_PER_REVOLUTION 3200  // Number of counts per revolution on the encoder on the motor

#define M_ENABLE  4                 // Motor enable pin -- must be set high to operate motors
#define M1_DIR    7                 // Motor 1 direction pin -- set true for cw rotation and false for ccw
#define M2_DIR    8                 // Motor 2 direction pin -- set true for cw rotation and false for ccw
#define M1_PWM    9                 // Motor 1 pwm pin -- set value between 0 and 255 (0 to not move, 255 is max speed)
#define M2_PWM   10                 // Motor 2 pwm pin -- set value between 0 and 255 (0 to not move, 255 is max speed)

// Define custom variable type for state machine
typedef enum {
  SEARCH_INITIAL,
  LOCATE_MARKER,
  ALIGN,
  APPROACH,
  ROT_90_CW,
  SEARCH_NEXT,
  STOP
} state_t;

// Controller Enable Flags
bool run_phi_controller = false;
bool run_phi_dot_controller = false;
bool run_rho_dot_controller = false;

// FIND_FIRST Active
bool find_first = false;

// FIND_LOC Active
bool find_loc = false;

// Marker location data
bool found_marker = false;
float marker_location_relative[2] = {0.00}; // [phi][rho]
float marker_location_absolute[2] = {0.00}; // [phi][rho]

// ITERATION TRACKING
int num_markers_traversed = 0;
int num_markers_total = 6;

// Current state variable
state_t current_state = SEARCH_INITIAL;

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

const float Kp_phi = 7;             // Positional controller proportional gain
const float Ki_phi = 7;             // Positional controller integral gain
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

float rho_set = 0;                  // Store distance to travel

int startTime = 0;                  // Store program start time
int currentTime = 0;                // Store current time

void setup() {
  // put your setup code here, to run once:
  pinMode(M_ENABLE, OUTPUT);        // Define enable pin as output
  pinMode(M1_DIR, OUTPUT);          // Define direction pin as output
  pinMode(M1_PWM, OUTPUT);          // Define pwm pin as output
  pinMode(M2_DIR, OUTPUT);          // Define direction pin as output
  pinMode(M2_PWM, OUTPUT);          // Define pwm pin as output

  Serial.begin(115200);             // Initialize Serial comms
  while (!Serial);                  // Wait for initialization

  digitalWrite(M_ENABLE, HIGH);     // IMPORTANT!! -- set enable pin high
  digitalWrite(M1_DIR, true);       // Set initial motor direction to cw
  analogWrite(M1_PWM, 0);           // Set initial motor speed to 0
  digitalWrite(M2_DIR, true);       // Set initial motor direction to cw
  analogWrite(M2_PWM, 0);           // Set initial motor speed to 0

  OCR0A = 0xAF;                     // Set TMR0 interrupt comparison value
  TIMSK0 |= _BV(OCIE0A);            // Enable TMR0 interrupt

  startTime = millis();             // Log program start time
}

void loop() {
  currentTime = millis();           // Record current time

  if (Serial.available() > 0) {
    dataRead();
  }

  // Set motor ouputs to the calculated values
  digitalWrite(M1_DIR, M1_Dir_Val);
  analogWrite(M1_PWM, M1_PWM_Val);
  digitalWrite(M2_DIR, M2_Dir_Val);
  analogWrite(M2_PWM, M2_PWM_Val);

  switch (current_state) {
    case SEARCH_INITIAL:    // Rotate on central axis until the first marker is found
      if (found_marker) {
        current_state = LOCATE_MARKER;

        find_first = false;
        found_marker = false;

        run_phi_controller = false;
        run_phi_dot_controller = false;
        run_rho_dot_controller = false;

        phi_set = phi;
        phi_dot_set = 0;
        rho_dot_set = 0;

        startTime = millis();
      } else {
        if (!find_first) {
          find_first = true;
          Serial.println("FIND_FIRST");
        }
        
        phi_dot_set = 0.3;
        rho_dot_set = 0;

        run_phi_controller = false;
        run_phi_dot_controller = true;
        run_rho_dot_controller = false;
      }
      break;

    case LOCATE_MARKER: // Robot stops, requests accurate marker location data, waits until rx
      if (found_marker) {
        current_state = ALIGN;

        find_loc = false;
        found_marker = false;

        run_phi_controller = false;
        run_phi_dot_controller = false;
        run_rho_dot_controller = false;

        marker_location_absolute[0] = marker_location_relative[0] + phi;
        marker_location_absolute[1] = marker_location_relative[1];

        phi_set = marker_location_absolute[0];
        phi_dot_set = 0;
        rho_dot_set = 0;

        startTime = millis();
      } else {
        if (!find_loc) {
          find_loc = true;
          Serial.println("FIND_LOC");
        }

        phi_set = phi;
        phi_dot_set = 0;
        rho_dot_set = 0;

        run_phi_controller = false;
        run_phi_dot_controller = false;
        run_rho_dot_controller = false;
      }
      break;

    case ALIGN: // Align robot direction with marker to travel to
      if (abs(marker_location_absolute[0] - phi) <= 0.02) {
        current_state = APPROACH;

        run_phi_controller = false;
        run_phi_dot_controller = false;
        run_rho_dot_controller = false;

        phi_set = phi;
        phi_dot_set = 0;
        rho_set = marker_location_absolute[1];
        rho_dot_set = 0;

        startTime = millis();

      } else {
        run_phi_controller = true;
        run_phi_dot_controller = true;
        run_rho_dot_controller = false;
      }
      break;

    case APPROACH: // Travel straight forward, length determined by distance from marker
      if (currentTime - startTime >= rho_set * 1825) {
        num_markers_traversed++;

        if (num_markers_traversed < num_markers_total) {
          current_state = ROT_90_CW;
        } else {
          current_state = STOP;
        }

        run_phi_controller = false;
        run_phi_dot_controller = false;
        run_rho_dot_controller = false;

        phi_set = marker_location_absolute[0] - (PI / 2);
        phi_dot_set = 0;
        rho_set = 0;
        rho_dot_set = 0;

        startTime = millis();

      } else {
        phi_set = phi;
        rho_dot_set = 0.5;

        run_phi_controller = true;
        run_phi_dot_controller = true;
        run_rho_dot_controller = true;
      }
      break;

    case ROT_90_CW: // Rotate 90 deg cw
      if (abs(phi_set - phi) <= 0.02) {
        current_state = SEARCH_NEXT;

        run_phi_controller = false;
        run_phi_dot_controller = false;
        run_rho_dot_controller = false;

        phi_set = phi;
        phi_dot_set = 0;
        rho_dot_set = 0;

        startTime = millis();

      } else {
        phi_set = marker_location_absolute[0] - (PI / 2);
        
        run_phi_controller = true;
        run_phi_dot_controller = true;
        run_rho_dot_controller = false;
      }
      break;

    case SEARCH_NEXT: // Drive slowly in a circle until the next marker is found
      if (found_marker) {
        current_state = LOCATE_MARKER;

        find_first = false;
        found_marker = false;

        run_phi_controller = false;
        run_phi_dot_controller = false;
        run_rho_dot_controller = false;

        phi_set = phi;
        phi_dot_set = 0;
        rho_dot_set = 0;

        startTime = millis();
      } else {
        if (!find_first) {
          find_first = true;
          Serial.println("FIND_FIRST");
        }

        phi_dot_set = 0.4;
        rho_dot_set = 0.15;

        run_phi_controller = false;
        run_phi_dot_controller = true;
        run_rho_dot_controller = true;
      }

      break;

    case STOP: // Stop the robot
      Serial.println("STOP");
      digitalWrite(M_ENABLE, LOW);
      phi_set = phi;
      phi_dot_set = 0;
      rho_dot_set = 0;
      run_phi_controller = false;
      run_phi_dot_controller = false;
      run_rho_dot_controller = false;
      break;
  }
}

// TMR0 interrupt service routine
SIGNAL(TIMER0_COMPA_vect) {
  Va = 0;
  delta_Va = 0;
  readEncoders();                                   // Get current position
  if (run_phi_controller) phi_controller();         // Calculate necessary angular speed
  if (run_phi_dot_controller) phi_dot_controller(); // Calculate deltaVa
  if (run_rho_dot_controller) rho_dot_controller(); // Calcualte Va
  setMotorVals();                                   // Scale ouput for PWM
}

// Record current position and velocity of the wheels
void readEncoders() {
  theta_1_previous = theta_1;   // Store previous value
  theta_2_previous = theta_2;   // Store previous value

  theta_1 = -2.0 * PI * enc1.read() / COUNTS_PER_REVOLUTION;  // Store position in radians
  theta_2 = 2.0 * PI * enc2.read() / COUNTS_PER_REVOLUTION;   // Store position in radians

  theta_dot_1 = (theta_1 - theta_1_previous) * 1000;          // Calculate angular velocity
  theta_dot_2 = (theta_2 - theta_2_previous) * 1000;          // Calculate angular velocity

  phi = - r * (theta_1 - theta_2) / d;                        // Calculate current angle
  phi_dot = - r * (theta_dot_1 - theta_dot_2) / d;            // Calculate angular velocity
  rho_dot = r * (theta_dot_1 + theta_dot_2) / 2.0;            // Calculate linear velocity
}

// Apply feedback control to calculate necessary angular velocity
void phi_controller() {
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

void dataRead() { // pass by reference marker_phi & phi_set
  String pi_data = Serial.readStringUntil('\n');
  int len = pi_data.length();
  int cmd = 2 * ((int) pi_data[0] - 48) + ((int) pi_data[1] - 48);

  switch (cmd) {
    case 0: 
      current_state = STOP;
      break;
    case 1:
      
      break;
    case 2: 
      found_marker = true;
      break;
    case 3: 
      found_marker = true;
      int angle_loc = pi_data.indexOf('a');
      int distance_loc = pi_data.indexOf('d');
      String angle_str = pi_data.substring(angle_loc + 1, distance_loc);
      String distance_str = pi_data.substring(distance_loc + 1, len);
      marker_location_relative[0] = angle_str.toFloat();
      marker_location_relative[1] = distance_str.toFloat();
      marker_location_absolute[0] = marker_location_relative[0] + phi;
      marker_location_absolute[1] = marker_location_relative[1];
      break;
  }  
}
