# Arduino

**This subfolder contains:**

**Demo1.ino**

    Final arduino code used for Demo 1: 
        - to set the distance/angle, modify: 
               #define NUM_FT_TO_MOVE 1
               #define NUM_DEG_TO_TURN 90
        - The program will rotate the bot the number fo degrees specified, then move forward the number of feet specified.
        - Implements linear velocity controller and angular position controller (with inner loop angular velocity controller)
        - Uses TMR0 interrupt (~1ms)

**tran_speed_controller.ino**
    
    Individual program for testing only the translational speed controller
        - Sets rho_dot_set
        - Applies feedback control algorithm (PI) to determine necessary pwm outputs for motor drivers
        - Uses TMR0 interrupt (~1ms)

**rot_speed_controller.ino**

    Individual program for testing only the rotational speed controller
        - Sets phi_dot_set
        - Applies feedback control algorithm (PI) to determine necessary pwm outputs for motor drivers
        - Uses TMR0 interrupt (~1ms)

**rot_pos_controller**

    Individual program for testing only the rotational position controller
        - Sets phi_set
        - Applies feedback control algorithm (PID) to determine necessary pwm outputs for motor drivers
        - Implements inner loop angular velocity controller found in rot_speed_controller.ino
        - Uses TMR0 interrupt (~1ms)

**arduino_serial_comms.ino**

    Test program for passing data back and forth between Pi and Arduino
    Not integrated for this demo
    
