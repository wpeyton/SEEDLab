# Open Loop Experiments

**This subfolder contains:**

**robot_open_loop_step_response_experiment.ino**

    This arduino code does the following: 
        - set motors to half power (if bool is rotating = true, motors spin in opposite directions)
        - report encoder positions to the serial monitor
        - stop after 3 seconds

**RobotOpenLoopResponse.xlsx**
    
    This excel file contains the collected data and calculates the angular velocity and linear velocity. 
    These are plotted to determine system model parameters.
