# MiniProject

**This folder contains:**

**mini_project.py**

    Python source code to run on Raspberry Pi which:
        - Gets set value from shell
        - Communicates state via I2C with arduino
        - Gets current wheel position via I2C from arduino
        - Displays current position of the wheel and set position on LCD via I2C
        
**mini_project.ino**
     
     C++ source code to run on Arduino Uno which: 
        - Reads in set value over I2C from Pi
        - Reads current position
        - Applies positional PI controller algorithm
        - Sets motor speed/direction
        - Sends current position value over I2C to Pi

**open_loop_step_response.ino**

    C++ source code to run on Arduino Uno which:
        - Sets motor on
        - Outputs encoder position and timestamp to serial monitor every 5 ms until position hits 10 rad

**closed_loop_step_response.ino**

    C++ source code to run on Arduino Uno which:
        - Sets desired position to 1.00 rad
        - Reads current position
        - Applies positional PI controller algorithm
        - Sets motor speed/direction
        - Outputs encoder position and timestamp to serial monitor every 5 ms for 5 sec
