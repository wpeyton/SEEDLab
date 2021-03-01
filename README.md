# SEEDLabMiniProject

# **This repo contains:**

# **mini_project.py**

#     Python source code to run on the Pi which:
#         - Gets set value from shell
#         - Communicates state via I2C with arduino
#         - Gets current wheel position via I2C from arduino
#         - Displays current position of the wheel and set position on LCD via I2C
        
# **mini_project.ino**

#     C++ source code to run on Arduino uno which: 
#         - Reads in set value over I2C from Pi
#         - Reads current position
#         - Applies positional PI controller algorithm
#         - Sets motor speed/direction
#         - Sends current position value over I2C to Pi
