**Serial Communication Dictionary**

**From Arduino to Pi:**

    - "FIND_FIRST\r\n": 
        - Pi should constantly search for any marker, and reply back as quickly as possible when one is found. 
        - Don't worry about calculating the position at this point, focus on running this as fast as possible so the robot can be stopped immediately when the marker is found.             - When marker is found, proper reply is "10\r\n"
    - "FIND_LOC\r\n": 
        -  Robot is currently stationary.
        -  Initialize a 6x3x2 matrix of floats to store data , with starting values of NaN
        -  Repeat the following three times: 
            -  For each marker in the image:
                -  Calculate the distance and angle
                -  Store the distance and angle in the matrix: 
                    -  matrix[markerId][iterationNo][0] = angle
                    -  matrix[markerId][iterationNo][1] = distance
        - For each marker, calculate the average dist and angle (store in another matrix) - if all values are NaN, save avg as NaN. If only some of the values are NaN, do not let these influence the avg.  
        -  Return the position of the marker that is furthest to the right in the image (most negative angle, or if none are negative smallest angle)
            -  Format "11aX.XXXdX.XXX\r\n" where X.XXX is replaced with the actual values. 
            -  If no markers were found (all NaNs), reply back "01\r\n". 
