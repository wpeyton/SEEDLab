# Demo 2

#Imports for Camera
from time import sleep
import cv2
import numpy as np
import cv2.aruco as aruco
import math
from math import log10, floor
import subprocess

#communications imports
import serial

imgScale = 107.5

width = 640
height = 480

aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_1000) 
###LCD Imports
##import board
##import busio
##import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd

if __name__ == '__main__': ## set up serial communication
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
    ser.flush()
print("Serial Established\n")

def sendData(angle, distance, mID):
    data = "a"
    angleStr = str(angle)
    distanceStr = str(distance)
    data = data + angleStr + "d"
    data = data + distanceStr
    print(data)
    x = data + "\n" #encodes info so that you can send a list
    ser.write((x).encode('utf-8'))
    return None

def recData():
    if ser.in_waiting > 0: #decodes data from arduino into a string
            line = ser.readline().rstrip()
            print(line)
    return None

def round_sig(x, sig=5):
    return round(x, sig-int(floor(log10(abs(x))))-1)



###LCD Initialization
### Modify this if you have a different sized Character LCD
##lcd_columns = 16
##lcd_rows = 2
### Initialise I2C bus.
##i2c = busio.I2C(board.SCL, board.SDA)
### Initialise the LCD class
##lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)
##lcd.clear()
##lcd.color = [100, 0, 0]
##lcd.message = "SEED\nGroup 10"



##def printAngle(ang, mID):
##    lcdMes = "ID: "
##    temp = str(mID)
##    lcdMes = lcdMes + temp
##    lcdMes = lcdMes + "\n"
##    angStr = "Angle: "
##    temp = str(ang)
##    angStr = angStr + temp
##    lcdMes = lcdMes + angStr
##    return lcdMes



## FIX ME!!!
cap = cv2.VideoCapture(0)
print("Video Started\n")
command = "v4l2-ctl -d 0 -c auto_exposure=1 -c exposure_time_absolute=200"
output = subprocess.call(command, shell=True)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
##cap.set(cv2.CAP_PROP_BRIGHTNESS, 0.6)
##cap.set(cv2.CAP_PROP_CONTRAST, 0.5)
##cap.set(cv2.CAP_PROP_FPS, 60)
##cap.set(cv2.CAP_PROP_EXPOSURE, .75)
###cap.set(cv2.CAP_MODE_GRAY,2) 

print("Setting Applied\n")




while(True):

    # Reset Variables
    position_x = 0
    position_y = 0
    position_center = 0
    position_angle = 0
    position_radians = 0 
    distance1 = 0
    distance2 = 0 

    # Capturing Image
    #camera.start_preview(alpha = 200)   # For testing  
    #camera.capture(rawCapture, format = "bgr")
    ret, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
    # Uncomment me to show the camera feed!
    # cv2.imshow('frame', gray)
    # if(cv2.waitKey(1) & 0xFF == ord('q')):
    # break
               
    # Instatiate Aruco Parameters
    parameters = aruco.DetectorParameters_create()
    
    # Detecting Aruco (Old Method)
    # resize = cv2.resize(image, None, fx=1, fy=1, interpolation = cv2.INTER_LINEAR)
    # grayImage = cv2.cvtColor(resize, cv2.COLOR_BGR2GRAY)
    # corners, ids, rejectedImgPoints = aruco.detectMarkers(grayImage, aruco_dict, parameters = parameters)
    
    # Detecting Aruco (New Method)
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters = parameters)
        
    if len(corners) == 0:
        print("NO ArUco marker detected") #sees if there is no aruco marker present and prints
        
    else:

        position_x += corners[0][0][0][0]
        position_x += corners[0][0][1][0]
        position_x += corners[0][0][2][0]
        position_x += corners[0][0][3][0]

        position_x /= 4
        
        y1 = corners[0][0][0][1]
        y2 = corners[0][0][2][1]
        y3 = corners[0][0][1][1]
        y4 = corners[0][0][3][1]
        
        imgHeight = (abs(y1-y2)+abs(y3-y4))/2
        print("Image Height: ")
        print(imgHeight)
        print("Position_x: ")
        print(position_x)
        position_center = position_x - (width/2)
        print("position_center: ")
        print(position_center)
        position_angle = (((position_center/(width/2))*30)) 
        position_radians = position_angle*(math.pi/180) * -1
        #print("\n\n\n")
        #print("Degrees: ", position_angle)
        #print("Radians: ",position_radians)
        #print("\n\n\n")
        distance1 = 1/((imgHeight)/imgScale) #93.5
        

        print("ArUco Marker Detected") #outputs marker angle and name
        print("ArUco ID:",ids[0])         
        #print("Angle: ", position_angle
            
                
        #new
        if(abs(position_radians) > 0.00001):
            angle_round = round_sig(position_radians)
        
        else:
            angle_round = 0.0;
        distance_round = round_sig(distance1)
        print("Angle:(Radians) ", angle_round)
        print("Distance:(Meters) ", distance_round)
        sendData(angle_round, distance_round, ids[0])

        sleep(0.01)
        #grayImage = aruco.drawDetectedMarkers(grayImage, corners)
        #cv2.imshow('test', grayImage)
        #lcd.message = printAngle(angle_round, ids[0])
        #sendData(angle_round, distance_round, ids[0])
        
   #cv2.waitKey(0)  
   #cv2.destroyAllWindows()
        


# Exit Image - for testing
#    cv2.waitKey(0)
#  cv2.destroyAllWindows()
