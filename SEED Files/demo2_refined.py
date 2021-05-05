# Demo 2

#Imports for Camera
from picamera import PiCamera
from picamera.array import PiRGBArray
from time import sleep
import cv2
import numpy as np
import cv2.aruco as aruco
import math
from math import log10, floor

#communications imports
import serial

#LCD Imports
import board
import busio
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd

if __name__ == '__main__': ## set up serial communication
    ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
    ser.flush()

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

#LCD Initialization
# Modify this if you have a different sized Character LCD
lcd_columns = 16
lcd_rows = 2
# Initialise I2C bus.
i2c = busio.I2C(board.SCL, board.SDA)
# Initialise the LCD class
lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)
lcd.clear()
lcd.color = [100, 0, 0]
lcd.message = "SEED\nGroup 10"



def printAngle(ang, mID):
    lcdMes = "ID: "
    temp = str(mID)
    lcdMes = lcdMes + temp
    lcdMes = lcdMes + "\n"
    angStr = "Angle: "
    temp = str(ang)
    angStr = angStr + temp
    lcdMes = lcdMes + angStr
    return lcdMes


# Camera Setup
camera = PiCamera()
camera.rotation = 180
rawCapture = PiRGBArray(camera)
width = 1920
height = 1088
fov = 29
camera.resolution = (width,height)
camera.framerate = 45
awb_mode = 'off'
camera.iso = 125
camera.brightness = 80
aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)


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
    camera.capture(rawCapture, format = "bgr")
    image = rawCapture.array
    #camera.stop_preview()   # For testing
    rawCapture.truncate(0)

    # Detecting Aruco
    resize = cv2.resize(image, None, fx=1, fy=1, interpolation = cv2.INTER_LINEAR)
    grayImage = cv2.cvtColor(resize, cv2.COLOR_BGR2GRAY)
    parameters = aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = aruco.detectMarkers(grayImage, aruco_dict, parameters = parameters)

    if len(corners) == 0:
        print("NO ArUco marker detected") #sees if there is no aruco marker present and prints
        
    else:

        for markers in corners: 
            for marker in markers:
                for corner in marker:
                    position_x += corner[0]
                    position_y += corner[1]

                    
                position_x /= 4
                position_y /= 4
                
        y1 = corners[0][0][0][0]
        y2 = corners[0][0][2][0]

        y3 = corners[0][0][1][0]
        y4 = corners[0][0][3][0]
        imgHeight = (abs(y1-y2)+abs(y3-y4))/2
        #print("Image Height: ")
        print(imgHeight)
        
        position_center = position_x - (width/2) 
        position_angle = (((position_center/(width/2))*30)) 
        position_radians = position_angle*(math.pi/180)
        #print("\n\n\n")
        #print("Degrees: ", position_angle)
        #print("Radians: ",position_radians)
        #print("\n\n\n")

        distance1 = 1/((imgHeight)/326) #93.5
        

        print("ArUco Marker Detected") #outputs marker angle and name
        print("ArUco ID:",ids[0])         
        #print("Angle: ", position_angle)
        
        #new
        angle_round = round_sig(position_radians)
        distance_round = round_sig(distance1)
        print("Angle:(Radians) ", angle_round)
        print("Distance:(Meters) ", distance_round)
        sendData(angle_round, distance_round, ids[0])
        #grayImage = aruco.drawDetectedMarkers(grayImage, corners)
        #cv2.imshow('test', grayImage)
        #lcd.message = printAngle(angle_round, ids[0])
        #sendData(angle_round, distance_round, ids[0])
        
  # cv2.waitKey(0)  
   #cv2.destroyAllWindows()
        


# Exit Image - for testing
#    cv2.waitKey(0)
#  cv2.destroyAllWindows()
