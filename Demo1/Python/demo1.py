# Demo 1

#Imports for Camera
from picamera import PiCamera
from picamera.array import PiRGBArray
from time import sleep
import cv2
import numpy as np
import cv2.aruco as aruco
#LCD Imports
import board
import busio
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd

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
camera.framerate = 30
awb_mode = 'off'
camera.iso = 125



while(True):

    # Reset Variables
    position_x = 0
    position_y = 0
    position_center = 0
    position_angle = 0

    # Capturing Image
    camera.start_preview(alpha = 200)   # For testing
    sleep(2)    
    camera.capture(rawCapture, format = "bgr")
    image = rawCapture.array
    camera.stop_preview()   # For testing
    rawCapture.truncate(0)

    # Detecting Aruco
    resize = cv2.resize(image, None, fx=.5, fy=.5, interpolation = cv2.INTER_LINEAR)
    grayImage = cv2.cvtColor(resize, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
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

        position_center = position_x - (width/2) #calculates the angle to the marker
        position_angle = (position_center/(width/2))*fov

        
        print("ArUco Marker Detected") #outputs marker angle and name
        lcd.message = "ArUco Marker\nDetected"
        sleep(.1)
        print("ArUco ID: ",ids)         
        print("Angle: ", position_angle)
        lcd.message = printAngle(position_angle, ids)
        
        grayImage = aruco.drawDetectedMarkers(grayImage, corners)
        cv2.imshow('test', grayImage)
        
 #   cv2.waitKey(0) 
#    cv2.destroyAllWindows()
        


    # Exit Image - for testing
#    cv2.waitKey(0)
  #  cv2.destroyAllWindows()


