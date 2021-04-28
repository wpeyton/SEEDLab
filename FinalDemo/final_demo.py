import serial

#Imports for Camera
from time import sleep
import cv2
import numpy as np
import cv2.aruco as aruco
import math
from math import log10, floor

# markerOrder = [0, 1, 2]
# targetMarker = markerOrder[0]
targetMarker = 0

ardString = ""

# Camera Setup
##camera = PiCamera()
##camera.rotation = 180
##rawCapture = PiRGBArray(camera)
width = 1920
height = 1088
##fov = 29
##camera.resolution = (width,height)
##camera.framerate = 22
##awb_mode = 'off'
##camera.iso = 100
##camera.brightness = 75

cap = cv2.VideoCapture(0)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
# cap.set(cv2.CAP_PROP_BRIGHTNESS, 0.8)
cap.set(cv2.CAP_PROP_CONTRAST, 0.9)
cap.set(cv2.CAP_PROP_FPS, 60)

aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_100) #updated


if __name__ == '__main__': ## set up serial communication
    ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
    ser.flush()

def sendCmd(cmd):
    data = cmd
    #data = data + "a"
    #angleStr = str(angle)
    #distanceStr = str(distance)
    #data = data + angleStr + "d"
    #data = data + distanceStr
    #print(data)
    x = data + "\n" #encodes info so that you can send a list
    ser.write((x).encode('utf-8'))
    return None

def sendData(cmd, angle, distance):
    data = cmd
    data = data + "a"
    angleStr = str(angle)
    distanceStr = str(distance)
    data = data + angleStr + "d"
    data = data + distanceStr
    print(data)
    x = data + "\n" #encodes info so that you can send a list
    ser.write((x).encode('utf-8'))
    return None

def recData():
    line = ser.readline().rstrip()
    line = line.decode('utf-8')
    print("Arduino Sent: ",line)
    return line

def round_sig(x, sig=5):
    return round(x, sig-int(floor(log10(abs(x))))-1)

while(True):

    if ser.in_waiting > 0: 
        ardString = recData()

    
    # Reset Variables
    position_x = 0
    position_y = 0
    position_center = 0
    position_angle = 0
    position_radians = 0 
    distance1 = 0
    distance2 = 0 

    if (ardString.find("FIND_FIRST") != -1):
        print("Entering Find First Loop")
        exitFlag = False
        while(exitFlag == False):
            # Capturing Image
            #print("Capturing Image")
##            camera.capture(rawCapture, format = "bgr")
##            image = rawCapture.array
##            rawCapture.truncate(0)
            
            ret, frame = cap.read()

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            #cv2.imshow('frame', gray)
            #if(cv2.waitKey(1) & 0xFF == ord('q')):
            #   break
                       

            # Detecting Aruco
#            resize = cv2.resize(image, None, fx=1, fy=1, interpolation = cv2.INTER_LINEAR)
#            grayImage = cv2.cvtColor(resize, cv2.COLOR_BGR2GRAY)
            parameters = aruco.DetectorParameters_create()
#            corners, ids, rejectedImgPoints = aruco.detectMarkers(grayImage, aruco_dict, parameters = parameters)
            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters = parameters)
            if len(corners) != 0:
                #print("Detected at least one Marker")
                x = 0
                while x < len(ids): 
                    if (ids[x][0] == targetMarker): # This looks for the target marker and then decides if it's in the list
                        sendCmd("10")
                        ardString = ""
                        print("Detected Marker with Correct ID")
                        exitFlag = True
                        print("Exiting Find First Loop")
                        break
                    x = x + 1

    if (ardString.find("FIND_LOC") != -1):
        exitFlag = False
        print("Entering Find Loc Loop")
        while(exitFlag == False):
            # Capturing Image
            #camera.start_preview(alpha = 200)   # For testing
            #print("Capturing Image")
##            camera.capture(rawCapture, format = "bgr")
##            image = rawCapture.array
##            #camera.stop_preview()   # For testing
##            rawCapture.truncate(0)

            ret, frame = cap.read()

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            #cv2.imshow('frame', gray)
            #if(cv2.waitKey(1) & 0xFF == ord('q')):
            #   break

            # Detecting Aruco
##            resize = cv2.resize(image, None, fx=1, fy=1, interpolation = cv2.INTER_LINEAR)
##            grayImage = cv2.cvtColor(resize, cv2.COLOR_BGR2GRAY)
            parameters = aruco.DetectorParameters_create()
##            corners, ids, rejectedImgPoints = aruco.detectMarkers(grayImage, aruco_dict, parameters = parameters)
            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters = parameters)
               
            if len(corners) == 0:
                print("NO Aruco marker detected") #sees if there is no aruco marker present and prints
                sendCmd("01")
                #CORNERS: [order in image (left to right)][?][Corner number, clockwise rotation starting @ top left][?]
            
            else:
                print("Detected at least one Marker")
                x = 0
                print("IDs: ", ids)
                while x < len(ids): 
                    if (ids[x][0] == targetMarker): # This looks for the target marker and then decides if it's in the list
                        print("Detected Marker with Correct ID")
                        y1 = corners[x][0][0][1]
                        y2 = corners[x][0][2][1]
                        y3 = corners[x][0][1][1]
                        y4 = corners[x][0][3][1]
                        imgHeight = (abs(y1-y2)+abs(y3-y4))/2
                        print("ID: ", ids[x][0])
                        # print("Height", imgHeight)
                        position_x = (corners[x][0][0][0])/4
                        position_x += (corners[x][0][1][0])/4
                        position_x += (corners[x][0][2][0])/4
                        position_x += (corners[x][0][3][0])/4
                        
                        position_center = position_x - (width/2) 
                        position_angle = (((position_center/(width/2))*30)) 
                        position_radians = position_angle*(math.pi/180)
                        print("Angle: ", position_radians)
                        distance1 = 1/(imgHeight/188)

                        #new ******************************
                        angle_round = round_sig(position_radians)
                        distance_round = round_sig(distance1)
                        #print("Angle:(Radians) ", angle_round)
                        print("Distance:(Meters) ", distance_round)
                        sendData("11", angle_round, distance_round)
                        exitFlag = True
                        print("Exiting Find Loc Loop")
                        break
                    x = x + 1

            
                ardString = ""
                # increment marker target
                # markerOrder.pop(0)
                # targetMarker = markerOrder[0]
                targetMarker = targetMarker + 1
                print("New Target: ", targetMarker)
                #print("Aruco ID[0]:", ids)
                #print("Size", len(ids))

            



            # LOOK IN IMAGE FOR MARKERS-
            # WHEN MARKERS ARE FOUND, CHECK TO SEE IF THE TARGET MARKER IS IN THE
            # LIST.
            # IF NOT, KEEP LOOKING. DO NOT SEND MARKER DATA IF IT IS NOT THE
            # TARGET MARKER.
            # CONTINUE TO ONLY TRANSMIT DATA ABOUT TARGET MARKER UNTIL MARKER IS REACHED
            # ARDUINO WILL SEND STATUS MESSAGES BACK TO THE PI.
            # ONCE THAT MARKER IS REACHED- ITERATE THE TARGET MARKER AND SEND THAT ONE
            # TO THE ARDUINO. CONTINUE PROCESS UNTIL WE FINISH THE LOOP. 
        
cap.release()
cv2.destroyAllWindows()





