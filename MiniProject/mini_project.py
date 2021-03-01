import smbus
import time
import board
import busio
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd


# for RPI version 1, use “bus = smbus.SMBus(0)”
bus = smbus.SMBus(1)
# Initialise I2C bus.
i2c = busio.I2C(board.SCL, board.SDA)
# This is the address we setup in the Arduino Program
address = 0x04
#initialize LCD
# Modify this if you have a different sized Character LCD
lcd_columns = 16
lcd_rows = 2

lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)

j = 0;


wheelPosStr = ""
wheelTargetPos = 0
wheelPos = 0
nextPos = 0

def readPos():
    #include code to parse block of data
    time.sleep(0.2)
    arr = bus.read_i2c_block_data(address ,0, 3)
    x = ""
    x = x + str(arr[0]) + "." + str(arr[1]) + str(arr[2])
    #print(x)
    currPos = x
    return currPos

def wheelTarget():
    targetPos = input("input next wheel pos from 1-4: ")
    #print(targetPos)
    return targetPos

def sendTarget(nextPos):
    bus.write_byte(address, nextPos)
    return nextPos

def lcdReadout(current, target):
    lcdStr = "Pos: " + str(current) + "\nTarget:" + target
    return lcdStr

wheelTargetPosChr = wheelTarget()
wheelTargetPos = int(wheelTargetPosChr)
print(wheelTargetPos)
    #set outside of main loop because right now we don't have the camera
    #telling us what we need to output

while True:
    try:
        
        time.sleep(.1)
        sendTarget(wheelTargetPos)
        wheelPos = readPos()
        #wheelTargetPos = wheelTarget()
        lcd.message = lcdReadout(wheelPos, wheelTargetPosChr)

        #wheelTargetPosChr = wheelTarget()
        #wheelTargetPos = int(wheelTargetPosChr)
        #print(wheelTargetPos)
        
    except OSError:
        print("I2C Error. Reconnect SLA pin")
