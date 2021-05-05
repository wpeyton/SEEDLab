import smbus
import time
import board
import busio
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
import serial


### LCD INITIALIZATION ###
# for RPI version 1, use “bus = smbus.SMBus(0)”
#bus = smbus.SMBus(1)
# Initialise I2C bus.
#i2c = busio.I2C(board.SCL, board.SDA)
# This is the address we setup in the Arduino Program
#address = 0x04
#initialize LCD
# Modify this if you have a different sized Character LCD
#lcd_columns = 16
#lcd_rows = 2
#lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)
### LCD INITIALIZATION COMPLETE ###

### SERIAL COMMUNICATION INITIALIZATION ###

line = ""
i = -90

#pi will send an x coordinate and a y coordinate
# x= 0 y= 10
# x= 10 y = 0
# x = 10 y = 10

if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
    ser.flush()

def sendData(data):
    data = str(data)
    x = data + "\n" #encodes info so that you can send a list#
    ser.write((x).encode('utf-8'))
    return None

def recData():
    if ser.in_waiting > 0: #decodes data from arduino into a string
            line = ser.readline().rstrip()
            print(line)
    return None




while True:
    try:
        time.sleep(.1)
        sendData(i)
        recData()
        i = i + 1
        if( i >= 90 ): #tested
            i = -90
        
    except OSError:
        print("I2C Error. Reconnect SLA pin")
