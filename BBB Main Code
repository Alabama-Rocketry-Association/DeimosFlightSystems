import smbus
import time
import adafruit_bmp3xx
import digitalio
import busio
import adafruit_bno055
#import adafruit_gps
#import Adafruit_BBIO.UART as UART
import board
import serial
from time import sleep
from time import sleep
import Adafruit_BBIO.GPIO as GPIO


relay = "P9_13"
GPIO.setup(relay,GPIO.OUT)
GPIO.output(relay, GPIO.LOW)

dummy = 0
#initialize Serial ports and set GPS output
#UART.setup("UART1")
#uart = serial.Serial(port = '/dev/ttyO1', baudrate=9600, timeout = 1000)
#gps = adafruit_gps.GPS(uart, debug=False)
#gps.send_command(b'PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
#gps.send_command(b'PMTK220,1000')
class multiplex:
    	#Multiplexer functions
    	def __init__(self, bus):
            	self.bus = smbus.SMBus(bus)
    	def channel(self, address=0x70,channel=0):
                if   (channel==0): action = 0x01
                elif (channel==1): action = 0x02
                elif (channel==2): action = 0x04
                elif (channel==3): action = 0x08
                elif (channel==4): action = 0x10
                elif (channel==5): action = 0x20
                elif (channel==6): action = 0x40
                elif (channel==7): action = 0x80
                self.bus.write_byte_data(address,0x04,action) #0x04 is the regi$)
if __name__ == '__main__':
        bus = 2
        address = 0x70
        i2c = busio.I2C('I2C2_SCL', 'I2C2_SDA')       
        while True:
    	#BMP sensor channel 0. Make sure to set local pressure for correct altitude
                bus = 2
                address = 0x70
                plexer = multiplex(bus)
                plexer.channel(address,0)
                bmp = adafruit_bmp3xx.BMP3XX_I2C(i2c)
                bmp.sea_level_pressure = 1013.5
                print('\n')
                print("Pressure: {:6.1f}".format(bmp.pressure))
                print("Temperature: {:5.2f}".format(bmp.temperature))
                print('Altitude: {} meters'.format(bmp.altitude))
        # Parachute Code. Deploys at around 213.36 meters = 700 feet       
                if (bmp.altitude > 213.36): 
                        dummy = 1
                        sleep(1)
                if (bmp.altitude < 213.36 and dummy == 1):
                        GPIO.output(relay, GPIO.HIGH)
                        sleep(2.5)
                        GPIO.output(relay,GPIO.LOW)
                sleep(0.1)
    	#BNO 055 IMU channel 1
                plexer.channel(address,1)
                sensor = adafruit_bno055.BNO055(i2c)
                print('\n')
                print('Accelerometer (m/s^2): {}'.format(sensor.acceleration))
                print('Magnetometer (microteslas): {}'.format(sensor.magnetic))
                sleep(0.1)
                
        
                        
#                print('Gyroscope (rad/sec): {}'.format(sensor.gyro))
#                print('Euler angle: {}'.format(sensor.euler))
#                print('Quaternion: {}'.format(sensor.quaternion))
#                print('Linear acceleration{}'.format(sensor.linear_acceleration))
#                print('Gravity (m/s^2): {}'.format(sensor.gravity))
#                print()
#                time.sleep(0.5)
    	#Adafuit V3 GPS over UART1
#           	uart = serial.Serial(port = '/dev/ttyO1', baudrate=9600, timeout=3000)
#           	gps = adafruit_gps.GPS(uart, debug=False)
            	#Set up GPS outputs (default selected)
#           	gps.send_command(b'PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
            	#Set update rate (1 second)
#           	gps.send_command(b'PMTK220,3000')
#            	gps.update()
#           	#Check for a fix
#            	if not gps.has_fix:
#                    	print('Waiting for fix...')
#                       	print()
#                    	continue
#            	print('Latitude: {0:.6f} degrees'.format(gps.latitude))
#            	print('Longitude: {0:.6f} degrees'.format(gps.longitude))
#            	print()
