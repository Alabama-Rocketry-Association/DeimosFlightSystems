import asyncio
from zoom9 import z9c, init_coroutine_methods
import smbus
import time
import adafruit_bmp3xx
import digitalio
import busio
import adafruit_bno055
# import adafruit_gps
# import Adafruit_BBIO.UART as UART
import board
import serial
from time import sleep
from time import sleep
import Adafruit_BBIO.GPIO as GPIO

global parVar
global radio

# initialize Serial ports and set GPS output
# UART.setup("UART1")
# uart = serial.Serial(port = '/dev/ttyO1', baudrate=9600, timeout = 1000)
# gps = adafruit_gps.GPS(uart, debug=False)
# gps.send_command(b'PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
# gps.send_command(b'PMTK220,1000')
class multiplex:
    # Multiplexer functions
    def __init__(self, bus):
        self.bus = smbus.SMBus(bus)

    def channel(self, address=0x70, channel=0):
        if (channel == 0):
            action = 0x01
        elif (channel == 1):
            action = 0x02
        elif (channel == 2):
            action = 0x04
        elif (channel == 3):
            action = 0x08
        elif (channel == 4):
            action = 0x10
        elif (channel == 5):
            action = 0x20
        elif (channel == 6):
            action = 0x40
        elif (channel == 7):
            action = 0x80
        self.bus.write_byte_data(address, 0x04, action)  # 0x04 is the regi$)

async def dataloop():
    #insert global variables into local context for ease of asynchronous communication between function loops
    global radio
    global parVar
    bus = 2
    address = 0x70
    i2c = busio.I2C('I2C2_SCL', 'I2C2_SDA')
    # BMP sensor channel 0. Make sure to set local pressure for correct altitude
    bus = 2
    address = 0x70
    plexer = multiplex(bus)
    plexer.channel(address, 0)
    bmp = adafruit_bmp3xx.BMP3XX_I2C(i2c)
    bmp.sea_level_pressure = 1013.5
    # BNO 055 IMU channel 1
    plexer.channel(address, 1)
    sensor = adafruit_bno055.BNO055(i2c)
    while True:
        data = {
            "P": bmp.pressure,
            "T": bmp.temperature,
            "ALT": bmp.altitude,
            "ACC": sensor.acceleration,
            "MAG": sensor.magnetic
        }
        # Parachute Code. Deploys at around 213.36 meters = 700 feet
        if (bmp.altitude > 213.36):
            parVar = 1
            sleep(1)
        if (bmp.altitude < 213.36 and parVar == 1):
            GPIO.output(relay, GPIO.HIGH)
            sleep(2.5)
            GPIO.output(relay, GPIO.LOW)
        if parVar == 2:
            #remote deploy
            GPIO.output(relay, GPIO.HIGH)
            sleep(2.5)
            GPIO.output(relay, GPIO.LOW)
        radio.to_send(data=data)
        await asyncio.sleep(0.1)

async def commands():
    global radio
    global parVar
    while 1:
        msg, status = radio.from_recv()
        if msg != None:
            if msg == "DEPLOY PARACHUTE":
                parVar = 2
        elif status == "FIN":
            """
            Should never be executed
            """
            pass
        await asyncio.sleep(1)

async def main(methods:list):
    await asyncio.gather(*[methods])

if __name__ == "__main__":
    radio = z9c(dev="/dev/ttyUSB0", baud=115200)
    relay = "P9_13"
    GPIO.setup(relay, GPIO.OUT)
    GPIO.output(relay, GPIO.LOW)
    parVar = 0
    methods = [
        radio.auto_run(),
        dataloop(),
        commands()
    ]
    loop = asyncio.get_event_loop()
    loop.run_until_complete(main(methods=methods))






