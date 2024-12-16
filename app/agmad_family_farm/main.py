# Distributed with a free-will license.
# Use it any way you want, profit or free, provided it fits in the licenses of its associated works.
# TSL2561
# This code is designed to work with the TSL2561_I2CS I2C Mini Module available from ControlEverything.com.
# https://www.controleverything.com/content/Light?sku=TSL2561_I2CS#tabs-0-product_tabset-2
import RPi.GPIO as GPIO
import smbus
import time
import adafruit_dht
import board
import digitalio
from board import SCLK, MISO, MOSI, CE0
import busio
import adafruit_vl53l0x
from adafruit_mcp3xxx.mcp3008 import MCP3008
from adafruit_mcp3xxx.analog_in import AnalogIn

i2c = busio.I2C(board.SCL, board.SDA)
#vl53 = adafruit_vl53l0x.VL53L0X(i2c)

PH_SENSOR_CHANNEL = 1 # ADC channel for pH sensor

WATER_LEVEL1_PIN = 20 # GPIO pin for Digital Water Level Sensor
WATER_LEVEL2_PIN = 21 # GPIO pin for Digital Water Level Sensor
GPIO.setmode(GPIO.BCM)

spi = busio.SPI(clock=SCLK, MISO=MISO, MOSI=MOSI)
cs = digitalio.DigitalInOut(CE0)
mcp = MCP3008(spi, cs)

def read_water_level():
    GPIO.setup(WATER_LEVEL1_PIN, GPIO.IN)
    GPIO.setup(WATER_LEVEL2_PIN, GPIO.IN)
    if GPIO.input(WATER_LEVEL1_PIN) == GPIO.HIGH:
        print("Water Level Sensor1: Water detected!")
    else:
        print("Water Level Sensor1: No water detected.")
    print("****************")
    if GPIO.input(WATER_LEVEL2_PIN) == GPIO.HIGH:
        print("Water Level Sensor2: Water detected!")
    else:
        print("Water Level Sensor2: No water detected.")
    print("-------------------------------------------------------------------------------------------------")

def read_adc(channel):
    chan = AnalogIn(mcp, channel)
    return chan.value

# Read pH value
def read_ph():
    raw_value = read_adc(PH_SENSOR_CHANNEL)
    ph_value = raw_value * (14 / 65535)  # Adjust for 10-bit (65535 is for 16-bit, adjust based on your ADC)
    print(f"pH Sensor: pH={ph_value:.2f}")
    print("-------------------------------------------------------------------------------------------------")

    return ph_value

def read_dht():
    # First DHT22 sensor
    dhtDevice1 = adafruit_dht.DHT22(board.D5)
    temperature_c = dhtDevice1.temperature
    humidity = dhtDevice1.humidity
    print("First DHT:")
    print(f"Temperature: {temperature_c:.1f}°C")
    print(f"Humidity: {humidity:.1f}%")
    print("*****************")
    # second DHT22 sensor
    dhtDevice2 = adafruit_dht.DHT22(board.D13)
    temperature_c = dhtDevice2.temperature
    humidity = dhtDevice2.humidity
    print("Second DHT:")
    print(f"Temperature: {temperature_c:.1f}°C")
    print(f"Humidity: {humidity:.1f}%")
    print("*****************")
    # DHT11 sensor
    dhtDevice3 = adafruit_dht.DHT11(board.D0)
    temperature_c = dhtDevice3.temperature
    humidity = dhtDevice3.humidity
    print("DHT11:")
    print(f"Temperature: {temperature_c:.1f}°C")
    print(f"Humidity: {humidity:.1f}%")
    print("-------------------------------------------------------------------------------------------------")

def read_laser():
    print("Laser Data:")
    print("Range: {0}mm".format(vl53.range))
    print("-----------------------------------------------------------------")


def read_light():
    # Get I2C bus
    bus = smbus.SMBus(6)

    # TSL2561 address, 0x39(57)
    # Select control register, 0x00(00) with command register, 0x80(128)
    #		0x03(03)	Power ON mode
    bus.write_byte_data(0x39, 0x00 | 0x80, 0x03)
    # TSL2561 address, 0x39(57)
    # Select timing register, 0x01(01) with command register, 0x80(128)
    #		0x02(02)	Nominal integration time = 402ms
    bus.write_byte_data(0x39, 0x01 | 0x80, 0x02)

    time.sleep(0.1)


    # Read data back from 0x0C(12) with command register, 0x80(128), 2 bytes
    # ch0 LSB, ch0 MSB
    data = bus.read_i2c_block_data(0x39, 0x0C | 0x80, 2)

    # Read data back from 0x0E(14) with command register, 0x80(128), 2 bytes
    # ch1 LSB, ch1 MSB
    data1 = bus.read_i2c_block_data(0x39, 0x0E | 0x80, 2)

    # Convert the data
    ch0 = data[1] * 256 + data[0]
    ch1 = data1[1] * 256 + data1[0]
    #--------------------------------------------------------------
    bus2 = smbus.SMBus(5)

    # TSL2561 address, 0x39(57)
    # Select control register, 0x00(00) with command register, 0x80(128)
    #		0x03(03)	Power ON mode
    bus2.write_byte_data(0x39, 0x00 | 0x80, 0x03)
    # TSL2561 address, 0x39(57)
    # Select timing register, 0x01(01) with command register, 0x80(128)
    #		0x02(02)	Nominal integration time = 402ms
    bus2.write_byte_data(0x39, 0x01 | 0x80, 0x02)

    time.sleep(0.1)

    # Read data back from 0x0C(12) with command register, 0x80(128), 2 bytes
    # ch0 LSB, ch0 MSB
    data2 = bus2.read_i2c_block_data(0x39, 0x0C | 0x80, 2)

    # Read data back from 0x0E(14) with command register, 0x80(128), 2 bytes
    # ch1 LSB, ch1 MSB
    data3 = bus2.read_i2c_block_data(0x39, 0x0E | 0x80, 2)

    # Convert the data
    ch2 = data2[1] * 256 + data2[0]
    ch3 = data3[1] * 256 + data3[0]
    # Output data to screen
    print ("Semsor 1 Data")
    print ("Full Spectrum(IR + Visible) :%d lux" %ch0)
    print ("Infrared Value :%d lux" %ch1)
    print ("Visible Value :%d lux" %(ch0 - ch1))
    print("*****************")
    #------------------------------------------------------------------
    print ("Semsor 2 Data")
    print ("Full Spectrum(IR + Visible) :%d lux" %ch2)
    print ("Infrared Value :%d lux" %ch3)
    print ("Visible Value :%d lux" %(ch2 - ch3))
    print("-----------------------------------------------------------------")
try:
    while True:
        print("\nReading sensors...")
        
        # Read DHT sensor
        read_dht()
        
        # Read Hall Effect sensor
        #read_hall()
        
        # Read Light sensor
        #read_light()
        
        # Read Laser Sensor
        #read_laser()

        # Read pH sensor
        read_ph()
        
        # Read temperature sensor
        #read_temperature()
        
        # Read water level sensor
        #read_water_level()
        
        time.sleep(4)  # Delay between readings

except KeyboardInterrupt:
    print("Exiting...")
finally:
    GPIO.cleanup()