# import board
# import busio
# import adafruit_vl53l0x

# i2c = busio.I2C(board.SCL, board.SDA)

# try:
#     vl53 = adafruit_vl53l0x.VL53L0X(i2c)
#     print("Sensor initialized!")
#     print("Range: {}mm".format(vl53.range))
# except Exception as e:
#     print("Error:", e)

# import Adafruit_DHT as dht
# from time import sleep

# # Set the GPIO pin connected to the DHT sensor
# DHT_PIN = 4

# while True:
#     # Read temperature and humidity data from the DHT sensor
#     humidity, temperature = dht.read_retry(dht.DHT11, DHT_PIN)
    
#     # Print the temperature and humidity data
#     print('Temp={0:0.1f}*C  Humidity={1:0.1f}%'.format(temperature, humidity))
    
#     # Wait 5 seconds before reading again
#     sleep(5)

    # SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

# import time
# import board
# import adafruit_dht

# # Initial the dht device, with data pin connected to:
# dhtDevice = adafruit_dht.DHT22(board.D18)

# # you can pass DHT22 use_pulseio=False if you wouldn't like to use pulseio.
# # This may be necessary on a Linux single board computer like the Raspberry Pi,
# # but it will not work in CircuitPython.
# # dhtDevice = adafruit_dht.DHT22(board.D18, use_pulseio=False)

# while True:
#     try:
#         # Print the values to the serial port
#         temperature_c = dhtDevice.temperature
#         temperature_f = temperature_c * (9 / 5) + 32
#         humidity = dhtDevice.humidity
#         print(
#             "Temp: {:.1f} F / {:.1f} C    Humidity: {}% ".format(
#                 temperature_f, temperature_c, humidity
#             )
#         )

#     except RuntimeError as error:
#         # Errors happen fairly often, DHT's are hard to read, just keep going
#         print(error.args[0])
#         time.sleep(2.0)
#         continue
#     except Exception as error:
#         dhtDevice.exit()
#         raise error

#     time.sleep(2.0)
#-----------------------------------------------------------------
# import busio
# import digitalio
# import board
# import adafruit_mcp3xxx.mcp3008 as MCP
# from adafruit_mcp3xxx.analog_in import AnalogIn

# # create the spi bus
# spi = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI)

# # create the cs (chip select)
# cs = digitalio.DigitalInOut(board.D5)

# # create the mcp object
# mcp = MCP.MCP3008(spi, cs)

# # create an analog input channel on pin 0
# chan = AnalogIn(mcp, MCP.P0)

# print('Raw ADC Value: ', chan.value)
# print('ADC Voltage: ' + str(chan.voltage) + 'V')
#-----------------------------------------------------------------------------
# import adafruit_dht
# import board

# # Initialize the DHT22 sensor using the correct GPIO pin
# # Replace `board.D5` with the appropriate GPIO pin for your setup
# dhtDevice = adafruit_dht.DHT22(board.D5)

# try:
#     # Read temperature and humidity
#     temperature_c = dhtDevice.temperature
#     humidity = dhtDevice.humidity
#     print(f"Temperature: {temperature_c:.1f}°C")
#     print(f"Humidity: {humidity:.1f}%")
# except RuntimeError as error:
#     # Handle reading errors (common with DHT sensors)
#     print(f"Error reading DHT sensor: {error}")
#-------------------------------------------------------------------------------
# import smbus2
# import time

# # I2C address of the VL53L0X
# VL53L0X_I2C_ADDR = 0x39

# # Register addresses
# SYSRANGE_START = 0x00
# RESULT_RANGE_STATUS = 0x14

# # I2C bus (typically 1 for Raspberry Pi)
# I2C_BUS = 1

# # VL53L0X Initialization
# class VL53L0X:
#     def __init__(self, i2c_bus, address):
#         self.bus = smbus2.SMBus(i2c_bus)
#         self.address = address

#     def write_byte(self, reg, value):
#         self.bus.write_byte_data(self.address, reg, value)

#     def read_byte(self, reg):
#         return self.bus.read_byte_data(self.address, reg)

#     def read_word(self, reg):
#         data = self.bus.read_i2c_block_data(self.address, reg, 2)
#         return (data[0] << 8) | data[1]

#     def start_ranging(self):
#         # Initialize the sensor and start ranging
#         self.write_byte(SYSRANGE_START, 0x01)

#     def get_distance(self):
#         # Read the range result
#         distance = self.read_word(RESULT_RANGE_STATUS + 10)
#         return distance

#     def stop_ranging(self):
#         # Stop ranging
#         self.write_byte(SYSRANGE_START, 0x00)

# # Main program
# if __name__ == "__main__":
#     try:
#         print("Initializing VL53L0X...")
#         vl53 = VL53L0X(I2C_BUS, VL53L0X_I2C_ADDR)

#         print("Starting ranging...")
#         vl53.start_ranging()

#         while True:
#             distance = vl53.get_distance()
#             print(f"Distance: {distance} mm")
#             time.sleep(0.5)

#     except KeyboardInterrupt:
#         print("Stopping ranging...")
#         vl53.stop_ranging()

#     except Exception as e:
#         print(f"Error: {e}")
#--------------------------------------------------------------------------------------------
# import smbus2
# import time

# # I2C address of the VL53L0X
# VL53L0X_I2C_ADDR = 0x29

# # Register addresses
# SYSRANGE_START = 0x00
# RESULT_RANGE_STATUS = 0x14
# VL53L0X_REG_IDENTIFICATION_MODEL_ID = 0xC0

# # I2C bus
# I2C_BUS = 1

# class VL53L0X:
#     def __init__(self, i2c_bus, address):
#         self.bus = smbus2.SMBus(i2c_bus)
#         self.address = address

#     def write_byte(self, reg, value):
#         self.bus.write_byte_data(self.address, reg, value)

#     def read_byte(self, reg):
#         return self.bus.read_byte_data(self.address, reg)

#     def read_word(self, reg):
#         data = self.bus.read_i2c_block_data(self.address, reg, 2)
#         return (data[0] << 8) | data[1]

#     def initialize(self):
#         # Check if the sensor responds
#         model_id = self.read_byte(VL53L0X_REG_IDENTIFICATION_MODEL_ID)
#         if model_id != 0xEE:  # Expected ID for VL53L0X
#             raise RuntimeError("Failed to find VL53L0X. Check wiring!")

#         # Set timing budget (optional)
#         self.write_byte(0x01, 0x02)  # Example timing setup

#         print("VL53L0X initialized.")

#     def start_ranging(self):
#         self.write_byte(SYSRANGE_START, 0x01)

#     def get_distance(self):
#         # Wait for the result to be ready
#         while (self.read_byte(RESULT_RANGE_STATUS) & 0x01) == 0:
#             time.sleep(0.01)  # Wait for 10ms

#         # Read distance in mm
#         distance = self.read_word(RESULT_RANGE_STATUS + 10)
#         return distance

#     def stop_ranging(self):
#         self.write_byte(SYSRANGE_START, 0x00)

# # Main program
# if __name__ == "__main__":
#     try:
#         print("Initializing VL53L0X...")
#         vl53 = VL53L0X(I2C_BUS, VL53L0X_I2C_ADDR)
#         vl53.initialize()

#         print("Starting ranging...")
#         vl53.start_ranging()

#         while True:
#             distance = vl53.get_distance()
#             print(f"Distance: {distance} mm")
#             time.sleep(0.5)

#     except KeyboardInterrupt:
#         print("Stopping ranging...")
#         vl53.stop_ranging()

#     except Exception as e:
#         print(f"Error: {e}")
#-----------------------------------------------------------------------------------------------------
# import serial
# import time
# import RPi.GPIO as GPIO

# # Water level sensor setup
# WATER_LEVEL_PIN = 20
# GPIO.setmode(GPIO.BCM)
# GPIO.setup(WATER_LEVEL_PIN, GPIO.IN)

# # UART setup
# SERIAL_PORT = "/dev/serial0"
# BAUD_RATE = 9600

# # Initialize UART communication
# ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
# time.sleep(2)  # Allow time for serial to initialize

# def send_command(command):
#     ser.write(f"{command}\n".encode("utf-8"))
#     time.sleep(0.5)  # Wait for response
#     response = ser.readline().decode("utf-8").strip()
#     print(f"ESP32 Response: {response}")

# def monitor_water_level():
#     try:
#         while True:
#             print("Checking water level...")
#             if GPIO.input(WATER_LEVEL_PIN) == GPIO.HIGH:
#                 print("Water level HIGH: Increasing fan speed & forward direction")
#                 send_command("FAN_FORWARD")  # Set fan direction to forward
#                 send_command("FAN_SPEED 200")  # Set fan speed (e.g., 200/255)
#                 time.sleep(5)  # Run fan for 5 seconds

#                 print("Reversing fan direction at low speed")
#                 send_command("FAN_REVERSE")
#                 send_command("FAN_SPEED 100")  # Reduce fan speed
#                 time.sleep(5)  # Run fan in reverse for 5 seconds

#                 print("Stopping fan")
#                 send_command("FAN_STOP")
#                 time.sleep(10)  # Delay before next check
#             else:
#                 print("Water level NORMAL: Fan remains OFF")
#                 send_command("FAN_STOP")
#             time.sleep(2)  # Check water level every 2 seconds
#     except KeyboardInterrupt:
#         print("Exiting...")
#     finally:
#         GPIO.cleanup()
#         ser.close()

# if __name__ == "__main__":
#     monitor_water_level()

import smbus
import RPi.GPIO as GPIO
import time
import serial


SERIAL_PORT = "/dev/serial0"
BAUD_RATE = 9600

ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
time.sleep(2)  # Allow time for serial to initialize

def send_command(command):
    ser.write(command)
    print("raspberry send a command")
    time.sleep(0.5)  # Wait for response
    response = ser.readline()
    print(f"ESP32 Response: {response}")
    
def read_light():
    # Get I2C bus
    bus = smbus.SMBus(1)

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
    
    # Output data to screen
    print ("Sensor 1 Data")
    print ("Full Spectrum(IR + Visible) :%d lux" %ch0)
    print ("Infrared Value :%d lux" %ch1)
    print ("Visible Value :%d lux" %(ch0 - ch1))
    print("*****************")

    if ch0 > 100:
        print("Water level HIGH: Increasing fan speed & forward direction")
        send_command(12)  # Set fan direction to forward
        send_command(200)  # Set fan speed (e.g., 200/255)
        time.sleep(5)  # Run fan for 5 seconds

        print("Reversing fan direction at low speed")
        send_command("FAN_REVERSE")
        send_command("FAN_SPEED 100")  # Reduce fan speed
        time.sleep(5)  # Run fan in reverse for 5 seconds

        print("Stopping fan")
        send_command([10,11])
        time.sleep(10)  # Delay before next check
    else:
        print("Water level NORMAL: Fan remains OFF")
        send_command("FAN_STOP")
    time.sleep(2)  # Check water level every 2 seconds
    #------------------------------------------------------------------

try:
    while True:
        read_light()
except KeyboardInterrupt:
    print("Exiting...")
finally:
    GPIO.cleanup()
    ser.close()
