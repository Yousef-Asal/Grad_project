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
import smbus2
import time

# I2C address of the VL53L0X
VL53L0X_I2C_ADDR = 0x29

# Register addresses
SYSRANGE_START = 0x00
RESULT_RANGE_STATUS = 0x14
VL53L0X_REG_IDENTIFICATION_MODEL_ID = 0xC0

# I2C bus
I2C_BUS = 1

class VL53L0X:
    def __init__(self, i2c_bus, address):
        self.bus = smbus2.SMBus(i2c_bus)
        self.address = address

    def write_byte(self, reg, value):
        self.bus.write_byte_data(self.address, reg, value)

    def read_byte(self, reg):
        return self.bus.read_byte_data(self.address, reg)

    def read_word(self, reg):
        data = self.bus.read_i2c_block_data(self.address, reg, 2)
        return (data[0] << 8) | data[1]

    def initialize(self):
        # Check if the sensor responds
        model_id = self.read_byte(VL53L0X_REG_IDENTIFICATION_MODEL_ID)
        if model_id != 0xEE:  # Expected ID for VL53L0X
            raise RuntimeError("Failed to find VL53L0X. Check wiring!")

        # Set timing budget (optional)
        self.write_byte(0x01, 0x02)  # Example timing setup

        print("VL53L0X initialized.")

    def start_ranging(self):
        self.write_byte(SYSRANGE_START, 0x01)

    def get_distance(self):
        # Wait for the result to be ready
        while (self.read_byte(RESULT_RANGE_STATUS) & 0x01) == 0:
            time.sleep(0.01)  # Wait for 10ms

        # Read distance in mm
        distance = self.read_word(RESULT_RANGE_STATUS + 10)
        return distance

    def stop_ranging(self):
        self.write_byte(SYSRANGE_START, 0x00)

# Main program
if __name__ == "__main__":
    try:
        print("Initializing VL53L0X...")
        vl53 = VL53L0X(I2C_BUS, VL53L0X_I2C_ADDR)
        vl53.initialize()

        print("Starting ranging...")
        vl53.start_ranging()

        while True:
            distance = vl53.get_distance()
            print(f"Distance: {distance} mm")
            time.sleep(0.5)

    except KeyboardInterrupt:
        print("Stopping ranging...")
        vl53.stop_ranging()

    except Exception as e:
        print(f"Error: {e}")
