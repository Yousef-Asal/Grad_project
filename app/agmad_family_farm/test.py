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
import Adafruit_DHT
import board

# Initialize the DHT22 sensor (use the correct GPIO pin)
dhtDevice = Adafruit_DHT.DHT22(board.D5)  # Replace D4 with your GPIO pin

try:
    # Get temperature and humidity readings
    temperature_c = dhtDevice.temperature
    humidity = dhtDevice.humidity
    print(f"Temperature: {temperature_c:.1f}°C")
    print(f"Humidity: {humidity:.1f}%")
except RuntimeError as error:
    # Errors happen fairly often with DHT sensors, just retry
    print(f"Error reading DHT sensor: {error}")