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

import Adafruit_DHT

SENSOR = Adafruit_DHT.DHT22
PIN = 6  # Replace with your GPIO pin

humidity, temperature = Adafruit_DHT.read_retry(SENSOR, PIN)
if humidity is not None and temperature is not None:
    print(f"Temp: {temperature:.1f} °C, Humidity: {humidity:.1f}%")
else:
    print("Failed to retrieve data from humidity sensor")