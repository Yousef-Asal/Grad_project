import board
import busio
import adafruit_vl53l0x

i2c = busio.I2C(board.SCL, board.SDA)

try:
    vl53 = adafruit_vl53l0x.VL53L0X(i2c)
    print("Sensor initialized!")
    print("Range: {}mm".format(vl53.range))
except Exception as e:
    print("Error:", e)