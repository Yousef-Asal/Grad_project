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
import serial
import requests
import json

SERIAL_PORT = "/dev/serial0"
BAUD_RATE = 9600
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
time.sleep(2)  # Allow time for serial to initialize

i2c = busio.I2C(board.SCL, board.SDA)  # laser 
#vl53 = adafruit_vl53l0x.VL53L0X(i2c)

water_pin = 20 # GPIO pin for Digital Water Level Sensor
nutrient_pin = 21 # GPIO pin for Digital Water Level Sensor
GPIO.setmode(GPIO.BCM)



drain_valve_state = 0
water_valve_state = 0
nutrients_valve_state = 0
plate1_open_valve_state = 0
plate1_drain_valve_state = 0
plate2_open_valve_state = 0
plate2_drain_valve_state = 0
pump_state = 0
plate1_heater_state = 0
plate2_heater_state = 0
plate1_fan_state = 0
plate2_fan_state = 0
led_line1_state = 0
led_line2_state = 0
led_line3_state = 0
mixer_state = 0




################################################################################################################
#**************************************************Sensors***************************************************#

#dht sensors readings
plate1_dht = adafruit_dht.DHT22(board.D5)
plate2_dht = adafruit_dht.DHT22(board.D13)
pcb_dht = adafruit_dht.DHT11(board.D0)

def read_plate_temp(plate):
    # First DHT22 sensor
    dht = plate1_dht if plate == "plate1" else plate2_dht
    try:
        temperature_c = dht.temperature
        humidity = dht.humidity
        print(f"{plate}DHT:")
        print(f"Temperature: {temperature_c:.1f}°C")
        print(f"Humidity: {humidity:.1f}%")
        print("*****************")
        return {"temp": temperature_c,"humidity": humidity}
    except RuntimeError as error:
    # Handle reading errors (common with DHT sensors)
        print(f"Error reading {plate} DHT sensor: {error}")
        return {"temp": null,"humidity": null}
    # second DHT22 sensor

    print("-------------------------------------------------------------------------------------------------")

def read_pcb_temp():
    try:
        temperature_c = pcb_dht.temperature
        humidity = pcb_dht.humidity
        print("pcb DHT:")
        print(f"Temperature: {temperature_c:.1f}°C")
        print(f"Humidity: {humidity:.1f}%")
        return {"temp": temperature_c,"humidity": humidity}

    except RuntimeError as error:
    # Handle reading errors (common with DHT sensors)
        print(f"Error reading pcb dht sensor: {error}")
        return {"temp": null,"humidity": null}

#**********************************************************#

def read_light(plate):
    # Get I2C bus
    # buses
    bus = smbus.SMBus((5)) if plate == "plate1" else smbus.SMBus((6))

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
    print (f"{plate} Data")
    print ("Full Spectrum(IR + Visible) :%d lux" %ch0)
    print ("Infrared Value :%d lux" %ch1)
    print ("Visible Value :%d lux" %(ch0 - ch1))
    print("*****************")
    return {"full_spectrum": ch0,"ir":ch1,"visible":(ch0 - ch1)}

#*******************************************************************#

# Read TDS value
TDS_SENSOR_CHANNEL = 1

def read_tds():
    raw_value = read_adc(TDS_SENSOR_CHANNEL)
    tds_value = raw_value * (1000 / 65535)  # Adjust for 10-bit (65535 is for 16-bit, adjust based on your ADC)
    ec_value = ((tds_value*2)/1000)
    print(f"TDS Sensor: ec={ec_value:.2f}")
    print("-------------------------------------------------------------------------------------------------")
    return ec_value

#********************************************************************#

# Read pH value
PH_SENSOR_CHANNEL = 0 # ADC channel for pH sensor

def read_ph():
    raw_value = read_adc(PH_SENSOR_CHANNEL)
    ph_value = raw_value * (14 / 65535)  # Adjust for 10-bit (65535 is for 16-bit, adjust based on your ADC)
    print(f"pH Sensor: pH={ph_value:.2f}")
    print("-------------------------------------------------------------------------------------------------")
    return ph_value

#*********************************************************************#

#Water level sensors
def read_water_level(tank):
    pin = 20 if tank == "water" else 21
    GPIO.setup(pin, GPIO.IN)
    if GPIO.input(pin) == GPIO.HIGH:
        print("Water Level Sensor1: Water detected!")
        return 1
    else:
        print("Water Level Sensor1: Water not detected!")
        return 0
    
#************************************************************************#

def read_laser():
    print("Laser Data:")
    print("Range: {0}mm".format(vl53.range))
    print("-----------------------------------------------------------------")
    return(vl53.range)

#//////////////////////////////////////////////////////////////////////

#read the analog signal from mcp
spi = busio.SPI(clock=SCLK, MISO=MISO, MOSI=MOSI)
cs = digitalio.DigitalInOut(CE0)
mcp = MCP3008(spi, cs)

def read_adc(channel):
    chan = AnalogIn(mcp, channel)
    return chan.value

#///////////////////////////////////////////////////////////////////////


################################################################################################################
#**************************************************Actuators***************************************************#
#to send a command to the esp
def send_command():
    command = f"{drain_valve_state}{water_valve_state}{nutrients_valve_state}{plate1_open_valve_state}{plate1_drain_valve_state}{plate2_open_valve_state}{plate2_drain_valve_state}{pump_state}{plate1_heater_state}{plate2_heater_state}{plate1_fan_state}{plate2_fan_state}{led_line1_state}{led_line2_state}{led_line3_state}{mixer_state}"
    ser.write(f"{command}\n".encode("utf-8"))
    print("raspberry send a command")
    time.sleep(0.5)  # Wait for response
    response = ser.readline().decode("utf-8").strip()
    print(f"ESP32 Response: {response}")

# def control_fans(plate,direction,speed):
#     print("Water level HIGH: Increasing fan speed & forward direction")
#     send_command("FAN_SPEED 200")  # Set fan speed (e.g., 200/255)
#     time.sleep(10)  # Run fan for 5 seconds
#     print("Stopping fan")
#     send_command("FAN_STOP")
#     time.sleep(10)  # Delay before next check

################################################################################################################
#*************************************************AWS**********************************************************#
# Your API Gateway link
def send_data():
    api_url = "https://wdy1m5yd7i.execute-api.us-east-1.amazonaws.com/RPI/RPI_DATA"
    # plate1_temp = read_plate_temp("plate1")
    # plate2_temp = read_plate_temp("plate2")
    # plate1_light = read_light("plate1")
    # plate2_light = read_light("plate2")
    # ec = read_tds()
    # ph = read_ph()
    # water_level = read_water_level("water")
    # nutrient_level = read_water_level("nutrients")




    # Data to send
    # sensors = {
    #     "plate1_temp": plate1_temp["temp"],
    #     "plate1_humidity": plate1_temp["humidity"],
    #     "plate2_temp": plate2_temp["temp"],
    #     "plate2_humidity": plate2_temp["humidity"],
    #     "plate1_light_visible": plate1_light["visible"],
    #     "plate2_light_visible": plate2_light["visible"],  # Assuming plate2 uses the same value for now
    #     "ec": ec,
    #     "ph": ph,
    #     "water_level": water_level,
    #     "nutrient_level": nutrient_level
    # }
    sensors = {
    "plate1_temp": 25.0,  # Plate 1 Temperature (°C)
    "plate1_humidity": 60.0,  # Plate 1 Humidity (%)
    "plate2_temp": 26.5,  # Plate 2 Temperature (°C)
    "plate2_humidity": 58.5,  # Plate 2 Humidity (%)
    "plate1_light_visible": 300,  # Plate 1 Visible Light Intensity (lux)
    "plate2_light_visible": 300,  # Plate 2 Visible Light Intensity (lux)
    "ec": 1.2,  # Electrical Conductivity (mS/cm)
    "ph": 6.5,  # pH Level
    "water_level": 75.0,  # Water Level (%)
    "nutrient_level": 50.0,  # Nutrient Level (%)
}
    actuators = {
        "drain_valve": 0,
        "water_valve": 1,
        "nutrients_valve": nutrients_valve_state,
        "plate1_open_valve": plate1_open_valve_state,
        "plate1_drain_valve": plate1_drain_valve_state,
        "plate2_open_valve": plate2_open_valve_state,
        "plate2_drain_valve": plate2_drain_valve_state,
        "pump": pump_state,
        "plate1_heater": plate1_heater_state,
        "plate2_heater": plate2_heater_state,
        "plate1_fan": plate1_fan_state,
        "plate2_fan": plate2_fan_state,
        "led_line1": led_line1_state,
        "led_line2": led_line2_state,
        "led_line3": led_line3_state,
        "mixer": mixer_state
    }

    # Create a payload dictionary
    payload = {"sensors": sensors, "actuators": actuators}

    headers = {"Content-Type": "application/json"}

    try:
        print("Payload:", json.dumps(payload, indent=4))  # Debugging
        response = requests.post(api_url, json=payload, headers=headers)

        if response.status_code == 200:
            print("Data sent successfully!")
            print("Response:", response.json())
        else:
            print(f"Failed to send data. Status code: {response.status_code}")
            print("Response:", response.text)
    except Exception as e:
        print("An error occurred:", e)

###################################################################################################
#********************************************Control**********************************************#
def tank_control():
    desired_water_level = 10
    mixer_height = 10
    required_time_for_plate = 60
    empty_plate_time = 30
    global water_valve_state
    global nutrients_valve_state
    global mixer_state
    global pump_state 
    global plate1_drain_valve_state
    global plate2_drain_valve_state
    global plate1_open_valve_state
    global plate2_open_valve_state

    while True:
        while True:
            water_level = read_laser()
            if water_level >= desired_water_level:
                water_valve_state = 0
                nutrients_valve_state = 0
                mixer_state = 1
                send_command()
                break
            else:
                water_valve_state = 1
                nutrients_valve_state = 1
                send_command()
        time.sleep(20)
        pump_state = 1
        mixer_state = 0
        plate1_drain_valve_state = 0
        plate2_drain_valve_state = 0
        plate1_open_valve_state = 1
        plate2_open_valve_state = 0
        send_command()
        while True:
            water_level = read_laser()
            if water_level >= mixer_height:
                pump_state = 0
                send_command()
                break
            else:
                continue
        time.sleep(required_time_for_plate)
        plate1_drain_valve_state = 1
        time.sleep(empty_plate_time)

def light_control():
    global led_line1_state
    global led_line2_state
    global led_line3_state

    on_time = 60
    off_time = 30

    while True:
        led_line1_state = 1
        led_line2_state = 1
        led_line3_state = 1
        send_command()
        time.sleep(on_time)

        led_line1_state = 0
        led_line2_state = 0
        led_line3_state = 0
        send_command()

        time.sleep(off_time)

def temp_control():
    global plate1_heater_state
    global plate1_fan_state

    min_temp = 30
    max_temp = 35
    time_for_checking = 60
    while True:
        output = read_plate_temp("plate1")
        temp = output["temp"]
        if temp < min_temp:
            plate1_heater_state = 1
            send_command()
        elif temp > max_temp:
            plate1_fan_state = 1
            send_command()
        else:
            plate1_fan_state = 0
            plate1_heater_state = 0
            send_command()
            break
    time.sleep(time_for_checking)

def camera_data():
    pass


try:
    while True:
        # print("*******************Reading sensors*****************************\n")
        # read_plate_temp("plate1")
        # print("done reading plate1 temp ")
        # read_water_level("water")
        # print("done reading water level ")
        # read_ph()
        # print("done reading ph ")
        # read_laser()
        # print("done reading laser ")
        send_data()

        # send_command()
        # Read DHT sensor
        #read_dht()
        
        # Read Light sensor
        # read_light()
        
        # Read Light sensor
        # read_tds()
        # Read Laser Sensor
        #read_laser()

        # Read pH sensor
        #read_ph()
        
        
        # Read water level sensor
        #read_water_level()
        
        time.sleep(4)  # Delay between readings

except KeyboardInterrupt:
    print("Exiting...")
finally:
    GPIO.cleanup()