# Distributed with a free-will license.
# Use it any way you want, profit or free, provided it fits in the licenses of its associated works.
# TSL2561
# This code is designed to work with the TSL2561_I2CS I2C Mini Module available from ControlEverything.com.
# https://www.controleverything.com/content/Light?sku=TSL2561_I2CS#tabs-0-product_tabset-2

import smbus
import time

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
    bus2 = smbus.SMBus(4)

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
        # read_dht()
        
        # Read Hall Effect sensor
        #read_hall()
        
        # Read Light sensor
        read_light()
        
        # Read pH sensor
        #read_ph()
        
        # Read temperature sensor
        #read_temperature()
        
        # Read water level sensor
        #read_water_level()
        
        time.sleep(4)  # Delay between readings

except KeyboardInterrupt:
    print("Exiting...")
finally:
    GPIO.cleanup()