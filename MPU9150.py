# coding: latin-1
"""
This module is designed to communicate with the MPU-9150 9 DoF sensor

busNumber               I�C bus on which the MPU-9150 is attached (Rev 1 is bus 0, Rev 2 is bus 1)
bus                     the smbus object used to talk to the I�C bus
addressMPU9150    The I�C address of the accelerometer chip
foundMPU9150      True if the accelerometer chip can be seen, False otherwise
printFunction           Function reference to call when printing text, if None "print" is used
gPerCount               Number of G represented by the LSB of the accelerometer at the current sensitivity
tempOffest              The offset to add to the temperature reading in �C
"""

# Import the libraries we need
import sys
import smbus
import struct
import math
import time
import paho.mqtt.client as mqtt
import numpy as np

# MQTT Broker
SERVER = "192.168.1.187"

# Fall detection module using accelerometer
last_readings = []
previousState = "Safe"
def fall_detect(reading, client):
    global previousState, last_readings
    reading = np.array(reading)
    last_readings.append(reading)
    if (len(last_readings) > 20):
        last_readings.pop(0)
        diff = 0

        # A fall is registered when a free fall is followed by a large impact in opposite direction
        for i in last_readings:
            for j in last_readings:
                diff = max(diff, np.linalg.norm(i-j))

        currentState = "Safe"
        if (diff > 2):  # Experimental thershold set to 2*accel. due to gravity
            currentState = "Unsafe"
            print "Fall Detected"

        if (previousState == "Safe" and currentState == "Unsafe"):
            client.publish("/fall", "Fall Detected")
        previousState = currentState


### MODULE DATA ###
# Shared values used by this module
global busNumber
global bus
global addressMPU9150
global foundMPU9150
global printFunction
global gPerCount
global tempOffest

# Constant values
addressMPU9150 = 0x68
addressMPU9150mag = 0x0c

# Check here for Rev 1 vs Rev 2 and select the correct bus
busNumber = 0

### MODULE FUNCTIONS ###


def Print(message):
    """
Print(message)

Wrapper used by the MPU-9150 module to print messages, will call printFunction if set, print otherwise
    """
    global printFunction
    if printFunction == None:
        print message
    else:
        printFunction(message)


def NoPrint(message):
    """
NoPrint(message)

Does nothing, intended for disabling diagnostic printout by using:
MPU-9150.printFunction = MPU-9150.NoPrint
    """
    pass


def Init(tryOtherBus=True):
    """
Init([tryOtherBus])

Prepare the I2C driver for talking to the MPU-9150
If tryOtherBus is True or omitted, this function will attempt to use the other bus if none of the MPU-9150 devices can be found on the current busNumber
    """
    global busNumber
    global bus
    global addressMPU9150
    global foundMPU9150
    global magxoffset, magyoffset, magzoffset

    # Read predetermined magnetometer offsets from file if present
    # File MAG3110offsets should contain 3 signed dicimal numbers separated by spaces.
    # Blank lines, lines starting with # and lines following a valid line are ignored.
    magxoffset = magyoffset = magzoffset = 0
    try:
        f = open('MPU9150offsets', 'r')
        line = "\n"
        while line == "\n" or line[0] == "#":
            line = f.readline()
        data = line.split()
        magxoffset = int(data[0])
        magyoffset = int(data[1])
        magzoffset = int(data[2])
        print("magxoffset=%d magyoffset=%d magzoffset=%d" %
              (magxoffset, magyoffset, magzoffset))
    except:
        print('No MPU9150offsets file found: offsets set to zero')

    # Open the bus
    Print('Loading MPU-9150 on bus %d' % (busNumber))
    bus = smbus.SMBus(busNumber)

    # Check for MPU-9150
    try:
        whoami = bus.read_byte_data(addressMPU9150, 117)
        foundMPU9150 = True
        Print('Found device on bus %d at %02X WhoAmI=%02X' %
              (busNumber, addressMPU9150, whoami))
    except:
        foundMPU9150 = False
        Print('Missing device at %02X' % (addressMPU9150))

    # See if we are missing chips
    if not foundMPU9150:
        if tryOtherBus:
            if busNumber == 1:
                busNumber = 0
            else:
                busNumber = 1
            Print('Trying bus %d instead' % (busNumber))
            Init(False)
        else:
            Print(
                'Are you sure your MPU-9150 is properly attached, and the I2C drivers are running?')
            bus = None
    else:
        InitMPU9150()


def InitMPU9150():
    """
InitMPU9150()

Initialises the accelerometer and magnetometer to default states
    """
    global bus
    global addressMPU9150
    global gPerCount

# Accelerometer config
    # Setup mode configuration
    register = 0x19             # Sample Rate Divider
    data = 9                   # Gyro o/p rate 1KHz / 10
    try:
        bus.write_byte_data(addressMPU9150, register, data)
    except:
        Print('Failed sending SMPRT_DIV!')

    # Config Ext Sync and digital lp filter
    register = 0x1A             # CONFIG
    data = 3                   # Ext sync dis'd, bandwidth 44Hz
    try:
        bus.write_byte_data(addressMPU9150, register, data)
    except:
        Print('Failed sending CONFIG!')

# Gyro Config:

    # Config Gyro
    register = 0x1B             # GYRO_CONFIG
    data = 0                   # +/- 250 degrees/sec
    try:
        bus.write_byte_data(addressMPU9150, register, data)
    except:
        Print('Failed sending GYRO_CONFIG!')

# Accelerometer config:

    # Config Gyro
    register = 0x1C             # ACCEL_CONFIG
    data = 0                   # +/- 2G
    try:
        bus.write_byte_data(addressMPU9150, register, data)
    except:
        Print('Failed sending ACCEL_CONFIG!')

# FIFO Enable:

    # Config FIFO
    register = 0x23             # FIFO_EN
    data = 0                   # Disabled
    try:
        bus.write_byte_data(addressMPU9150, register, data)
    except:
        Print('Failed sending FIFO_EN!')

# I2C config: All registers left at their zero valuesafter reset
# Likewise INT_ENABLE, USER_CTRL, PWR_MGMT_2

    # INT Pin / Bypass Enable
    register = 0x37             # INT_PIN_CFG
    data = 2                   # I2C bypass enable to access magnetometer
    try:
        bus.write_byte_data(addressMPU9150, register, data)
    except:
        Print('Failed sending INT_PIN_CFG!')

    # Power Management 1
    register = 0x6B             # PWR_MGMT_1
    data = 1                   # Clock from X Gyro
    try:
        bus.write_byte_data(addressMPU9150, register, data)
    except:
        Print('Failed sending PWR_MGMT_1!')

# Magnetometer config:

    # Check for MPU-9150 magnetometer
    try:
        id = bus.read_byte_data(addressMPU9150mag, 0)
        info = bus.read_byte_data(addressMPU9150mag, 1)
        Print('Found magnetometer ID=%02X Info=%02X' %
              (id, info))
    except:
        Print('Missing magnetomete')

    # Control
    register = 0x0A             # CTRL
    data = 1                   # Single measurement mode
    try:
        bus.write_byte_data(addressMPU9150mag, register, data)
    except:
        Print('Failed sending CTRL!')


def ReadAccelerometer():
    global bus

    try:
        [xh, xl, yh, yl, zh, zl] = bus.read_i2c_block_data(
            addressMPU9150, 0x3B, 6)
        bytes = struct.pack('BBBBBB', xl, xh, yl, yh, zl, zh)
        x, y, z = struct.unpack('hhh', bytes)
        return float(x)/16384, float(y) / 16384, float(z) / 16384
    except:
        Print('Failed reading Gyro')


def ReadGyro():
    global bus

    try:
        [xh, xl, yh, yl, zh, zl] = bus.read_i2c_block_data(
            addressMPU9150, 0x43, 6)
        bytes = struct.pack('BBBBBB', xl, xh, yl, yh, zl, zh)
        x, y, z = struct.unpack('hhh', bytes)
        return float(x)/131.072, float(y) / 131.072, float(z) / 131.072
    except:
        Print('Failed reading Accelerometer')
        return 0, 0, 0
        return 0, 0, 0


def ReadTemperature():
    """
temp = ReadTemperature()

Reads the die temperature of the compass in degrees Celsius
    """
    global bus
    global addressMPU9150
    global tempOffest

    # Read the data from the compass chip
    try:
        [temph, templ] = bus.read_i2c_block_data(addressMPU9150, 0x41, 2)
    except:
        Print('Failed reading Temp')
        return 0

    bytes = struct.pack('BB', templ, temph)
    temp = struct.unpack('h', bytes)
    return float(temp[0]) / 340 + 35


def ReadCompassRaw():
    """
x, y, z = ReadCompassRaw()

Reads the X, Y and Z axis raw magnetometer readings
    """
    global bus
    global magxoffset, magyoffset, magzoffset

    # Set single measurement mode
    register = 0x0A             # CTRL
    data = 1                   # Single measurement mode
    try:
        bus.write_byte_data(addressMPU9150mag, register, data)
    except:
        Print('Failed sending CTRL!')
    time.sleep(0.01)

    # Wait for dataready
    register = 0x02             # Status 1
    try:
        status = 0
        while (status & 1) == 0:
            status = bus.read_byte_data(addressMPU9150mag, register)
    except:
        Print('Failed reading ST1!')

    # Read the data from the compass chip
    try:
        [xl, xh, yl, yh, zl, zh] = bus.read_i2c_block_data(
            addressMPU9150mag, 3, 6)
    except:
        Print('Failed reading registers!')
        status = 0
        xh = 0
        xl = 0
        yh = 0
        yl = 0
        zh = 0
        zl = 0

    # Convert from unsigned to correctly signed values
    bytes = struct.pack('BBBBBB', xl, xh, yl, yh, zl, zh)
    x, y, z = struct.unpack('hhh', bytes)

    return x - magxoffset, y - magyoffset, z - magzoffset


### STARTUP ROUTINES ###
# Default user settings
printFunction = None
tempOffset = 0


def start():
    try:
        # Start the MPU-9150 module (sets up devices)
        Init()
        client = mqtt.Client()
        client.connect(SERVER, 1883, 60)
        client.loop_start()
        time.sleep(2)
        # Loop indefinitely
        while True:
            fall_detect(ReadAccelerometer(), client)
            time.sleep(0.1)
    except KeyboardInterrupt:
        # User aborted
        print ''
