# Glove controller program in python2
import RPi.GPIO as GPIO
import Adafruit_ADS1x15
import paho.mqtt.client as mqtt
import time

# Raspberry Pin declarations in BOARD scheme
PIN_BUTTON1 = 40  # Push Buttons 1 and 2
PIN_BUTTON2 = 38
PIN_TRIGGER = 7   # Trigger and Echo for Ultrasonic sensor
PIN_ECHO = 11
PIN_BUZZER = 12
SERVER = "192.168.1.187" # MQTT Broker

# Medicine alarm section
BEEP = False
def on_message(client, userdata, message):
    global BEEP
    print message.topic + " " + message.payload.decode()
    if message.topic == "/alarm" and message.payload.decode() == "yes":
        print "alarm on" # Start time dictated by companion android app
        BEEP = True
    elif (message.topic == "/locker/0" and message.payload.decode() == "open"):
        print "alarm off" # End when smart locker is opened thereby ensuring that dose is taken
        BEEP = False

# Gesture recogition from 3 flex sensors connected by ADC
adc = Adafruit_ADS1x15.ADS1115()
GAIN = 1
# Threshold is the average of ADC reading when a flex sensor is straight and bend.
thresholds = [16000, 18000, 19000]
prevGesture = -1
AUTOMATION = False
def gesture(client):
    global prevGesture, low, AUTOMATION
    values = [adc.read_adc(i, gain=GAIN) for i in range(3)]
    # print values
    # Encode gesture into 3-bit binary number, eg, 5 (101) means 1st and 3rd are bend
    gesture = 0
    multiplier = 1
    for i in range(3):
        if values[i] < thresholds[i]:
            gesture += multiplier
        multiplier *= 2
    # Only publish when gesture is changed to avoid repitition, according to context.
    if gesture is not prevGesture:
        print "Gesture " + str(gesture)
        if (AUTOMATION):
            client.publish("/home_automation", "Gesture " + str(gesture))
        else:
            client.publish("/gesture", "Gesture " + str(gesture))
    prevGesture = gesture

# Obstacle Detection is used only when needed using push button
BUTTON_STATE = False
def buttonState():
    global BUTTON_STATE
    INPUT = GPIO.input(PIN_BUTTON1)
    if (INPUT):
        BUTTON_STATE = not BUTTON_STATE
        print "Obstacle Detection " + str(BUTTON_STATE)

# Routine for distance calculation in UltraSonic sensor
def distanceCalc():
    GPIO.output(PIN_TRIGGER, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(PIN_TRIGGER, GPIO.LOW)

    while GPIO.input(PIN_ECHO) == 0:
        pulse_start_time = time.time()
    while GPIO.input(PIN_ECHO) == 1:
        pulse_end_time = time.time()

    pulse_duration = pulse_end_time - pulse_start_time
    distance = round(pulse_duration * 17150, 2)
    print "Distance:", distance, "cm"
    return distance

# Actuates the buzzer if obstacle within 50cm with linearly increasing freq 
def detect_obstacle():
    while True:
        buttonState()
        if (not BUTTON_STATE):
            return
        distance = distanceCalc()
        if (distance < 50):
            halfTime = 100.0/(500 - 10*distance)
            GPIO.output(PIN_BUZZER, GPIO.HIGH)
            time.sleep(halfTime)
            GPIO.output(PIN_BUZZER, GPIO.LOW)
            time.sleep(halfTime)
        else:
            return

# Listen for context changes between Gesture-to-Speech and Home Automation
def context_switch():
    global AUTOMATION
    INPUT = GPIO.input(PIN_BUTTON2)
    if (INPUT):
        AUTOMATION = not AUTOMATION
        print "Home Automation " + str(AUTOMATION)

def main():
    try:
        # Setup sensors and MQTT connections
        GPIO.setmode(GPIO.BOARD)

        GPIO.setup(PIN_BUTTON1, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(PIN_BUTTON2, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

        GPIO.setup(PIN_TRIGGER, GPIO.OUT)
        GPIO.setup(PIN_ECHO, GPIO.IN)
        GPIO.output(PIN_TRIGGER, GPIO.LOW)

        GPIO.setup(PIN_BUZZER, GPIO.OUT)
        GPIO.output(PIN_BUZZER, GPIO.LOW)

        client = mqtt.Client()
        client.connect(SERVER, 1883, 60)
        client.subscribe("/alarm")
        client.subscribe("/locker/0")
        client.on_message = on_message
        client.loop_start()
        time.sleep(2)
        
        # Main loop integrating all above functionalities
        while (True):
            if BEEP:
                BUTTON_STATE = False
                GPIO.output(PIN_BUZZER, GPIO.HIGH)
                time.sleep(0.5)
                GPIO.output(PIN_BUZZER, GPIO.LOW)
                time.sleep(0.4)
            context_switch()
            detect_obstacle()
            gesture(client)
            time.sleep(1)
    
    except KeyboardInterrupt:
        client.disconnect()
        GPIO.cleanup()
