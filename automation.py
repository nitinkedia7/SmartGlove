# Contoller program for Home automation server in python2
import paho.mqtt.client as mqtt
import RPi.GPIO as GPIO

# Relay switch pin declaration
SWITCH_A = 8
# MQTT broker
SERVER = "192.168.1.187"


def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
    client.subscribe("/home_automation")


LIGHT_STATE = False  # Denotes current state of the gesture controlled bulb


def on_message(client, userdata, msg):
	global LIGHT_STATE
	print msg.topic + " " + msg.payload.decode()
	if(msg.topic == "/home_automation" and msg.payload.decode() == "Gesture 5"):
		print "Light Toggle"
		GPIO.output(SWITCH_A, LIGHT_STATE)
		LIGHT_STATE = not LIGHT_STATE


try:
	GPIO.setmode(GPIO.BOARD)

	# Set light pin as output
	GPIO.setup(SWITCH_A, GPIO.OUT)
	GPIO.output(SWITCH_A, True)
	while True:
		client = mqtt.Client()
        	client.connect(SERVER, 1883, 60)
			# Setup callback functions
        	client.on_connect = on_connect
        	client.on_message = on_message

        	client.loop_forever()

        	break

except KeyboardInterrupt:
    print("Stopped by User!")
    GPIO.cleanup()
    client.disconnect()
