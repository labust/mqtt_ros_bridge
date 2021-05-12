import os
import time
import sys
import random
import paho.mqtt.client as mqtt
import json
import time
from Phidget22.Phidget import *
from Phidget22.Devices.Log import *
from Phidget22.LogLevel import *
from Phidget22.Devices.GPS import *
from gpiozero import CPUTemperature


#THINGSBOARD_HOST = 'localhost'
#THINGSBOARD_HOST = '192.168.1.5'
THINGSBOARD_HOST = 'labustbuoy.ddnsfree.com'
ACCESS_TOKEN = 'ABC123'

time.sleep(1)

# Data capture and upload interval in seconds. Less interval will eventually hang the DHT22.
INTERVAL=60

sensor_data = {'temperature': 0, 'humidity': 0, 'latitude': 0, 'longitude': 0}
humidity = 0.0
temperature = 0.0
lat = 43.9334177
lon = 15.4436

def onPositionChange(self, latitude, longitude, altitude):
	print("Latitude: " + str(latitude))
	print("Longitude: " + str(longitude))
	print("Altitude: " + str(altitude))
	print("----------")
	lat = latitude
	lon = longitude

def onPositionFixStateChange(self, positionFixState):
	print("PositionFixState: " + str(positionFixState))

def onHeadingChange(self, heading, velocity):
	print("Heading: " + str(heading))
	print("Velocity: " + str(velocity))

next_reading = time.time() 

client = mqtt.Client()

# Set access token
client.username_pw_set(ACCESS_TOKEN)

# Connect to ThingsBoard using default MQTT port and 60 seconds keepalive interval
client.connect(THINGSBOARD_HOST, 1883, 60)

#Log.enable(LogLevel.PHIDGET_LOG_INFO, "phidgetlog.log")
gps0 = GPS()

# Register for events before calling open
gps0.setOnPositionChangeHandler(onPositionChange)
gps0.setOnPositionFixStateChangeHandler(onPositionFixStateChange)
gps0.setOnHeadingChangeHandler(onHeadingChange)

gps0.openWaitForAttachment(5000)

client.loop_start()

try:
    while True:
        cpu = CPUTemperature()
        temperature = cpu.temperature
        humidity = random.uniform(0,100)
        #temperature = random.uniform(10,40)
        humidity = round(humidity, 2)
        temperature = round(temperature, 2)
        print(u"Temperature: {:g}\u00b0C, Humidity: {:g}%, Latitude: {:g}, Longitude: {:g}".format(temperature, humidity, lat, lon))
        sensor_data['temperature'] = temperature
        sensor_data['humidity'] = humidity
        sensor_data['latitude'] = lat
        sensor_data['longitude'] = lon

        # Sending humidity and temperature data to ThingsBoard
        client.publish('v1/devices/me/telemetry', json.dumps(sensor_data), 1)

        next_reading += INTERVAL
        sleep_time = next_reading-time.time()
        if sleep_time > 0:
            time.sleep(sleep_time)
except KeyboardInterrupt:
    pass

client.loop_stop()
client.disconnect()
gps0.close()
