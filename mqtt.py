import paho.mqtt.client as mqtt

client = mqtt.Client("robocupID")
client.connect("test.mosquitto.org",1883,60)


