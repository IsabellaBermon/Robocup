import paho.mqtt.client as mqtt
global flagPayload

# Definit en true para enviar datos del primer frame 
flagPayload = True

client = mqtt.Client("robocupID")
client.connect("test.mosquitto.org",1883,60)

# def on_message(client, userdata, msg):
#     print(f"Mensaje recibido en el tema {msg.topic}: {msg.payload.decode()}")
#     flagPayload = True

# client.on_message = on_message

# client.subscribe("request/Betty")
