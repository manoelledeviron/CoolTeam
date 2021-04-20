from firebase import firebase
import paho.mqtt.client as mqtt
import json

def on_connect(client, userdata, flags, rc):  # The callback for when the client connects to the broker
    print("Connected with result code {0}".format(str(rc)))  # Print result of connection attempt
    client.subscribe("v3/4332@ttn/devices/433/up")  # Subscribe to the topic
    

def on_message(client, userdata, msg):  # The callback for when a PUBLISH message is received from the server.
    m_decode=str(msg.payload.decode("utf-8","ignore")) #make json of string
    m_in=json.loads(m_decode) #decode json data
    write_to_db(m_in)
    

def write_to_db(data):
    date = data["received_at"][0:10]
    time = data["received_at"][11:19]
    fb = firebase.FirebaseApplication("https://lora-tracking-test-mode-default-rtdb.firebaseio.com/")
    if "temperature_1" in data["uplink_message"]["decoded_payload"].keys():
        temp = data["uplink_message"]["decoded_payload"]["temperature_1"]
        send_d = {
            "date":date,
            "time":time,
            "temp":temp
        }
        fb.post("/lora-tracking-test-mode-default-rtdb/Temps",send_d)
    if "gps_1" in data["uplink_message"]["decoded_payload"].keys():
        altitude = data["uplink_message"]["decoded_payload"]["gps_1"]["altitude"]
        longitude = data["uplink_message"]["decoded_payload"]["gps_1"]["longitude"]
        latitude = data["uplink_message"]["decoded_payload"]["gps_1"]["latitude"]
        send_d = {
            "date":date,
            "time":time,
            "altitude":altitude,
            "longitude":longitude,
            "latitude":latitude
        }
        fb.post("/lora-tracking-test-mode-default-rtdb/Locations",send_d)
        

client = mqtt.Client("sub_tracking") 
client.on_connect = on_connect  # Define callback function for successful connection
client.on_message = on_message  # Define callback function for receipt of a message
client.username_pw_set(username="4332@ttn",password="NNSXS.N4T354NOO223R7C4Z3DQ2N6WGSXC2E3FKARMR6A.63IL2W2TEZPPCY2UUQ7764LX2EZQNJC5NL3N5DYBZ656HYPJ6HOA")
client.connect('eu1.cloud.thethings.network', 1883)
client.loop_forever() 
