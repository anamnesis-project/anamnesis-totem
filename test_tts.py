import paho.mqtt.client as mqtt
import sys
MQTT_BROKER = "localhost"
MQTT_PORT = 1883
TOPIC_PUBLISH = "voice/speak"

def publish_message(text: str):
    try:
        client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1, "tts_tester_script")
        print(f"Conecting MQTT broker at {MQTT_BROKER}...")
        client.connect(MQTT_BROKER, MQTT_PORT, 60)
        print(f"Publishing topic '{TOPIC_PUBLISH}': '{text}'")
        msg_info = client.publish(TOPIC_PUBLISH, text, qos=1)
        msg_info.wait_for_publish()
        client.disconnect()
        
        print("Message sent!")

    except ConnectionRefusedError:
        print(f"ERROR could not connect to MQTT broker.")
    except Exception as e:
        print(f"Unexpected error: {e}")

if __name__ == "__main__":
    if len(sys.argv) > 1:
        message_to_send = " ".join(sys.argv[1:])
    else:
        message_to_send = "I AM FEELING SICK"
        print("No text provided, using default message.")

    publish_message(message_to_send)