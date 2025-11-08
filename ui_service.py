import time
import asyncio
import aiomqtt as mqtt
import logging
from os import path
from flask import Flask, send_from_directory
from flask_socketio import SocketIO
import json
from enum import Enum

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

MQTT_BROKER = "localhost"
MQTT_PORT = 1883
UI_SEND = "ui/send"
UI_RECEIVE = "ui/receive"

mqtt_client = None

class States(Enum):
    IDLE        = "idle"
    FORMS       = "forms"
    TEMPERATURE = "temperature"
    MEASURES    = "measures"
    FINISHED    = "finished"
    CANCELLED   = "cancelled"

current_state = States.IDLE

data = {
    "temperature": 37.0,
    "name": "Murilo Kenji Unten"
}

app = Flask(__name__, static_folder="ui")
app.config['SECRET_KEY'] = 'totem_secret_key'

socketio = SocketIO(app)

@app.route("/")
def index():
    return send_from_directory(str(app.static_folder), "index.html")

# Serve other static files (style.css, app.js, etc.)
@app.route("/<path:path>")
def static_file(path):
    return send_from_directory(str(app.static_folder), path)

@socketio.on("connect")
def handle_connect():
    global current_state
    print("client connected")
    send_sync({"asdf": "whatever"})
    send_state(current_state, "")

@socketio.on("disconnect")
def handle_disconnect():
    print("client disconnected")

@socketio.on("client_message")
def handle_receive(msg):
    global mqtt_client
    print("message received", msg)
    if mqtt_client is not None:
        a = mqtt_client.publish(UI_RECEIVE, msg)

    # # Uncomment this for testing running this file directly 
    # data = json.loads(msg)
    # match data["type"]:
    #     case "command":
    #         match data["action"]:
    #             case "start":
    #                 handle_start()
    #             case "cancel":
    #                 handle_cancel()
    #             case _:
    #                 # ignore unknown action
    #                 print("received invalid command")
    #
    #     case "name":
    #         receive_name(data["value"])
    #     case "cpf":
    #         receive_cpf(data["value"])
    #     case _:
    #         # ignore unknown type
    #         print("received invalid message")

def send_state(state, msg, step=""):
    payload = {
        "type": "state",
        "state": state.value,
        "msg": msg,
        "step": step
    }
    output_string = json.dumps(payload)
    print("sending message:", output_string)
    socketio.emit("server_message", output_string)

def send_data(field, value):
    payload = {
        "type": "data",
        field: value
    }
    output_string = json.dumps(payload)
    print("sending message:", output_string)
    socketio.emit("server_message", output_string)

def send_sync(data):
    payload = {
        "type": "sync",
        "data": data
    }
    output_string = json.dumps(payload)
    print("sending message:", output_string)
    socketio.emit("server_message", output_string)

def handle_start():
    global current_state
    if current_state is not States.IDLE:
        send_sync(data)
        send_state(States.IDLE, "")

    current_state = States.FORMS
    send_state(current_state, "What is your name?", "name")

def handle_cancel():
    global current_state
    current_state = States.IDLE
    # TODO cancel everything
    send_state(current_state, "")
    pass

def receive_name(name):
    print("name:", name)
    send_state(current_state, "What is your cpf", "cpf")
    # TODO implement

def receive_cpf(cpf):
    print("name:", cpf)
    send_state(current_state, "What is your date of birth", "age")
    time.sleep(5)
    send_state(States.MEASURES, "Follow the instructions on the screen", "temperature")
    time.sleep(5)
    send_state(States.MEASURES, "Follow the instructions on the screen", "oxymeter")
    time.sleep(5)
    send_state(States.MEASURES, "Follow the instructions on the screen", "pressure")
    time.sleep(5)
    # TODO implement

async def handle_mqtt():
    try:
        async with mqtt.Client(MQTT_BROKER, port=MQTT_PORT) as client:
            logging.info(f"Conected to MQTT Broker: {MQTT_BROKER}.")
            await client.subscribe(UI_SEND)

            global mqtt_client
            mqtt_client = client

            logging.info("waiting for message...")
            async for message in client.messages:
                if message.topic.matches(UI_SEND):
                    socketio.emit("server_message", message.payload)
                elif message.topic.matches(UI_RECEIVE):
                    await client.publish(UI_RECEIVE)

    except mqtt.exceptions.MqttError as e:
        logging.critical(f"ERROR: Could not connecto to MQTT at {MQTT_BROKER}:{MQTT_PORT}.")
        logging.critical(f"Detail: {e}")

def serve_blocking():
    print("Starting server on port 5000")
    socketio.run(app, host="0.0.0.0", port=5000, debug=True)

async def serve():
    loop = asyncio.get_event_loop()
    await loop.run_in_executor(None, serve_blocking)

async def main():
    await asyncio.gather(
        handle_mqtt(),
        serve(),
    )

if __name__ == '__main__':
    asyncio.run(main())

