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
main_loop = None # To store the main asyncio event loop

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

async def ui_send_state(client, state, msg, step=""):
    payload = {
        "type": "state",
        "state": state,
        "msg": msg,
        "step": step
    }
    output_string = json.dumps(payload)
    await client.publish(UI_SEND, output_string)

async def ui_send_data(client, field, value):
    payload = {
        "type": "data",
        "field": field,
        "value": value,
    }

    output_string = json.dumps(payload)
    await client.publish(UI_SEND, output_string)

async def ui_start(client):
    await ui_send_state(client, "forms", "What is your name?", "name")

async def ui_cancel(client):
    await ui_send_state(client, "idle", "")

async def ui_mic_on(client):
    await ui_send_data(client, "mic", True)

async def ui_mic_off(client):
    await ui_send_data(client, "mic", False)

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
    send_sync({ "mic": False })
    send_state(current_state, "")

@socketio.on("disconnect")
def handle_disconnect():
    print("client disconnected")

async def _async_handle_receive(msg):
    """This is the asynchronous logic that will run on the main loop."""
    global mqtt_client
    print("message received (async handler)", msg)
    if mqtt_client is not None:
        try:
            await mqtt_client.publish(UI_RECEIVE, msg)
        except Exception as e:
            logging.error(f"Error publishing to MQTT: {e}")

    # # Uncomment this for testing running this file directly 
    # # Note: This part is synchronous, but it's okay inside the async function
    # # for testing. For real use, consider 'await asyncio.sleep' instead of 'time.sleep'
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

@socketio.on("client_message")
def handle_receive(msg):
    """
    This is the synchronous SocketIO handler.
    It schedules the async logic to run on the main asyncio loop.
    """
    global main_loop
    if main_loop:
        # Safely schedule the coroutine to run on the main event loop from this thread
        asyncio.run_coroutine_threadsafe(_async_handle_receive(msg), main_loop)
    else:
        logging.error("Main asyncio loop is not available.")


def send_state(state, msg, step=""):
    payload = {
        "type": "state",
        "state": state.value,
        "msg": msg,
        "step": step
    }
    output_string = json.dumps(payload)
    print("sending message:", output_string)
    # socketio.emit is thread-safe
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
    print("cpf:", cpf) # Corrected from "name:"
    send_state(current_state, "What is your date of birth?", "age")
    # Note: time.sleep() blocks the worker thread.
    # This is fine for testing, but for production,
    # this logic should be in an async function with asyncio.sleep
    # or handled by the external MQTT logic.
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
            logging.info(f"Connected to MQTT Broker: {MQTT_BROKER}.")
            await client.subscribe(UI_SEND)

            global mqtt_client
            mqtt_client = client

            logging.info("waiting for message...")
            async for message in client.messages:
                if message.topic.matches(UI_SEND):
                    # Forward message from MQTT to SocketIO client
                    socketio.emit("server_message", message.payload.decode()) # Decode payload
                # Removed the UI_RECEIVE match, as publishing is handled by _async_handle_receive
                # elif message.topic.matches(UI_RECEIVE):
                #     await client.publish(UI_RECEIVE)

    except mqtt.exceptions.MqttError as e:
        logging.critical(f"ERROR: Could not connect to MQTT at {MQTT_BROKER}:{MQTT_PORT}.")
        logging.critical(f"Detail: {e}")
    except Exception as e:
        logging.error(f"An error occurred in handle_mqtt: {e}")
        # Optionally, add a retry mechanism
        await asyncio.sleep(5)
        asyncio.create_task(handle_mqtt()) # Relaunch task

def serve_blocking():
    print("Starting server on port 5000")
    socketio.run(app, host="0.0.0.0", port=5000, debug=False, allow_unsafe_werkzeug=True)

async def serve():
    global main_loop
    # Get the running event loop in the main thread
    main_loop = asyncio.get_running_loop() 
    
    loop = asyncio.get_event_loop()
    await loop.run_in_executor(None, serve_blocking)

async def main():
    await asyncio.gather(
        handle_mqtt(),
        serve(),
    )

if __name__ == '__main__':
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logging.info("Shutting down...")
