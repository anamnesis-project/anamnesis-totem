from asyncio import current_task
from os import path
from flask import Flask, send_from_directory
from flask_socketio import SocketIO
import json
from enum import Enum

class States(Enum):
    IDLE        = "idle"
    FORMS       = "forms"
    TEMPERATURE = "temperature"
    OXYMETER    = "oxymeter"
    PRESSURE    = "pressure"
    INTERVIEW   = "interview"
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
    print("message received", msg)
    data = json.loads(msg)
    match data["type"]:
        case "command":
            match data["action"]:
                case "start":
                    handle_start()
                case "cancel":
                    handle_cancel()
                case _:
                    # ignore unknown action
                    print("received invalid command")

        case "name":
            receive_name(data["value"])
        case "cpf":
            receive_cpf(data["value"])
        case _:
            # ignore unknown type
            print("received invalid message")

def send_state(state, msg):
    payload = {
        "type": "state",
        "state": state.value,
        "msg": msg
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
    send_state(current_state, "What is your name?")

def handle_cancel():
    global current_state
    current_state = States.IDLE
    # TODO cancel everything
    send_state(current_state, "")
    pass

def receive_name(name):
    print("name:", name)
    # TODO implement

def receive_cpf(cpf):
    print("name:", cpf)
    # TODO implement

if __name__ == '__main__':
    print("Starting server on port 5000")
    socketio.run(app, host='0.0.0.0', port=5000)

