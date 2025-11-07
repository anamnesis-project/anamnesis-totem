const ws = io();
const app = document.getElementById("app");

let currentView = "idle";
let globalData = {};

const types = Object.freeze({
    STATE: "state",
    DATA:  "data",
    SYNC:  "sync"
});

const states = Object.freeze({
    IDLE:      "idle",
    FORMS:     "forms",
    MEASURES:  "measures",
    INTERVIEW: "interview",
    FINISHED:  "finished",
    CANCELLED: "cancelled"
});

const FormStep = Object.freeze({
    NAME:     "name",
    CPF:      "cpf",
    QUESTION: "question"
});

const MeasuresStep = Object.freeze({
    TEMPERATURE: "temperature",
    OXYMETER:    "oxymeter",
    PRESSURE:    "pressure"
});

ws.on("connect", () => console.log("Connected to WebSocket"));

ws.on("server_message", async (event) => {
    console.log(event)
    const data = JSON.parse(event);
    switch (data.type) {
        case types.STATE: {
            await updateState(data.state, data.step, data.msg);
            break;
        }
        case types.DATA: {
            updateData(data.field, data.value);
            break;
        }
        case types.SYNC: {
            syncData(data.data);
            break;
        }
        default: {
            // ignore malformed message
            break;
        }
    }
});

function sendStart() {
    const payload = JSON.stringify({
        type: "command",
        action: "start"
    });

    console.log("sending message:", payload);
    ws.emit("client_message", JSON.stringify({ type: "command", action: "start" }));
}

function sendCancel() {
    const payload = JSON.stringify({
        type: "command",
        action: "cancel"
    });

    console.log("sending message:", payload);
    ws.emit("client_message", payload);
}

function submitName(event) {
    event.preventDefault();

    const name = document.getElementById("nameInput").value;
    const payload = JSON.stringify({
        type: "name",
        value: name
    });

    console.log("sending message:", payload);
    ws.emit("client_message", payload);

    return false;
}

function submitCPF(event) {
    event.preventDefault();

    const cpf = document.getElementById("cpfInput").value;
    const payload = JSON.stringify({
        type: "cpf",
        value: cpf
    });

    console.log("sending message:", payload);
    ws.emit("client_message", payload);

    return false;
}

async function loadView(name, msg) {
    const res = await fetch(`views/${name}.html`);
    const html = await res.text();
    app.innerHTML = html;
    if (msg) {
        document.getElementById("msg").textContent = msg;
    }
    currentView = name;
    updateViewData();
    console.log("Loaded view:", name);
}

function updateViewData() {
    switch (currentView) {
        case "forms": {
            
            break;
        }
        case "temperature": {
            
            break;
        }
        case "oxygen": {
            
            break;
        }
        case "pressure": {
            
            break;
        }
        case "interview": {
            
            break;
        }
        default: {
            // do nothing on stateless views
            break;
        }
    }
}

async function updateState(state, step, msg) {
    switch (state) {
        case states.IDLE: {
            await loadView("idle");
            break;
        }
        case states.FORMS: {
            switch (step) {
                case FormStep.NAME: {
                    await loadView("name", msg);
                    break;
                }
                case FormStep.CPF: {
                    await loadView("cpf", msg);
                    break;
                }
                default: {
                    await loadView("forms", msg);
                    break;
                }
            }
            break;
        }
        case states.MEASURES: {
            switch (step) {
                case MeasuresStep.TEMPERATURE: {
                    await loadView("temperature", msg);
                    break;
                }
                case MeasuresStep.OXYMETER: {
                    await loadView("oxymeter", msg);
                    break;
                }
                case MeasuresStep.PRESSURE: {
                    await loadView("pressure", msg);
                    break;
                }
                default: {
                    // invalid step
                    break;
                }
            }
            break;
        }
        case states.INTERVIEW: {
            await loadView("interview");
            break;
        }
        case states.FINISHED: {
            await loadView("finished");
            break;
        }
        case states.CANCELLED: {
            await loadView("cancelled");
            break;
        }
        default: {
            // ignore malformed message
            break;
        }
    }
}

function updateData(field, value) {
    globalData["field"] = value;
    updateViewData();
}

function syncData(data) {
    globalData = data;
}
