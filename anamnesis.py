import asyncio
import aiomqtt as mqtt
from gemini_service import process_answer_context, interview_context, measure_context
from enum import Enum
import json
import os
import logging
import requests

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] (%(name)s) %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S"
)
log = logging.getLogger("Anamnesis")

MQTT_BROKER = "localhost"
MQTT_PORT = 1883
FW_INPUT = "fw/input"
FW_OUTPUT = "fw/output"
TOPIC_PROMPT = "llm/prompt"
LLM_RESPONSE = "llm/response"
TOPIC_SPEAK = "voice/speak/request" 
SPEAK_RESPONSE = "voice/speak/response"
MIC_START = "voice/mic_stt/start"
MIC_STOP = "voice/mic_stt/stop"
TOPIC_TRANSCRIPTION = "voice/mic_stt/transcription" 
TOPIC_DB_REQUEST = "db/request" 
TOPIC_DB_RESPONSE = "db/response"
UI_SEND = "ui/send"
UI_RECEIVE = "ui/receive"
TOPIC_CAM = "cam/input"
CAM_OUTPUT = "cam/output"

MAX_QUESTIONS = 5

SPEAK_SUCCESS_PAYLOAD = "ok"
STT_FAIL_PAYLOAD = 'failed'
MIC_START_PAYLOAD = "start"
LLM_CONTINUE_PAYLOAD = 'Continue'

FIRST_QUESTION = "What brings you here today?"
END_SENTENCE = "Thanks for the information. You can leave now."
LLM_ENOUGH = "I got enough info"
LLM_END_PAYLOAD = "End session"
FW_FAIL_PAYLOAD = "failed"
LLM_FAIL_PAYLOAD = "failed"

class Forms(Enum):
    NAME = (0, "name", "Hi, I am Anna, I'm a virtual assistant and I'm here to collect some information to speed up your check-in. Please, answer my questions and follow my instructions. Tell me, What is your name?")
    CPF = (1, "cpf", "What is your CPF?")
    AGE = (2, "age", "What is you day of birth?")
    SEX = (3, "sex", "What is your biological sex?")
    HEIGHT = (4, "height", "What is your height in centimeters?")
    WEIGHT = (5, "weight", "What is your weight in kilograms?")
    OCCUPATION = (6, "occupation", "What is your occupation?")
    MEDICATIONS = (7, "medications", "Are you taking any medications? If so, please list them.")
    ALLERGIES = (8, "allergies", "Do you have any known allergies? If so, please list them.")
    DISEASES = (9, "diseases", "Do you have any chronic illnesses? If so, please list them.")

    @property
    def index(self):
        return self.value[0]

    @property
    def step(self):
        return self.value[1]

    @property
    def question(self):
        return self.value[2]
    
    @classmethod
    def get_by_index(cls, index):
        for member in cls:
            if member.index == index:
                return member

class Measures(Enum):
    TEMPERATURE = (0, "temperature", "Now, we are measuring some vital signs. Please, place your forehead in front of the thermometer as shown on the screen. Tell me when you are ready")
    OXYMETER = (1, "oxymeter", "Please, put your finger on the oxymeter as shown on the screen.")
    PRESSURE_OPEN_DOOR = (2, "pressure", "Please, grab the cuff inside the totem and place it on your bare arm, and tell me when you are ready...")
    PRESSURE_START_MONITOR = (3, "pressure", "Please put the cuff back in the cabinet and tell me when it is done")
    PRESSURE_CLOSE_DOOR = (4, "pressure", "")

    @property
    def index(self):
        return self.value[0]

    @property
    def step(self):
        return self.value[1]

    @property
    def speach(self):
        return self.value[2]
    
    @classmethod
    def get_by_index(cls, index):
        for member in cls:
            if member.index == index:
                return member

class State(Enum):
    FORMS = 0
    MEASURES = 1
    INTERVIEW = 2
    IDLE = 3

class SessionState:
    def __init__(self):
        self.main_state = State.IDLE
        self.forms_state = 0
        self.measures_state = 0
        self.jsonPost = {
            "patient": {
                "name": "Luis Inacio",
                "cpf": "11111111111",
                "dateOfBirth": "2000-01-13T00:00:00Z",
                "sex": "M"
            },
            "weight": 10,
            "height": 99,
            "heartRate": 66,
            "systolicPressure": 12,
            "diastolicPressure": 8,
            "temperature": 36.5,
            "oxygenSaturation": 98,
            "occupation": "student",
            "medications": [],
            "allergies": ["eggs"],
            "diseases": ["dengue", "chikungunya"],
            "interview": [
                {
                    "question": "What brings you here today?",
                    "answer": "im feeling week"
                }
            ]
        }
        self.interview = []
        self.dinamic_context = interview_context

    def reset(self):
        self.__init__()       

async def main():
    Session = SessionState()
    try:
        async with mqtt.Client(MQTT_BROKER, port=MQTT_PORT) as client:
            
            await client.subscribe(SPEAK_RESPONSE)
            await client.subscribe(TOPIC_TRANSCRIPTION)
            await client.subscribe(FW_OUTPUT)
            await client.subscribe(LLM_RESPONSE)
            await client.subscribe(TOPIC_DB_RESPONSE)
            await client.subscribe(UI_RECEIVE)
            await client.subscribe(CAM_OUTPUT)
            log.info(f"Connected to Broker {MQTT_BROKER}.")
            async for message in client.messages:
                try:
                    payload = message.payload.decode('utf-8')
                    log.info(f"<== Message: Topic '{message.topic}': '{payload}'")
                except UnicodeDecodeError:
                    log.warning(f"Non UTF8 message got from {message.topic}")
                    continue

                if message.topic.matches(UI_RECEIVE):
                    ui_message = json.loads(payload)
                    if ui_message["type"] == "command":
                        if ui_message["action"] == "start" and Session.main_state == State.IDLE:
                            Session.main_state = State.FORMS
                            await ui_start(client)
                            question = Forms.get_by_index(Session.forms_state).question
                            await client.publish(TOPIC_SPEAK, question)
                        if ui_message["action"] == "cancel":
                            Session.reset()
                            await ui_cancel(client)

                if Session.main_state == State.FORMS:
                    await run_forms_flow(client, message, payload, Session)
                    
                elif Session.main_state == State.MEASURES:
                    await run_measures_flow(client, message, payload, Session)
                    if Session.main_state == State.INTERVIEW:
                        Session.interview.append(FIRST_QUESTION)
                        Session.dinamic_context += "\n[You]: " + FIRST_QUESTION
                        await ui_send_state(client, "interview", FIRST_QUESTION)
                        await client.publish(TOPIC_SPEAK, FIRST_QUESTION)

                elif Session.main_state == State.INTERVIEW:
                    await run_interview_flow(client, message, payload, Session)

    except mqtt.MqttError as e:
        log.critical(f"Erro crítico de MQTT: {e}. Encerrando.")
    except KeyboardInterrupt:
        log.info("Service ended by user")

async def run_forms_flow(client, message, payload, Session):
    if message.topic.matches(UI_RECEIVE):
        ui_message = json.loads(payload)
        if ui_message.get("type") == "command":
            return

        Session.jsonPost["patient"][ui_message["type"]] = ui_message["value"]
        Session.forms_state += 1
        print('form state:', Session.forms_state)
        question = Forms.get_by_index(Session.forms_state).question
        step = Forms.get_by_index(Session.forms_state).step
        await ui_send_state(client, "forms", question, step)
        await client.publish(TOPIC_SPEAK, question)

    elif message.topic.matches(SPEAK_RESPONSE):
        if payload != SPEAK_SUCCESS_PAYLOAD:
            log.warning(f"Audio_player_service failed: '{payload}'")
            return False #???
        if Session.forms_state >= Forms.AGE.index:
            await client.publish(MIC_START, MIC_START_PAYLOAD)
            
    elif message.topic.matches(TOPIC_TRANSCRIPTION):
        if payload == STT_FAIL_PAYLOAD:
            log.warning(f"Mic_stt_service failed: '{payload}'")
            return False #???
        content = build_llm_prompt(payload, Session)
        await client.publish(TOPIC_PROMPT, content)

    elif message.topic.matches(LLM_RESPONSE):
        if payload == LLM_FAIL_PAYLOAD:
            log.warning(f"Gemini_service failed: '{payload}'")
            return False #???
        elif payload == LLM_END_PAYLOAD:
            #RESET ALL
            return False
        Session.jsonPost[Forms.get_by_index(Session.forms_state).name] = payload
        Session.forms_state += 1
        print('form state:', Session.forms_state)
        if Session.forms_state >= len(Forms): #DISEASES
            Session.main_state = State.MEASURES
            print('Changing to MEASURES')
            speach = Measures.get_by_index(Session.measures_state).speach
            step =  Measures.get_by_index(Session.measures_state).step
            await ui_send_state(client, "measures", speach, step)
            await client.publish(TOPIC_SPEAK, Measures.get_by_index(Session.measures_state).speach)
        else:
            question = Forms.get_by_index(Session.forms_state).question
            step = Forms.get_by_index(Session.forms_state).step
            await ui_send_state(client, "forms", question, step)
            await client.publish(TOPIC_SPEAK, question)

async def run_measures_flow(client, message, payload, Session):
    if message.topic.matches(FW_OUTPUT):
        print('caiu fw: ' + str(Session.measures_state))
        if payload == FW_FAIL_PAYLOAD:
            log.warning(f"Firmware_service failed: '{payload}'")
            return False #???
        if Session.measures_state == Measures.TEMPERATURE.index:
            if payload.startswith("T:OK"):
                try:
                    parts = payload.split(':')
                    value = float(parts[2])
                    Session.jsonPost["temperature"] = value
                    log.info(f"Temperature recorded: {value}°C")
                    Session.measures_state += 1
                    speach = Measures.get_by_index(Session.measures_state).speach
                    step = Measures.get_by_index(Session.measures_state).step
                    await ui_send_state(client, "measures", speach, step)
                    await client.publish(TOPIC_SPEAK, speach)

                except (IndexError, ValueError):
                    log.warning(f"Invalid temperature '{payload}'")
            elif payload.startswith("T:ERR"):
                log.warning("Temperature measurement error.")
            else:
                log.warning(f"Unexpected temperature payload: '{payload}'")
        
        elif Session.measures_state == Measures.OXYMETER.index:
            if payload.startswith("O:OK"):
                try:
                    parts = payload.split(':')
                    value = int(parts[2])
                    Session.jsonPost["oxygen_saturation"] = value
                    log.info(f"Oxymeter recorded: {value}%")
                    Session.measures_state += 1
                    speach = Measures.get_by_index(Session.measures_state).speach
                    step = Measures.get_by_index(Session.measures_state).step
                    await ui_send_state(client, "measures", speach, step)
                    await client.publish(FW_INPUT, Measures.get_by_index(Session.measures_state).name)
                except (IndexError, ValueError):
                    log.warning(f"Invalid oxymeter '{payload}'")
            elif payload.startswith("O:ERR"):
                log.warning("Oxymeter measurement error.")
            else:
                log.warning(f"Unexpected oxymeter payload: '{payload}'")

        elif Session.measures_state == Measures.PRESSURE_OPEN_DOOR.index: 
            if payload.startswith("P1:OK"): #SUCCESS OPEN DOOR
                print('abriu')
                speach = Measures.get_by_index(Session.measures_state).speach
                step = Measures.get_by_index(Session.measures_state).step
                await ui_send_state(client, "measures", speach, step)
                await client.publish(TOPIC_SPEAK, speach)
                Session.measures_state += 1
                #VOICE COMMAND TO START MONITORING
                print('COMANDO PARA INICIAR MONITORAMENTO DE PRESSAO')

            elif payload.startswith("P1:ERR"):
                log.warning("Open pressure monitor door error.")
            else:
                log.warning(f"Unexpected open pressure monitor door payload: '{payload}'")

        elif Session.measures_state == Measures.PRESSURE_START_MONITOR.index:
            if payload.startswith("P:OK"):
                print('CAIU CAM')
                await client.publish(TOPIC_CAM, 'START READING')
            elif payload.startswith("P:ERR"):
                log.warning("Start pressure monitor error.")
            else:
                log.warning(f"Unexpected start pressure monitor payload: '{payload}'")

        elif Session.measures_state == Measures.PRESSURE_CLOSE_DOOR.index:
            if payload.startswith("P2:OK"):
                Session.main_state = State.INTERVIEW
                return True
            elif payload.startswith("P2:ERR"):
                log.warning("Close pressure monitor door error.")
            else:
                log.warning(f"Unexpected close pressure monitor door payload: '{payload}'")

    elif message.topic.matches(TOPIC_TRANSCRIPTION):
        if payload == STT_FAIL_PAYLOAD:
            log.warning(f"Mic_stt_service failed: '{payload}'")
            return False #???
        prompt = build_llm_prompt(payload, Session)
        await client.publish(TOPIC_PROMPT, prompt)

    elif message.topic.matches(LLM_RESPONSE):
        if payload == LLM_CONTINUE_PAYLOAD:
            await client.publish(FW_INPUT, Measures.get_by_index(Session.measures_state).name)
        else:
            log.warning(f"Unexpected start pressure monitor payload: '{payload}'")
            await client.publish(MIC_START, MIC_START_PAYLOAD)
            return False

    elif message.topic.matches(SPEAK_RESPONSE):
        if payload != SPEAK_SUCCESS_PAYLOAD:
            log.warning(f"Audio_player_service falhou: '{payload}'")
            return False #???
        if Session.measures_state in [Measures.TEMPERATURE.index,
                                      Measures.PRESSURE_OPEN_DOOR.index, 
                                      Measures.PRESSURE_START_MONITOR.index,
                                      Measures.PRESSURE_CLOSE_DOOR.index]: 
            await client.publish(MIC_START, MIC_START_PAYLOAD)

        if Session.measures_state == Measures.OXYMETER.index:
            await client.publish(FW_INPUT, Measures.get_by_index(Session.measures_state).name)
    
    elif message.topic.matches(CAM_OUTPUT):
        if payload.startswith("CAM:ERR"):
            log.warning("Camera error during pressure measurement.")
            return False
        elif payload.startswith("CAM:OK"):
            parts = payload.split(':')
            systolic_pressure = int(parts[2])
            diastolic_pressure = int(parts[3])
            heart_rate = int(parts[4])
            Session.jsonPost["systolic_pressure"] = systolic_pressure
            Session.jsonPost["diastolic_pressure"] = diastolic_pressure
            Session.jsonPost["heart_rate"] = heart_rate
            await client.publish(TOPIC_SPEAK, Measures.get_by_index(Session.measures_state).speach)
            Session.measures_state += 1
            
        
async def run_interview_flow(client, message, payload, Session):
    if message.topic.matches(SPEAK_RESPONSE):
        if payload != SPEAK_SUCCESS_PAYLOAD:
            log.warning(f"Audio_player_service falhou: '{payload}'")
            return False #???
        await client.publish(MIC_START, MIC_START_PAYLOAD)
            
    elif message.topic.matches(TOPIC_TRANSCRIPTION):
        if payload == STT_FAIL_PAYLOAD:
            log.warning(f"Mic_stt_service failed: '{payload}'")
            return False #???
        Session.interview.append(payload)
        if len(Session.interview)/2 >= MAX_QUESTIONS:
            return await end_session(Session, client, persist=True)
        prompt = build_llm_prompt(payload, Session)
        await client.publish(TOPIC_PROMPT, prompt)
        return True #???

    elif message.topic.matches(LLM_RESPONSE):
        if payload == LLM_FAIL_PAYLOAD:
            log.warning(f"Gemini_service failed: '{payload}'")
            return False #???
        elif payload == LLM_END_PAYLOAD:
            return await end_session(Session, client, persist=False)
        elif payload == LLM_ENOUGH:
            log.info("Got enough info")
            return await end_session(Session, client, persist=True)
        Session.interview.append(payload)
        Session.dinamic_context += "\n[You]: " + payload
        # TODO is this correct??
        await client.publish(TOPIC_SPEAK, payload)
        await ui_send_state(client, "interview", payload)
        
def build_llm_prompt(message, Session):
    if Session.main_state == State.FORMS:
        question = Forms.get_by_index(Session.forms_state).question
        prompt = (process_answer_context + question + '\nAnswer:\n' + message + '\nOutput:')
        return prompt
    elif Session.main_state == State.MEASURES:
        question = Measures.get_by_index(Session.measures_state).speach
        prompt = (measure_context + "\n[Question]" + question + "\n[Patient]: "+ message)
        print("prompt: " + prompt)
        return prompt
    elif Session.main_state == State.INTERVIEW:
        Session.dinamic_context += "\n[Patient]: " + message
        return Session.dinamic_context

def insert_cli(Session):
    url = 'https://kickless-untaxing-neil.ngrok-free.dev/reports'
    i_list = []
    for i in range(0, len(Session.interview), 2):
        item = {
            "question": Session.interview[i],
            "answer": Session.interview[i+1]
        }
        i_list.append(item)
    age = Session.jsonPost.get("AGE")
    date_of_birth = f"{age}T00:00:00Z" if age else None
    weight_val = Session.jsonPost.get("WEIGHT")
    weight = int(weight_val) if weight_val is not None else None
    height_val = Session.jsonPost.get("HEIGHT")
    height = int(height_val) if height_val is not None else None
    meds_str = Session.jsonPost.get("MEDICATIONS")
    medications = [med.strip() for med in meds_str.split(',')] if meds_str else None
    allergies_str = Session.jsonPost.get("ALLERGIES")
    allergies = [allg.strip() for allg in allergies_str.split(',')] if allergies_str else None
    diseases_str = Session.jsonPost.get("DISEASES")
    diseases = [dis.strip() for dis in diseases_str.split(',')] if diseases_str else None
    requestBody = {
        "patient": {
            "name": "Luis Inacio",
            "cpf": "11111111111",
            "dateOfBirth": date_of_birth,
            "sex": Session.jsonPost.get("SEX"),
        },
        "weight": weight,
        "height": height,
        "heartRate": Session.jsonPost.get("heart_rate"),
        "systolicPressure": Session.jsonPost.get('systolic_pressure'),
        "diastolicPressure": Session.jsonPost.get('diastolic_pressure'),
        "temperature": Session.jsonPost.get('temperature'),
        "oxygenSaturation": Session.jsonPost.get('oxygen_saturation'),
        "medications": medications,
        "allergies": allergies,
        "diseases": diseases,
        "interview": i_list
    }
    print(requestBody)
    response = requests.post(url, json=requestBody)
    print('\nstatus code: ' + str(response.status_code))
    print('\nresponse: ' + response.text)

async def end_session(Session, client, persist):
    if persist:
        insert_cli(Session)
    Session.reset()
    await client.publish(TOPIC_SPEAK, END_SENTENCE)
    await ui_cancel(client)

async def ui_start(client):
    await ui_send_state(client, "forms", "What is your name?", "name")

async def ui_cancel(client):
    await ui_send_state(client, "idle", "")

async def ui_send_state(client, state, msg, step=""):
    payload = {
        "type": "state",
        "state": state,
        "msg": msg,
        "step": step
    }
    output_string = json.dumps(payload)
    await client.publish(UI_SEND, output_string)

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        log.info("Program interrupted")
