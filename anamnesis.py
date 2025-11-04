import asyncio
import aiomqtt as mqtt
from gemini_service import process_answer_context, interview_context
from enum import Enum
import os
import logging

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


SPEAK_SUCCESS_PAYLOAD = "ok"
STT_FAIL_PAYLOAD = 'failed'
MIC_START_PAYLOAD = "start"

FIRST_QUESTION = "What brings you here today?"
LLM_ENOUGH = "I got enough info"
LLM_REPEAT_PAYLOAD = "Repeat"
LLM_END_PAYLOAD = "End session"
FW_FAIL_PAYLOAD = "failed"
LLM_FAIL_PAYLOAD = "failed"

class Forms(Enum):
    AGE = (0, "Hi, I am Anna, I'm a virtual assistant and I'm here to collect some information to speed up your check-in. Please, answer my questions and follow my instructions. Tell me, how old are you?")
    SEX = (1, "What is your biological sex?")
    OCCUPATION = (2, "What is your occupation?")
    MEDICATIONS = (3, "Are you taking any medications? If so, please list them.")
    ALLERGIES = (4, "Do you have any known allergies? If so, please list them.")
    DISEASES = (5, "Do you have any chronic illnesess? If so, please list them.")

    @property
    def index(self):
        return self.value[0]

    @property
    def question(self):
        return self.value[1]
    
    @classmethod
    def get_by_index(cls, index):
        for member in cls:
            if member.index == index:
                return member

class Measures(Enum):
    TEMPERATURE = (0, "Now, we are measuring some vital signs. Please, place your forehead in front of the thermometer as shown on the screen.")
    OXYMETER = (1, "Please, put your finger on the oxymeter as shown on the screen.")
    PRESSURE_OPEN_DOOR = (2, "Please, grab the cuff inside the totem and place it on your bare arm...")
    PRESSURE_START_MONITOR = (3, "Please place the monitor back and say done when it is done")
    PRESSURE_CLOSE_DOOR = (4, "")

    @property
    def index(self):
        return self.value[0]

    @property
    def speach(self):
        return self.value[1]
    
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

main_state = State.FORMS
forms_state = 0
measures_state = 0
jsonPost = {}
interview = []
dinamic_context = interview_context

async def main():
    global main_state, dinamic_context
    try:
        async with mqtt.Client(MQTT_BROKER, port=MQTT_PORT) as client:
            
            await client.subscribe(SPEAK_RESPONSE)
            await client.subscribe(TOPIC_TRANSCRIPTION)
            await client.subscribe(FW_OUTPUT)
            await client.subscribe(LLM_RESPONSE)
            await client.subscribe(TOPIC_DB_RESPONSE)
            log.info(f"Connected to Broker {MQTT_BROKER}.")
            question = Forms.get_by_index(forms_state).question
            await client.publish(TOPIC_SPEAK, question)
            async for message in client.messages:
                try:
                    payload = message.payload.decode('utf-8')
                    log.info(f"<== Message: Topic '{message.topic}': '{payload}'")
                except UnicodeDecodeError:
                    log.warning(f"Non UTF8 message got from {message.topic}")
                    continue

                if main_state == State.FORMS:
                    await run_forms_flow(client, message, payload)
                    if main_state == State.MEASURES:
                        await client.publish(TOPIC_SPEAK, Measures.get_by_index(measures_state).speach)
                        #await client.publish(TOTEM SCREEN)
                        await client.publish(FW_INPUT, Measures.get_by_index(measures_state).name)
                    
                elif main_state == State.MEASURES:
                    await run_measures_flow(client, message, payload)
                    if main_state == State.INTERVIEW:
                        interview.append(FIRST_QUESTION)
                        dinamic_context += "\n[You]: " + FIRST_QUESTION
                        await client.publish(TOPIC_SPEAK, FIRST_QUESTION)

                elif main_state == State.INTERVIEW:
                    await run_interview_flow(client, message, payload)

    except mqtt.MqttError as e:
        log.critical(f"Erro crítico de MQTT: {e}. Encerrando.")
    except KeyboardInterrupt:
        log.info("Orquestrador encerrado pelo usuário.")

async def run_forms_flow(client, message, payload):
    global main_state, forms_state
    if message.topic.matches(SPEAK_RESPONSE):
        print('caiu')
        if payload != SPEAK_SUCCESS_PAYLOAD:
            log.warning(f"Audio_player_service failed: '{payload}'")
            return False #???
        await client.publish(MIC_START, MIC_START_PAYLOAD)
        return True #???
            
    elif message.topic.matches(TOPIC_TRANSCRIPTION):
        if payload == STT_FAIL_PAYLOAD:
            log.warning(f"Mic_stt_service failed: '{payload}'")
            return False #???
        content = build_llm_prompt(payload, State(main_state))
        await client.publish(TOPIC_PROMPT, content)
        return True #???

    elif message.topic.matches(LLM_RESPONSE):
        if payload == LLM_FAIL_PAYLOAD:
            log.warning(f"Gemini_service failed: '{payload}'")
            return False #???
        elif payload == LLM_REPEAT_PAYLOAD:
            question = Forms.get_by_index(forms_state).question
            await client.publish(TOPIC_SPEAK, question)
            return True
        elif payload == LLM_END_PAYLOAD:
            #RESET ALL
            return False
        jsonPost[Forms.get_by_index(forms_state)] = message
        forms_state += 1
        print('form state:', forms_state)
        if forms_state >= 5: #DISEASES
            main_state = State.MEASURES
            print('Changing to MEASURES')
        else:
            question = Forms.get_by_index(forms_state).question
            await client.publish(TOPIC_SPEAK, question)
        return True #???

async def run_measures_flow(client, message, payload):
    global main_state, measures_state
    if message.topic.matches(FW_OUTPUT):
        if payload == FW_FAIL_PAYLOAD:
            log.warning(f"Firmware_service failed: '{payload}'")
            return False #???
        if measures_state == Measures.TEMPERATURE.index:
            if payload.startswith("T:OK"):
                try:
                    parts = payload.split(':')
                    value = float(parts[2])
                    jsonPost["temperature"] = value
                    log.info(f"Temperature recorded: {value}°C")
                    measures_state += 1
                    #await client.publish(SCREEN SHOW RESULT)
                    await client.publish(TOPIC_SPEAK, Measures.get_by_index(measures_state).speach)
                    await client.publish(FW_INPUT, Measures.get_by_index(measures_state).name)

                except (IndexError, ValueError):
                    log.warning(f"Invalid temperature '{payload}'")
            elif payload.startswith("T:ERR"):
                log.warning("Temperature measurement error.")
            else:
                log.warning(f"Unexpected temperature payload: '{payload}'")
        
        elif measures_state == Measures.OXYMETER.index:
            if payload.startswith("O:OK"):
                try:
                    parts = payload.split(':')
                    value = int(parts[2])
                    jsonPost["oxygen_saturation"] = value
                    log.info(f"Oxymeter recorded: {value}%")
                    measures_state += 1
                    #await client.publish(SCREEN SHOW RESULT)
                    await client.publish(TOPIC_SPEAK, Measures.get_by_index(measures_state).speach)
                    await client.publish(FW_INPUT, Measures.get_by_index(measures_state).name)
                except (IndexError, ValueError):
                    log.warning(f"Invalid oxymeter '{payload}'")
            elif payload.startswith("O:ERR"):
                log.warning("Oxymeter measurement error.")
            else:
                log.warning(f"Unexpected oxymeter payload: '{payload}'")

        elif measures_state == Measures.PRESSURE_OPEN_DOOR.index:
            if payload.startswith("P0:OK"):
                measures_state += 1
                #await client.publish(SCREEN SHOW RESULT)
                await client.publish(TOPIC_SPEAK, Measures.get_by_index(measures_state).speach)
                #VOICE COMMAND TO START MONITORING
                await client.publish(TOPIC_SPEAK, 'TELL ME WHEN READY')
                await client.publish(MIC_START, MIC_START_PAYLOAD)
            elif payload.startswith("P0:ERR"):
                log.warning("Open pressure monitor door error.")
            else:
                log.warning(f"Unexpected open pressure monitor door payload: '{payload}'")

    elif message.topic.matches(TOPIC_TRANSCRIPTION):
        if measures_state == Measures.PRESSURE_START_MONITOR.index:
            if payload.startswith("P1:OK"):
                measures_state += 1
                await client.publish(FW_INPUT, Measures.get_by_index(measures_state).name)
                await client.publish(TOPIC_CAM, 'START READING')
            elif payload.startswith("P1:ERR"):
                log.warning("Start pressure monitor error.")
            else:
                log.warning(f"Unexpected start pressure monitor payload: '{payload}'")

    elif message.topic.matches(CAM_OUTPUT):
        elif measures_state == Measures.PRESSURE_CLOSE_DOOR.index:
            if payload.startswith("P2:OK"):
                measures_state += 1
            elif payload.startswith("P2:ERR"):
                log.warning("Close pressure monitor door error.")
            else:
                log.warning(f"Unexpected close pressure monitor door payload: '{payload}'")

        return True


         
        
        #processes_fw_output(payload)
        #main_state = State.INTERVIEWS

async def run_interview_flow(client, message, payload):
    global main_state, dinamic_context
    if message.topic.matches(SPEAK_RESPONSE):
        if payload != SPEAK_SUCCESS_PAYLOAD:
            log.warning(f"Audio_player_service falhou: '{payload}'")
            return False #???
        await client.publish(MIC_START, MIC_START_PAYLOAD)
        return True #???
            
    elif message.topic.matches(TOPIC_TRANSCRIPTION):
        if payload == STT_FAIL_PAYLOAD:
            log.warning(f"Mic_stt_service failed: '{payload}'")
            return False #???
        interview.append(payload)
        if len(interview)/2 >= 10:
            a=1 #STOP
        prompt = build_llm_prompt(payload, State(main_state))
        await client.publish(TOPIC_PROMPT, prompt)
        return True #???

    elif message.topic.matches(LLM_RESPONSE):
        if payload == LLM_FAIL_PAYLOAD:
            log.warning(f"Gemini_service failed: '{payload}'")
            return False #???
        elif payload == LLM_REPEAT_PAYLOAD:
            question = Forms.get_by_index(forms_state).question
            await client.publish(TOPIC_SPEAK, question)
            return True
        elif payload == LLM_END_PAYLOAD:
            #RESET ALL
            return False
        elif payload == LLM_ENOUGH:
            log.info("Got enough info")
            main_state = State.IDLE
        interview.append(payload)
        dinamic_context += "\n[You]: " + payload
        await client.publish(TOPIC_SPEAK, payload)
        
def build_llm_prompt(message, status):
    global dinamic_context, forms_state
    if status == State.FORMS:
        question = Forms.get_by_index(forms_state).question
        prompt = (process_answer_context + question + '\nAnswer:\n' + message + '\nOutput:')
        return prompt
    if status == State.INTERVIEW:
        dinamic_context += "\n[Patient]: " + message
        return dinamic_context

#def insert_db(client, information, payload):

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        log.info("Program interrupted")