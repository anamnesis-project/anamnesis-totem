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


SPEAK_SUCCESS_PAYLOAD = "success"
STT_FAIL_PAYLOAD = 'failed'
MIC_START_PAYLOAD = "start"

FORMS_CONTEXT = """
Your task is to process a user answer and extract the core data for a response
to be stored in a database, this answer comes from a speach-to-text service, so
it can have some unaccurate data, try to fix it if has some mistake. Example:
Input = Question: Do you have any known allergies? If so, please list them.
Answer: Yes I have to allergies, I am allergic to milk and I don't fell well taking dipyrone.
Output = milk, dipyrone.
Question: How old are you? 
Answer: I am an industrial engineering student.
"""

main_state = 0
forms_state = 0
forms_flow_state = 0

class Forms(Enum):
    AGE = (0, "How old are you?")
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

# --- Definição dos Estados da Máquina ---
class State(Enum):
    FORMS = 0
    MEASURES = 1
    INTERVIEW = 2

async def main():
    try:
        async with mqtt.Client(MQTT_BROKER, port=MQTT_PORT) as client:
            
            await client.subscribe(SPEAK_RESPONSE)
            await client.subscribe(TOPIC_TRANSCRIPTION)
            await client.subscribe(FW_OUTPUT)
            await client.subscribe(LLM_RESPONSE)
            await client.subscribe(TOPIC_DB_RESPONSE)
            log.info(f"Connected to Broker {MQTT_BROKER}.")
            run_forms_flow(client)
            question = Forms(forms_state).question
            await client.publish(TOPIC_SPEAK, question)
            async for message in client.messages:
                try:
                    payload = message.payload.decode('utf-8')
                    log.info(f"<== MENSAGEM: Tópico '{message.topic}': '{payload}'")
                except UnicodeDecodeError:
                    log.warning(f"Mensagem não-UTF8 recebida em {message.topic}. Ignorando.")
                    continue

                if main_state == State.FORMS:
                    (
                        new_main_state, 
                        new_sub_state, 
                        new_form_index
                    ) = await run_forms_flow(client, message, payload)
                    
                    # Verifica se houve uma transição de ESTADO PRINCIPAL
                    if new_main_state != current_state:
                        log.info(f"[TRANSIÇÃO] Mudando de {current_state.name} para {new_main_state.name}")
                        
                        # Dispara a primeira ação do NOVO estado
                        if new_main_state == MainState.MEASURES:
                            log.info(f"[ESTADO: {new_main_state.name}] Iniciando fluxo.")
                            # Ex: await start_measures_flow(client)
                        
                        elif new_main_state == MainState.INTERVIEW:
                            # ...
                            pass
                    
                    # Atualiza os estados para a próxima iteração do loop
                    current_state = new_main_state
                    current_sub_state = new_sub_state
                    current_form_index = new_form_index

                elif current_state == MainState.MEASURES:
                    log.info(f"[ESTADO: {current_state.name}] (Lógica não implementada)")
                    # (new_main_state, ...) = await run_measures_flow(...)
                    # current_state = new_main_state
                
                elif current_state == MainState.INTERVIEW:
                    log.info(f"[ESTADO: {current_state.name}] (Lógica não implementada)")
                    # ...

                if current_state == MainState.COMPLETED:
                    log.info("Todos os fluxos concluídos. Encerrando.")
                    break # Sai do loop 'async for'

    except mqtt.MqttError as e:
        log.critical(f"Erro crítico de MQTT: {e}. Encerrando.")
    except KeyboardInterrupt:
        log.info("Orquestrador encerrado pelo usuário.")

# --- Ponto de Entrada do Script ---
if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        log.info("Programa interrompido.")

async def run_forms_flow(client, message, payload):
    if message.topic.matches(SPEAK_RESPONSE):
        if payload != SPEAK_SUCCESS_PAYLOAD:
            log.warning(f"Audio_player_service falhou: '{payload}'")
            return False #???
        await client.publish(MIC_START, MIC_START_PAYLOAD)
        return True #???
            
    elif message.topic.matches(TOPIC_TRANSCRIPTION):
        if payload != STT_FAIL_PAYLOAD:
            log.warning(f"Mic_stt_service falhou: '{payload}'")
            return False #???
        content = build_llm_prompt(payload, State.FORMS)
        await client.publish(TOPIC_PROMPT, content)
        return True #???

    elif message.topic.matches(LLM_RESPONSE):
        if payload != LLM_FAIL_PAYLOAD:
            log.warning(f"Gemini_service failed: '{payload}'")
            return False #???
        insert_db(client, payload, main_state, forms_state) #TODO 

    elif message.topic.matches(TOPIC_DB_RESPONSE):
        if payload != DB_FAIL_PAYLOAD:
            log.warnning("DB_service failed")
            return False #???
        forms_state += 1
        if forms_state > 5:
            main_state += 1
        else:
            question = Forms(forms_state).question
            await client.publish(TOPIC_SPEAK, question)
        return True #???

def build_llm_prompt(message, status):
    if status == State.FORMS:
        question = Forms(forms_state).question
        prompt = (process_answer_context + question + '\nAnswer:\n' + message + '\nOutput:')
        return prompt
    
def insert_db(client, information, main_state, forms_state=None):
    if main_state == State.FORMS:
        json = {
            
        }

# --- Ponto de Entrada do Script ---
if __name__ == "__main__":
    # Remove a variável 'state' global, pois a lógica agora
    # está encapsulada dentro da função 'main'
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        log.info("Programa interrompido.")