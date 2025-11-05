import asyncio
import aiomqtt as mqtt
# import pyaudio # REMOVIDO
# import websockets # REMOVIDO
import logging
import os
from dotenv import load_dotenv

logging.basicConfig(level=logging.INFO)
load_dotenv()
server_ip = os.environ.get('SERVER_PATH')


MQTT_BROKER = "localhost"
MQTT_PORT = 1883
MIC_START = "voice/mic_stt/start"
MIC_STOP = "voice/mic_stt/stop"
TOPIC_TRANSCRIPTION = "voice/mic_stt/transcription" # Para onde vai o texto final

WEBSOCKET_URI = "ws://"+ server_ip +"/ws/stt"

# Função send_audio_async REMOVIDA
# Função receive_text_and_publish REMOVIDA

async def stt_session_manager(mqtt_client, stt_active_event):
    """
    MODIFICADO: Em vez de gerenciar áudio/websocket, agora espera por
    input no terminal quando o evento stt_active_event é disparado.
    """
    while True:
        # 1. Espera pelo comando 'start'
        await stt_active_event.wait()
        logging.info("Detected 'start' command, waiting for terminal input...")
        
        try:
            # 2. Solicita o input no terminal
            # Usamos print com flush=True para garantir que o prompt apareça imediatamente
            print("\n[MODO TESTE] Digite a transcrição simulada e pressione Enter: ", end="", flush=True)

            # 3. Roda o input() bloqueante em uma thread separada
            # Isso impede que o input() trave o loop de eventos do asyncio
            transcription = await asyncio.to_thread(input)

            # 4. Verifica se o evento ainda está ativo (não recebeu 'stop' enquanto digitava)
            if stt_active_event.is_set():
                logging.info(f"Simulating transcription: '{transcription}'")
                # 5. Publica o texto digitado no tópico de transcrição
                await mqtt_client.publish(TOPIC_TRANSCRIPTION, transcription)
                logging.info(f"Published simulated text to '{TOPIC_TRANSCRIPTION}'")
            else:
                logging.info("Input received, but 'stop' command was issued. Discarding.")

        except asyncio.CancelledError:
            logging.info("STT session (test mode) cancelled.")
        except Exception as e:
            logging.error(f"Unexpected error in STT service (test mode): {e}")
        finally:
            # 6. Limpa o evento e volta ao estado de espera
            logging.info("STT session (test mode) ended, waiting on idle state")
            stt_active_event.clear()

async def handle_mqtt_commands(client, stt_active_event):
    """
    (Esta função permanece inalterada)
    Escuta os tópicos MQTT de 'start' e 'stop' 
    para controlar o stt_active_event.
    """
    logging.info(f"Listening topics: '{MIC_START}' and '{MIC_STOP}'")
    try:
        await client.subscribe(MIC_START)
        await client.subscribe(MIC_STOP)
        
        async for message in client.messages:
            if message.topic.matches(MIC_START):
                if not stt_active_event.is_set():
                    logging.info("'start' command received. Activating STT (test mode).")
                    stt_active_event.set()
                else:
                    logging.warning("'start' command received but STT already on.")
                    
            elif message.topic.matches(MIC_STOP):
                if stt_active_event.is_set():
                    logging.info("stop' command received. Deactivating STT (test mode).")
                    stt_active_event.clear()
                else:
                    logging.warning("'stop' command received, but STT already off.")
                    
    except Exception as e:
        logging.error(f"Error in MQTT loop: {e}")


async def main():
    """
    (Esta função permanece inalterada)
    Configura o cliente MQTT e inicia as tarefas principais.
    """
    logging.info("mic_stt_service starting... (TEST MODE)")
    
    stt_active_event = asyncio.Event()
    
    try:
        async with mqtt.Client(MQTT_BROKER, port=MQTT_PORT) as client:
            mqtt_commands_task = asyncio.create_task(
                handle_mqtt_commands(client, stt_active_event)
            )
            stt_manager_task = asyncio.create_task(
                stt_session_manager(client, stt_active_event)
            )
            
            await asyncio.gather(mqtt_commands_task, stt_manager_task)
            
    except mqtt.exceptions.MqttError as e:
        logging.critical(f"ERROR: Could not connect to MQTT broker {MQTT_BROKER}:{MQTT_PORT}.")
        logging.critical(f"Detail: {e}")
    except KeyboardInterrupt:
        logging.info("mic_stt_service stoped by used.")
    finally:
        logging.info("mic_stt_service ended.")

if __name__ == "__main__":
    asyncio.run(main())