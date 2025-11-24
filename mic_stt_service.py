import asyncio
import websockets
import aiomqtt as mqtt
import pyaudio
import logging
import os
from dotenv import load_dotenv
from ctypes import *
from contextlib import contextmanager

# --- CONFIGURAÇÃO DE LOGS ---
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
load_dotenv()

# --- CONSTANTES ---
server_ip = os.environ.get('SERVER_PATH', 'localhost') # Fallback para localhost se nulo
MQTT_BROKER = "localhost"
MQTT_PORT = 1883
MIC_START = "voice/mic_stt/start"
MIC_STOP = "voice/mic_stt/stop"
TOPIC_TRANSCRIPTION = "voice/mic_stt/transcription"
WEBSOCKET_URI = f"ws://{server_ip}/ws/stt" # f-string é mais limpo

WEBSOCKET_URI = "ws://"+ server_ip +"/ws/stt"

SAMPLE_RATE = 16000 #48000
CHUNK_SIZE = 2048 #4096
FORMAT = pyaudio.paInt16
CHANNELS = 1

# --- O TRUQUE PARA SILENCIAR O ALSA ---
ERROR_HANDLER_FUNC = CFUNCTYPE(None, c_char_p, c_int, c_char_p, c_int, c_char_p)

def py_error_handler(filename, line, function, err, fmt):
    pass # Não faz nada, engole o erro

c_error_handler = ERROR_HANDLER_FUNC(py_error_handler)

@contextmanager
def no_alsa_error():
    try:
        asound = cdll.LoadLibrary('libasound.so')
        asound.snd_lib_error_set_handler(c_error_handler)
        yield
        asound.snd_lib_error_set_handler(None) # Restaura o handler padrão
    except OSError:
        # Se não achar a libasound (ex: rodando no Windows), apenas segue a vida
        yield 

# --- FUNÇÕES DO SISTEMA ---

async def send_audio_async(websocket, stream, active_event):
    logging.info("Start sending audio chunks...")
    try:
        while active_event.is_set():
            # Use asyncio.to_thread para não bloquear o loop de eventos com I/O de áudio
            audio_chunk = await asyncio.to_thread(
                stream.read, CHUNK_SIZE, exception_on_overflow=False
            )
            await websocket.send(audio_chunk)
            # Removi o logging.info aqui de dentro, senão gera spam infinito
    except asyncio.CancelledError:
        logging.info("Sending audio task cancelled")
    except Exception as e:
        logging.error(f"Error sending audio: {e}")
    finally:
        logging.info("Stopped sending audio chunks")


async def receive_text_and_publish(websocket, mqtt_client, active_event):
    logging.info("Waiting for transcription...")
    try:
        async for message in websocket:
            logging.info(f"<< Received transcription: {message}")
            
            # Se a mensagem for válida, publica e encerra
            if message:
                await mqtt_client.publish(TOPIC_TRANSCRIPTION, message)
                logging.info("Transcription published. Stopping session.")
                break 
            
    except asyncio.CancelledError:
        logging.info("Receive transcription task cancelled")
    except websockets.exceptions.ConnectionClosed as e:
        logging.warning(f"STT server closed connection: {e.code}")
    except Exception as e:
        logging.error(f"Error receiving transcription: {e}")
    finally:
        active_event.clear() # Garante que o loop de envio pare também


async def stt_session_manager(mqtt_client, stt_active_event):
    # 1. Instanciamos o PyAudio FORA do loop while
    # Isso faz o scan de drivers apenas uma vez no boot do programa.
    p = pyaudio.PyAudio()
    logging.info("PyAudio initialized (single instance).")

    try:
        while True:
            await stt_active_event.wait()
            logging.info("Detected 'start' command, starting STT service...")
            
            stream = None
            try:
                # 2. Usamos a instância 'p' já existente para abrir o stream
                stream = p.open(
                    format=FORMAT,
                    channels=CHANNELS,
                    rate=SAMPLE_RATE,
                    input=True,
                    frames_per_buffer=CHUNK_SIZE
                )
                logging.info("Microphone active")
                
                async with websockets.connect(WEBSOCKET_URI) as websocket:
                    logging.info(f"Connecting to STT server at: {WEBSOCKET_URI}")
                    
                    send_task = asyncio.create_task(
                        send_audio_async(websocket, stream, stt_active_event)
                    )
                    receive_task = asyncio.create_task(
                        receive_text_and_publish(websocket, mqtt_client, stt_active_event)
                    )
                    
                    await asyncio.gather(send_task, receive_task)
                
            except asyncio.CancelledError:
                logging.info("STT session cancelled.")
                stt_active_event.clear()
            except ConnectionRefusedError:
                logging.error(f"STT connection refused: {WEBSOCKET_URI}")
                stt_active_event.clear()
            except Exception as e:
                logging.error(f"Unexpected error at STT service: {e}")
                stt_active_event.clear()
            finally:
                # 3. No loop interno, fechamos APENAS o stream
                if stream:
                    stream.stop_stream()
                    stream.close()
                    logging.info("Closed audio stream.")
                
                logging.info("STT session ended, waiting on idle state")

    except asyncio.CancelledError:
        # Captura o cancelamento do programa principal para fechar o PyAudio corretamente
        logging.info("Stopping Session Manager...")
    finally:
        # 4. O PyAudio só é encerrado quando o script todo morre
        if p:
            p.terminate()
            logging.info("PyAudio ended.")

async def handle_mqtt_commands(client, stt_active_event):
    logging.info(f"Listening MQTT topics: '{MIC_START}' / '{MIC_STOP}'")
    await client.subscribe(MIC_START)
    await client.subscribe(MIC_STOP)
    
    async for message in client.messages:
        if message.topic.matches(MIC_START):
            if not stt_active_event.is_set():
                logging.info("CMD: START")
                stt_active_event.set()
            else:
                logging.debug("CMD: START ignored (already active)")
                
        elif message.topic.matches(MIC_STOP):
            if stt_active_event.is_set():
                logging.info("CMD: STOP")
                stt_active_event.clear()
            else:
                logging.debug("CMD: STOP ignored (already idle)")

async def main():
    logging.info("Service starting...")
    stt_active_event = asyncio.Event()
    
    try:
        async with mqtt.Client(MQTT_BROKER, port=MQTT_PORT) as client:
            logging.info(f"Connected to MQTT Broker {MQTT_BROKER}")
            
            await asyncio.gather(
                handle_mqtt_commands(client, stt_active_event),
                stt_session_manager(client, stt_active_event)
            )
            
    except mqtt.exceptions.MqttError as e:
        logging.critical(f"MQTT Error: {e}")
    except KeyboardInterrupt:
        logging.info("Service stopped by user.")

if __name__ == "__main__":
    asyncio.run(main())