import asyncio
import websockets
import aiomqtt as mqtt
import pyaudio
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

SAMPLE_RATE = 16000 #48000
CHUNK_SIZE = 2048 #4096
FORMAT = pyaudio.paInt16
CHANNELS = 1

async def send_audio_async(websocket, stream, active_event):
    """
    Send audio chunk to WebSocket in an async loop.
    """
    logging.info("Sending audio chunk...")
    try:
        while active_event.is_set():
            audio_chunk = await asyncio.to_thread(
                stream.read, CHUNK_SIZE, exception_on_overflow=False
            )
            await websocket.send(audio_chunk)
    except asyncio.CancelledError:
        logging.info("Error: cancelled sending audio")
    except Exception as e:
        logging.error(f"Error: failed to send audio: {e}")
    finally:
        logging.info("Finished sending audio")


async def receive_text_and_publish(websocket, mqtt_client, active_event):
    logging.info("Waiting for transcription...")
    try:
        async for message in websocket:
            logging.info(f"<< Received transcription: {message}")
            await mqtt_client.publish(TOPIC_TRANSCRIPTION, message)
            logging.info("Stopping session after receiving transcription.")
            break

    except asyncio.CancelledError:
        logging.info("Error: cancelled receiving transcription")
    except websockets.exceptions.ConnectionClosed as e:
        logging.info(f"STT server closed connection: {e.code}")
    except Exception as e:
        logging.error(f"Error receiving transcription: {e}")
    finally:
        logging.info("Finished receiving transcription")
        active_event.clear()


async def stt_session_manager(mqtt_client, stt_active_event):
    while True:
        await stt_active_event.wait()
        logging.info("Detected 'start' command, starting STT service...")
        p = None
        stream = None
        try:
            p = pyaudio.PyAudio()
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
            if stream:
                stream.stop_stream()
                stream.close()
                logging.info("Closed audio stream.")
            if p:
                p.terminate()
                logging.info("PyAudio ended.")
            
            logging.info("STT session ended, waiting on idle state")
            #stt_active_event.clear()

async def handle_mqtt_commands(client, stt_active_event):
    logging.info(f"Listening topics: '{MIC_START}' and '{MIC_STOP}'")
    try:
        await client.subscribe(MIC_START)
        await client.subscribe(MIC_STOP)
        
        async for message in client.messages:
            if message.topic.matches(MIC_START):
                if not stt_active_event.is_set():
                    logging.info("'start' command received. Activating STT.")
                    stt_active_event.set()
                else:
                    logging.warning("'start' command received but STT already on.")
                    
            elif message.topic.matches(MIC_STOP):
                if stt_active_event.is_set():
                    logging.info("stop' command received. Deactivating STT.")
                    # Limpa o evento, o que fará as tarefas 'send/receive' pararem
                    stt_active_event.clear()
                else:
                    logging.warning("'stop' command received, but STT already off.")
                    
    except Exception as e:
        logging.error(f"Error in MQTT loop: {e}")


async def main():
    logging.info("mic_stt_service starting...")
    
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