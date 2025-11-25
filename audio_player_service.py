import asyncio
import websockets
import aiomqtt as mqtt
import asyncio.subprocess
import os
from dotenv import load_dotenv

load_dotenv()
#server_path = os.environ.get('SERVER_PATH')
server_path = "192.168.18.47:8000"

MQTT_BROKER = "localhost"
MQTT_PORT = 1883
TOPIC_SPEAK = "voice/speak/request"
SPEAK_RESPONSE = "voice/speak/response"
# MQTT topic to listen for text
WEBSOCKET_URI = "ws://" + server_path + "/ws/tts" # TTS SERVER WebSocket URI
AUDIO_FILENAME = "speak.wav" # temporary file

SPEAKER_SUCCESS_PAYLOAD = 'ok'

async def tts_via_websocket(text: str, filename: str):
    """
    Connect to TTS via WebSocket, send text, receives and save audio file
    """
    print(f"Connecting to TTS Websocket in {WEBSOCKET_URI}...")
    try:
        async with websockets.connect(WEBSOCKET_URI) as websocket:
            await websocket.send(text)
            print(f"> Text sent to TTS: {text}")

            while True:
                message = await websocket.recv()
                if isinstance(message, bytes):
                    with open(filename, "wb") as f:
                        f.write(message)
                    print(f"Received audio file saved in '{filename}'")

                elif isinstance(message, str):
                    print(f"< TTS Confirmation: {message}")
                    break
        
        return True

    except websockets.exceptions.ConnectionClosedError as e:
        print(f"Error: TTS Websocket closed: {e}")
    except ConnectionRefusedError as e:
        print(f"Error: TTS Websocket refused connection")
    except Exception as e:
        print(f"Unexpected error: {e}")
    
    return False


async def play_audio_file(filename: str):
    """
    Play audio file using 'aplay' command asynchronously.
    """
    print(f"Playing file: {filename}")
    try:
        proc = await asyncio.create_subprocess_exec(
            'aplay', filename,
            stdout=asyncio.subprocess.DEVNULL,
            stderr=asyncio.subprocess.PIPE
        )
        _, stderr_data = await proc.communicate()
        
        if proc.returncode != 0:
            print(f"Error playing audio (aplay): {stderr_data.decode().strip()}")
        else:
            print("Audio played successfully.")
            
    except Exception as e:
        print(f"Unexpected error playing audio: {e}")


async def main():
    """
    Manage MQTT conection and message loop.
    """
    print("Starting audio player service")
    
    try:
        # Conect to MQTT broker
        async with mqtt.Client(MQTT_BROKER, port=MQTT_PORT) as client:
            print(f"Conected to MQTT Broker at {MQTT_BROKER}.")
            
            # subscribe to listen topic
            await client.subscribe(TOPIC_SPEAK)
            print(f"Listening MQTT topic: '{TOPIC_SPEAK}'")
            print("Waiting for text messages...")
            
            async for message in client.messages:
                try:
                    text = message.payload.decode('utf-8')
                    print(f"\n--- New MQTT message ---")
                    print(f"  > Received text: '{text}'")
                    
                    success_tts = await tts_via_websocket(text, AUDIO_FILENAME)
                    
                    # Play audio
                    if success_tts:
                        await play_audio_file(AUDIO_FILENAME)
                        await client.publish(SPEAK_RESPONSE, SPEAKER_SUCCESS_PAYLOAD)
                    else:
                        print("Failed playing audio from TTS server.")
                    
                except Exception as e:
                    print(f"Unexpected error in message loop: {e}")

    except Exception as e:
        print(f"ERROR: Failed to connect to MQTT: {e}")

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nInterrupted by user.")