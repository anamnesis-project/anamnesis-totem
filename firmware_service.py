import asyncio
import aiomqtt as mqtt
from google import genai
import json
import logging
import os
import serial
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

MQTT_BROKER = "localhost"
MQTT_PORT = 1883
FW_INPUT = "fw/input"
FW_OUTPUT = "fw/output"

try:
    ser = serial.Serial('/dev/ttyS0', 115200, timeout=1)
    ser.flush()
    print("Serial port connected")
except serial.SerialException as e:
    ser = None
    print(f"Failed connecting to serial Port: ({e})")

async def fw_communication(command: str) -> str:
    try:
        logging.info("Sending command to firmware...")
        await asyncio.to_thread(ser.write, command.encode('ascii'))
        
        response_bytes = await asyncio.to_thread(ser.readline)
        if not response_bytes:
            logging.warning("No response from firmware")
            return False
        response = response_bytes.decode('utf-8').rstrip()
        print(f"[Pi <- FW] Response: {response}")
        return json.loads(response)

    except Exception as e:
        logging.error(f"Error calling Gemini API: {e}")
        return "Error calling Gemini API"

async def main():
    try:
        async with mqtt.Client(MQTT_BROKER, port=MQTT_PORT) as client:
            logging.info(f"Conected to MQTT Broker: {MQTT_BROKER}.")
            await client.subscribe(FW_INPUT)

            logging.info("Waiting for command...")
            async with client.messages() as messages:
                async for message in messages:
                    if message.topic.matches(FW_INPUT):
                        command = message.payload.decode('utf-8')
                        logging.info(f"Received command: {command}")
                        response = await fw_communication(command)
                        await client.publish(FW_OUTPUT, response)
                        
    except mqtt.exceptions.MqttError as e:
        logging.critical(f"ERROR: Could not connecto to MQTT at {MQTT_BROKER}:{MQTT_PORT}.")
        logging.critical(f"Detail: {e}")
    except KeyboardInterrupt:
        logging.info("Firmware Service terminated by user.")
    finally:
        logging.info("Firmware Service finished.")

if __name__ == "__main__":
    asyncio.run(main())

