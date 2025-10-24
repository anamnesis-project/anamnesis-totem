import asyncio
import aiomqtt as mqtt
from google import genai
import logging
import os
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

MQTT_BROKER = "localhost"
MQTT_PORT = 1883
TOPIC_PROMPT = "llm/prompt"
TOPIC_RESPONSE = "llm/response"

GEMINI_API_KEY = 'getAPIKEY'

client = genai.Client(api_key=GEMINI_API_KEY)

async def async_generate_content(context_to_send: str) -> str:
    try:
        logging.info("Sending prompt to Gemini...")
        response = await asyncio.to_thread(
            client.models.generate_content,
            model="gemini-2.5-flash",
            contents=context_to_send
        )
        logging.info("Got response from Gemini.")
        clean_text = response.text.replace('*', '').strip()
        return clean_text
    except Exception as e:
        logging.error(f"Error calling Gemini API: {e}")
        return "Error calling Gemini API"

async def main():
    try:
        async with mqtt.Client(MQTT_BROKER, port=MQTT_PORT) as client:
            logging.info(f"Conected to MQTT Broker: {MQTT_BROKER}.")
            await client.subscribe(TOPIC_PROMPT)

            logging.info("Waiting for prompt...")
            async with client.messages() as messages:
                async for message in messages:
                    if message.topic.matches(TOPIC_PROMPT):
                        prompt = message.payload.decode('utf-8')
                        logging.info(f"Received prompt: {prompt}")
                        response = await async_generate_content(prompt)
                        logging.info(f"Sending Gemini response ({TOPIC_RESPONSE}): {response}")
                        await client.publish(TOPIC_RESPONSE, response)
                        
    except mqtt.exceptions.MqttError as e:
        logging.critical(f"ERROR: Could not connecto to MQTT at {MQTT_BROKER}:{MQTT_PORT}.")
        logging.critical(f"Detail: {e}")
    except KeyboardInterrupt:
        logging.info("Gemini Service terminated by user.")
    finally:
        logging.info("Gemini Service finished.")

if __name__ == "__main__":
    asyncio.run(main())

