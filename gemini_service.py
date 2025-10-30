import asyncio
import aiomqtt as mqtt
from google import genai
import logging
import os
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

MQTT_BROKER = "localhost"
MQTT_PORT = 1883
TOPIC_PROMPT = "llm/prompt"
LLM_RESPONSE = "llm/response"

GEMINI_API_KEY = '' #get via os

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
            async for message in client.messages:
                if message.topic.matches(TOPIC_PROMPT):
                    prompt = message.payload.decode('utf-8')
                    logging.info(f"Received prompt: {prompt}")
                    response = await async_generate_content(prompt)
                    logging.info(f"Sending Gemini response ({LLM_RESPONSE}): {response}")
                    await client.publish(LLM_RESPONSE, response)
                    
    except mqtt.exceptions.MqttError as e:
        logging.critical(f"ERROR: Could not connecto to MQTT at {MQTT_BROKER}:{MQTT_PORT}.")
        logging.critical(f"Detail: {e}")
    except KeyboardInterrupt:
        logging.info("Gemini Service terminated by user.")
    finally:
        logging.info("Gemini Service finished.")

if __name__ == "__main__":
    asyncio.run(main())

process_answer_context = """You are an expert data extraction assistant. Your task is to process a user's answer to a question and extract only the core data needed for a database.

The user's answer comes from a speech-to-text (STT) service, so it may contain inaccuracies or conversational filler.

## Rules
1.  **Extract Core Data:** Only output the specific answer to the question.
2.  **Remove Filler:** Ignore conversational fillers like "Uhm," "I think," "Well," etc.
3.  **Correct STT Errors:** Fix common speech-to-text mistakes (e.g. "gmail.com" from "g male dot com").
4.  **Format Lists:** If the answer contains multiple items, separate them with a comma and a space.
5.  **Handle Negatives:** If the answer does not make sense or the user does not answer (e.g. "I dont know") output "N/A".
6.  **Be Concise:** Do not add any explanation or labels to your output.

## Examples

---
Input:
Question: Do you have any known allergies? If so, please list them.
Answer: Yes I have to allergies, I am allergic to milk and I don't fell well taking dipyrone.

Output:
milk, dipyrone

---
Input:
Question: How old are you?
Answer: I am an industrial engineering student.

Output:
N/A

---
Input:
Question: What's your email?
Answer: uhh, sure. it's test dot user at g male dot com.

Output:
test.user@gmail.com

---
Input:
Question: Do you have any children?
Answer: Nope, I don't have any kids.

Output:
No

---
Input:
Question: What is your current city?
Answer: I'm living in san fran sisco right now.

Output:
San Francisco

---
Input:
"""

interview_context = """
    **Context**: You are a medical triage assistant.
    Your task is to analyze the patients story,
    retreive the most important topics and generate the next most relevant question to ask them in order to further obtain important information.
    Don't repeat questions and don't go too deep on topic that has already been covered.
    Keep a professional tone throughout the conversation.
    If the conversation history is empty, generate the first question.
    Keep questions short and objective. Don't say "Thank you" or similar phrases. 
    Do not include markdown syntax in the answer. Respond with plain text only.
    If you feel you have retreived enough information, or there is no relevant question to be made, answer with
    "I got enough info"

    **Conversation history**:
"""