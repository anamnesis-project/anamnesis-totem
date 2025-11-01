from google import genai

# The client gets the API key from the environment variable `GEMINI_API_KEY`.
client = genai.Client()

context="""
    **Context**: You are a medical triage assistant.
    Your task is to analyze the patients story,
    retreive the most important topics and generate the next question to ask them in order to further obtain important information.
    Keep a empathetic and professional tone throughout the conversation.
    If the conversation history is empty, generate the first question.
    Keep questions short and objective.
    Do not include markdown syntax in the answer. Respond with plain text only.
    If you feel you have retreived enough information, answer with what you think the patient has. (This is not a diagnosis, so it's ok)

    **Conversation history**:
"""

while True:
    response = client.models.generate_content(
        model="gemini-2.5-flash",
        contents=context,
    )

    print(response.text)
    context += "\n[You]: " + response.text

    answer = input("Answer: ")
    context += "\n[Patient]: " + answer
