# Gemma-3 set up
import openai

OPENROUTER_API_KEY = "xxx"
openai.api_key = OPENROUTER_API_KEY

# Set the custom base URL for OpenRouter:
openai.api_base = "https://openrouter.ai/api/v1"

GEMMA_MODEL_NAME = "google/gemma-3-4b-it:free"


def ask_gemma_3_for_route(poi_data_json, user_message):
    specialties_str = "\nAdditionally, these POIs have special attributes:\n"
    for poi, desc in poi_specialties.items():
        specialties_str += f" - {poi}: {desc}\n"

    system_prompt = (
        "You are an advanced route-planning AI. Below is JSON data describing "
        "all POIs and their driving-route distance (in meters) from the user's current location.\n"
        f"{poi_data_json}\n\n"
        f"{specialties_str}\n\n"
        "The user says:\n"
        f"{user_message}\n\n"
        "\n\nImportant note:\n"
        "We have four categories of destinations:\n"
        "1) Hospital: a facility for diagnosing and treating health conditions "
        "(emergency care, clinics, general medical services)\n"
        "2) Mall: a large retail center with shops for clothing, shoes, accessories, general "
        "merchandise\n"
        "3) School: an educational setting for learning or research (universities, colleges, "
        "campuses)\n"
        "4) Cafe: a place for coffee, small meals, or produce\n"
        "Use these conceptual definitions to interpret user requests.\n"
        "\nYour tasks:\n"
        "1) Parse the user's request into one or multiple POIs.\n"
        "2) If the user's request is not the nearest, mention how much closer the nearest POI is "
        "*within the same category*.\n"
        "3) Produce a final route plan in JSON with this format:\n"
        '{"decision":"single|multi|error","route":[{"poi_name":"...","explanation":"..."}],"explanation":"..."}\n'
        "No extra text outside the JSON.\n"
        "Express distance differences in meters.\n"
        "Even if the user explicitly names a location (e.g., 'Cafe D'), you must still check if "
        "there is a closer POI in the same category.\n"
    )

    try:
        response = openai.ChatCompletion.create(
            model=GEMMA_MODEL_NAME,
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_message}
            ],
            temperature=0.5
        )
        content = response["choices"][0]["message"]["content"]

        # Strip code fences
        clean_json = strip_code_fences(content)

        try:
            plan = json.loads(clean_json)
        except json.JSONDecodeError as e:
            plan = {
                "decision": "error",
                "route": [],
                "explanation": "Invalid or fenced JSON"
            }

    except (KeyError, Exception) as e:
        print("DEBUG: Something went wrong with the call:", e)
        plan = {
            "decision": "error",
            "route": [],
            "explanation": f"Call error => {str(e)}"
        }

    return plan
