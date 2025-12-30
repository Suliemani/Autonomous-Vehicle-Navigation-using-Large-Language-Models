import openai
import json
import time
import re

# Import POI data and distance calculation functions
from POI_data import poi_specialties, compute_legal_distances

openai.api_key = "xxx"


def ask_gpt_for_route(poi_data_json, user_message, model="gpt-4"):
    # Convert POI specialties into formatted string for the prompt
    specialties_str = "\nAdditionally, these POIs have special attributes:\n"
    for poi, desc in poi_specialties.items():
        specialties_str += f" - {poi}: {desc}\n"

    # Construct system prompt with instructions for GPT
    system_prompt = (
        "You are an advanced route-planning AI. Below is JSON data describing "
        "all POIs and their driving-route distance (in meters) from the user's current location.\n"
        + poi_data_json +
        "\n\n" +
        specialties_str +
        "\n\nThe user says:\n"
        + user_message +
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
        # Call OpenAI API to get route suggestion
        response = openai.ChatCompletion.create(
            model=model,
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_message},
            ],
            temperature=0.5
        )
        content = response["choices"][0]["message"]["content"]
        plan = json.loads(content)

    except (json.JSONDecodeError, KeyError):
        # Return error if response parsing fails
        plan = {
            "decision": "error",
            "route": [],
            "explanation": "Invalid GPT response or JSON parse error"
        }

    return plan


def get_welcome_message():
    # Generate friendly welcome message using GPT
    prompt = (
        "You are an advanced vehicle AI. Please provide a short, friendly welcome "
        "message. Mention that the user can ask to drive to a hospital, school, "
        "cafe, or mall. If their request isn't the nearest in that same category, "
        "I'll suggest a closer alternative. Type 'requested location' or 'suggested location' "
        "to confirm. End with 'Have a great ride!'"
    )

    response = openai.ChatCompletion.create(
        model="gpt-4",
        messages=[
            {"role": "system", "content": "You are a helpful assistant."},
            {"role": "user", "content": prompt},
        ],
        temperature=0.5
    )

    return response["choices"][0]["message"]["content"].strip()


def parse_nearest_poi_from_gpt_explanation(expl_text):
    # Extract nearest POI mentioned in GPT's explanation text
    pattern = re.compile(
        r"([A-Z][\w]*(?:\s+[A-Z][\w]*)?) is .*closer",
        re.IGNORECASE
    )
    match = pattern.search(expl_text)
    if match:
        return match.group(1).strip()
    return None


def gpt_worker(in_q, out_q, vehicle, grp):
    # Worker thread that processes user requests through GPT
    while True:
        user_text = in_q.get()
        if user_text is None:
            break

        # Measure response time and get distance data
        start_t = time.time()
        distance_data = compute_legal_distances(vehicle, grp)
        distances_json = json.dumps(distance_data, indent=2)

        # Get route suggestion from GPT
        plan = ask_gpt_for_route(distances_json, user_text)
        response_time = time.time() - start_t

        # Put result in output queue
        out_q.put((user_text, plan, response_time))
