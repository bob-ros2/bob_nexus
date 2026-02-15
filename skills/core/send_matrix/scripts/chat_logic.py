import os
import time

import requests


def send_matrix_message(text: str):
    """
    Sends a message to the primary Matrix bridge room.
    """
    token = os.getenv("MATRIX_TOKEN")
    homeserver = os.getenv("MATRIX_HOMESERVER", "https://uavcgbs1.selfhost.eu").rstrip("/")
    room_id = os.getenv("MATRIX_ROOM_ID")

    if not token or not room_id:
        return "Error: Matrix environment variables (TOKEN, ROOM_ID) not set."

    url = f"{homeserver}/_matrix/client/v3/rooms/{room_id}/send/m.room.message/{int(time.time())}"
    headers = {"Authorization": f"Bearer {token}", "Content-Type": "application/json"}
    payload = {"msgtype": "m.text", "body": text}

    try:
        response = requests.put(url, headers=headers, json=payload)
        response.raise_for_status()
        return "Message successfully relayed to Matrix."
    except Exception as e:
        return f"Failed to send to Matrix: {e}"
