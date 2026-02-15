import os
import requests
import time

def send_message(body: str):
    """
    Sends a text message to the configured Matrix room.
    """
    token = os.getenv("MATRIX_TOKEN")
    homeserver = os.getenv("MATRIX_HOMESERVER", "https://uavcgbs1.selfhost.eu").rstrip("/")
    room_id = os.getenv("MATRIX_ROOM_ID")
    
    if not token or not room_id:
        return "Error: MATRIX_TOKEN or MATRIX_ROOM_ID not set in environment."
        
    txid = int(time.time())
    url = f"{homeserver}/_matrix/client/v3/rooms/{room_id}/send/m.room.message/{txid}"
    headers = {"Authorization": f"Bearer {token}", "Content-Type": "application/json"}
    payload = {"msgtype": "m.text", "body": body}
    
    try:
        response = requests.put(url, headers=headers, json=payload)
        response.raise_for_status()
        return f"Message sent to Matrix: {body}"
    except Exception as e:
        return f"Failed to send Matrix message: {str(e)}"
