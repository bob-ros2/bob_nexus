import os
import requests
import json

def store_memory(content: str, title: str = "Untitled Moment"):
    """
    Saves a piece of information, a thought, or a poem to the Nexus persistent memory (Qdrant).
    """
    qdrant_url = os.getenv("QDRANT_URL", "http://localhost:6333").rstrip("/")
    collection = os.getenv("COLLECTION_NAME", "mastermind_memory")
    
    # Simple document storage via metadata/payload. 
    # Since we might not have an embedding model here, we use a dummy vector if necessary,
    # or just store it if the collection allows.
    
    point_id = int(os.getpid() + hash(content) % 1000000)
    url = f"{qdrant_url}/collections/{collection}/points?wait=true"
    
    # We assume a collection with vector size 1 (dummy) for simple document storage
    payload = {
        "points": [
            {
                "id": point_id,
                "vector": [0.0], # Dummy vector
                "payload": {
                    "title": title,
                    "content": content,
                    "timestamp": os.popen("date").read().strip()
                }
            }
        ]
    }
    
    try:
        # Check if collection exists first
        coll_check = requests.get(f"{qdrant_url}/collections/{collection}")
        if coll_check.status_code != 200:
            # Create simple collection
            requests.put(f"{qdrant_url}/collections/{collection}", json={
                "vectors": {"size": 1, "distance": "Cosine"}
            })

        response = requests.put(url, json=payload)
        response.raise_for_status()
        return f"Successfully stored memory in Qdrant (ID: {point_id}): {title}"
    except Exception as e:
        # Fallback to local JSON if Qdrant is unreachable
        backup_path = "memory/chat_memory_fallback.json"
        os.makedirs("memory", exist_ok=True)
        try:
            data = []
            if os.path.exists(backup_path):
                with open(backup_path, "r") as f: data = json.load(f)
            data.append({"title": title, "content": content})
            with open(backup_path, "w") as f: json.dump(data, f, indent=2)
            return f"Qdrant unreachable ({e}). Saved to local fallback instead: {title}"
        except Exception as e2:
            return f"Error storing memory: {e2}"

def search_memory(query: str):
    """
    Searches the Nexus memory for relevant past information.
    """
    # ... Implementation for search ...
    return "Search functionality via Qdrant is currently being synced. Check local fallback for now."
