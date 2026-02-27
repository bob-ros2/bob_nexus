import os
import json
from datetime import datetime

# Optional: try to import qdrant-client
try:
    from qdrant_client import QdrantClient
    from qdrant_client.http.models import Distance, VectorParams
    HAS_QDRANT = True
except ImportError:
    HAS_QDRANT = False

class MemoryManager:
    def __init__(self):
        self.backend = os.getenv("MEMORY_BACKEND", "").lower()
        self.qdrant_url = os.getenv("QDRANT_URL")
        self.collection = os.getenv("COLLECTION_NAME", "nexus_memory")
        self.storage_dir = "memory"
        os.makedirs(self.storage_dir, exist_ok=True)
        
        # Logic to decide backend
        if self.backend == "json":
            self.mode = "json"
        elif self.qdrant_url and HAS_QDRANT:
            self.mode = "qdrant_remote"
            self.client = QdrantClient(url=self.qdrant_url)
        elif HAS_QDRANT:
            self.mode = "qdrant_local"
            db_path = os.path.join(self.storage_dir, "qdrant_db")
            self.client = QdrantClient(path=db_path)
        else:
            self.mode = "json"

        # Initialize collection if needed
        if "qdrant" in self.mode:
            self._ensure_collection()

    def _ensure_collection(self):
        try:
            collections = self.client.get_collections().collections
            exists = any(c.name == self.collection for c in collections)
            if not exists:
                self.client.create_collection(
                    collection_name=self.collection,
                    vectors_config=VectorParams(size=1, distance=Distance.COSINE),
                )
        except Exception:
            self.mode = "json" # Force fallback on error

    def save_memory(self, content: str, metadata: dict = None):
        """
        Saves a piece of information, a thought, or a fact to persistent memory.
        """
        if metadata is None: metadata = {}
        metadata["timestamp"] = datetime.now().isoformat()
        
        if "qdrant" in self.mode:
            try:
                import uuid
                from qdrant_client.http.models import PointStruct
                self.client.upsert(
                    collection_name=self.collection,
                    points=[
                        PointStruct(
                            id=str(uuid.uuid4()),
                            vector=[0.0], # Dummy vector for now (metadata search)
                            payload={"content": content, **metadata}
                        )
                    ]
                )
                return f"Memory stored in Qdrant ({self.mode})"
            except Exception as e:
                return f"Qdrant error: {e}. Consider switching to MEMORY_BACKEND=json"
        
        # JSON Backend
        return self._save_json(content, metadata)

    def _save_json(self, content, metadata):
        path = os.path.join(self.storage_dir, "memory.json")
        data = []
        if os.path.exists(path):
            with open(path, "r") as f:
                try: data = json.load(f)
                except: data = []
        
        data.append({"content": content, "metadata": metadata})
        with open(path, "w") as f:
            json.dump(data, f, indent=2)
        return "Memory stored in local JSON file."

    def search_memory(self, query: str, limit: int = 5):
        """
        Searches memory for relevant information.
        """
        if "qdrant" in self.mode:
            try:
                # Simple keyword search via filters since we don't have embeddings here yet
                # Real vector search would happen if we added an embedding step
                results = self.client.scroll(
                    collection_name=self.collection,
                    limit=limit,
                    with_payload=True
                )
                points = [p.payload for p in results[0]]
                return points if points else "No records found in Qdrant."
            except Exception as e:
                return f"Search error: {e}"

        # JSON Search (Simple contains)
        return self._search_json(query, limit)

    def _search_json(self, query, limit):
        path = os.path.join(self.storage_dir, "memory.json")
        if not os.path.exists(path): return "Memory is empty."
        
        with open(path, "r") as f:
            try: data = json.load(f)
            except: return "Memory file corrupted."
            
        matches = [m for m in data if query.lower() in m["content"].lower()]
        return matches[:limit] if matches else f"No matches for '{query}'"

# --- LLM Tools ---
_manager = None

def _get_manager():
    global _manager
    if _manager is None: _manager = MemoryManager()
    return _manager

def save_memory(content: str):
    """Stores information persistently."""
    return _get_manager().save_memory(content)

def search_memory(query: str):
    """Searches past memories by keyword."""
    return _get_manager().search_memory(query)
