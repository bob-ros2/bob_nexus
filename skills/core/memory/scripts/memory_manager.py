import argparse
import json
import os
import sys
from datetime import datetime


class MemoryManager:
    def __init__(self, storage_path, backend="qdrant"):
        self.storage_path = storage_path
        self.backend = os.getenv("MEMORY_BACKEND", backend)
        self.collection_name = os.getenv("COLLECTION_NAME", "mastermind_memory")

        os.makedirs(self.storage_path, exist_ok=True)

        if self.backend == "qdrant":
            try:
                from qdrant_client import QdrantClient

                # Local path mode for Qdrant
                qdrant_path = os.path.join(self.storage_path, "qdrant_db")
                self.client = QdrantClient(path=qdrant_path)
                print(f"Initialized Qdrant backend at {qdrant_path}")
            except ImportError:
                print("Warning: qdrant-client not found. Falling back to JSON.")
                self.backend = "json"

    def save(self, content, metadata=None):
        if metadata is None:
            metadata = {}
        metadata["timestamp"] = datetime.now().isoformat()

        if self.backend == "qdrant":
            # For simplicity in this script, we'll use a simple collection setup
            # In a real scenario, we'd need embeddings here.
            # If we don't have embeddings, we'll just store it as a record if using qdrant-client
            # Actually, qdrant-client local mode is best used with actual vectors.
            # As a fallback-within-fallback, we'll use JSON if we can't do vector search easily.
            self._save_json(content, metadata)
        else:
            self._save_json(content, metadata)

    def _save_json(self, content, metadata):
        file_path = os.path.join(self.storage_path, "memory_store.json")
        data = []
        if os.path.exists(file_path):
            try:
                with open(file_path, "r") as f:
                    data = json.load(f)
            except Exception:
                pass

        data.append({"content": content, "metadata": metadata})

        with open(file_path, "w") as f:
            json.dump(data, f, indent=2)
        print(f"Memory saved to JSON store at {file_path}")

    def search(self, query, limit=5):
        if self.backend == "json":
            return self._search_json(query, limit)
        else:
            # Fallback to JSON search if vector search is not implemented here
            return self._search_json(query, limit)

    def _search_json(self, query, limit):
        file_path = os.path.join(self.storage_path, "memory_store.json")
        if not os.path.exists(file_path):
            return []

        with open(file_path, "r") as f:
            data = json.load(f)

        # Simple keyword matching as fallback
        results = []
        query_words = query.lower().split()
        for item in data:
            content_lower = item["content"].lower()
            score = sum(1 for word in query_words if word in content_lower)
            if score > 0:
                results.append((score, item))

        # Sort by score
        results.sort(key=lambda x: x[0], reverse=True)
        return [r[1] for r in results[:limit]]


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Memory Manager Skill Script")
    parser.add_argument("command", choices=["save", "search", "info"])
    parser.add_argument("--content", help="Content to save")
    parser.add_argument("--query", help="Query to search")
    parser.add_argument("--path", help="Storage path", default="memory")

    args = parser.parse_args()

    manager = MemoryManager(args.path)

    if args.command == "save":
        if not args.content:
            print("Error: --content required for save")
            sys.exit(1)
        manager.save(args.content)
    elif args.command == "search":
        if not args.query:
            print("Error: --query required for search")
            sys.exit(1)
        results = manager.search(args.query)
        print(json.dumps(results, indent=2))
    elif args.command == "info":
        print(f"Backend: {manager.backend}")
        print(f"Storage: {os.path.abspath(args.path)}")
