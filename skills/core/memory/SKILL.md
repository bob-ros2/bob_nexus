---
name: memory
description: Persistent information storage and retrieval via JSON or Qdrant.
---
# Persistent Memory Skill

This skill allows an entity to store and retrieve information persistently. It automatically switches between Qdrant (local/server) and JSON backends depending on the environment.

## Instructions

The following tools are available directly to the AI:

1. **`save_memory(content: str)`**: Saves important facts, preferences, or thoughts.
2. **`search_memory(query: str)`**: Searches past memories for relevant context using keyword matching.

## Configuration (Environment Variables)

- **`MEMORY_BACKEND`**: Force a backend (`json`, `local`, `remote`).
- **`QDRANT_URL`**: Set this to use a remote Qdrant server.
- **`COLLECTION_NAME`**: The name of the collection to use (default: `nexus_memory`).

## Usage Guidelines
- Use this to remember user preferences, past conversations, or facts.
- The JSON backend is used by default on Raspberry Pi or when Qdrant is unavailable.
- For high-performance vector search in larger swarms, configure a central Qdrant server via `QDRANT_URL`.
