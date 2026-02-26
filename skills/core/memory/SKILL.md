---
name: memory
description: Unified memory skill for storing and searching information. Supports Qdrant (local file) and JSON backends.
---

# Persistent Memory Skill

This skill allows an entity to store and retrieve information persistently. It serves as a fallback or local alternative to the global Qdrant tools.

## Instructions

1. **Save Memory**: Call `run_skill_script('memory', 'scripts/memory_manager.py', ['save', '--content', 'your info here', '--path', 'memory'])`.
2. **Search Memory**: Call `run_skill_script('memory', 'scripts/memory_manager.py', ['search', '--query', 'your query', '--path', 'memory'])`.
3. **Get Memory Info**: Call `run_skill_script('memory', 'scripts/memory_manager.py', ['info', '--path', 'memory'])`.

## Configuration
- **Backend**: Set `MEMORY_BACKEND=json` in `.env` to force JSON storage. Default is `qdrant` (local file mode).
- **Storage Path**: Data is stored relative to the entity's root by default.

## Usage Guidelines
- Use this to remember user preferences, past conversations, or facts.
- It is particularly useful for entities that need to operate offline or when the primary Qdrant DB is unavailable.
