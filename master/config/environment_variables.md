# Centale Environment Variables

The following environment variables are defined in `master/config/.env` and are used across the Bob Nexus ecosystem.

## LLM & API Configuration
- `MASTER_API_URL`: The endpoint for the primary LLM API (e.g., OpenAI, DeepSeek, or a local provider).
- `MASTER_API_KEY`: The authentication key for the LLM API.
- `MASTER_API_MODEL`: The specific model ID to use (e.g., `deepseek-chat`).

## Shared Memory & Database
- `QDRANT_URL`: The URL of the Qdrant vector database used for long-term memory.
- `QDRANT_API_KEY`: API key for Qdrant (defaults to `none` if not secured).
- `COLLECTION_NAME`: The Qdrant collection used for memory storage (e.g., `mastermind_memory`).

## Infrastructure & Support
- `LLAMA_CPP_PORT`: Port for the local `llama.cpp` inference server.
- `LLAMA_CPP_HF_REPO`: HuggingFace repository ID for local GGUF models.
- `LLAMA_CPP_HF_FILE`: The specific GGUF file name to load.
- `TWITCH_STREAM_KEY`: Stream key for entities utilizing the visual streaming features.

## Observability Stack
- `OBS_GRAFANA_PORT`: Web interface port for Grafana.
- `OBS_LOKI_PORT`: API port for Loki log aggregation.
- `OBS_PROMTAIL_PORT`: Port for Promtail log shipping metrics.
