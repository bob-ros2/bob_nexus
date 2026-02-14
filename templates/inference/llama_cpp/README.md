# Pure Inference Template: llama.cpp

This template launches a `llama.cpp` server with NVIDIA CUDA support.
It is intended for "Brain Entities" that provide LLM inference to the rest of the swarm.

## Variables
- `NAME`: Automatically set to the entity name.
- `PORT`: External port for the OAI-compatible API (Default: 8080).
- `HF_REPO`: HuggingFace repository to download from (Default: Qwen/Qwen2.5-0.5B-Instruct-GGUF).
- `HF_FILE`: Specific GGUF file in the repo (Default: qwen2.5-0.5b-instruct-q4_k_m.gguf).

## Usage
Spawn as a pure infrastructure entity:
```bash
./master/cli.sh spawn infrastructure brain-01 inference/llama_cpp
./master/cli.sh up brain-01
```

Once running, any assistant can use `http://brain-01:8080/v1` as their `MASTER_API_URL` (if internal networking is set up) or use the host's mapping.
