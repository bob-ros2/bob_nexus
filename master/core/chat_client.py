#!/usr/bin/env python3
import os
import sys
import argparse
from dotenv import load_dotenv

# Add project root to path for imports
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from chat_backend import get_backend

def str2bool(v):
    if isinstance(v, bool):
        return v
    if v.lower() in ('yes', 'true', 't', 'y', '1'):
        return True
    elif v.lower() in ('no', 'false', 'f', 'n', '0'):
        return False
    else:
        raise argparse.ArgumentTypeError('Boolean value expected.')

def main():
    # Load .env from master/config (standard for persistence)
    config_env = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "config", ".env")
    
    if os.path.exists(config_env):
        load_dotenv(config_env)
    else:
        load_dotenv() # Fallback to standard (.env in current dir)

    parser = argparse.ArgumentParser(description="Experiment 7! Generic Chat Client")
    parser.add_argument("--backend", choices=["oai", "bob_llm"], default="oai", help="Chat backend to use")
    parser.add_argument("--stream", type=str2bool, default=True, help="Enable/disable streaming (default: True)")
    
    # OAI Specific
    parser.add_argument("--oai_api_url", default=os.getenv("MASTER_API_URL"), help="OpenAI Compatible API URL")
    parser.add_argument("--oai_api_key", default=os.getenv("MASTER_API_KEY"), help="OpenAI Compatible API Key")
    parser.add_argument("--oai_api_model", default=os.getenv("MASTER_API_MODEL", "gpt-4o"), help="Model name")
    parser.add_argument("--aoi_system_prompt", help="System prompt for OAI backend")
    parser.add_argument("--oia_history", type=int, default=10, help="History length (number of messages to keep)")

    # bob_llm Specific
    parser.add_argument("--root_ns", default=os.getenv("ROOT_NS", ""), help="Root namespace for ROS topics")
    parser.add_argument("--topic_in", default="llm_prompt", help="ROS Input topic for prompts")
    parser.add_argument("--topic_out", default="llm_stream", help="ROS Output topic for responses")

    args = parser.parse_args()

    # Prepend ROOT_NS to topics if set
    topic_in = args.topic_in
    topic_out = args.topic_out
    if args.root_ns:
        ns = args.root_ns if args.root_ns.startswith("/") else f"/{args.root_ns}"
        if not topic_in.startswith("/"):
            topic_in = f"{ns}/{topic_in}".replace("//", "/")
        if not topic_out.startswith("/"):
            topic_out = f"{ns}/{topic_out}".replace("//", "/")

    # Initialize Backend
    try:
        backend = get_backend(
            args.backend,
            oai_api_url=args.oai_api_url,
            oai_api_key=args.oai_api_key,
            oai_api_model=args.oai_api_model,
            system_prompt=args.aoi_system_prompt,
            history_limit=args.oia_history,
            topic_in=topic_in,
            topic_out=topic_out
        )
    except Exception as e:
        print(f"\033[91m[Error] Failed to initialize backend: {e}\033[0m")
        sys.exit(1)

    print(f"\033[94m-- Experiment 7! Chat Client ({args.backend}) --\033[0m")
    print("\033[90mType 'exit' or 'quit' to end the session.\033[0m")
    
    while True:
        try:
            # User Input
            user_input = input("\033[1;32mYou: \033[0m").strip()
            
            if not user_input:
                continue
                
            if user_input.lower() in ["exit", "quit"]:
                print("Goodbye!")
                break

            # Response Display
            print("\033[1;34mLLM: \033[0m", end="", flush=True)
            
            # Use Backend
            full_content = ""
            for chunk in backend.send_message(user_input, stream=args.stream):
                print(chunk, end="", flush=True)
                full_content += chunk
            
            print("\n") # New line after the whole response

        except KeyboardInterrupt:
            print("\nSession interrupted. Goodbye!")
            break
        except Exception as e:
            print(f"\n\033[91m[Runtime Error]: {e}\033[0m")

if __name__ == "__main__":
    main()
