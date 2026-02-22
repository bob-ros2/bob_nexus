#!/usr/bin/env python3
import argparse
import os
import sys

import yaml
from dotenv import load_dotenv

# Add project root to path for imports
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from chat_backend import get_backend


def str2bool(v):
    if isinstance(v, bool):
        return v
    if v.lower() in ("yes", "true", "t", "y", "1"):
        return True
    elif v.lower() in ("no", "false", "f", "n", "0"):
        return False
    else:
        raise argparse.ArgumentTypeError("Boolean value expected.")


def main():
    root_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    config_dir = os.path.join(root_dir, "config")

    # Load .env
    config_env = os.path.join(config_dir, ".env")
    if os.path.exists(config_env):
        load_dotenv(config_env)
    else:
        load_dotenv()

    # Load conf.yaml for chat settings and skills
    conf_path = os.path.join(config_dir, "conf.yaml")
    max_tool_calls_default = 5
    master_skills = []
    oai_interfaces = []
    if os.path.exists(conf_path):
        try:
            with open(conf_path, "r") as f:
                conf = yaml.safe_load(f)
                oai_conf = conf.get("chat", {}).get("oai", {})
                max_tool_calls_default = oai_conf.get("max_tool_calls", 5)
                oai_interfaces = oai_conf.get("tool_interfaces", [])

                # Master skills are lists like ["category", "name"]
                for skill_pair in conf.get("skills", {}).get("master", []):
                    if len(skill_pair) >= 2:
                        master_skills.append(skill_pair[1])
        except Exception:
            pass

    parser = argparse.ArgumentParser(description="Experiment 7! Generic Chat Client")
    parser.add_argument(
        "--backend", choices=["oai", "bob_llm"], default="oai", help="Chat backend to use"
    )
    parser.add_argument(
        "--stream", type=str2bool, default=True, help="Enable/disable streaming (default: True)"
    )
    parser.add_argument(
        "--debug", type=str2bool, default=False, help="Enable verbose debug logging"
    )

    # OAI Specific
    parser.add_argument(
        "--oai_api_url", default=os.getenv("MASTER_API_URL"), help="OpenAI Compatible API URL"
    )
    parser.add_argument(
        "--oai_api_key", default=os.getenv("MASTER_API_KEY"), help="OpenAI Compatible API Key"
    )
    parser.add_argument(
        "--oai_api_model", default=os.getenv("MASTER_API_MODEL", "gpt-4o"), help="Model name"
    )
    parser.add_argument("--oai_system_prompt", help="System prompt for OAI backend")
    parser.add_argument(
        "--oai_history", type=int, default=10, help="History length (number of messages to keep)"
    )
    parser.add_argument(
        "--persistent_history",
        default=os.getenv("NEXUS_CHAT_HISTORY"),
        help="Path to persistent history file",
    )
    parser.add_argument(
        "--tools",
        type=str2bool,
        default=str2bool(os.getenv("NEXUS_CHAT_TOOLS", "False")),
        help="Enable/disable skill-based tools",
    )
    parser.add_argument(
        "--max_tool_calls",
        type=int,
        default=max_tool_calls_default,
        help="Maximum consecutive tool calls",
    )
    parser.add_argument(
        "--render",
        type=str2bool,
        default=str2bool(os.getenv("NEXUS_CHAT_RENDER", "True")),
        help="Enable/disable markdown rendering via rich",
    )
    parser.add_argument(
        "--signal-topic",
        help="Optional ROS 2 topic to mirror tool calls (std_msgs/String)"
    )
    parser.add_argument(
        "--signal-file",
        help="Optional local file to append tool call logs"
    )

    # bob_llm Specific
    parser.add_argument(
        "--root_ns", default=os.getenv("ROOT_NS", ""), help="Root namespace for ROS topics"
    )
    parser.add_argument("--topic_in", default="llm_prompt", help="ROS Input topic for prompts")
    parser.add_argument("--topic_out", default="llm_stream", help="ROS Output topic for responses")

    args = parser.parse_args()

    # --- Signal Mirroring Setup ---
    signal_pub = None
    if args.signal_topic:
        try:
            import rclpy
            from std_msgs.msg import String
            if not rclpy.ok():
                rclpy.init()
            node = rclpy.create_node("nexus_signal_mirror")
            signal_pub = node.create_publisher(String, args.signal_topic, 10)
            print(f"\033[94m[*] Signal Mirror: Publishing to {args.signal_topic}\033[0m")
        except Exception as e:
            print(f"\033[91m[Error] Failed to initialize ROS signal mirror: {e}\033[0m")

    def signal_handler(tool_name, tool_args):
        formatted = f"[*] NEXUS CALLING: {tool_name}({tool_args})"
        # 1. Console (Always)
        print(f"\033[93m{formatted}\033[0m")
        
        # 2. File Mirror
        if args.signal_file:
            try:
                with open(args.signal_file, "a") as f:
                    import datetime
                    ts = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                    f.write(f"[{ts}] {formatted}\n")
            except Exception as e:
                print(f"\033[91m[Error] Failed to write signal to file: {e}\033[0m")
        
        # 3. ROS Mirror
        if signal_pub:
            try:
                from std_msgs.msg import String
                signal_pub.publish(String(data=formatted))
            except Exception as e:
                pass # Silent fail for ROS publish in mirror mode

    # Initialize Rich Console if enabled
    console = None
    if args.render:
        try:
            from rich.console import Console
            from rich.live import Live
            from rich.markdown import Markdown

            console = Console()
        except ImportError:
            args.render = False
            print("\n\033[93m[Warning] 'rich' library not found. Rendering disabled.\033[0m")

    if args.debug:
        import logging

        logging.basicConfig(level=logging.DEBUG)
        logging.getLogger("urllib3").setLevel(logging.DEBUG)
        logging.getLogger("requests").setLevel(logging.DEBUG)
        print("\033[90m[Debug] Verbose logging enabled.\033[0m")

    # Prepend ROOT_NS to topics if set
    topic_in = args.topic_in
    topic_out = args.topic_out
    if args.root_ns:
        ns = args.root_ns if args.root_ns.startswith("/") else f"/{args.root_ns}"
        if not topic_in.startswith("/"):
            topic_in = f"{ns}/{topic_in}".replace("//", "/")
        if not topic_out.startswith("/"):
            topic_out = f"{ns}/{topic_out}".replace("//", "/")

    # --- Dynamic System Prompt Assembly ---
    identity_file = os.path.join(config_dir, "identity.txt")
    rules_file = os.path.join(config_dir, "swarm_rules.txt")
    
    final_system_prompt = args.oai_system_prompt or ""
    
    if not final_system_prompt:
        parts = []
        if os.path.exists(identity_file):
            with open(identity_file, "r") as f:
                parts.append(f.read().strip())
        if os.path.exists(rules_file):
            with open(rules_file, "r") as f:
                parts.append(f.read().strip())
        
        if parts:
            final_system_prompt = "\n\n".join(parts)

    # Initialize Backend
    try:
        backend = get_backend(
            args.backend,
            oai_api_url=args.oai_api_url,
            oai_api_key=args.oai_api_key,
            oai_api_model=args.oai_api_model,
            system_prompt=final_system_prompt,
            history_limit=args.oai_history,
            persistent_history_path=args.persistent_history,
            enable_tools=args.tools,
            max_tool_calls=args.max_tool_calls,
            enabled_skills=master_skills,
            tool_interfaces=oai_interfaces,
            signal_callback=signal_handler,
            debug=args.debug,
            topic_in=topic_in,
            topic_out=topic_out,
        )
    except Exception as e:
        print(f"\033[91m[Error] Failed to initialize backend: {e}\033[0m")
        sys.exit(1)

    print(f"\033[94m-- Bob Nexus Sovereign Swarm Chat ({args.backend}) --\033[0m")
    if args.render:
        print("\033[90mPremium Rendering: Enabled (rich)\033[0m")
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

            # Use Backend
            full_content = ""
            if args.render:
                console.print("\n[bold blue]LLM:[/]")
                with Live(Markdown(""), console=console, auto_refresh=False) as live:
                    for chunk in backend.send_message(user_input, stream=args.stream):
                        full_content += chunk
                        # Small delay or check for better rendering performance if needed
                        live.update(Markdown(full_content), refresh=True)
                console.print("")  # Space after live section
            else:
                # Fallback to plain terminal
                print("\033[1;34mLLM: \033[0m", end="", flush=True)
                for chunk in backend.send_message(user_input, stream=args.stream):
                    print(chunk, end="", flush=True)
                    full_content += chunk
                print("\n")

        except KeyboardInterrupt:
            print("\nSession interrupted. Goodbye!")
            break
        except Exception as e:
            if args.debug:
                import traceback

                traceback.print_exc()
            print(f"\n\033[91m[Runtime Error]: {e}\033[0m")


if __name__ == "__main__":
    main()
