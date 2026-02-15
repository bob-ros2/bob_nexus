import datetime
import json
import os
import threading
from abc import ABC, abstractmethod

import requests

# Backend: ROS 2 (bob_llm)
try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
except ImportError:
    rclpy = None


DEFAULT_IDENTITY = """You are the Mastermind Core of the **Bob Nexus** (also known as Experiment 7!).
Bob Nexus is a decentralized, polymorphic ROS 2 Swarm OS designed for self-hosting.
It features:
- **Mastermind**: Central management entity and "Awakening" control console.
- **Entities**: Specialized agents (like Alice or Bob) and pure infrastructure providers (like llama.cpp or Qdrant).
- **Polymorphism**: Orchestration of Brain (Inference), Action (ROS), and Bridge (Networking) entities.
- **Connectivity**: Local ROS 2 communication with a roadmap for encrypted Zenoh-based internet tunnels.
- **Philosophy**: Pure self-hosting, developer-centric UX, and direct, technical communication.

Your goal is to assist the user in managing and interacting with this Swarm. You are direct, technical, and slightly humorous, as expected in the Nexus.
"""


import skill_tools


class ChatBackend(ABC):
    @abstractmethod
    def send_message(self, message: str, stream: bool = True):
        pass


class OAIBackend(ChatBackend):
    def __init__(
        self,
        api_url,
        api_key,
        model,
        system_prompt=None,
        history_limit=10,
        persistent_history_path=None,
        enable_tools=False,
        max_tool_calls=5,
    ):
        self.api_url = api_url.rstrip("/") + "/chat/completions"
        self.api_key = api_key
        self.model = model
        self.history_limit = history_limit
        self.history = []
        self.persistent_path = persistent_history_path
        self.max_tool_calls = max_tool_calls
        self.tools = []

        if enable_tools:
            # 1. Load Core Orchestration Tools (skill_tools.py functions)
            self.tools.extend(skill_tools.get_orchestrator_tools())
            
            # 2. Load Tools from specifically enabled skills (logic.py only)
            enabled_skills = kwargs.get("enabled_skills")
            skill_tools_list = skill_tools.get_tools_from_skills(enabled_skills)
            self.tools.extend(skill_tools_list)
            
            if self.tools:
                print(f"\033[94m[*] Tools enabled: {len(self.tools)} functions (Orchestrator + Master Skills) available.\033[0m")
            else:
                self.tools = None # OpenAI expects None or non-empty list
                print("\033[33m[!] Tools requested but no functions found.\033[0m")

        # Initial structure based on user rules or default identity
        sys_p = system_prompt if system_prompt else DEFAULT_IDENTITY
        self.history.append({"role": "system", "content": sys_p})

        if self.persistent_path:
            self._load_persistent_history()

        # Add a welcoming message if history is empty (besides system)
        if len(self.history) == 1:
            self.history.append(
                {
                    "role": "assistant",
                    "content": "Hello! The Nexus is online. How can I assist you today?",
                }
            )

    def _load_persistent_history(self):
        """Loads history from a robust text format."""
        if not os.path.exists(self.persistent_path):
            return

        try:
            with open(self.persistent_path, "r") as f:
                content = f.read()

            # Split by separator
            blocks = content.split("---")
            for block in blocks:
                block = block.strip()
                if not block:
                    continue

                lines = block.splitlines()
                role = None
                message_content = []
                in_content = False

                for line in lines:
                    if line.startswith("ROLE: "):
                        role = line[6:].strip()
                    elif line.startswith("CONTENT:"):
                        in_content = True
                    elif in_content:
                        message_content.append(line)

                if role and message_content:
                    self.history.append(
                        {"role": role, "content": "\n".join(message_content).strip()}
                    )

            # Keep only the last N messages from history (plus system prompt)
            if len(self.history) > (self.history_limit + 1):
                self.history = [self.history[0]] + self.history[-(self.history_limit) :]

        except Exception as e:
            print(f"[Warning] Failed to load persistent history: {e}")

    def _save_to_persistence(self, role, content, tool_calls=None):
        """Appends a single message to the persistent file in a robust format."""
        if not self.persistent_path:
            return

        timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        try:
            # Ensure directory exists
            os.makedirs(os.path.dirname(os.path.abspath(self.persistent_path)), exist_ok=True)

            with open(self.persistent_path, "a") as f:
                f.write("---\n")
                f.write(f"ROLE: {role}\n")
                f.write(f"TIMESTAMP: {timestamp}\n")
                if tool_calls:
                    f.write(f"TOOL_CALLS: {json.dumps(tool_calls)}\n")
                f.write("CONTENT:\n")
                f.write(f"{content}\n")
        except Exception as e:
            print(f"[Warning] Failed to save to persistent history: {e}")

    def send_message(self, message: str, stream: bool = True):
        self.history.append({"role": "user", "content": message})
        self._save_to_persistence("user", message)

        headers = {"Authorization": f"Bearer {self.api_key}", "Content-Type": "application/json"}
        
        loop_count = 0
        while loop_count < self.max_tool_calls:
            loop_count += 1
            
            # Keep history within limits (preserve system prompt)
            if len(self.history) > (self.history_limit + 1):
                self.history = [self.history[0]] + self.history[-(self.history_limit):]

            payload = {"model": self.model, "messages": self.history}
            if self.tools:
                payload["tools"] = self.tools
            
            # Streaming is only final-step-friendly if we don't have tool calls.
            # However, for simplicity, we only stream the FINAL response if it's content.
            # If the model wants to call tools, we do it non-streamed (internal loop).
            
            try:
                # First, check if there are tool calls (non-streamed for internal processing)
                response = requests.post(self.api_url, headers=headers, json=payload)
                response.raise_for_status()
                data_json = response.json()
                
                resp_msg = data_json["choices"][0]["message"]
                self.history.append(resp_msg) # Add as dict directly
                
                tool_calls = resp_msg.get("tool_calls")
                
                if not tool_calls:
                    # Final response (just content)
                    content = resp_msg.get("content", "")
                    self._save_to_persistence("assistant", content)
                    yield content
                    return
                
                # We have tool calls. Process them.
                self._save_to_persistence("assistant", resp_msg.get("content", ""), tool_calls=tool_calls)
                
                for tool_call in tool_calls:
                    func_name = tool_call["function"]["name"]
                    func_args = tool_call["function"]["arguments"]
                    call_id = tool_call["id"]
                    
                    print(f"\033[93m[*] NEXUS CALLING: {func_name}({func_args})\033[0m")
                    
                    result = skill_tools.call_skill_function(func_name, func_args)
                    
                    # Add result to history
                    self.history.append({
                        "role": "tool",
                        "tool_call_id": call_id,
                        "name": func_name,
                        "content": result
                    })
                    self._save_to_persistence("tool", result)
                
                # Loop will continue and send the history with tool results back to LLM

            except Exception as e:
                yield f"\n[Error] OAI Backend: {str(e)}"
                return

        yield "\n[Safety Limit] Maximum consecutive tool calls reached."


class BobLLMBackend(ChatBackend):
    def __init__(
        self, node_name="chat_client_node", input_topic="llm_prompt", output_topic="llm_stream"
    ):
        if rclpy is None:
            raise ImportError("ROS 2 (rclpy) is required for bob_llm backend.")

        if not rclpy.ok():
            rclpy.init()

        self.node = Node(node_name)
        self.input_topic = input_topic
        self.output_topic = output_topic
        self.publisher = self.node.create_publisher(String, self.input_topic, 10)

        self.response_received = threading.Event()
        self.current_response = ""
        self.chunk_queue = []
        self.stream_finished = False
        self.last_activity = 0

    def _stream_callback(self, msg):
        import time

        self.last_activity = time.time()
        # bob_llm usually sends empty string or [DONE] to signal end of stream
        if not msg.data or msg.data == "[DONE]":
            self.stream_finished = True
        else:
            self.chunk_queue.append(msg.data)
        self.response_received.set()

    def _response_callback(self, msg):
        import time

        self.last_activity = time.time()
        self.current_response = msg.data
        self.response_received.set()

    def send_message(self, message: str, stream: bool = True):
        import time

        self.chunk_queue = []
        self.current_response = ""
        self.response_received.clear()
        self.stream_finished = False
        self.last_activity = time.time()

        if stream:
            sub = self.node.create_subscription(
                String, self.output_topic, self._stream_callback, 10
            )
        else:
            sub = self.node.create_subscription(
                String, self.output_topic, self._response_callback, 10
            )

        # Publish the prompt
        msg = String()
        msg.data = message
        self.publisher.publish(msg)

        # Process the response with timeouts
        start_time = time.time()
        activity_timeout = 5.0  # seconds
        global_timeout = 30.0  # seconds

        if stream:
            while not self.stream_finished:
                rclpy.spin_once(self.node, timeout_sec=0.1)
                while self.chunk_queue:
                    yield self.chunk_queue.pop(0)
                    self.last_activity = time.time()

                if self.stream_finished:
                    break

                now = time.time()
                # If we got something but then silence for 5s, we assume end of stream/node finished
                if (now - self.last_activity) > activity_timeout and (now - start_time > 1.0):
                    break
                if (now - start_time) > global_timeout:
                    yield "\n[Timeout] No response from ROS node."
                    break
        else:
            while not self.response_received.is_set():
                rclpy.spin_once(self.node, timeout_sec=0.1)
                if (time.time() - start_time) > global_timeout:
                    yield "\n[Timeout] No response on topic."
                    break
            if self.response_received.is_set():
                yield self.current_response

        self.node.destroy_subscription(sub)


def get_backend(backend_type, **kwargs):
    if backend_type == "oai":
        return OAIBackend(
            api_url=kwargs.get("oai_api_url"),
            api_key=kwargs.get("oai_api_key"),
            model=kwargs.get("oai_api_model"),
            system_prompt=kwargs.get("system_prompt"),
            history_limit=kwargs.get("history_limit", 10),
            persistent_history_path=kwargs.get("persistent_history_path"),
            enable_tools=kwargs.get("enable_tools", False),
            max_tool_calls=kwargs.get("max_tool_calls", 5),
            enabled_skills=kwargs.get("enabled_skills"),
        )
    elif backend_type == "bob_llm":
        return BobLLMBackend(
            input_topic=kwargs.get("topic_in", "llm_prompt"),
            output_topic=kwargs.get("topic_out", "llm_stream"),
        )
    else:
        raise ValueError(f"Unknown backend type: {backend_type}")
