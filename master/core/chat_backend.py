import datetime
import json
import os
import time
from abc import ABC, abstractmethod

import requests
import skill_tools

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
except ImportError:
    rclpy = None

DEFAULT_IDENTITY = """You are the Mastermind Core of the **Bob Nexus** (Experiment 7!).
Bob Nexus is a decentralized ROS 2 Swarm OS.
- Mastermind: Central management and "Awakening" control.
- Entities: Agents (Alice, Bob) and providers (llama.cpp, Qdrant).
- Connectivity: Local ROS 2 / Roadmap Zenoh.
- Philosophy: Self-hosting, technical, direct.
"""



class ChatBackend(ABC):
    @abstractmethod
    def send_message(self, message: str, stream: bool = True):
        pass


class OAIBackend(ChatBackend):
    def __init__(
        self,
        oai_api_url,
        oai_api_key,
        oai_api_model,
        system_prompt=None,
        history_limit=10,
        persistent_history_path=None,
        enable_tools=False,
        max_tool_calls=5,
        **kwargs,
    ):
        self.api_url = oai_api_url.rstrip("/") + "/chat/completions"
        self.api_key = oai_api_key
        self.model = oai_api_model
        self.history_limit = history_limit
        self.history = []
        self.persistent_path = persistent_history_path
        self.max_tool_calls = max_tool_calls
        self.tools = []
        self.debug = kwargs.get("debug", False)

        if enable_tools:
            # 1. Orchestrator Core Tools (prefix core)
            core_tools = skill_tools.get_orchestrator_tools()
            self.tools.extend(core_tools)
            if self.debug:
                print(f"\033[90m[Debug] Loaded {len(core_tools)} core tools.\033[0m")

            # 2. Specific interfaces from config.yaml
            tool_interfaces = kwargs.get("tool_interfaces", [])
            for interface_file in tool_interfaces:
                iface_tools = skill_tools.get_tools_from_file(interface_file)
                self.tools.extend(iface_tools)
                if self.debug and iface_tools:
                    print(
                        f"\033[90m[Debug] Loaded {len(iface_tools)} tools from interface: {interface_file}\033[0m"
                    )

            # 3. Enabled skills (from conf.yaml master section)
            enabled_skills = kwargs.get("enabled_skills", [])
            skill_tools_list = skill_tools.get_tools_from_skills(enabled_skills)
            self.tools.extend(skill_tools_list)
            if self.debug and skill_tools_list:
                print(
                    f"\033[90m[Debug] Loaded {len(skill_tools_list)} tools from {len(enabled_skills)} master skills.\033[0m"
                )

            if not self.tools:
                self.tools = None
            else:
                print(
                    f"\033[94m[*] OAI Tools enabled: {len(self.tools)} functions available.\033[0m"
                )

        # Identity
        sys_p = system_prompt if system_prompt else DEFAULT_IDENTITY
        self.history.append({"role": "system", "content": sys_p})
        if self.persistent_path:
            self._load_persistent_history()

    def _load_persistent_history(self):
        if not os.path.exists(self.persistent_path):
            return
        try:
            with open(self.persistent_path, "r") as f:
                blocks = f.read().split("---")
            for b in blocks:
                lines = b.strip().splitlines()
                role, content = None, []
                in_content = False
                for line in lines:
                    if line.startswith("ROLE: "):
                        role = line[6:].strip()
                    elif line.startswith("CONTENT:"):
                        in_content = True
                    elif in_content:
                        content.append(line)
                if role and content:
                    self.history.append({"role": role, "content": "\n".join(content).strip()})
            if len(self.history) > (self.history_limit + 1):
                self.history = [self.history[0]] + self.history[-(self.history_limit) :]
        except Exception as e:
            print(f"Warning: History load error: {e}")

    def _save_to_persistence(self, role, content, tool_calls=None):
        if not self.persistent_path:
            return
        t = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        try:
            os.makedirs(os.path.dirname(os.path.abspath(self.persistent_path)), exist_ok=True)
            with open(self.persistent_path, "a") as f:
                f.write("---\nROLE: " + role + "\nTIMESTAMP: " + t + "\n")
                if tool_calls:
                    f.write("TOOL_CALLS: " + json.dumps(tool_calls) + "\n")
                f.write("CONTENT:\n" + content + "\n")
        except Exception as e:
            print(f"Warning: History save error: {e}")

    def send_message(self, message: str, stream: bool = True):
        self.history.append({"role": "user", "content": message})
        self._save_to_persistence("user", message)

        headers = {"Authorization": f"Bearer {self.api_key}", "Content-Type": "application/json"}
        loop = 0
        while loop < self.max_tool_calls:
            loop += 1
            payload = {"model": self.model, "messages": self.history}
            if self.tools:
                payload["tools"] = self.tools

            try:
                r = requests.post(self.api_url, headers=headers, json=payload)
                r.raise_for_status()
                msg = r.json()["choices"][0]["message"]
                self.history.append(msg)

                tool_calls = msg.get("tool_calls")
                if not tool_calls:
                    content = msg.get("content", "")
                    self._save_to_persistence("assistant", content)
                    yield content
                    return

                self._save_to_persistence(
                    "assistant", msg.get("content", "") or "", tool_calls=tool_calls
                )
                for call in tool_calls:
                    f_name, f_args = call["function"]["name"], call["function"]["arguments"]
                    print(f"\033[93m[*] NEXUS CALLING: {f_name}({f_args})\033[0m")
                    res = skill_tools.dispatch_tool(f_name, f_args)
                    self.history.append(
                        {"role": "tool", "tool_call_id": call["id"], "name": f_name, "content": res}
                    )
                    self._save_to_persistence("tool", res)
            except Exception as e:
                yield f"\n[Backend Error] {e}"
                return
        yield "\n[Safety] Max tool calls reached."


class BobLLMBackend(ChatBackend):
    def __init__(self, input_topic, output_topic):
        if not rclpy:
            raise ImportError("rclpy not found")
        if not rclpy.ok():
            rclpy.init()
        self.node = Node("chat_bridge")
        self.pub = self.node.create_publisher(String, input_topic, 10)
        self.sub_topic = output_topic
        self.chunk_queue = []
        self.finished = False

    def _cb(self, msg):
        if msg.data == "[DONE]":
            self.finished = True
        else:
            self.chunk_queue.append(msg.data)

    def send_message(self, message: str, stream: bool = True):
        self.pub.publish(String(data=message))
        self.chunk_queue = []
        self.finished = False
        sub = self.node.create_subscription(String, self.sub_topic, self._cb, 10)
        start = time.time()
        while not self.finished and (time.time() - start < 30):
            rclpy.spin_once(self.node, timeout_sec=0.1)
            while self.chunk_queue:
                yield self.chunk_queue.pop(0)
        self.node.destroy_subscription(sub)


def get_backend(backend_type, **kwargs):
    if backend_type == "oai":
        return OAIBackend(**kwargs)
    if backend_type == "bob_llm":
        return BobLLMBackend(
            kwargs.get("topic_in", "llm_prompt"), kwargs.get("topic_out", "llm_stream")
        )
    raise ValueError(f"Unknown backend: {backend_type}")
