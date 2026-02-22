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

DEFAULT_IDENTITY = "You are a helpful and technical AI assistant."


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
        self.signal_callback = kwargs.get("signal_callback")

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
                    msg = (
                        f"\033[90m[Debug] Loaded {len(iface_tools)} tools "
                        f"from interface: {interface_file}\033[0m"
                    )
                    print(msg)

            # 3. Enabled skills (from conf.yaml master section)
            enabled_skills = kwargs.get("enabled_skills", [])
            skill_tools_list = skill_tools.get_tools_from_skills(enabled_skills)
            self.tools.extend(skill_tools_list)
            if self.debug and skill_tools_list:
                msg = (
                    f"\033[90m[Debug] Loaded {len(skill_tools_list)} tools "
                    f"from {len(enabled_skills)} master skills.\033[0m"
                )
                print(msg)

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
                role, content, tool_calls = None, [], None
                tool_call_id, tool_name = None, None
                in_content = False
                for line in lines:
                    if line.startswith("ROLE: "):
                        role = line[6:].strip()
                    elif line.startswith("TOOL_CALLS: "):
                        try:
                            tool_calls = json.loads(line[12:].strip())
                        except:
                            pass
                    elif line.startswith("TOOL_CALL_ID: "):
                        tool_call_id = line[14:].strip()
                    elif line.startswith("TOOL_NAME: "):
                        tool_name = line[11:].strip()
                    elif line.startswith("CONTENT:"):
                        in_content = True
                    elif in_content:
                        content.append(line)
                
                if role:
                    # DeepSeek specific: Skip tool messages that lack an ID
                    if role == "tool" and not tool_call_id:
                        if self.debug:
                            print(f"\033[93m[Debug] Skipping incomplete tool message in history.\033[0m")
                        continue
                    
                    msg = {"role": role, "content": "\n".join(content).strip()}
                    if tool_calls:
                        msg["tool_calls"] = tool_calls
                    if role == "tool":
                        msg["tool_call_id"] = tool_call_id or "unknown"
                        msg["name"] = tool_name or "unknown"
                    self.history.append(msg)
            
            self._trim_history()
        except Exception as e:
            print(f"Warning: History load error: {e}")

    def _save_to_persistence(self, role: str, content: str, tool_calls: list = None, tool_call_id: str = None, tool_name: str = None):
        if not self.persistent_path:
            return
        t = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        try:
            os.makedirs(os.path.dirname(os.path.abspath(self.persistent_path)), exist_ok=True)
            with open(self.persistent_path, "a") as f:
                f.write("---\nROLE: " + role + "\nTIMESTAMP: " + t + "\n")
                if tool_calls:
                    f.write("TOOL_CALLS: " + json.dumps(tool_calls) + "\n")
                if tool_call_id:
                    f.write("TOOL_CALL_ID: " + tool_call_id + "\n")
                if tool_name:
                    f.write("TOOL_NAME: " + tool_name + "\n")
                f.write("CONTENT:\n" + content + "\n")
        except Exception as e:
            print(f"Warning: History save error: {e}")

    def _trim_history(self):
        """
        Trims history while ensuring message sequences (like tool calls) are not orphaned.
        Essential for DeepSeek API consistency.
        """
        if len(self.history) <= (self.history_limit + 1):
            return

        # Start from the target window
        start_idx = len(self.history) - self.history_limit
        
        # Move back until we find a 'user' message to start the window cleanly
        # (index 0 is always system, so we don't go below 1)
        while start_idx > 1 and self.history[start_idx]["role"] != "user":
            start_idx -= 1
            
        self.history = [self.history[0]] + self.history[start_idx:]

    def send_message(self, message: str, stream: bool = True):
        self.history.append({"role": "user", "content": message})
        self._save_to_persistence("user", message)

        self._trim_history()

        headers = {"Authorization": f"Bearer {self.api_key}", "Content-Type": "application/json"}
        loop = 0
        while loop < self.max_tool_calls:
            loop += 1
            payload = {"model": self.model, "messages": self.history, "stream": stream}
            if self.tools:
                payload["tools"] = self.tools

            if self.debug:
                print(f"\033[90m[Debug] Sending {len(self.history)} messages to API (Loop {loop}).\033[0m")

            try:
                r = requests.post(self.api_url, headers=headers, json=payload, stream=stream)
                r.raise_for_status()

                if stream:
                    full_content = ""
                    tool_calls_raw = {}
                    
                    for line in r.iter_lines():
                        if line:
                            decoded_line = line.decode("utf-8").strip()
                            if decoded_line.startswith("data: "):
                                if decoded_line == "data: [DONE]":
                                    break
                                
                                try:
                                    json_str = decoded_line[6:].strip()
                                    if not json_str:
                                        continue
                                    chunk = json.loads(json_str)
                                    if not chunk.get("choices"):
                                        continue
                                        
                                    delta = chunk["choices"][0].get("delta", {})
                                    
                                    # Text content
                                    if "content" in delta and delta["content"]:
                                        content = delta["content"]
                                        full_content += content
                                        yield content
                                    
                                    # Tool calls
                                    if "tool_calls" in delta:
                                        for t_chunk in delta["tool_calls"]:
                                            idx = t_chunk.get("index", 0)
                                            if idx not in tool_calls_raw:
                                                tool_calls_raw[idx] = {"id": None, "type": "function", "function": {"name": "", "arguments": ""}}
                                            
                                            if "id" in t_chunk:
                                                tool_calls_raw[idx]["id"] = t_chunk["id"]
                                            if "function" in t_chunk:
                                                f_delta = t_chunk["function"]
                                                if "name" in f_delta:
                                                    tool_calls_raw[idx]["function"]["name"] += f_delta["name"]
                                                if "arguments" in f_delta:
                                                    tool_calls_raw[idx]["function"]["arguments"] += f_delta["arguments"]
                                except Exception as e:
                                    if self.debug:
                                        print(f"\033[90m[Debug] Stream chunk parse error: {e} | Line: {decoded_line}\033[0m")
                                    continue
                    
                    # Construct assistant message from accumulated data
                    msg = {"role": "assistant", "content": full_content or None}
                    # Filter out tool calls that lack IDs or names (safety for DeepSeek)
                    tool_calls = []
                    if tool_calls_raw:
                        for tc in tool_calls_raw.values():
                            if tc.get("id") and tc.get("function", {}).get("name"):
                                tool_calls.append(tc)
                    
                    if tool_calls:
                        msg["tool_calls"] = tool_calls
                    
                    self.history.append(msg)
                    self._save_to_persistence("assistant", full_content or "", tool_calls=tool_calls)
                    
                    if not tool_calls:
                        return
                    
                    # Execute tool calls
                    for call in tool_calls:
                        f_name, f_args = call["function"]["name"], call["function"]["arguments"]
                        if self.signal_callback:
                            self.signal_callback(f_name, f_args)
                        else:
                            print(f"\033[93m[*] NEXUS CALLING: {f_name}({f_args})\033[0m")
                        res = skill_tools.dispatch_tool(f_name, f_args)
                        self.history.append(
                            {"role": "tool", "tool_call_id": call["id"], "name": f_name, "content": res}
                        )
                        self._save_to_persistence("tool", res, tool_call_id=call["id"], tool_name=f_name)

                else:
                    # Non-streaming fallback
                    try:
                        data = r.json()
                        msg = data["choices"][0]["message"]
                        self.history.append(msg)
                        content = msg.get("content", "")
                        tool_calls = msg.get("tool_calls")
                        
                        self._save_to_persistence("assistant", content or "", tool_calls=tool_calls)
                        yield content or ""
                        
                        if not tool_calls:
                            return
                            
                        for call in tool_calls:
                            f_name, f_args = call["function"]["name"], call["function"]["arguments"]
                            if self.signal_callback:
                                self.signal_callback(f_name, f_args)
                            else:
                                print(f"\033[93m[*] NEXUS CALLING: {f_name}({f_args})\033[0m")
                            res = skill_tools.dispatch_tool(f_name, f_args)
                            self.history.append(
                                {"role": "tool", "tool_call_id": call["id"], "name": f_name, "content": res}
                            )
                            self._save_to_persistence("tool", res, tool_call_id=call["id"], tool_name=f_name)
                    except Exception as e:
                        yield f"\n[Backend Error] Failed to parse non-streaming response: {e}"
                        return

            except requests.exceptions.HTTPError as e:
                try:
                    err_msg = r.json()
                except:
                    err_msg = r.text
                yield f"\n[Backend Error] {e}\nResponse: {err_msg}"
                return
            except Exception as e:
                if self.debug:
                    import traceback
                    traceback.print_exc()
                yield f"\n[Backend Error] {e}"
                return

            # Safety: If we reach this point, we hit max_tool_calls.
            # Force one last call WITHOUT tools to close the sequence for DeepSeek.
            if loop >= self.max_tool_calls:
                if self.debug:
                    print(f"\033[93m[Debug] Safety limit reached. Forcing turn closure.\033[0m")
                yield "\n[Safety] Max tool calls reached. Closing turn..."
                # Temporarily disable tools for the closing turn
                original_tools = self.tools
                self.tools = None
                try:
                    # We continue one more iteration to get a final text response
                    # This will loop once more and then exit via 'if not tool_calls: return'
                    continue 
                finally:
                    self.tools = original_tools

        yield "\n[Safety] Maximum recursion depth reached."


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
