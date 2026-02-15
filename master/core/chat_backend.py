import json
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


class ChatBackend(ABC):
    @abstractmethod
    def send_message(self, message: str, stream: bool = True):
        pass


class OAIBackend(ChatBackend):
    def __init__(self, api_url, api_key, model, system_prompt=None, history_limit=10):
        self.api_url = api_url.rstrip("/") + "/chat/completions"
        self.api_key = api_key
        self.model = model
        self.history_limit = history_limit
        self.history = []

        # Initial structure based on user rules
        if system_prompt:
            self.history.append({"role": "system", "content": system_prompt})
            self.history.append({"role": "assistant", "content": "I am ready to help."})
        else:
            self.history.append(
                {"role": "assistant", "content": "Hello! How can I assist you today?"}
            )

    def send_message(self, message: str, stream: bool = True):
        self.history.append({"role": "user", "content": message})

        # Keep history within limits
        # We need to preserve the first 1 or 2 messages (system/initial assistant)
        fixed_offset = 2 if self.history[0]["role"] == "system" else 1
        if len(self.history) > (self.history_limit + fixed_offset):
            # Remove the oldest user/assistant pairs after the fixed header
            self.history = self.history[:fixed_offset] + self.history[-(self.history_limit) :]

        headers = {"Authorization": f"Bearer {self.api_key}", "Content-Type": "application/json"}

        payload = {"model": self.model, "messages": self.history, "stream": stream}

        try:
            response = requests.post(self.api_url, headers=headers, json=payload, stream=stream)
            response.raise_for_status()

            full_response = ""
            if stream:
                for line in response.iter_lines():
                    if not line:
                        continue

                    line_text = line.decode("utf-8").strip()
                    if line_text.startswith("data: "):
                        data_str = line_text[6:].strip()
                        if data_str == "[DONE]":
                            break

                        try:
                            data_json = json.loads(data_str)
                            content = data_json["choices"][0].get("delta", {}).get("content", "")
                            if content:
                                full_response += content
                                yield content
                        except json.JSONDecodeError:
                            continue
            else:
                data_json = response.json()
                full_response = data_json["choices"][0]["message"]["content"]
                yield full_response

            self.history.append({"role": "assistant", "content": full_response})

        except Exception as e:
            yield f"\n[Error] OAI Backend: {str(e)}"


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
        )
    elif backend_type == "bob_llm":
        return BobLLMBackend(
            input_topic=kwargs.get("topic_in", "llm_prompt"),
            output_topic=kwargs.get("topic_out", "llm_stream"),
        )
    else:
        raise ValueError(f"Unknown backend type: {backend_type}")
