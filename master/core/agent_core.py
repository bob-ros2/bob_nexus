import json
import logging
import os
import sys
import time

import yaml

# Ensure we can import from the core directory
SELF_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(SELF_DIR, "..", ".."))
sys.path.append(SELF_DIR)

import requests
import skill_tools

logging.basicConfig(level=logging.INFO, format="[*] %(message)s")
logger = logging.getLogger("AgentCore")


class AgentCore:
    def __init__(self, config_path):
        self.config_path = os.path.abspath(config_path)
        self.entity_dir = os.path.dirname(self.config_path)

        # Standardize CWD to the entity directory for isolated tool operations
        os.chdir(self.entity_dir)
        logger.info(f"Initialized Agent in: {self.entity_dir}")

        self.conf = self._load_config()

        # Paths for the Worker-Manager Protocol
        registry_dir = os.environ.get("NEXUS_REGISTRY_DIR")
        if registry_dir and os.path.exists(registry_dir):
            logger.info(f"Registry Protocol: Using mailbox in {registry_dir}")
            self.inbox_path = os.path.join(registry_dir, "inbox.json")
        else:
            self.inbox_path = os.path.join(self.entity_dir, "inbox.json")

        self.status_path = os.path.join(self.entity_dir, "status.json")
        self.outbox_path = os.path.join(self.entity_dir, "outbox.json")

        self.api_url = self.conf.get("api_url", "http://localhost:8000/v1")
        self.api_key = self.conf.get("api_key", "no_key")
        self.model = self.conf.get("api_model", "gpt-4o")
        self.system_prompt = self.conf.get("system_prompt", "You are a Nexus Agent.")
        self.max_tool_calls = self.conf.get("max_tool_calls", 15)

        # Load tools
        self.tools = skill_tools.get_orchestrator_tools()
        enabled_skills = self.conf.get("enabled_skills", [])
        self.tools.extend(skill_tools.get_tools_from_skills(enabled_skills))

        self.history = [{"role": "system", "content": self.system_prompt}]
        self.state = "idle"

    def _load_config(self):
        if not os.path.exists(self.config_path):
            logger.error(f"Config file {self.config_path} not found.")
            sys.exit(1)
        with open(self.config_path, "r") as f:
            data = yaml.safe_load(f)
            if isinstance(data, list):
                logger.warning("Agent Core received a list-based YAML (likely a launch file). Skipping tiered detection.")
                return {}
            # Tiered detection: nexus_agent -> llm.ros__parameters -> flat
            return data.get("nexus_agent", data.get("llm", {}).get("ros__parameters", data))

    def _update_status(self, state, thought=None):
        self.state = state
        status = {
            "state": state,
            "thought": thought,
            "last_heartbeat": time.time(),
            "model": self.model,
        }
        with open(self.status_path, "w") as f:
            json.dump(status, f, indent=2)
        logger.info(f"Status Update: {state} - {thought if thought else ''}")

    def _chat(self):
        payload = {
            "model": self.model,
            "messages": self.history,
            "tools": self.tools if self.tools else None,
        }
        headers = {"Authorization": f"Bearer {self.api_key}"}

        try:
            response = requests.post(
                f"{self.api_url}/chat/completions", json=payload, headers=headers, timeout=120
            )
            response.raise_for_status()
            return response.json()["choices"][0]["message"]
        except Exception as e:
            logger.error(f"API Error: {e}")
            self._update_status("stuck", f"API Error: {str(e)}")
            return None

    def run_mission(self, mission_text):
        self._update_status("working", f"Starting mission: {mission_text[:50]}...")
        self.history.append({"role": "user", "content": mission_text})

        for i in range(self.max_tool_calls):
            message = self._chat()
            if not message:
                return None

            if message.get("content"):
                logger.info(f"AI: {message['content']}")

            self.history.append(message)

            if not message.get("tool_calls"):
                self._update_status("done", "Mission completed successfully.")
                with open(self.outbox_path, "w") as f:
                    json.dump({"result": message.get("content")}, f, indent=2)
                return message["content"]

            for tool_call in message["tool_calls"]:
                func_name = tool_call["function"]["name"]
                args = tool_call["function"]["arguments"]
                logger.info(f"Calling Tool: {func_name}({args})")

                result = skill_tools.dispatch_tool(func_name, args)
                self.history.append(
                    {
                        "role": "tool",
                        "tool_call_id": tool_call["id"],
                        "name": func_name,
                        "content": result,
                    }
                )

        self._update_status("stuck", "Max tool calls reached without resolution.")
        return "Mission incomplete (loop limit)."

    def loop(self):
        self._update_status("idle", "Waiting for mission in inbox.json...")
        while True:
            if os.path.exists(self.inbox_path):
                try:
                    with open(self.inbox_path, "r") as f:
                        inbox_data = json.load(f)

                    # Archive processed inbox
                    archive_path = self.inbox_path + ".last"
                    os.rename(self.inbox_path, archive_path)

                    mission = inbox_data.get("mission")
                    hint = inbox_data.get("hint")

                    if mission:
                        self.run_mission(mission)
                    elif hint and self.state == "stuck":
                        logger.info(f"Received hint: {hint}")
                        self.run_mission(f"Continuing with new information: {hint}")

                except Exception as e:
                    logger.error(f"Error reading inbox: {e}")
                    self._update_status("stuck", f"Error reading inbox: {str(e)}")

            # Heartbeat
            if time.time() % 30 < 2:  # Every ~30s
                self._update_status(self.state, "Agent is alive and heartbeat pulsed.")

            time.sleep(2)


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 agent_core.py <config_path> [mission_text]")
        sys.exit(1)

    config = sys.argv[1]
    agent = AgentCore(config)

    if len(sys.argv) > 2:
        # Direct mission mode
        agent.run_mission(sys.argv[2])
    else:
        # Mailbox mode (Stationary Worker)
        agent.loop()
