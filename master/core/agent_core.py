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

# --- Logging Setup (Swarm 10.4) ---
LOG_FORMAT = "[*] %(message)s"
registry_dir = os.environ.get("NEXUS_REGISTRY_DIR")
if registry_dir and os.path.exists(registry_dir):
    log_file = os.path.join(registry_dir, "stdout.log")
    logging.basicConfig(
        level=logging.INFO,
        format=LOG_FORMAT,
        handlers=[
            logging.FileHandler(log_file),
            logging.StreamHandler(sys.stdout)
        ]
    )
else:
    logging.basicConfig(level=logging.INFO, format=LOG_FORMAT)

logger = logging.getLogger("AgentCore")


class AgentCore:
    def __init__(self, config_path):
        self.config_path = os.path.abspath(config_path)
        self.entity_dir = os.path.dirname(self.config_path)

        # Standardize CWD to the entity directory for isolated tool operations
        os.chdir(self.entity_dir)
        logger.info(f"Initialized Agent in: {self.entity_dir}")

        self.conf = self._load_config()

        # Paths for the Worker-Manager Protocol (Swarm 10.3)
        registry_dir = os.environ.get("NEXUS_REGISTRY_DIR")
        if registry_dir and os.path.exists(registry_dir):
            logger.info(f"Registry Protocol: Using mailbox in {registry_dir}")
            self.inbox_path = os.path.join(registry_dir, "inbox.json")
            self.status_path = os.path.join(registry_dir, "status.json")
            self.outbox_path = os.path.join(registry_dir, "outbox.json")
        else:
            logger.info(f"Registry Protocol: No registry dir found. Using local mailbox in {self.entity_dir}")
            self.inbox_path = os.path.join(self.entity_dir, "inbox.json")
            self.status_path = os.path.join(self.entity_dir, "status.json")
            self.outbox_path = os.path.join(self.entity_dir, "outbox.json")

        self.api_url = self.conf.get("api_url", "http://localhost:8000/v1")
        self.api_key = self.conf.get("api_key", "no_key")
        self.model = self.conf.get("api_model", "gpt-4o")
        self.system_prompt = self.conf.get("system_prompt", "You are a Nexus Agent.")
        self.max_tool_calls = self.conf.get("max_tool_calls", 15)

        # Load tools (Swarm 10.6: Auto-discover local skills)
        self.tools = skill_tools.get_orchestrator_tools()
        
        # 1. Explicitly enabled skills from config
        all_skill_names = self.conf.get("enabled_skills", [])
        
        # 2. Discover linked skills in local directory
        local_skills_dir = os.path.join(self.entity_dir, "skills")
        if os.path.exists(local_skills_dir):
            for entry in os.listdir(local_skills_dir):
                if os.path.isdir(os.path.join(local_skills_dir, entry)) and entry not in all_skill_names:
                    all_skill_names.append(entry)
        
        if all_skill_names:
            logger.info(f"Loading tools from {len(all_skill_names)} skills: {all_skill_names}")
            self.tools.extend(skill_tools.get_tools_from_skills(all_skill_names))
        else:
            logger.warning("No skills enabled or discovered.")

        self.history = [{"role": "system", "content": self.system_prompt}]
        self.state = "idle"
        self.current_thought = "Waiting for mission..."
        
        # Swarm 10.5: Immediate status write to announce presence
        self._update_status("idle", self.current_thought)

    def _load_config(self):
        if not os.path.exists(self.config_path):
            logger.error(f"Config file {self.config_path} not found.")
            sys.exit(1)
        with open(self.config_path, "r") as f:
            data = yaml.safe_load(f)
            if isinstance(data, list):
                logger.warning("Agent Core received a list-based YAML (likely a launch file). Skipping tiered detection.")
                return {}
            # Tiered detection: nexus_agent -> flat
            return data.get("nexus_agent", data)

    def _update_status(self, state, thought=None):
        self.state = state
        if thought is not None:
            self.current_thought = thought

        status = {
            "name": getattr(self, "name", os.environ.get("NAME", "unknown")),
            "state": state,
            "thought": self.current_thought,
            "last_heartbeat": time.time(),
            "model": self.model,
        }
        try:
            with open(self.status_path, "w") as f:
                json.dump(status, f, indent=2)
        except Exception as e:
            logger.error(f"Failed to write status: {e}")
        
        if thought:
            logger.info(f"Status Update: {state} - {thought}")


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

            # Swarm 10.8: Emergency Stop Check
            if os.path.exists(self.inbox_path):
                try:
                    with open(self.inbox_path, "r") as f:
                        inbox_cmd = json.load(f)
                    if inbox_cmd.get("command") == "abort":
                        logger.warning("Emergency Stop: Aborting mission as requested.")
                        archive_path = self.inbox_path + ".last"
                        os.rename(self.inbox_path, archive_path)
                        self._update_status("done", "Mission aborted by user.")
                        return "Mission aborted."
                except:
                    pass

            for tool_call in message["tool_calls"]:
                func_name = tool_call["function"]["name"]
                args = tool_call["function"]["arguments"]
                logger.info(f"Calling Tool: {func_name}({args})")

                result = skill_tools.dispatch_tool(func_name, args)
                
                # IMPORTANT: LLM APIs require 'content' to be a string
                if not isinstance(result, str):
                    result = json.dumps(result)

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

            # Heartbeat (Swarm 10.7: No more thought-shredding)
            if time.time() % 30 < 2:  # Every ~30s
                self._update_status(self.state)

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
