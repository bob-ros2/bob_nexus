import os
import subprocess
import unittest
import json
import time
import sys
from threading import Thread
from flask import Flask, request, jsonify

# --- Mock OAI Server ---
app = Flask(__name__)

@app.route('/v1/chat/completions', methods=['POST'])
def chat_completions():
    data = request.json
    messages = data.get('messages', [])
    
    # Check if the last message is a tool execution result
    if messages and messages[-1].get('role') == 'tool':
        return jsonify({
            "choices": [{
                "message": {
                    "role": "assistant",
                    "content": "I have successfully saved your poem to the Nexus memory."
                }
            }]
        })
        
    last_user_msg = ""
    for m in reversed(messages):
        if m.get('role') == 'user':
            last_user_msg = m.get('content', '')
            break

    if "save memory" in last_user_msg.lower():
        return jsonify({
            "choices": [{
                "message": {
                    "role": "assistant",
                    "content": None,
                    "tool_calls": [{
                        "id": "call_sanity_123",
                        "type": "function",
                        "function": {
                            "name": "memory__store_memory",
                            "arguments": json.dumps({
                                "title": "Artifact 7",
                                "content": "A poem about the awakening."
                            })
                        }
                    }]
                }
            }]
        })
    
    return jsonify({
        "choices": [{
            "message": {
                "role": "assistant",
                "content": "Identity confirmed. I am the Nexus Mastermind."
            }
        }]
    })

def run_mock_server():
    import logging
    log = logging.getLogger('werkzeug')
    log.setLevel(logging.ERROR) # Quiet down flask
    app.run(port=5858, debug=False, use_reloader=False)

# --- Test Suite ---

class NexusIntegrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Start mock server
        cls.server_thread = Thread(target=run_mock_server, daemon=True)
        cls.server_thread.start()
        time.sleep(2)

        # Environment setup for tests
        os.environ["MASTER_API_URL"] = "http://localhost:5858/v1"
        os.environ["MASTER_API_KEY"] = "test-nexus-key"
        os.environ["MASTER_API_MODEL"] = "test-model"
        os.environ["NEXUS_CHAT_TOOLS"] = "True"
        os.environ["QDRANT_URL"] = "http://localhost:6333"

    def test_01_onboarding(self):
        """Onboarding script sanity check (non-interactive)."""
        process = subprocess.run(["./onboarding.sh"], input="n\n", text=True, capture_output=True)
        self.assertTrue(os.path.exists("master/config/.env"), msg=f"Onboarding failed to create .env. Out: {process.stdout} Err: {process.stderr}")
        self.assertTrue(os.path.exists("master/config/conf.yaml"))

    def test_02_cli_infrastructure(self):
        """CLI tool and entity operations."""
        import shutil
        # Cleanup previously failed runs
        for p in ["entities/master/test_mastermind", "entities/assistant/test_alice"]:
            if os.path.exists(p):
                shutil.rmtree(p)

        # Help check
        res = subprocess.run(["./cli.sh", "-h"], capture_output=True, text=True)
        self.assertEqual(res.returncode, 0, msg=f"cli.sh -h failed. Err: {res.stderr}")
        
        # Status check
        res = subprocess.run(["./cli.sh", "status"], capture_output=True, text=True)
        self.assertEqual(res.returncode, 0, msg=f"cli.sh status failed. Out: {res.stdout} Err: {res.stderr}")

        # Spawn Mastermind Entity
        res = subprocess.run(["./cli.sh", "spawn", "master", "test_mastermind", "bob_llm"], capture_output=True, text=True)
        self.assertEqual(res.returncode, 0, msg=f"Spawn mastermind failed. Out: {res.stdout} Err: {res.stderr}")
        self.assertTrue(os.path.exists("entities/master/test_mastermind/llm.yaml"), msg="llm.yaml missing after spawn")

        # Spawn Assistant Entity
        res = subprocess.run(["./cli.sh", "spawn", "assistant", "test_alice", "bob_llm"], capture_output=True, text=True)
        self.assertEqual(res.returncode, 0, msg=f"Spawn assistant failed. Out: {res.stdout} Err: {res.stderr}")
        self.assertTrue(os.path.exists("entities/assistant/test_alice/llm.yaml"), msg="llm.yaml missing after spawn")

    def test_03_oai_chat_and_tools(self):
        """OAI Chat Client initialization and tool calling logic."""
        # Note: We use chat_client.py directly to avoid TTY issues with chat.sh
        chat_cmd = [sys.executable, "master/core/chat_client.py", "--tools", "True", "--max_tool_calls", "3"]
        process = subprocess.Popen(chat_cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        
        stdout, stderr = process.communicate(input="save memory for me please\nexit\n", timeout=20)
        
        # Verify tool call was logged to stdout
        self.assertIn("NEXUS CALLING: memory__store_memory", stdout, msg=f"Tool call log missing. Out: {stdout}")
        # Verify the final response after tool execution
        self.assertIn("successfully saved your poem", stdout, msg=f"Success response missing. Out: {stdout}")

    def test_04_bob_llm_backend_init(self):
        """Check if bob_llm backend is importable and initializes (dry run)."""
        sys.path.append(os.path.join(os.getcwd(), "master/core"))
        try:
            import chat_backend
            self.assertTrue(hasattr(chat_backend, 'BobLLMBackend'))
        except ImportError as e:
            self.fail(f"Failed to import chat_backend or rclpy: {e}")

if __name__ == "__main__":
    unittest.main()
