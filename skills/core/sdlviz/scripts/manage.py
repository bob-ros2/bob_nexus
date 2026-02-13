import subprocess
import sys


def run_command(cmd):
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, check=False)
        return {"stdout": result.stdout, "stderr": result.stderr, "returncode": result.returncode}
    except Exception as e:
        return {"error": str(e)}


def main():
    if len(sys.argv) < 2:
        print("Usage: manage.py [up|down|info]")
        sys.exit(1)

    cmd_type = sys.argv[1]

    # In this stage, we are implementing high-level control via ROS 2 CLI
    if cmd_type == "up":
        # Placeholder for starting the node - in a real scenario, this would trigger a systemd service or docker container
        print("Starting SDLViz node via Mastermind Orchestrator...")
        # Example: subprocess.Popen(["ros2", "run", "bob_sdlviz", "sdlviz", "--ros-args", "-r", "__ns:=/bob"])
        print("SDLViz is now UP (Logic simulation)")

    elif cmd_type == "down":
        print("Stopping SDLViz node...")
        print("SDLViz is now DOWN")

    elif cmd_type == "info":
        print("Retrieving SDLViz node information...")
        # Real ROS 2 CLI call
        res = run_command(["ros2", "node", "info", "/bob/sdlviz"])
        if res.get("returncode") == 0:
            print(res["stdout"])
        else:
            print("Node /bob/sdlviz not found or ROS 2 environment not sourced correctly.")


if __name__ == "__main__":
    main()
