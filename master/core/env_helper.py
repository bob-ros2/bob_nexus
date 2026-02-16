import os
import sys

import yaml


def get_ros_setup_cmd(root_dir):
    conf_path = os.path.join(root_dir, "master", "config", "conf.yaml")
    if not os.path.exists(conf_path):
        return ""

    try:
        with open(conf_path, "r") as f:
            conf = yaml.safe_load(f)

        setups = conf.get("ros", {}).get("setups", [])
        if not setups:
            return ""

        # Build the source command
        # We only include files that actually exist to avoid bash errors
        valid_setups = []
        for s in setups:
            # Resolve relative paths relative to root_dir
            full_path = s if os.path.isabs(s) else os.path.abspath(os.path.join(root_dir, s))

            if os.path.exists(full_path):
                valid_setups.append(f"source {full_path}")
            else:
                # Log to stderr so we don't pollute the command output
                sys.stderr.write(
                    f"Warning: ROS setup path not found: {s} (resolved: {full_path})\n"
                )

        return " && ".join(valid_setups) if valid_setups else ""
    except Exception as e:
        sys.stderr.write(f"Error reading conf.yaml: {e}\n")
        return ""


if __name__ == "__main__":
    # If run as script, print the source command
    # Usage: python3 env_helper.py [root_dir]
    root = sys.argv[1] if len(sys.argv) > 1 else os.getcwd()
    print(get_ros_setup_cmd(root))
