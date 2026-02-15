import json
import os
import signal
import subprocess

import yaml


class BaseDriver:
    def __init__(self, root_dir):
        self.root_dir = root_dir

    def up(self, entity_dir, config_path, ros_source):
        raise NotImplementedError()

    def down(self, manifest):
        raise NotImplementedError()

    def get_status(self, manifest):
        raise NotImplementedError()


class HostDriver(BaseDriver):
    def up(self, entity_dir, config_path, ros_source):
        env = os.environ.copy()
        # Build source prefix
        source_prefix = f"{ros_source} && " if ros_source else ""
        command = [
            "/bin/bash",
            "-c",
            f"{source_prefix}export BOB_LAUNCH_CONFIG={config_path} && "
            f"ros2 launch bob_launch generic.launch.py",
        ]

        log_out = open(os.path.join(entity_dir, "stdout.log"), "w")
        log_err = open(os.path.join(entity_dir, "stderr.log"), "w")

        process = subprocess.Popen(
            command, env=env, preexec_fn=os.setsid, stdout=log_out, stderr=log_err, text=True
        )
        # With os.setsid, PGID is equal to PID of the spawned process
        return {"pid": process.pid, "pgid": process.pid, "status": "running"}

    def down(self, manifest):
        pid = manifest.get("pid")
        pgid = manifest.get("pgid")

        if not pid and not pgid:
            return False

        # If we don't have a stored PGID, try to get it from the PID
        if not pgid and pid:
            try:
                pgid = os.getpgid(pid)
            except ProcessLookupError:
                return True  # Process group already gone

        try:
            # 1. Be polite: SIGINT (ROS likes this)
            os.killpg(pgid, signal.SIGINT)

            import time

            time.sleep(0.5)

            # Check if leader still exists
            try:
                os.kill(pid, 0)
                # 2. Be firmer: SIGTERM
                os.killpg(pgid, signal.SIGTERM)
                time.sleep(1.0)
            except (ProcessLookupError, TypeError):
                # Leader gone, but wait a bit for children
                time.sleep(0.5)

            # 3. Final PURGE: SIGKILL to the entire group
            try:
                os.killpg(pgid, signal.SIGKILL)
            except ProcessLookupError:
                pass

            return True
        except (ProcessLookupError, PermissionError):
            return True
        except Exception:
            return False

    def get_status(self, manifest):
        pid = manifest.get("pid")
        if not pid:
            return "stopped"
        try:
            os.kill(pid, 0)
            return "running"
        except (ProcessLookupError, OSError):
            return "stopped"


class DockerDriver(BaseDriver):
    def __init__(self, root_dir, config):
        super().__init__(root_dir)
        self.config = config
        self.image = config.get("image", "mastermind:latest")
        self.network = config.get("network", "mastermind_net")
        self.global_compose_file = config.get("compose_file", "docker-compose.yaml")

    def up(self, entity_dir, config_path, ros_source):
        entity_name = os.path.basename(entity_dir)

        # 1. Search for a compose file
        # Priority: Entity-local > Global/Template from conf.yaml
        local_compose = os.path.join(entity_dir, "docker-compose.yaml")
        compose_path = None

        if os.path.exists(local_compose):
            compose_path = local_compose
        else:
            # Check global path (resolve relative to root if needed)
            g_path = self.global_compose_file
            if not os.path.isabs(g_path):
                g_path = os.path.join(self.root_dir, g_path)

            if os.path.exists(g_path):
                compose_path = g_path

        if compose_path:
            # Use Docker Compose
            print(f"[*] Orchestration: Using compose file {compose_path}")
            command = [
                "docker",
                "compose",
                "-p",
                entity_name,
                "--project-directory",
                self.root_dir,
                "-f",
                compose_path,
                "up",
                "-d",
            ]

            # Forward environment for the compose context
            env = os.environ.copy()
            env["BOB_LAUNCH_CONFIG"] = config_path
            env["ROS_DOMAIN_ID"] = str(self.config.get("domain_id", 42))
            ds = self.config.get("discovery_server")
            if ds:
                env["ROS_DISCOVERY_SERVER"] = ds

            result = subprocess.run(command, env=env, capture_output=True, text=True)
            if result.returncode != 0:
                raise Exception(f"Docker Compose failed: {result.stderr}")

            return {"type": "swarm-compose", "compose_file": compose_path, "status": "running"}

        # 2. Fallback to Docker Run
        # Cleanup existing container with same name
        subprocess.run(["docker", "rm", "-f", entity_name], capture_output=True)

        # Discovery Server logic
        ds = self.config.get("discovery_server")
        if ds:
            command += ["-e", f"ROS_DISCOVERY_SERVER={ds}"]

        command += [
            self.image,
            "/bin/bash",
            "-c",
            f"{ros_source} && ros2 launch bob_launch generic.launch.py",
        ]

        result = subprocess.run(command, capture_output=True, text=True)
        if result.returncode != 0:
            raise Exception(f"Docker failed: {result.stderr}")

        container_id = result.stdout.strip()
        return {"container_id": container_id, "status": "running"}

    def down(self, manifest):
        entity_name = manifest.get("name")  # Ensure we have the project name
        if manifest.get("type") == "swarm-compose":
            compose_path = manifest.get("compose_file")
            print(f"[*] Orchestration: Stopping compose project {entity_name}")
            subprocess.run(
                [
                    "docker",
                    "compose",
                    "-p",
                    entity_name,
                    "--project-directory",
                    self.root_dir,
                    "-f",
                    compose_path,
                    "down",
                ],
                capture_output=True,
            )
            return True

        container_id = manifest.get("container_id")
        if container_id:
            subprocess.run(["docker", "rm", "-f", container_id], capture_output=True)
            return True
        return False

    def get_status(self, manifest):
        if manifest.get("type") == "swarm-compose":
            entity_name = manifest.get("name")
            res = subprocess.run(
                ["docker", "compose", "-p", entity_name, "ps", "--format", "json"],
                capture_output=True,
                text=True,
            )
            if res.returncode == 0 and res.stdout.strip():
                # If there are any containers in the project, consider it running
                try:
                    data = json.loads(res.stdout)
                    if isinstance(data, list) and len(data) > 0:
                        return "running"
                    if isinstance(data, dict):
                        return "running"  # Some versions return a single dict
                except Exception:
                    pass
            return "stopped"

        # Fallback to single container check
        container_id = manifest.get("container_id")
        if not container_id:
            return "stopped"

        res = subprocess.run(
            ["docker", "inspect", "-f", "{{.State.Running}}", container_id],
            capture_output=True,
            text=True,
        )
        if res.returncode == 0 and res.stdout.strip() == "true":
            return "running"
        return "stopped"


class Deployer:
    def __init__(self, root_dir):
        self.root_dir = os.path.abspath(root_dir)
        self.conf = self._load_conf()
        self.driver = self._init_driver()

    def _load_conf(self):
        conf_path = os.path.join(self.root_dir, "master", "config", "conf.yaml")
        if os.path.exists(conf_path):
            with open(conf_path, "r") as f:
                return yaml.safe_load(f)
        return {"orchestration": {"mode": "host"}}

    def _init_driver(self):
        orch = self.conf.get("orchestration", {})
        mode = orch.get("mode", "host")
        if mode == "swarm":
            return DockerDriver(self.root_dir, orch)
        return HostDriver(self.root_dir)

    def up_local(self, entity_dir):
        """
        Starts an entity locally or via Docker depending on driver.
        """
        # Find a suitable config file. Prioritize llm.yaml
        config_path = os.path.join(entity_dir, "llm.yaml")
        if not os.path.exists(config_path):
            import glob

            yamls = glob.glob(os.path.join(entity_dir, "*.yaml"))
            if yamls:
                config_path = yamls[0]
            else:
                return {"error": f"No .yaml config found in {entity_dir}"}

        from env_helper import get_ros_setup_cmd

        ros_source = get_ros_setup_cmd(self.root_dir)

        try:
            result = self.driver.up(entity_dir, config_path, ros_source)

            # Save manifest
            manifest_path = os.path.join(entity_dir, "manifest.json")
            manifest = {
                "name": os.path.basename(entity_dir),
                "status": "running",
                "config": config_path,
                "type": self.conf.get("orchestration", {}).get("mode", "host"),
                **result,
            }
            with open(manifest_path, "w") as f:
                json.dump(manifest, f, indent=2)

            return manifest
        except Exception as e:
            return {"error": str(e)}

    def down_local(self, entity_dir):
        """
        Stops an entity.
        """
        manifest_path = os.path.join(entity_dir, "manifest.json")
        if not os.path.exists(manifest_path):
            return {"error": "Manifest not found."}

        with open(manifest_path, "r") as f:
            manifest = json.load(f)

        if self.driver.down(manifest):
            manifest["status"] = "stopped"
            manifest["pid"] = None
            manifest["container_id"] = None
            with open(manifest_path, "w") as f:
                json.dump(manifest, f, indent=2)
            return {"status": "stopped"}
        else:
            return {"status": "failed to stop or already dead"}
