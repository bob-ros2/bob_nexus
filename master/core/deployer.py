import json
import os
import signal
import subprocess

import yaml


class BaseDriver:
    def __init__(self, root_dir):
        self.root_dir = root_dir

    def up(self, entity_dir, config_path, ros_source, category="assistant", policy="ro"):
        raise NotImplementedError()

    def down(self, manifest):
        raise NotImplementedError()

    def get_status(self, manifest):
        raise NotImplementedError()


class HostDriver(BaseDriver):
    def up(self, entity_dir, config_path, ros_source, category="assistant", policy="ro"):
        env = os.environ.copy()

        # Load entity-specific .env if it exists
        local_env_path = os.path.join(entity_dir, ".env")
        if os.path.exists(local_env_path):
            from dotenv import dotenv_values

            local_vars = dotenv_values(local_env_path)
            env.update({k: v for k, v in local_vars.items() if v is not None})

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
        self.image = config.get("image", "bob-nexus:latest")
        self.network = config.get("network", "mastermind_net")
        self.global_compose_file = config.get("compose_file", "docker-compose.yaml")

    def _prepare_secrets(self, entity_dir, config_path):
        """
        Scans the config for any occurrence of '/run/secrets/' strings
        and prepares the local secrets directory.
        """
        if not config_path or not os.path.exists(config_path):
            return []

        import yaml
        secrets_identified = []

        try:
            with open(config_path, "r") as f:
                # We load it and search recursively for strings starting with /run/secrets/
                content = yaml.safe_load(f)

                def find_secret_refs(data):
                    refs = []
                    if isinstance(data, dict):
                        for v in data.values():
                            refs.extend(find_secret_refs(v))
                    elif isinstance(data, list):
                        for item in data:
                            refs.extend(find_secret_refs(item))
                    elif isinstance(data, str) and data.startswith("/run/secrets/"):
                        # Extract the filename
                        refs.append(os.path.basename(data))
                    return refs

                secrets_identified = find_secret_refs(content)
        except Exception as e:
            print(f"[!] Warning: Failed to scan config for secret references: {e}")

        if not secrets_identified:
            return []

        local_secrets_dir = os.path.join(entity_dir, "secrets")
        if not os.path.exists(local_secrets_dir):
            os.makedirs(local_secrets_dir, exist_ok=True)

        vault_dir = os.path.join(self.root_dir, "master", "secrets")
        valid_count = 0

        for secret_file in set(secrets_identified):
            vault_path = os.path.join(vault_dir, secret_file)
            # Support fuzzy extension matching if requested file lacks it
            if not os.path.exists(vault_path):
                for ext in [".json", ".jsonc", ".yaml", ".yml", ".txt"]:
                    if os.path.exists(vault_path + ext):
                        vault_path += ext
                        break

            if os.path.exists(vault_path):
                target_path = os.path.join(local_secrets_dir, secret_file)
                if os.path.exists(target_path) or os.path.islink(target_path):
                    os.remove(target_path)
                os.symlink(vault_path, target_path)
                valid_count += 1
            else:
                print(f"[!] Warning: Secret file '{secret_file}' not found in {vault_dir}")

        return secrets_identified

    def up(self, entity_dir, config_path, ros_source, category="assistant", policy="ro"):
        entity_name = os.path.basename(entity_dir)

        # Prepare Secrets
        self._prepare_secrets(entity_dir, config_path)

        # 1. Search for compose files
        # We use a layered approach: Global Blueprint + Local Override
        local_compose = os.path.join(entity_dir, "docker-compose.yaml")
        compose_files = []

        # Global path
        g_path = self.global_compose_file
        if not os.path.isabs(g_path):
            g_path = os.path.join(self.root_dir, g_path)
        if os.path.exists(g_path):
            compose_files.append(g_path)

        # Local override
        if os.path.exists(local_compose):
            compose_files.append(local_compose)

        if compose_files:
            # Use Docker Compose
            print(f"[*] Orchestration: Using compose layers: "
                  f"{', '.join(compose_files)}")
            command = [
                "docker",
                "compose",
                "-p",
                entity_name,
                "--project-directory",
                entity_dir,
            ]
            for cf in compose_files:
                command.extend(["-f", cf])

            command.extend(["up", "-d"])

            # Forward environment for the compose context
            env = os.environ.copy()

            # SAE Permission Fix: Map host user to container
            env["HOST_UID"] = str(os.getuid())
            env["HOST_GID"] = str(os.getgid())
            env["HOST_NEXUS_DIR"] = self.root_dir

            # Load entity-specific .env if it exists
            local_env_path = os.path.join(entity_dir, ".env")
            if os.path.exists(local_env_path):
                print(f"[*] Orchestration: Loading local environment from "
                      f"{local_env_path}")
                from dotenv import dotenv_values

                local_vars = dotenv_values(local_env_path)
                # Filter out None values and update
                env.update({k: v for k, v in local_vars.items() if v is not None})

            # Translate config_path to container path
            rel_config = os.path.relpath(config_path, self.root_dir)
            env["BOB_LAUNCH_CONFIG"] = os.path.join("/app", rel_config)
            env["IMAGE_NAME"] = self.image
            env["NAME"] = entity_name
            env["ROS_DOMAIN_ID"] = str(self.config.get("domain_id", 42))
            ds = self.config.get("discovery_server")
            if ds:
                env["ROS_DISCOVERY_SERVER"] = ds

            # Workspace Policy Forwarding
            env["WORKSPACE_POLICY"] = policy
            env["ENTITY_CATEGORY"] = category
            # Map policy to mount mode for volumes
            env["WORKSPACE_MODE"] = "rw" if policy == "rw" else "ro"
            local_ws = os.path.join(entity_dir, "ros2_ws")
            env["HAS_LOCAL_WS"] = "true" if os.path.exists(local_ws) else "false"

            result = subprocess.run(command, env=env, capture_output=True, text=True)
            if result.returncode != 0:
                raise Exception(f"Docker Compose failed: {result.stderr}")

            return {
                "type": "swarm-compose",
                "compose_files": compose_files,
                "status": "running"
            }

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
        entity_name = manifest.get("name")
        if manifest.get("type") == "swarm-compose":
            c_files = manifest.get("compose_files", [])
            if not c_files and manifest.get("compose_file"):
                c_files = [manifest.get("compose_file")]
            # Use the first file to find the directory context
            entity_dir = os.path.dirname(c_files[0]) if c_files \
                else os.getcwd()

            print(f"[*] Orchestration: Stopping compose project {entity_name}")
            cmd = [
                "docker", "compose", "-p", entity_name,
                "--project-directory", entity_dir
            ]
            for cf in c_files:
                cmd.extend(["-f", cf])
            cmd.append("down")

            subprocess.run(cmd, capture_output=True)
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
                    # Some versions return a JSON array, others return one JSON object per line
                    output = res.stdout.strip()
                    if output.startswith("["):
                        data = json.loads(output)
                        if isinstance(data, list) and len(data) > 0:
                            return "running"
                    else:
                        # Try to parse line by line (NDJSON)
                        lines = output.splitlines()
                        for line in lines:
                            if line.strip():
                                data = json.loads(line)
                                if isinstance(data, dict):
                                    return "running"
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

        orch = self.conf.get("orchestration", {})
        self.host_driver = HostDriver(self.root_dir)
        self.docker_driver = DockerDriver(self.root_dir, orch)

        # Default driver for legacy property access
        self.default_driver = self.host_driver
        if orch.get("mode") == "swarm":
            self.default_driver = self.docker_driver

    def _load_conf(self):
        conf_path = os.path.join(self.root_dir, "master", "config", "conf.yaml")
        if os.path.exists(conf_path):
            with open(conf_path, "r") as f:
                return yaml.safe_load(f) or {}
        return {"orchestration": {"mode": "host"}}

    @property
    def driver(self):
        """Returns the default driver. Maintained for legacy CLI access."""
        return self.default_driver

    def get_status(self, manifest):
        """
        Returns the status of an entity by delegating to the correct driver.
        """
        driver_type = manifest.get("type", "host")
        if driver_type in ["swarm", "swarm-compose"]:
            return self.docker_driver.get_status(manifest)
        return self.host_driver.get_status(manifest)

    def _pre_start_assemble(self, entity_dir):
        """
        Runs the TemplateEngine on entity files just before starting to ensure
        that .env changes are respected.
        """
        from template_engine import TemplateEngine

        local_env = os.path.join(entity_dir, ".env")
        global_env = os.path.join(self.root_dir, "master", "config", ".env")

        # We need the absolute path on the HOST for volume mounting
        # root_dir is already absolute from __init__

        engine = TemplateEngine(
            global_env,
            extra_vars={
                "HOST_NEXUS_DIR": self.root_dir,
                "HOST_ENTITY_DIR": os.path.abspath(entity_dir),
                "NAME": os.path.basename(entity_dir),
                "ENTITY_DIR": entity_dir,  # Backward compat
            },
        )

        if os.path.exists(local_env):
            from dotenv import dotenv_values

            engine.env_vars.update(dotenv_values(local_env))

        # Re-process core config files
        targets = ["docker-compose.yaml", "llm.yaml", "launch.yaml"]
        import glob

        all_yamls = glob.glob(os.path.join(entity_dir, "*.yaml"))
        for y in all_yamls:
            bname = os.path.basename(y)
            if bname not in targets:
                targets.append(bname)

        for t in targets:
            path = os.path.join(entity_dir, t)
            if os.path.exists(path) and not os.path.islink(path):
                try:
                    engine.process_file(path, path)
                except Exception as e:
                    print(f"    [!] SAE Warning: Failed to assemble {t}: {e}")

    def up_local(self, entity_dir):
        """
        Starts an entity locally or via Docker depending on configuration.
        """
        # 1. Self-Assembly: Refresh configuration from .env before up
        self._pre_start_assemble(entity_dir)

        # Determine driver for this entity
        local_compose = os.path.join(entity_dir, "docker-compose.yaml")

        # Priority:
        # 1. If docker-compose.yaml exists -> Always use Docker (Pure Container)
        # 2. Global mode
        if os.path.exists(local_compose):
            active_driver = self.docker_driver
        else:
            active_driver = self.default_driver

        # Find a suitable config file. Prioritize llm.yaml
        config_path = os.path.join(entity_dir, "llm.yaml")
        if not os.path.exists(config_path):
            import glob

            yamls = glob.glob(os.path.join(entity_dir, "*.yaml"))
            # Skip docker-compose from being treated as ROS config if possible
            other_yamls = [y for y in yamls if "docker-compose" not in os.path.basename(y)]
            if other_yamls:
                config_path = other_yamls[0]
            elif yamls:
                config_path = yamls[0]
            else:
                config_path = local_compose if os.path.exists(local_compose) else None

        # Determine category and policy for governance
        # entity_dir is something like /app/entities/assistant/alice
        category = os.path.basename(os.path.dirname(entity_dir))

        policies = self.conf.get("orchestration", {}).get("workspace_policies", {})
        policy = policies.get(category, policies.get("defaults", "ro"))

        from env_helper import get_ros_setup_cmd

        ros_source = get_ros_setup_cmd(self.root_dir)

        try:
            result = active_driver.up(entity_dir, config_path, ros_source, category=category, policy=policy)

            # Save manifest
            manifest_path = os.path.join(entity_dir, "manifest.json")
            manifest_type = "host"
            if active_driver == self.docker_driver:
                manifest_type = "swarm"  # Basic docker run
                if result.get("type") == "swarm-compose":
                    manifest_type = "swarm-compose"

            manifest = {
                "name": os.path.basename(entity_dir),
                "status": "running",
                "config": config_path,
                "type": manifest_type,
                **result,
            }
            with open(manifest_path, "w") as f:
                json.dump(manifest, f, indent=2)

            return manifest
        except Exception as e:
            return {"error": str(e)}

    def down_local(self, entity_dir):
        """
        Stops an entity by delegating to the correct driver.
        """
        manifest_path = os.path.join(entity_dir, "manifest.json")
        if not os.path.exists(manifest_path):
            return {"error": "Manifest not found."}

        with open(manifest_path, "r") as f:
            manifest = json.load(f)

        driver_type = manifest.get("type", "host")
        active_driver = self.host_driver
        if driver_type in ["swarm", "swarm-compose"]:
            active_driver = self.docker_driver

        if active_driver.down(manifest):
            manifest["status"] = "stopped"
            # Clear identifiers
            manifest["pid"] = None
            manifest["pgid"] = None
            manifest["container_id"] = None
            with open(manifest_path, "w") as f:
                json.dump(manifest, f, indent=2)
            return {"status": "stopped"}
        else:
            return {"status": "failed to stop or already dead"}
