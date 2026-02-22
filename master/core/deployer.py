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

    def down(self, manifest, entity_dir=None):
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
        exec_cmd = f"ros2 launch bob_launch generic.launch.py"
        
        command = [
            "/bin/bash",
            "-c",
            f"{source_prefix}{exec_cmd}",
        ]

        log_out = open(os.path.join(entity_dir, "stdout.log"), "w")
        log_err = open(os.path.join(entity_dir, "stderr.log"), "w")

        process = subprocess.Popen(
            command, env=env, preexec_fn=os.setsid, stdout=log_out, stderr=log_err, text=True
        )
        return {"pid": process.pid, "pgid": process.pid, "status": "running"}

    def down(self, manifest, entity_dir=None):
        pid = manifest.get("pid")
        pgid = manifest.get("pgid")

        if not pid and not pgid:
            return False

        if not pgid and pid:
            try:
                pgid = os.getpgid(pid)
            except ProcessLookupError:
                return True

        try:
            os.killpg(pgid, signal.SIGINT)
            import time
            time.sleep(0.5)

            try:
                os.kill(pid, 0)
                os.killpg(pgid, signal.SIGTERM)
                time.sleep(1.0)
            except (ProcessLookupError, TypeError):
                time.sleep(0.5)

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
        self.network = config.get("network", "bridge")
        self.global_compose_file = config.get("compose_file", "master/templates/blueprint.yaml")

    def _get_docker_env(self, entity_dir, category, policy):
        """
        Calculates basic docker-compose orchestration variables for interpolation.
        Note: Actual environment variables are passed via the .env file.
        """
        entity_name = os.path.basename(entity_dir)
        project_prefix = os.environ.get("COMPOSE_PROJECT_NAME", "nexus")
        
        return {
            "NAME": entity_name,
            "VOLUME_NAME": f"nexus_entity_{entity_name}_root",
            "ENTITY_CONTAINER_NAME": f"{project_prefix}_{entity_name}",
            "HOST_NEXUS_DIR": os.getenv("HOST_NEXUS_DIR", self.root_dir),
            "REDIS_HOST": os.getenv("REDIS_HOST", "bob_nexus_redis"),
        }

    def up(self, entity_dir, config_path, ros_source, category="assistant", policy="ro"):
        entity_name = os.path.basename(entity_dir)
        local_compose = os.path.join(entity_dir, "docker-compose.yaml")
        
        # Manifest check for specific image overrides
        nexus_manifest = {}
        manifest_path = os.path.join(entity_dir, "nexus.yaml")
        if os.path.exists(manifest_path):
            with open(manifest_path, "r") as f:
                nexus_manifest = yaml.safe_load(f) or {}

        # Basic Environment for interpolation
        env = os.environ.copy()
        env.update(self._get_docker_env(entity_dir, category, policy))
        if nexus_manifest.get("image"):
            env["IMAGE_NAME"] = nexus_manifest["image"]

        # Decide if we use Docker
        run_script = os.path.join(entity_dir, "run.sh")
        
        # Compose files
        compose_files = []
        if os.path.exists(local_compose):
            compose_files.append(local_compose)
        else:
            g_path = self.global_compose_file
            if not os.path.isabs(g_path):
                g_path = os.path.join(self.root_dir, g_path)
            if os.path.exists(g_path):
                compose_files.append(g_path)

        if (os.path.exists(local_compose) or (
            not os.path.exists(run_script) and self.config.get("mode") == "swarm"
        )):
            print(f"[*] Orchestration: Using compose layers: {', '.join(compose_files)}")
            compose_base = ["/usr/bin/docker", "compose"]
            try:
                subprocess.run(["/usr/bin/docker", "compose", "version"], check=True, capture_output=True)
            except:
                compose_base = ["docker-compose"]

            command = compose_base + ["-p", entity_name, "--project-directory", entity_dir]
            for cf in compose_files:
                command.extend(["-f", cf])
            command.extend(["up", "-d"])

            result = subprocess.run(command, env=env, capture_output=True, text=True)
            if result.returncode != 0:
                raise Exception(f"Docker Compose failed: {result.stderr}")

            # Store paths relative to entity_dir for cross-context compatibility (Swarm 7.0)
            rel_compose = [os.path.relpath(cf, entity_dir) for cf in compose_files]
            rel_config = os.path.relpath(config_path, entity_dir) if config_path else None

            return {
                "type": "swarm-compose",
                "compose_files": rel_compose,
                "status": "running",
                "category": category,
                "policy": policy,
                "config": rel_config,
            }

        if os.path.exists(run_script):
            return HostDriver(self.root_dir).up(entity_dir, config_path, ros_source, category, policy)

        raise Exception("No valid entrypoint found")

    def down(self, manifest, entity_dir=None):
        entity_name = manifest.get("name")
        if manifest.get("type") == "swarm-compose":
            if not entity_dir:
                entity_dir = os.getcwd()

            # Resolve relative paths against current entity_dir (Swarm 7.0)
            rel_files = manifest.get("compose_files", [])
            abs_files = [os.path.join(entity_dir, f) if not os.path.isabs(f) else f for f in rel_files]
            
            # Environment for interpolation
            env = os.environ.copy()
            env.update(self._get_docker_env(entity_dir, manifest.get("category"), manifest.get("policy")))

            compose_base = ["/usr/bin/docker", "compose"]
            try:
                subprocess.run(["/usr/bin/docker", "compose", "version"], check=True, capture_output=True)
            except:
                compose_base = ["docker-compose"]

            # Standardized command with explicit project directory
            cmd = compose_base + ["-p", entity_name, "--project-directory", entity_dir]
            for cf in abs_files:
                cmd.extend(["-f", cf])
            cmd.append("down")

            result = subprocess.run(cmd, env=env, capture_output=True)
            return result.returncode == 0
        return False

    def get_status(self, manifest):
        if manifest.get("type") == "swarm-compose":
            entity_name = manifest.get("name")
            compose_base = ["/usr/bin/docker", "compose"]
            try:
                subprocess.run(["/usr/bin/docker", "compose", "version"], check=True, capture_output=True)
            except:
                compose_base = ["docker-compose"]

            res = subprocess.run(
                compose_base + ["-p", entity_name, "ps", "--format", "json"],
                capture_output=True, text=True,
            )
            return "running" if res.returncode == 0 and res.stdout.strip() else "stopped"
        return "stopped"


class Deployer:
    def __init__(self, root_dir):
        self.root_dir = os.path.abspath(root_dir)
        self.conf = self._load_conf()
        orch = self.conf.get("orchestration", {})
        self.host_driver = HostDriver(self.root_dir)
        self.docker_driver = DockerDriver(self.root_dir, orch)

    def _load_conf(self):
        conf_path = os.path.join(self.root_dir, "master", "config", "conf.yaml")
        if os.path.exists(conf_path):
            with open(conf_path, "r") as f:
                return yaml.safe_load(f) or {}
        return {"orchestration": {"mode": "host"}}

    def _pre_start_assemble(self, entity_dir, category, policy, refresh=False):
        """
        Consolidated Assembly Logic for Swarm 8.0 Hermetic Isolation.
        1. Hierarchy discovery Template -> Local -> Master
        2. Dynamic Orchestration Overrides (Signals)
        3. .env Generation
        4. Blueprint Materialization (Materializing docker-compose.yaml)
        """
        from template_engine import TemplateEngine
        from dotenv import dotenv_values

        entity_name = os.path.basename(entity_dir)
        local_env_path = os.path.join(entity_dir, ".env")
        global_env_path = os.path.join(self.root_dir, "master", "config", ".env")
        nexus_manifest_path = os.path.join(entity_dir, "nexus.yaml")

        # 0. Load Manifest Overrides
        nexus_manifest = {}
        if os.path.exists(nexus_manifest_path):
            with open(nexus_manifest_path, "r") as f:
                nexus_manifest = yaml.safe_load(f) or {}

        # 1. Discover Template
        template_dir = None
        if nexus_manifest.get("blueprint"):
            template_dir = os.path.join(self.root_dir, "templates", nexus_manifest["blueprint"])
        else:
            template_dir = os.path.join(self.root_dir, "templates", category)

        # 2. Build Consolidated Environment
        final_env = {}
        if os.path.exists(global_env_path):
            final_env.update(dotenv_values(global_env_path))
        if template_dir and os.path.exists(os.path.join(template_dir, ".env")):
            final_env.update(dotenv_values(os.path.join(template_dir, ".env")))
        if os.path.exists(local_env_path):
            final_env.update(dotenv_values(local_env_path))

        # 3. Dynamic Signals (Overrides)
        project_prefix = os.environ.get("COMPOSE_PROJECT_NAME", "nexus")
        
        # Determine Entrypoint
        entrypoint = nexus_manifest.get("entrypoint")
        if not entrypoint:
            if os.path.exists(os.path.join(entity_dir, "bob_launch.yaml")):
                entrypoint = None
            elif os.path.exists(os.path.join(entity_dir, "agent.yaml")):
                entrypoint = "python3 /app/master/core/agent_core.py /root/agent.yaml"
            elif os.path.exists(os.path.join(entity_dir, "run.sh")):
                entrypoint = "/bin/bash /root/run.sh"
            else:
                entrypoint = "/app/master/core/entrypoint.sh"

        onboarding = nexus_manifest.get("onboarding", True) # Default to true
        overlay_priority = nexus_manifest.get("overlay_priority", False)
        
        orchestration_signals = {
            "NAME": entity_name,
            "ENTITY_CATEGORY": category,
            "WORKSPACE_POLICY": policy,
            "VOLUME_NAME": f"nexus_entity_{entity_name}_root",
            "ENTITY_CONTAINER_NAME": f"{project_prefix}_{entity_name}",
            "REDIS_HOST": os.getenv("REDIS_HOST", "bob_nexus_redis"),
            "ENABLE_ONBOARDING": "true" if onboarding else "false",
            "IMAGE_NAME": nexus_manifest.get("image", self.docker_driver.image),
            "NEXUS_REGISTRY_DIR": f"/app/entities/{category}/{entity_name}",
            "NEXUS_OVERLAY_PRIORITY": "true" if overlay_priority else "false",
            "NEXUS_REFRESH": "true" if refresh else "false",
        }
        if entrypoint:
            orchestration_signals["ENTITY_ENTRYPOINT"] = entrypoint
        
        final_env.update(orchestration_signals)

        # 4. Write Final .env (For Docker Compose and Entrypoint)
        with open(local_env_path, "w") as f:
            for k, v in sorted(final_env.items()):
                if v is not None:
                    safe_v = str(v).replace("\\", "\\\\").replace('"', '\\"')
                    f.write(f"{k}=\"{safe_v}\"\n")

        # 4.1 Build Environment YAML for Blueprint Injection
        whitelist = ["ENTITY_", "MASTER_", "NEXUS_", "BOB_", "ROS_", "API_", "MATRIX_", "QDRANT_", "REDIS_", "HEADSCALE_"]
        explicit = ["NAME", "WORKSPACE_POLICY", "WORKSPACE_MODE", "ENTITY_ENTRYPOINT", 
                    "ENABLE_ONBOARDING", "ENTITY_CONTAINER_NAME", "VOLUME_NAME"]
        
        env_lines = []
        for k, v in sorted(final_env.items()):
            if any(k.startswith(p) for p in whitelist) or k in explicit:
                safe_v = str(v).replace('"', '\\"')
                env_lines.append(f"      - \"{k}={safe_v}\"")
        
        orchestration_signals["ENTITY_ENV_YAML"] = "\n".join(env_lines)

        # 5. Materialize Blueprint
        engine = TemplateEngine(local_env_path, extra_vars=orchestration_signals)
        
        # Standard targets
        targets = ["docker-compose.yaml", "llm.yaml", "launch.yaml", "bob_launch.yaml", "agent.yaml"]
        import glob
        all_yamls = glob.glob(os.path.join(entity_dir, "*.yaml"))
        for y in all_yamls:
            bname = os.path.basename(y)
            if bname not in targets:
                targets.append(bname)

        # Source Blueprint (Swarm 9.1: Smart Discovery with Fallback)
        orch = self.conf.get("orchestration", {})
        g_config_path = orch.get("compose_file", "master/templates/blueprint.yaml")
        
        # 1. Primary path from config
        g_path = g_config_path
        if not os.path.isabs(g_path):
            g_path = os.path.join(self.root_dir, g_path)
        
        # 2. Fallback to framework default if not found
        if not os.path.exists(g_path):
            fallback_path = "/app/templates/blueprint.yaml"
            if os.path.exists(fallback_path):
                print(f"    [*] Blueprint fallback: Using framework default {fallback_path}")
                g_path = fallback_path

        # 3. Final validation
        if not os.path.exists(g_path):
             raise Exception(f"Orchestration Error: Blueprint not found! Tried {g_config_path} and {fallback_path if 'fallback_path' in locals() else '/app/templates/blueprint.yaml'}")

        local_compose = os.path.join(entity_dir, "docker-compose.yaml")
        if os.path.exists(g_path) and not os.path.islink(local_compose):
            print(f"[*] Orchestration: Syncing blueprint for {entity_name}")
            engine.process_file(g_path, local_compose)

        # Process other files
        for t in targets:
            path = os.path.join(entity_dir, t)
            if os.path.exists(path) and not os.path.islink(path) and path != g_path:
                try:
                    engine.process_file(path, path)
                except Exception as e:
                    print(f"    [!] SAE Warning: Failed to assemble {t}: {e}")

    def up_local(self, entity_dir, refresh=False):
        category = os.path.basename(os.path.dirname(entity_dir))
        policies = self.conf.get("orchestration", {}).get("workspace_policies", {})
        policy = policies.get(category, policies.get("defaults", "ro"))

        self._pre_start_assemble(entity_dir, category, policy, refresh=refresh)
        local_compose = os.path.join(entity_dir, "docker-compose.yaml")
        bob_launch = os.path.join(entity_dir, "bob_launch.yaml")
        agent_config = os.path.join(entity_dir, "agent.yaml")
        run_script = os.path.join(entity_dir, "run.sh")

        if (os.path.exists(local_compose) or os.path.exists(bob_launch) or os.path.exists(agent_config)):
            active_driver = self.docker_driver
        elif os.path.exists(run_script):
            active_driver = self.host_driver
        else:
            active_driver = self.docker_driver

        config_path = os.path.join(entity_dir, "agent.yaml")
        if not os.path.exists(config_path):
            config_path = os.path.join(entity_dir, "llm.yaml")
        if not os.path.exists(config_path):
            import glob
            yamls = glob.glob(os.path.join(entity_dir, "*.yaml"))
            other_yamls = [y for y in yamls if "docker-compose" not in os.path.basename(y)]
            if other_yamls:
                config_path = other_yamls[0]

        from env_helper import get_ros_setup_cmd
        ros_source = get_ros_setup_cmd(self.root_dir)

        try:
            result = active_driver.up(entity_dir, config_path, ros_source, category=category, policy=policy)
            manifest_path = os.path.join(entity_dir, "manifest.json")
            manifest_type = "host" if active_driver == self.host_driver else "swarm-compose"
            manifest = {"name": os.path.basename(entity_dir), "status": "running", "type": manifest_type, **result}
            with open(manifest_path, "w") as f:
                json.dump(manifest, f, indent=2)
            return manifest
        except Exception as e:
            return {"error": str(e)}

    def down_local(self, entity_dir):
        manifest_path = os.path.join(entity_dir, "manifest.json")
        if not os.path.exists(manifest_path):
            return {"error": "Manifest not found."}
        with open(manifest_path, "r") as f:
            manifest = json.load(f)
        driver_type = manifest.get("type", "host")
        active_driver = self.host_driver
        if driver_type in ["swarm", "swarm-compose"]:
            active_driver = self.docker_driver
        if active_driver.down(manifest, entity_dir=entity_dir):
            manifest["status"] = "stopped"
            manifest["pid"] = None
            manifest["pgid"] = None
            manifest["container_id"] = None
            with open(manifest_path, "w") as f:
                json.dump(manifest, f, indent=2)
            return {"status": "stopped"}
        return {"status": "failed to stop or already dead"}

    def get_status(self, manifest):
        driver_type = manifest.get("type", "host")
        if driver_type in ["swarm", "swarm-compose"]:
            return self.docker_driver.get_status(manifest)
        return self.host_driver.get_status(manifest)
