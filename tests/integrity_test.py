import glob
import os

import pytest
import yaml

ROOT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


def test_config_validity():
    """Check if the main configuration exists and is valid YAML."""
    conf_path = os.path.join(ROOT_DIR, "master", "config", "conf.yaml")
    assert os.path.exists(conf_path), "conf.yaml missing"
    with open(conf_path, "r") as f:
        conf = yaml.safe_load(f)
        assert "ros" in conf, "Config missing 'ros' section"
        assert "orchestration" in conf, "Config missing 'orchestration' section"
        assert "skills" in conf, "Config missing 'skills' section"
        assert "master" in conf["skills"], "Skills config missing 'master' category"
        assert "assistant" in conf["skills"], "Skills config missing 'assistant' category"
        assert "defaults" in conf["skills"], "Skills config missing 'defaults' category"


def test_templates_integrity():
    """Verify that all template leaf folders contain a valid configuration."""
    templates_dir = os.path.join(ROOT_DIR, "templates")
    if not os.path.exists(templates_dir):
        pytest.skip("Templates directory not found")

    for root, dirs, files in os.walk(templates_dir):
        if root == templates_dir:
            continue

        # Skip known non-template subfolders, composers, or categories
        skip_list = ["composers", "config", "dashboards", "inference"]
        if any(skip in root for skip in skip_list):
            # Special case: check subfolders of inference
            if "inference/" in root and not any(s in root for s in ["config", "dashboards"]):
                pass # Continue to check actual templates inside inference
            else:
                continue

        # A template folder should have a config if it's a leaf or specific top-level
        # We'll check folders that are either direct children or second-level children
        rel_path = os.path.relpath(root, templates_dir)
        depth = len(rel_path.split(os.sep))

        if depth <= 2 and not dirs:
            yamls = glob.glob(os.path.join(root, "*.yaml"))
            if not yamls:
                yamls = glob.glob(os.path.join(root, "docker-compose.yaml"))

            assert len(yamls) > 0, (
                f"Template folder '{root}' contains no configuration (.yaml or docker-compose.yaml)"
            )


def test_skills_integrity():
    """Verify that every linked skill has a SKILL.md."""
    skills_dir = os.path.join(ROOT_DIR, "skills")
    if not os.path.exists(skills_dir):
        pytest.skip("Skills directory not found")

    skill_files = glob.glob(os.path.join(skills_dir, "**/SKILL.md"), recursive=True)
    # At least one skill should exist in a healthy setup
    assert len(skill_files) > 0, "No skills found in skills/ directory"


def test_entity_manifests():
    """Verify that existing entities have valid manifest.json files if they are running."""
    entities_dir = os.path.join(ROOT_DIR, "entities")
    if not os.path.exists(entities_dir):
        pytest.skip("Entities directory not found")

    for category in os.listdir(entities_dir):
        cat_path = os.path.join(entities_dir, category)
        if not os.path.isdir(cat_path):
            continue

        for entity in os.listdir(cat_path):
            ent_path = os.path.join(cat_path, entity)
            manifest_path = os.path.join(ent_path, "manifest.json")
            if os.path.exists(manifest_path):
                import json

                with open(manifest_path, "r") as f:
                    m = json.load(f)
                    assert "status" in m, f"Manifest for {entity} missing status"
                    assert "type" in m, f"Manifest for {entity} missing deployment type"


def test_composers_validity():
    """Check if all orchestration templates are valid YAML."""
    composers_dir = os.path.join(ROOT_DIR, "templates", "composers")
    if not os.path.exists(composers_dir):
        pytest.skip("Composers directory not found")

    for item in os.listdir(composers_dir):
        if item.endswith(".yaml") or item.endswith(".yml"):
            path = os.path.join(composers_dir, item)
            with open(path, "r") as f:
                content = yaml.safe_load(f)
                assert content is not None, f"Composer {item} is empty or invalid"


def test_docker_network_integrity():
    """Verify docker network exists if swarm mode is enabled."""
    conf_path = os.path.join(ROOT_DIR, "master", "config", "conf.yaml")
    if not os.path.exists(conf_path):
        pytest.skip("conf.yaml missing")

    with open(conf_path, "r") as f:
        conf = yaml.safe_load(f)
        orch = conf.get("orchestration", {})
        if orch.get("mode") != "swarm":
            pytest.skip("Not in swarm mode")

        network_name = orch.get("network")
        assert network_name, "Swarm mode enabled but no network name defined"

        # Try to check if docker network exists
        import subprocess

        try:
            res = subprocess.run(
                [
                    "docker",
                    "network",
                    "ls",
                    "--filter",
                    f"name={network_name}",
                    "--format",
                    "{{.Name}}",
                ],
                capture_output=True,
                text=True,
                timeout=5,
            )
            if res.returncode == 0:
                found_networks = res.stdout.strip().split("\n")
                assert network_name in found_networks, (
                    f"Docker network '{network_name}' not found. Create it: docker network create {network_name}"
                )
            else:
                pytest.skip(f"Docker command failed (permissions?): {res.stderr}")
        except (subprocess.TimeoutExpired, FileNotFoundError):
            pytest.skip("Docker not accessible or timed out, skipping network check")
