import glob
import importlib.util
import inspect
import json
import os
import sys

import yaml

# Project root calculation
SELF_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(SELF_DIR, "..", ".."))


def get_entity_skills_dir():
    """
    Intelligently find the skills directory of the current entity.
    """
    curr = os.getcwd()
    while curr and curr != "/":
        candidate = os.path.join(curr, "skills")
        if os.path.exists(candidate) and os.path.isdir(candidate):
            return candidate
        if curr == PROJECT_ROOT:
            break
        curr = os.path.dirname(curr)

    global_skills = os.path.join(PROJECT_ROOT, "skills")
    return global_skills if os.path.exists(global_skills) else None


def resolve_skill_path(skill_name):
    """Resolves path for a skill name using entity context first."""
    base = get_entity_skills_dir()
    if base:
        p = os.path.join(base, skill_name)
        if os.path.exists(p):
            return os.path.abspath(p)

    # Global fallback (Nexus Pool)
    matches = glob.glob(
        os.path.join(PROJECT_ROOT, "skills", "**", skill_name, "SKILL.md"), recursive=True
    )
    if matches:
        return os.path.dirname(os.path.abspath(matches[0]))
    return None


# --- Orchestrator Core Tools (Exposed to LLM) ---


def list_available_skills():
    """
    Returns a list of all functional skills available in the Nexus.
    Use this to identify capabilities like memory, vision, or communication.
    """
    base = get_entity_skills_dir()
    if not base:
        return []

    skills = []
    files = glob.glob(os.path.join(base, "**/SKILL.md"), recursive=True)
    for f_path in files:
        try:
            with open(f_path, "r") as f:
                content = f.read()
                if content.startswith("---"):
                    data = yaml.safe_load(content.split("---")[1])
                    skills.append(
                        {"name": data.get("name"), "description": data.get("description")}
                    )
        except Exception:
            continue
    return skills


def load_skill_md(skill_name: str):
    """
    Loads the detailed documentation (SKILL.md) for a specific skill.
    Use this if you are unsure how to use a specific skill or what parameters it expects.
    """
    root = resolve_skill_path(skill_name)
    if not root:
        return f"Error: Skill '{skill_name}' not found."
    try:
        with open(os.path.join(root, "SKILL.md"), "r") as f:
            return f.read()
    except Exception as e:
        return str(e)


def read_skill_resource(skill_name: str, resource_path: str):
    """
    Reads a specific resource file (e.g. template, config, JSON) from a skill's directory.
    """
    root = resolve_skill_path(skill_name)
    if not root:
        return f"Error: Skill '{skill_name}' not found."
    full_path = os.path.join(root, resource_path)
    if not os.path.exists(full_path):
        return f"Error: Resource '{resource_path}' not found."
    try:
        with open(full_path, "r") as f:
            return f.read()
    except Exception as e:
        return str(e)
