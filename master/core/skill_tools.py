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


def function_to_openai_tool(func):
    """
    Converts a Python function to an OpenAI-compatible tool definition using its docstring.
    """
    name = func.__name__
    doc = inspect.getdoc(func) or "No description available."
    
    # Simple parser for parameters (assumes standard types for now)
    sig = inspect.signature(func)
    parameters = {
        "type": "object",
        "properties": {},
        "required": []
    }
    
    for param_name, param in sig.parameters.items():
        parameters["properties"][param_name] = {
            "type": "string", # Default to string
            "description": f"Parameter {param_name}"
        }
        if param.default is inspect.Parameter.empty:
            parameters["required"].append(param_name)
            
    return {
        "type": "function",
        "function": {
            "name": name,
            "description": doc,
            "parameters": parameters,
        }
    }

def get_orchestrator_tools():
    """
    Returns the list of core orchestrator tools as OpenAI definitions.
    """
    funcs = [
        list_available_skills,
        load_skill_md,
        read_skill_resource,
    ]
    return [function_to_openai_tool(f) for f in funcs]


def get_tools_from_skills(skill_names: list):
    """
    Dynamically loads all functions from the specified skills.
    """
    tools = []
    for name in skill_names:
        path = resolve_skill_path(name)
        if not path:
            continue

        # Look for a python file with the same name as the skill or 'logic.py'
        logic_files = [os.path.join(path, f"{os.path.basename(path)}.py"), os.path.join(path, "logic.py"), os.path.join(path, f"{name}.py")]
        
        target_file = None
        for lf in logic_files:
            if os.path.exists(lf):
                target_file = lf
                break
        
        if not target_file:
            continue

        try:
            module_name = f"skill_{name.replace('/', '_')}"
            spec = importlib.util.spec_from_file_location(module_name, target_file)
            mod = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(mod)

            # Export all functions from the module
            for attr_name in dir(mod):
                attr = getattr(mod, attr_name)
                if inspect.isfunction(attr) and not attr_name.startswith("_"):
                    tools.append(function_to_openai_tool(attr))
        except Exception as e:
            print(f"Error loading skill {name}: {e}")
    
    return tools
