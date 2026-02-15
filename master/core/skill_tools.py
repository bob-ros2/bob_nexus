import glob
import importlib.util
import inspect
import json
import os
import subprocess
import sys

import yaml

# Determine project root relative to this file (master/core/skill_tools.py)
ROOT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
SKILLS_DIR = os.path.join(ROOT_DIR, "skills")


def get_entity_skills_dir():
    """
    Attempts to find the current entity's skills directory using environment variables.
    """
    launch_config = os.environ.get("BOB_LAUNCH_CONFIG")
    if launch_config and os.path.exists(launch_config):
        # Assume launch_config is in entities/category/name/llm.yaml
        entity_dir = os.path.dirname(launch_config)
        skills_dir = os.path.join(entity_dir, "skills")
        if os.path.exists(skills_dir):
            return skills_dir
    return None


def resolve_skill_path(skill_name):
    """
    Resolves the actual path to a skill's directory.
    """
    # 1. Local check
    entity_skills = get_entity_skills_dir()
    if entity_skills:
        local_path = os.path.join(entity_skills, skill_name)
        if os.path.exists(local_path):
            return os.path.abspath(local_path)

    # 2. Global recursive check
    search_pattern = os.path.join(SKILLS_DIR, "**", skill_name, "SKILL.md")
    matches = glob.glob(search_pattern, recursive=True)
    if matches:
        return os.path.dirname(os.path.abspath(matches[0]))

    return None


# --- Core Tools (Exposed to LLM) ---

def list_available_skills():
    """
    Returns a list of all available skills in the Nexus with their names and descriptions.
    """
    skills = []
    search_dir = SKILLS_DIR
    skill_files = glob.glob(os.path.join(search_dir, "**/SKILL.md"), recursive=True)

    for file_path in skill_files:
        try:
            with open(file_path, "r") as f:
                content = f.read()
                if content.startswith("---"):
                    parts = content.split("---")
                    if len(parts) >= 2:
                        frontmatter = parts[1]
                        data = yaml.safe_load(frontmatter)
                        skills.append(
                            {"name": data.get("name"), "description": data.get("description")}
                        )
        except Exception:
            continue
    return skills


def load_skill(skill_name: str):
    """
    Loads the full content of a skill's SKILL.md file to understand its full capabilities and instructions.
    """
    root_path = resolve_skill_path(skill_name)
    if not root_path:
        return f"Error: Skill '{skill_name}' not found."

    skill_file = os.path.join(root_path, "SKILL.md")
    try:
        with open(skill_file, "r") as f:
            return f.read()
    except Exception as e:
        return f"Error reading skill: {str(e)}"


def read_skill_resource(skill_name: str, resource_path: str):
    """
    Reads a file from a skill's directory (e.g. templates or configs).
    """
    root_path = resolve_skill_path(skill_name)
    if not root_path:
        return f"Error: Skill '{skill_name}' not found."

    full_path = os.path.join(root_path, resource_path)
    if not os.path.exists(full_path):
        return f"Error: Resource '{resource_path}' not found."

    try:
        with open(full_path, "r") as f:
            return f.read()
    except Exception as e:
        return f"Error reading resource: {str(e)}"


# --- OAI Tool Discovery ---

def _get_python_type(annotation):
    if annotation == str: return "string"
    if annotation == int: return "integer"
    if annotation == bool: return "boolean"
    if annotation == float: return "number"
    return "string"


def _module_to_tools(module, prefix=""):
    tools = []
    for name, func in inspect.getmembers(module, inspect.isfunction):
        if name.startswith("_") or name == "main":
            continue
        
        doc = inspect.getdoc(func)
        if not doc:
            continue
            
        sig = inspect.signature(func)
        properties = {}
        required = []

        for param_name, param in sig.parameters.items():
            if param_name == "self": continue
            p_type = _get_python_type(param.annotation)
            properties[param_name] = {"type": p_type, "description": f"Parameter {param_name}"}
            if param.default == inspect.Parameter.empty:
                required.append(param_name)

        tool_name = f"{prefix}__{name}" if prefix else name
        tools.append({
            "type": "function",
            "function": {
                "name": tool_name,
                "description": doc,
                "parameters": {
                    "type": "object",
                    "properties": properties,
                    "required": required,
                },
            },
        })
    return tools


def get_orchestrator_tools():
    """Exports core orchestration tools from this file."""
    import skill_tools
    return _module_to_tools(skill_tools, prefix="core")


def get_tools_from_skills(enabled_skills=None):
    """Restricted tool discovery from 'logic.py' files only."""
    tools = []
    all_skills = list_available_skills()

    for s in all_skills:
        skill_name = s.get("name")
        if enabled_skills and skill_name not in enabled_skills:
            continue

        root_path = resolve_skill_path(skill_name)
        if not root_path: continue

        potential_files = [
            os.path.join(root_path, "scripts", "logic.py"),
            os.path.join(root_path, "logic.py"),
        ]
        
        for py_file in potential_files:
            if not os.path.exists(py_file): continue
            try:
                spec = importlib.util.spec_from_file_location(f"skill_logic_{skill_name}", py_file)
                module = importlib.util.module_from_spec(spec)
                spec.loader.exec_module(module)
                tools.extend(_module_to_tools(module, prefix=skill_name))
            except Exception as e:
                sys.stderr.write(f"Warning: Failed to load tools from {py_file}: {e}\n")
    return tools


def call_skill_function(full_func_name, args_json):
    """Dispatches calls to core or skill functions."""
    try:
        kwargs = json.loads(args_json) if isinstance(args_json, str) else args_json

        if full_func_name.startswith("core__"):
            import skill_tools
            func_name = full_func_name[6:]
            func = getattr(skill_tools, func_name)
            result = func(**kwargs)
        else:
            if "__" not in full_func_name: return f"Error: Invalid format '{full_func_name}'"
            skill_name, func_name = full_func_name.split("__", 1)
            root_path = resolve_skill_path(skill_name)
            if not root_path: return f"Error: Skill '{skill_name}' not found."

            py_file = None
            for p in [os.path.join(root_path, "scripts", "logic.py"), os.path.join(root_path, "logic.py")]:
                if os.path.exists(p):
                    py_file = p
                    break
            
            if not py_file: return f"Error: logic.py for {skill_name} not found."
            spec = importlib.util.spec_from_file_location("exec_mod", py_file)
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            func = getattr(module, func_name)
            result = func(**kwargs)

        return json.dumps(result) if isinstance(result, (dict, list)) else str(result)
    except Exception as e:
        return f"Error calling {full_func_name}: {str(e)}"
