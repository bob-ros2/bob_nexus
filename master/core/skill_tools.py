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


# --- Modular Tool Discovery & Execution ---


def _get_json_type(annotation):
    if annotation is str:
        return "string"
    if annotation is int:
        return "integer"
    if annotation is bool:
        return "boolean"
    if annotation is float:
        return "number"
    return "string"


def get_tools_from_module(module, prefix=""):
    """Extracts functions from a module. STRICT: Only functions defined IN the module."""
    tools = []
    # FIX: Filter by __module__ to avoid exporting imported functions/classes
    for name, func in inspect.getmembers(
        module, lambda x: inspect.isfunction(x) and x.__module__ == module.__name__
    ):
        if (
            name.startswith("_")
            or name == "main"
            or name.startswith("get_tools")
            or name == "register"
        ):
            continue
        doc = inspect.getdoc(func)
        if not doc:
            continue

        sig = inspect.signature(func)
        properties = {}
        required = []
        for p_name, p in sig.parameters.items():
            if p_name == "self":
                continue
            properties[p_name] = {
                "type": _get_json_type(p.annotation),
                "description": f"Parameter {p_name}",
            }
            if p.default == inspect.Parameter.empty:
                required.append(p_name)

        tools.append(
            {
                "type": "function",
                "function": {
                    "name": f"{prefix}__{name}" if prefix else name,
                    "description": doc,
                    "parameters": {
                        "type": "object",
                        "properties": properties,
                        "required": required,
                    },
                },
            }
        )
    return tools


def get_tools_from_file(py_file, prefix=""):
    """Loads a tool interface file and extracts functions."""
    abs_path = (
        os.path.abspath(os.path.join(PROJECT_ROOT, py_file))
        if not os.path.isabs(py_file)
        else py_file
    )
    if not os.path.exists(abs_path):
        return []
    try:
        spec = importlib.util.spec_from_file_location("tool_interface", abs_path)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        return get_tools_from_module(mod, prefix=prefix)
    except Exception as e:
        sys.stderr.write(f"Error loading {py_file}: {e}\n")
        return []


def get_tools_from_skills(skill_names):
    """Loads tools from 'chat_logic.py' in enabled skills."""
    tools = []
    for name in skill_names or []:
        root = resolve_skill_path(name)
        if not root:
            continue
        for candidate in ["chat_logic.py", "scripts/chat_logic.py"]:
            path = os.path.join(root, candidate)
            if os.path.exists(path):
                tools.extend(get_tools_from_file(path, prefix=name))
                break
    return tools


def get_orchestrator_tools():
    """Public tools from THIS file."""
    import skill_tools

    return get_tools_from_module(skill_tools, prefix="core")


def dispatch_tool(full_name, args_json):
    """The central execution gate for all LLM tool calls."""
    try:
        kwargs = json.loads(args_json) if isinstance(args_json, str) else args_json
        prefix, func_name = full_name.split("__", 1) if "__" in full_name else ("", full_name)

        # 1. Core Tools
        if prefix == "core" or not prefix:
            import skill_tools

            if hasattr(skill_tools, func_name):
                res = getattr(skill_tools, func_name)(**kwargs)
                return json.dumps(res) if isinstance(res, (dict, list)) else str(res)

        # 2. Skill Tools
        root = resolve_skill_path(prefix)
        if root:
            for candidate in ["chat_logic.py", "scripts/chat_logic.py"]:
                path = os.path.join(root, candidate)
                if os.path.exists(path):
                    spec = importlib.util.spec_from_file_location("exec_mod", path)
                    mod = importlib.util.module_from_spec(spec)
                    spec.loader.exec_module(mod)
                    if hasattr(mod, func_name):
                        res = getattr(mod, func_name)(**kwargs)
                        return json.dumps(res) if isinstance(res, (dict, list)) else str(res)

        return f"Error: Tool '{full_name}' not found."
    except Exception as e:
        return f"Error executing {full_name}: {str(e)}"
