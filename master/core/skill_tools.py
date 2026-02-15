import glob
import importlib.util
import inspect
import json
import os
import subprocess
import sys

import yaml

# Project root calculation
SELF_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(SELF_DIR, "..", ".."))

def get_entity_skills_dir():
    """
    Intelligently find the skills directory of the current entity.
    Priority:
    1. A directory named 'skills' in the Current Working Directory.
    2. A directory named 'skills' in any parent directory (up to project root).
    3. Fallback to global project 'skills' folder.
    """
    curr = os.getcwd()
    while curr and curr != "/":
        candidate = os.path.join(curr, "skills")
        if os.path.exists(candidate) and os.path.isdir(candidate):
            return candidate
        if curr == PROJECT_ROOT: break
        curr = os.path.dirname(curr)
    
    global_skills = os.path.join(PROJECT_ROOT, "skills")
    return global_skills if os.path.exists(global_skills) else None

def resolve_skill_path(skill_name):
    """Resolves path for a skill name using entity context first."""
    base = get_entity_skills_dir()
    if base:
        p = os.path.join(base, skill_name)
        if os.path.exists(p): return os.path.abspath(p)
    
    # Global fallback
    matches = glob.glob(os.path.join(PROJECT_ROOT, "skills", "**", skill_name, "SKILL.md"), recursive=True)
    if matches: return os.path.dirname(os.path.abspath(matches[0]))
    return None

# --- Core LLM Support ---

def list_available_skills():
    """Lists skills available in the current context."""
    base = get_entity_skills_dir()
    if not base: return []
    
    skills = []
    files = glob.glob(os.path.join(base, "**/SKILL.md"), recursive=True)
    for f_path in files:
        try:
            with open(f_path, "r") as f:
                content = f.read()
                if content.startswith("---"):
                    data = yaml.safe_load(content.split("---")[1])
                    skills.append({"name": data.get("name"), "description": data.get("description")})
        except Exception: continue
    return skills

def load_skill_md(skill_name: str):
    """Loads SKILL.md for LLM context."""
    root = resolve_skill_path(skill_name)
    if not root: return f"Error: Skill '{skill_name}' not found."
    try:
        with open(os.path.join(root, "SKILL.md"), "r") as f: return f.read()
    except Exception as e: return str(e)

# --- Dynamic Tool Discovery ---

def _get_json_type(annotation):
    if annotation == str: return "string"
    if annotation == int: return "integer"
    if annotation == bool: return "boolean"
    if annotation == float: return "number"
    return "string"

def get_tools_from_module(module, prefix=""):
    """Extracts suitable functions from a module to OAI tool format."""
    tools = []
    for name, func in inspect.getmembers(module, inspect.isfunction):
        if name.startswith("_") or name == "main": continue
        doc = inspect.getdoc(func)
        if not doc: continue
            
        sig = inspect.signature(func)
        properties = {}
        required = []
        for p_name, p in sig.parameters.items():
            if p_name == "self": continue
            properties[p_name] = {"type": _get_json_type(p.annotation), "description": f"Parameter {p_name}"}
            if p.default == inspect.Parameter.empty: required.append(p_name)

        tools.append({
            "type": "function",
            "function": {
                "name": f"{prefix}__{name}" if prefix else name,
                "description": doc,
                "parameters": {"type": "object", "properties": properties, "required": required}
            }
        })
    return tools

def get_tools_from_file(py_file, prefix=""):
    """Loads a file and extracts tools."""
    abs_path = os.path.abspath(os.path.join(PROJECT_ROOT, py_file)) if not os.path.isabs(py_file) else py_file
    if not os.path.exists(abs_path): return []
    try:
        spec = importlib.util.spec_from_file_location("dynamic_mod", abs_path)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        return get_tools_from_module(mod, prefix=prefix)
    except Exception as e:
        sys.stderr.write(f"Error loading {py_file}: {e}\n")
        return []

def get_tools_from_skills(skill_names):
    """Loads tools from 'chat_logic.py' in the given skills."""
    tools = []
    for name in (skill_names or []):
        root = resolve_skill_path(name)
        if not root: continue
        # Only support chat_logic.py for clean OAI interfaces
        for candidate in ["chat_logic.py", "scripts/chat_logic.py"]:
            path = os.path.join(root, candidate)
            if os.path.exists(path):
                tools.extend(get_tools_from_file(path, prefix=name))
                break
    return tools

def call_tool(full_name, args_json):
    """Executes a tool call."""
    try:
        kwargs = json.loads(args_json) if isinstance(args_json, str) else args_json
        prefix, func_name = full_name.split("__", 1) if "__" in full_name else ("", full_name)

        # 1. Check Orchestrator tools (this file)
        if prefix == "core" or not prefix:
            import skill_tools
            if hasattr(skill_tools, func_name):
                res = getattr(skill_tools, func_name)(**kwargs)
                return json.dumps(res) if isinstance(res, (dict, list)) else str(res)

        # 2. Check Skill tools
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
        return f"Error calling {full_name}: {str(e)}"
