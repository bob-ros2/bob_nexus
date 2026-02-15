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
    Priority:
    1. Local entity skills directory (symlinks)
    2. Global skills pool (recursive search)
    """
    # 1. Local check
    entity_skills = get_entity_skills_dir()
    if entity_skills:
        local_path = os.path.join(entity_skills, skill_name)
        if os.path.exists(local_path):
            return os.path.abspath(local_path)

    # 2. Global recursive check
    # We look for a directory named 'skill_name' that contains a 'SKILL.md'
    search_pattern = os.path.join(SKILLS_DIR, "**", skill_name, "SKILL.md")
    matches = glob.glob(search_pattern, recursive=True)
    if matches:
        return os.path.dirname(os.path.abspath(matches[0]))

    return None


def list_available_skills():
    """
    Returns a list of available skills with their names and descriptions.
    """
    skills = []
    # If we are in an entity, list only its linked skills
    entity_skills = get_entity_skills_dir()
    if entity_skills:
        search_dir = entity_skills
    else:
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
    Loads the full content of a specific skill's SKILL.md file.
    """
    root_path = resolve_skill_path(skill_name)
    if not root_path:
        return f"Error: Skill '{skill_name}' not resolved in entity skills or global pool."

    skill_file = os.path.join(root_path, "SKILL.md")
    if not os.path.exists(skill_file):
        return f"Error: SKILL.md missing for '{skill_name}' at {root_path}."

    try:
        with open(skill_file, "r") as f:
            return f.read()
    except Exception as e:
        return f"Error reading skill: {str(e)}"


# --- OAI Tool Implementation ---

def _get_python_type(annotation):
    """Maps Python types to JSON Schema types."""
    if annotation == str:
        return "string"
    if annotation == int:
        return "integer"
    if annotation == bool:
        return "boolean"
    if annotation == float:
        return "number"
    return "string"


def get_tools_from_skills():
    """
    Discovers all skills and converts their 'logic.py' or 'scripts/*.py' functions into OAI tools.
    """
    tools = []
    skills = list_available_skills()

    for s in skills:
        skill_name = s.get("name")
        root_path = resolve_skill_path(skill_name)
        if not root_path:
            continue

        # Look for logic files
        potential_files = [
            os.path.join(root_path, "scripts", "logic.py"),
            os.path.join(root_path, "logic.py"),
        ]
        # Also include any .py in scripts/ if no logic.py found
        if not any(os.path.exists(p) for p in potential_files):
            scripts_dir = os.path.join(root_path, "scripts")
            if os.path.exists(scripts_dir):
                potential_files.extend(glob.glob(os.path.join(scripts_dir, "*.py")))

        for py_file in potential_files:
            if not os.path.exists(py_file):
                continue

            try:
                # Load module dynamically
                spec = importlib.util.spec_from_file_location(f"skill_logic_{skill_name}", py_file)
                module = importlib.util.module_from_spec(spec)
                spec.loader.exec_module(module)

                # Inspect functions
                for name, func in inspect.getmembers(module, inspect.isfunction):
                    # Only exported functions (no leading underscore)
                    if name.startswith("_"):
                        continue
                    
                    # Get docstring for description
                    doc = inspect.getdoc(func) or f"Execute {name} from {skill_name}"
                    
                    # Inspect parameters
                    sig = inspect.signature(func)
                    properties = {}
                    required = []

                    for param_name, param in sig.parameters.items():
                        if param_name == "self":
                            continue
                        
                        p_type = _get_python_type(param.annotation)
                        properties[param_name] = {
                            "type": p_type,
                            "description": f"Parameter {param_name}"
                        }
                        if param.default == inspect.Parameter.empty:
                            required.append(param_name)

                    tool = {
                        "type": "function",
                        "function": {
                            "name": f"{skill_name}__{name}",
                            "description": doc,
                            "parameters": {
                                "type": "object",
                                "properties": properties,
                                "required": required,
                            },
                        },
                    }
                    tools.append(tool)

            except Exception as e:
                sys.stderr.write(f"Warning: Failed to load tools from {py_file}: {e}\n")

    return tools


def call_skill_function(full_func_name, args_json):
    """
    Executes a function from a skill logic file.
    Format: skill_name__function_name
    """
    try:
        if "__" not in full_func_name:
             return f"Error: Invalid function format '{full_func_name}'"
        
        skill_name, func_name = full_func_name.split("__", 1)
        root_path = resolve_skill_path(skill_name)
        if not root_path:
            return f"Error: Skill '{skill_name}' not found."

        # Same logic to find the file
        py_file = None
        potential_files = [
            os.path.join(root_path, "scripts", "logic.py"),
            os.path.join(root_path, "logic.py"),
        ]
        for p in potential_files:
            if os.path.exists(p):
                py_file = p
                break
        
        if not py_file:
            scripts_dir = os.path.join(root_path, "scripts")
            if os.path.exists(scripts_dir):
                for f in glob.glob(os.path.join(scripts_dir, "*.py")):
                    # We need to know which file contains the function...
                    # For now, let's assume it's the first one or we scan them all.
                    # Simplification: reload them until found.
                    spec = importlib.util.spec_from_file_location("tmp_load", f)
                    module = importlib.util.module_from_spec(spec)
                    spec.loader.exec_module(module)
                    if hasattr(module, func_name):
                        py_file = f
                        break
        
        if not py_file:
            return f"Error: Logic file for {skill_name} not found."

        # Execute
        spec = importlib.util.spec_from_file_location("exec_mod", py_file)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        
        func = getattr(module, func_name)
        
        # Load args
        if isinstance(args_json, str):
            kwargs = json.loads(args_json)
        else:
            kwargs = args_json
        
        result = func(**kwargs)
        if isinstance(result, (dict, list)):
            return json.dumps(result)
        return str(result)

    except Exception as e:
        return f"Error calling {full_func_name}: {str(e)}"


# --- Legacy Support ---

def read_skill_resource(skill_name: str, resource_path: str):
    """
    Reads a resource file associated with a skill.
    """
    root_path = resolve_skill_path(skill_name)
    if not root_path:
        return f"Error: Skill '{skill_name}' not found."

    full_path = os.path.join(root_path, resource_path)

    # Security check
    skill_root = os.path.abspath(root_path)
    target_path = os.path.abspath(full_path)
    if not target_path.startswith(skill_root):
        return "Error: Access denied. Resource path must be within the skill directory."

    if not os.path.exists(target_path):
        return (
            f"Error: Resource '{resource_path}' not found for skill '{skill_name}' at {root_path}."
        )

    try:
        with open(target_path, "r") as f:
            return f.read()
    except Exception as e:
        return f"Error reading resource: {str(e)}"


def run_skill_script(skill_name: str, script_path: str, args=None):
    """
    Executes a script found in the 'scripts/' directory of a skill.
    """
    root_path = resolve_skill_path(skill_name)
    if not root_path:
        return f"Error: Skill '{skill_name}' not found."

    full_path = os.path.join(root_path, script_path)

    # Security check
    skill_root = os.path.abspath(root_path)
    target_path = os.path.abspath(full_path)
    if not target_path.startswith(skill_root):
        return "Error: Access denied. Script path must be within the skill directory."

    if not os.path.exists(target_path):
        return f"Error: Script '{script_path}' not found for skill '{skill_name}'."

    if isinstance(args, str):
        import shlex

        args = shlex.split(args)
    elif args is None:
        args = []

    try:
        if target_path.endswith(".py"):
            cmd = [sys.executable, target_path] + args
        elif target_path.endswith(".sh"):
            cmd = ["bash", target_path] + args
        else:
            cmd = [target_path] + args

        result = subprocess.run(cmd, capture_output=True, text=True, check=False)

        if result.stdout:
            sys.stdout.write(result.stdout)
            sys.stdout.flush()
        if result.stderr:
            sys.stderr.write(result.stderr)
            sys.stderr.flush()

        return {"stdout": result.stdout, "stderr": result.stderr, "returncode": result.returncode}
    except Exception as e:
        return f"Error executing script: {str(e)}"
