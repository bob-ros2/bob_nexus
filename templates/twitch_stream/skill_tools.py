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

# Global registry to map tool names to function objects for dispatch
TOOL_REGISTRY = {}


def get_entity_skills_dir():
    """
    Intelligently find the skills directory of the current entity.
    """
    # Check environment variable first (set by deployer .env)
    ent_dir = os.environ.get("ENTITY_DIR")
    if ent_dir:
        p = os.path.join(ent_dir, "skills")
        if os.path.exists(p) and os.path.isdir(p):
            return p

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
    
    # Map Python types to JSON Schema types
    TYPE_MAP = {
        str: 'string',
        int: 'integer',
        float: 'number',
        bool: 'boolean',
        list: 'array',
        dict: 'object'
    }

    # Simple parser for parameters (assumes standard types for now)
    sig = inspect.signature(func)
    parameters = {
        "type": "object",
        "properties": {},
        "required": []
    }
    
    for param_name, param in sig.parameters.items():
        if param.annotation is inspect.Parameter.empty:
            param_type = 'string'
        else:
            param_type = TYPE_MAP.get(param.annotation, 'string')

        parameters["properties"][param_name] = {
            "type": param_type,
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


def dispatch_tool(func_name: str, args):
    """
    Executes a tool from the registry. Handles JSON string or dict arguments.
    """
    if func_name not in TOOL_REGISTRY:
        return f"Error: Tool '{func_name}' not found in registry."
    
    func = TOOL_REGISTRY[func_name]
    
    # Handle JSON string args from LLM
    if isinstance(args, str):
        try:
            args = json.loads(args)
        except Exception as e:
            return f"Error parsing arguments: {e}"
            
    try:
        if isinstance(args, dict):
            return func(**args)
        return func()
    except Exception as e:
        return f"Error executing {func_name}: {e}"


def get_orchestrator_tools():
    """
    Returns the list of core orchestrator tools as OpenAI definitions.
    """
    funcs = [
        list_available_skills,
        load_skill_md,
        read_skill_resource,
    ]
    for f in funcs:
        TOOL_REGISTRY[f.__name__] = f
        
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
                    TOOL_REGISTRY[attr_name] = attr
                    tools.append(function_to_openai_tool(attr))
        except Exception as e:
            print(f"Error loading skill {name}: {e}")
    
    return tools


def get_tools_from_file(file_path):
    """
    Loads tools from a specific python file (interface definition).
    """
    if not os.path.exists(file_path):
        return []

    try:
        module_name = f"interface_{os.path.basename(file_path).replace('.', '_')}"
        spec = importlib.util.spec_from_file_location(module_name, file_path)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)

        tools = []
        for attr_name in dir(mod):
            attr = getattr(mod, attr_name)
            if inspect.isfunction(attr) and not attr_name.startswith("_") and attr_name != "register":
                TOOL_REGISTRY[attr_name] = attr
                tools.append(function_to_openai_tool(attr))
        return tools
    except Exception as e:
        print(f"Error loading tools from {file_path}: {e}")
        return []


def register(module, node):
    """
    Universal registration bridge for bob_llm.
    Automatically discovers and loads all skills linked in the entity's local 'skills/' folder.
    """
    node.get_logger().info("[Skill Bridge] Initializing universal tool bridge...")
    
    # 1. Register core orchestrator tools
    all_tools = get_orchestrator_tools()
    # Also export them to the module so llm_node can find them
    for t_def in all_tools:
        f_name = t_def['function']['name']
        if f_name in TOOL_REGISTRY:
            setattr(module, f_name, TOOL_REGISTRY[f_name])

    # 2. Find local skills
    skills_dir = get_entity_skills_dir()
    if not skills_dir:
        node.get_logger().warn("[Skill Bridge] No local skills directory found.")
        return all_tools

    # Scan for directories in skills/ (these are our linked skills)
    skill_names = []
    for item in os.listdir(skills_dir):
        if os.path.isdir(os.path.join(skills_dir, item)):
            skill_names.append(item)
    
    if not skill_names:
        node.get_logger().info("[Skill Bridge] No linked skills found in /skills.")
    else:
        node.get_logger().info(f"[Skill Bridge] Discovered {len(skill_names)} skills: {skill_names}")

    # 3. Load tools from each skill
    for name in skill_names:
        path = os.path.join(skills_dir, name)
        node.get_logger().info(f"[Skill Bridge] Checking skill path: {path}")
        
        # Look for logic files (standardized locations)
        logic_candidates = [
            os.path.join(path, "scripts", "chat_logic.py"), # New standard
            os.path.join(path, "logic.py"),                 # Simple standard
            os.path.join(path, f"{name}.py")               # Legacy
        ]
        
        target_file = None
        for lf in logic_candidates:
            if os.path.exists(lf):
                target_file = lf
                node.get_logger().info(f"[Skill Bridge] Found logic file: {lf}")
                break
            else:
                node.get_logger().debug(f"[Skill Bridge] Candidate not found: {lf}")
        
        if target_file:
            node.get_logger().info(f"  -> Loading tools from skill '{name}' ({os.path.basename(target_file)})")
            
            # Use get_tools_from_file but handle registration with the node
            try:
                module_name = f"skill_{name.replace('/', '_')}"
                spec = importlib.util.spec_from_file_location(module_name, target_file)
                mod = importlib.util.module_from_spec(spec)
                spec.loader.exec_module(mod)
                
                # If the skill has its own specialized register, call it
                has_reg = hasattr(mod, "register")
                is_call = callable(getattr(mod, "register", None))
                node.get_logger().info(f"     [Skill Bridge] {name} has register: {has_reg}, is callable: {is_call}")
                
                if has_reg and is_call:
                    node.get_logger().info(f"     [Skill Bridge] Calling {name}.register()")
                    skill_tools = mod.register(mod, node)
                    
                    # Capture functions from this mod and export them to the bridge module
                    # This is key! llm_node looks in 'module' (skill_tools)
                    for t_def in skill_tools:
                        f_name = t_def['function']['name']
                        if hasattr(mod, f_name):
                            func_obj = getattr(mod, f_name)
                            TOOL_REGISTRY[f_name] = func_obj
                            setattr(module, f_name, func_obj)
                            node.get_logger().debug(f"     [Skill Bridge] Exported tool: {f_name}")
                    
                    all_tools.extend(skill_tools)
                else:
                    # Generic discovery
                    node.get_logger().info(f"     [Skill Bridge] Falling back to generic discovery for {name}")
                    for attr_name in dir(mod):
                        attr = getattr(mod, attr_name)
                        if inspect.isfunction(attr) and not attr_name.startswith("_"):
                            TOOL_REGISTRY[attr_name] = attr
                            setattr(module, attr_name, attr)
                            all_tools.append(function_to_openai_tool(attr))
                            node.get_logger().debug(f"     [Skill Bridge] Exported tool (generic): {attr_name}")
            except Exception as e:
                node.get_logger().error(f"  [!] Failed to load skill '{name}': {e}")
        else:
            node.get_logger().warn(f"  [!] No logic file found for skill '{name}'")
    
    node.get_logger().info(f"[Skill Bridge] Registration complete. Total tools: {len(all_tools)}")
    return all_tools
