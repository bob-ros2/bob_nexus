import os
import json
import sys
import subprocess

# Calculate project root from environment or relative path
PROJECT_ROOT = os.getenv("BOB_NEXUS_DIR")
if not PROJECT_ROOT:
    SELF_DIR = os.path.dirname(os.path.abspath(__file__))
    PROJECT_ROOT = os.path.abspath(os.path.join(SELF_DIR, "..", "..", ".."))

sys.path.append(os.path.join(PROJECT_ROOT, "master", "core"))

from manager import EntityManager
from deployer import Deployer

ENTITIES_DIR = os.path.join(PROJECT_ROOT, "entities")

def list_swarm():
    """
    Returns a list of all entities across all categories and their current status.
    Limited by NEXUS_MAX_LIST_LENGTH (default 20).
    """
    max_len = int(os.getenv("NEXUS_MAX_LIST_LENGTH", 20))
    
    manager = EntityManager(PROJECT_ROOT)
    deployer = Deployer(PROJECT_ROOT)
    
    if not os.path.exists(ENTITIES_DIR):
        return f"Error: Entities directory not found at {ENTITIES_DIR}."
    
    entities = []
    count = 0
    categories = sorted(os.listdir(ENTITIES_DIR))
    for cat in categories:
        cat_path = os.path.join(ENTITIES_DIR, cat)
        if not os.path.isdir(cat_path):
            continue
        for ent in sorted(os.listdir(cat_path)):
            if count >= max_len:
                break
            
            ent_path = os.path.join(cat_path, ent)
            manifest_path = os.path.join(ent_path, "manifest.json")
            status = "stopped"
            if os.path.exists(manifest_path):
                try:
                    with open(manifest_path, "r") as f:
                        m = json.load(f)
                        status = deployer.get_status(m)
                except:
                    status = "error"
            
            entities.append({
                "category": cat,
                "name": ent,
                "status": status
            })
            count += 1
        if count >= max_len:
            break
            
    return entities

def get_status(name: str, category: str = None):
    """
    Retrieves detailed status of a specific entity, including its current task summary from 'task.md'.
    Use this to monitor the progress of a delegated task.
    """
    manager = EntityManager(PROJECT_ROOT)
    if not category:
        category, _ = manager.find_entity(name)
    
    if not category:
        return f"Error: Entity '{name}' not found."
        
    ent_dir = os.path.join(ENTITIES_DIR, category, name)
    status = {"name": name, "category": category, "active": False, "task_summary": "No active tasks."}
    
    # Check if running
    manifest_path = os.path.join(ent_dir, "manifest.json")
    if os.path.exists(manifest_path):
        from deployer import Deployer
        deployer = Deployer(PROJECT_ROOT)
        with open(manifest_path, "r") as f:
            m = json.load(f)
            status["active"] = (deployer.get_status(m) == "running")

    # Read task.md
    task_file = os.path.join(ent_dir, "task.md")
    if os.path.exists(task_file):
        with open(task_file, "r") as f:
            content = f.read()
            # Extract first section or first 500 chars
            status["task_summary"] = content[:500] + ("..." if len(content) > 500 else "")
            
    return status

def assign_instruction(name: str, instruction: str, category: str = None):
    """
    Delivers a task or instruction to an entity. Writes to 'instruction.md' which triggers 
    the entity's autonomous loop. This is the primary tool for DELEGATING work to other agents (like 'neo').
    """
    manager = EntityManager(PROJECT_ROOT)
    if not category:
        category, _ = manager.find_entity(name)
        
    if not category:
        return f"Error: Entity '{name}' not found."
    
    ent_dir = os.path.join(ENTITIES_DIR, category, name)
    instr_path = os.path.join(ent_dir, "instruction.md")
    
    try:
        with open(instr_path, "w") as f:
            f.write(f"# MISSION INSTRUCTION\n\n{instruction}\n")
        return f"Success: Instruction delivered to {name}."
    except Exception as e:
        return f"Error delivering instruction: {str(e)}"

def control_entity(action: str, name: str, category: str = None, template: str = None):
    """
    Controls an entity life-cycle. 
    Actions: 'up', 'down', 'spawn'.
    'category' and 'template' are required for 'spawn'.
    """
    manager = EntityManager(PROJECT_ROOT)
    deployer = Deployer(PROJECT_ROOT)
    
    try:
        if action == "spawn":
            if not category or not template:
                return "Error: 'category' and 'template' are required for 'spawn'."
            manager.spawn_entity(category, name, template)
            return f"Success: Entity {name} spawned in {category}."
            
        # For up/down, we need the category
        if not category:
            category, _ = manager.find_entity(name)
        
        if not category:
            return f"Error: Could not find entity '{name}'."
            
        entity_dir = os.path.join(ENTITIES_DIR, category, name)
        
        if action == "up":
            res = deployer.up_local(entity_dir)
            return f"Success: {name} is going up. Response: {res}"
        elif action == "down":
            res = deployer.down_local(entity_dir)
            return f"Success: {name} is going down. Response: {res}"
        else:
            return f"Error: Unknown action '{action}'."
            
    except Exception as e:
        return f"Error: {str(e)}"

def get_logs(name: str, lines: int = 50):
    """
    Retrieves the last N lines of logs for a specific entity from its Docker container.
    """
    manager = EntityManager(PROJECT_ROOT)
    category, _ = manager.find_entity(name)
    
    if not category:
        return f"Error: Entity '{name}' not found."
    
    try:
        # Check if container exists and get logs
        logs = subprocess.check_output(
            ["docker", "logs", "--tail", str(lines), name], 
            text=True, 
            stderr=subprocess.STDOUT
        )
        return logs
    except subprocess.CalledProcessError as e:
        return f"Error retrieving logs for {name}: {e.output}"
    except Exception as e:
        return f"Failed to retrieve logs for {name}: {str(e)}"
