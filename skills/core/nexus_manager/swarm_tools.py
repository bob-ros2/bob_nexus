import os
import json
import yaml
import glob

NEXUS_DIR = "/app"

def swarm_status():
    """
    Scans the entities directory and returns status for all entities.
    """
    entities = []
    status_files = glob.glob(f"{NEXUS_DIR}/entities/*/*/status.json")
    
    for sf in status_files:
        try:
            with open(sf, 'r') as f:
                data = json.load(f)
                rel_path = os.path.relpath(sf, f"{NEXUS_DIR}/entities")
                parts = rel_path.split(os.sep)
                entities.append({
                    "category": parts[0],
                    "name": parts[1],
                    "status": data.get("status", "unknown"),
                    "last_active": data.get("last_active", "unknown")
                })
        except:
            pass
    return entities

def swarm_delegate(target_category, target_name, mission_prompt):
    """
    Delegates a mission to another entity.
    """
    target_dir = os.path.join(NEXUS_DIR, "entities", target_category, target_name)
    if not os.path.exists(target_dir):
        return {"error": f"Target entity {target_category}/{target_name} not found."}
    
    inbox_path = os.path.join(target_dir, "inbox.json")
    mission = {
        "source": "prime_coordinator",
        "prompt": mission_prompt,
        "type": "technical_mission"
    }
    
    try:
        with open(inbox_path, 'w') as f:
            json.dump(mission, f, indent=2)
        return {"success": f"Mission delegated to {target_name}."}
    except Exception as e:
        return {"error": str(e)}

def swarm_check_result(target_category, target_name):
    """
    Checks for the outbox of a target entity.
    """
    outbox_path = os.path.join(NEXUS_DIR, "entities", target_category, target_name, "outbox.json")
    if not os.path.exists(outbox_path):
        return {"status": "pending", "message": "No result in outbox yet."}
    
    try:
        with open(outbox_path, 'r') as f:
            result = json.load(f)
        return {"status": "completed", "result": result}
    except Exception as e:
        return {"error": str(e)}
