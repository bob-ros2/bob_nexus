import os
import json
import time

# Resolve storage path
TASK_FILE = "tasks.json"

def _load_tasks():
    if not os.path.exists(TASK_FILE):
        return []
    try:
        with open(TASK_FILE, "r") as f:
            return json.load(f)
    except:
        return []

def _save_tasks(tasks):
    with open(TASK_FILE, "w") as f:
        json.dump(tasks, f, indent=2)

def add_swarm_task(name: str, description: str, assignee: str = "Neo"):
    """
    Creates a new mission in the persistent task list.
    Assignee defaults to 'Neo' (the Coder).
    Returns the new Task ID.
    """
    tasks = _load_tasks()
    task_id = int(time.time())
    new_task = {
        "id": task_id,
        "name": name,
        "description": description,
        "assignee": assignee,
        "status": "pending",
        "created_at": time.ctime(),
        "completed_at": None,
        "review_report": None
    }
    tasks.append(new_task)
    _save_tasks(tasks)
    return f"Success: Task '{name}' added (ID: {task_id}). Assigned to {assignee}."

def list_swarm_tasks(include_completed: bool = False):
    """
    Returns a list of all active tasks. 
    Set 'include_completed' to True to see history.
    """
    tasks = _load_tasks()
    if not include_completed:
        tasks = [t for t in tasks if t["status"] == "pending"]
    
    if not tasks:
        return "No tasks found."
    
    return tasks

def complete_swarm_task(task_id: int, review_report: str):
    """
    Marks a task as completed and stores the review findings.
    Use this after verifying the work of the assignee.
    """
    tasks = _load_tasks()
    found = False
    task_name = "Unknown"
    
    for t in tasks:
        if t["id"] == task_id:
            t["status"] = "completed"
            t["completed_at"] = time.ctime()
            t["review_report"] = review_report
            task_name = t["name"]
            found = True
            break
    
    if found:
        _save_tasks(tasks)
        return (
            f"Success: Task {task_id} ('{task_name}') marked as completed.\n"
            "REPORT ARCHIVED: You should now use the 'memory' skill to store a permanent "
            f"record of this completion for future swarms. Report content:\n\n{review_report}"
        )
    return f"Error: Task ID {task_id} not found."
