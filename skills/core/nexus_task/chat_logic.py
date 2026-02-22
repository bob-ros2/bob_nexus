import os

# Calculate project root from environment or relative path
PROJECT_ROOT = os.getenv("BOB_NEXUS_DIR")
if not PROJECT_ROOT:
    SELF_DIR = os.path.dirname(os.path.abspath(__file__))
    PROJECT_ROOT = os.path.abspath(os.path.join(SELF_DIR, "..", "..", ".."))
TASK_FILE = os.path.join(PROJECT_ROOT, "nexus_task.md")

def get_task_list():
    """
    Reads the current global task list (nexus_task.md).
    """
    if not os.path.exists(TASK_FILE):
        return "No nexus_task.md found. Create one with update_task_list."
    
    try:
        with open(TASK_FILE, "r") as f:
            return f.read()
    except Exception as e:
        return f"Error reading task list: {str(e)}"

def update_task_list(content: str):
    """
    Overwrites the global task list with new content.
    """
    try:
        with open(TASK_FILE, "w") as f:
            f.write(content)
        return "Success: nexus_task.md updated."
    except Exception as e:
        return f"Error updating task list: {str(e)}"

def add_task_item(item: str):
    """
    Appends a new item to the task list.
    """
    try:
        if not os.path.exists(TASK_FILE):
            with open(TASK_FILE, "w") as f:
                f.write("# Nexus Tasks\n\n")
        
        with open(TASK_FILE, "a") as f:
            f.write(f"- [ ] {item}\n")
        return f"Success: Added task: {item}"
    except Exception as e:
        return f"Error adding task item: {str(e)}"
