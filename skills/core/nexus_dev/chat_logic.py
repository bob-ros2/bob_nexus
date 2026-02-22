import os

# Calculate project root from environment or relative path
PROJECT_ROOT = os.getenv("BOB_NEXUS_DIR")
if not PROJECT_ROOT:
    SELF_DIR = os.path.dirname(os.path.abspath(__file__))
    PROJECT_ROOT = os.path.abspath(os.path.join(SELF_DIR, "..", "..", ".."))

def _is_safe_path(path):
    """
    Ensures the path is within the project root to prevent data leakage or system damage.
    """
    abs_path = os.path.abspath(os.path.join(PROJECT_ROOT, path))
    return abs_path.startswith(PROJECT_ROOT)

def read_code(path: str):
    """
    Reads the content of a file within the project.
    """
    if not _is_safe_path(path):
        return "Error: Path out of bounds."
    
    full_path = os.path.abspath(os.path.join(PROJECT_ROOT, path))
    if not os.path.exists(full_path):
        return f"Error: File '{path}' not found."
    
    try:
        with open(full_path, "r") as f:
            return f.read()
    except Exception as e:
        return f"Error reading file: {str(e)}"

def edit_code(path: str, content: str, create: bool = False):
    """
    Writes or overwrites a file. 'create' flag allows creating new files.
    """
    if not _is_safe_path(path):
        return "Error: Path out of bounds."
    
    full_path = os.path.abspath(os.path.join(PROJECT_ROOT, path))
    if not create and not os.path.exists(full_path):
        return f"Error: File '{path}' not found. Use create=True to create new files."
    
    try:
        # Ensure parent directory exists
        os.makedirs(os.path.dirname(full_path), exist_ok=True)
        with open(full_path, "w") as f:
            f.write(content)
        return f"Success: File '{path}' updated."
    except Exception as e:
        return f"Error writing file: {str(e)}"

def list_dir(path: str = "."):
    """
    Lists directory contents to explore the project structure.
    """
    if not _is_safe_path(path):
        return "Error: Path out of bounds."
    
    full_path = os.path.abspath(os.path.join(PROJECT_ROOT, path))
    if not os.path.exists(full_path) or not os.path.isdir(full_path):
        return f"Error: Directory '{path}' not found."
    
    try:
        items = os.listdir(full_path)
        result = []
        for item in items:
            item_path = os.path.join(full_path, item)
            suffix = "/" if os.path.isdir(item_path) else ""
            result.append(f"{item}{suffix}")
        return sorted(result)
    except Exception as e:
        return f"Error listing directory: {str(e)}"
