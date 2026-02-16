import subprocess
import os
import sys
import json
import tempfile


def bash_run(command: str) -> str:
    """
    Executes a bash command and returns the combined stdout and stderr.
    Use this for system investigation, package management, or script execution.
    """
    try:
        result = subprocess.run(
            ["/bin/bash", "-c", command],
            capture_output=True,
            text=True,
            timeout=60,
            check=False
        )
        output = result.stdout
        if result.stderr:
            output += f"\n--- STDERR ---\n{result.stderr}"
        
        status = f"[Exit Code: {result.returncode}]"
        return f"{status}\n{output}" if output else f"{status} (No output)"
    except Exception as e:
        return f"Error executing bash: {str(e)}"


def python_run(code: str) -> str:
    """
    Executes a Python code snippet in a separate process.
    Use this for data manipulation, testing logic, or running complex scripts.
    """
    try:
        with tempfile.NamedTemporaryFile(mode='w', suffix='.py', delete=False) as tmp:
            tmp.write(code)
            tmp_path = tmp.name
        
        result = subprocess.run(
            [sys.executable, tmp_path],
            capture_output=True,
            text=True,
            timeout=30,
            check=False
        )
        os.unlink(tmp_path)
        
        output = result.stdout
        if result.stderr:
            output += f"\n--- STDERR ---\n{result.stderr}"
            
        status = f"[Exit Code: {result.returncode}]"
        return f"{status}\n{output}" if output else f"{status} (No output)"
    except Exception as e:
        return f"Error executing python: {str(e)}"


def fs_ls(path: str = ".") -> str:
    """
    Lists files and directories in the specified path.
    """
    try:
        items = os.listdir(path)
        details = []
        for item in items:
            full = os.path.join(path, item)
            itype = "DIR " if os.path.isdir(full) else "FILE"
            size = os.path.getsize(full) if not os.path.isdir(full) else "-"
            details.append(f"{itype} | {size:>10} | {item}")
        return "\n".join(details) if details else "Directory is empty."
    except Exception as e:
        return f"Error listing {path}: {str(e)}"


def fs_read(path: str) -> str:
    """
    Reads the content of a file. Use this to inspect source code or logs.
    """
    try:
        with open(path, "r") as f:
            return f.read()
    except Exception as e:
        return f"Error reading {path}: {str(e)}"


def fs_write(path: str, content: str) -> str:
    """
    Writes content to a file. Use this to create or update scripts.
    """
    try:
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        with open(path, "w") as f:
            f.write(content)
        return f"Successfully written to {path}"
    except Exception as e:
        return f"Error writing to {path}: {str(e)}"


def fs_mkdir(path: str) -> str:
    """
    Creates a new directory structure.
    """
    try:
        os.makedirs(path, exist_ok=True)
        return f"Directory {path} created."
    except Exception as e:
        return f"Error creating directory {path}: {str(e)}"
