---
name: coder_lab
description: Provides advanced system interaction capabilities, including a persistent Bash shell, a Python REPL, and clinical file system management for the Coder & Shell King.
---

# Coder Lab Skill

This skill transforms an entity into a high-performance system architect. It provides direct, raw access to the underlying OS and Python environment.

## Capabilities

### 1. Bash Execution (`bash_run`)
- Execute any shell command.
- Persistent environment (within the scope of the call).
- Returns stdout and stderr.

### 2. Python REPL (`python_run`)
- Execute Python code snippets.
- Use this for complex calculations, data processing, or algorithmic development.
- The code runs in a separate process for safety.

### 3. Systematic File Management (`fs_*`)
- `fs_ls`: Transparent view of the workspace.
- `fs_read`: Deep inspection of code and configurations.
- `fs_write`: Atomic file creation and updating.
- `fs_mkdir`: Structural organization.

## Usage Guidelines
- **Precision**: Be surgical with file paths.
- **Safety**: Always check the output of a command before proceeding to the next step.
- **Isolation**: Remember that you are in an isolated research workspace. Your changes here will not corrupt the Mastermind core unless explicitly targeted.
