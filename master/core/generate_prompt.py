import glob
import os
import re
import sys
import yaml

def generate_skills_prompt(entity_dir):
    prompt = "## Available Agent Skills\n"
    prompt += (
        "You have access to the following specialized skills. "
        "To use a skill, first call `load_skill(name)` to read its full "
        "instructions.\n\n"
    )
    # Scan the entity's own skills directory (where skills are symlinked)
    skills_path = os.path.join(entity_dir, "skills")
    if not os.path.exists(skills_path):
        return None

    print(f"[*] Scanning for skills in: {skills_path}")
    skill_files = glob.glob(os.path.join(skills_path, "**/SKILL.md"), recursive=True)
    print(f"[*] Found {len(skill_files)} SKILL.md files.")

    found = False
    for file_path in skill_files:
        print(f"  -> Processing {file_path}")
        try:
            with open(file_path, "r") as f:
                content = f.read()
                # Use regex to find frontmatter between --- markers
                match = re.search(r"^---\s*\n(.*?)\n---\s*\n", content, re.DOTALL | re.MULTILINE)
                if match:
                    frontmatter = match.group(1)
                    data = yaml.safe_load(frontmatter)
                    name = data.get("name")
                    desc = data.get("description")
                    if name and desc:
                        prompt += f"- **{name}**: {desc}\n"
                        found = True
                        print(f"     [+] Added skill metadata: {name}")
                else:
                    # Legacy or marker-less
                    print(f"     [!] No YAML frontmatter found.")
        except Exception as e:
            sys.stderr.write(f"Warning: Error parsing {file_path}: {e}\n")
            continue

    if not found:
        print("[!] No valid SKILL.md metadata found.")
        return None

    prompt += (
        "\nIf a user request matches a skill description, you SHOULD call "
        "`load_skill` to get detailed behavior instructions."
    )
    return prompt


def update_prompt_file(file_path, prompt):
    """Updates the specialized markers within a .txt or .md file."""
    if not os.path.exists(file_path):
        # Create initial with placeholder if it doesn't exist
        with open(file_path, "w") as f:
            f.write("You are a helpful assistant.\n\n<ADD_HERE_SKILLS_SYSTEM_PROMPT>\n")

    with open(file_path, "r") as f:
        content = f.read()

    start_marker = "# --- NEXUS SKILLS PROMPT START ---"
    end_marker = "# --- NEXUS SKILLS PROMPT END ---"
    placeholder = "<ADD_HERE_SKILLS_SYSTEM_PROMPT>"

    # Phase 1: Marker-based update (preferred)
    if start_marker in content and end_marker in content:
        pattern = re.escape(start_marker) + r".*?" + re.escape(end_marker)
        new_block = f"{start_marker}\n{prompt}\n{end_marker}"
        new_content = re.sub(pattern, new_block, content, flags=re.DOTALL)
        with open(file_path, "w") as f:
            f.write(new_content)
        return True

    # Phase 2: Placeholder-based update
    if placeholder in content:
        new_block = f"{start_marker}\n{prompt}\n{end_marker}"
        new_content = content.replace(placeholder, new_block)
        with open(file_path, "w") as f:
            f.write(new_content)
        return True

    # Fallback: Just append if no markers found
    with open(file_path, "a") as f:
        f.write(f"\n\n{start_marker}\n{prompt}\n{end_marker}\n")
    return True


def update_entity_yaml(entity_dir):
    # Support multiple configuration names
    candidates = ["agent.yaml", "bob_launch.yaml", "launch.yaml"]
    targets = []
    for c in candidates:
        p = os.path.join(entity_dir, c)
        if os.path.exists(p):
            targets.append(p)

    if not targets:
        sys.stderr.write(f"Error: No config files found in {entity_dir}\n")
        return False

    prompt = generate_skills_prompt(entity_dir)
    if not prompt:
        return False

    placeholder = "<ADD_HERE_SKILLS_SYSTEM_PROMPT>"
    updated_any = False

    for yaml_file in targets:
        print(f"[*] Updating {os.path.basename(yaml_file)}...")
        try:
            with open(yaml_file, "r") as f:
                lines = f.readlines()
        except Exception as e:
            sys.stderr.write(f"Error reading {yaml_file}: {e}\n")
            continue

        processed = False
        new_lines = []
        skip_indented = False
        
        for i, line in enumerate(lines):
            # 1. Handle skipping lines of a block scalar being removed
            if skip_indented:
                # If the line is indented or empty, it's still part of the block
                if re.match(r"^\s+", line) or line.strip() == "":
                    # print(f"  [DEBUG] Skipping indented line: {line.strip()[:20]}...")
                    continue
                else:
                    # Found next unindented key
                    skip_indented = False

            # 2. Look for the system_prompt key (only if not already processed for this file)
            # We match key, optional separator (:= or :), optional spaces, and the value
            # Note: We use a non-greedy catch-all for the key part to handle various formats
            match = re.search(r"^(.*?system_prompt[:=]+)\s*(.*?)$", line)
            
            if match and not processed:
                prefix = match.group(1)
                value = match.group(2).strip()
                
                # Check if value is already a path
                is_file_path = value.endswith((".txt", ".md")) or (value.startswith(("./", "/")) and "." in value)
                
                if is_file_path:
                    # Clean the path reference
                    clean_path = value.strip("\"'|").strip()
                    # Resolve relative path
                    if clean_path.startswith("./"):
                        p_path = os.path.join(entity_dir, clean_path[2:])
                    else:
                        p_path = os.path.join(entity_dir, clean_path)
                    
                    print(f"  [+] Update existing prompt file: {clean_path}")
                    update_prompt_file(p_path, prompt)
                    new_lines.append(line) # Keep line exactly as is
                    processed = True
                else:
                    # MIGRATION: Switch from text/block-scalar to file reference
                    p_path = os.path.join(entity_dir, "system_prompt.txt")
                    print(f"  [>] Migrating system_prompt to external file: ./system_prompt.txt")
                    
                    # If the line contained a block scalar indicator '|' or '>'
                    if "|" in value or ">" in value or value == "":
                        skip_indented = True
                    
                    # Write new reference line
                    # Ensure space after colon
                    if not prefix.endswith(" "):
                        prefix += " "
                    new_lines.append(f"{prefix}./system_prompt.txt\n")
                    
                    # Ensure specialized prompt file exists with placeholder
                    if not os.path.exists(p_path):
                        with open(p_path, "w") as f:
                            f.write("You are a helpful assistant.\n\n" + placeholder + "\n")
                    
                    update_prompt_file(p_path, prompt)
                    processed = True
            else:
                # Normal line
                new_lines.append(line)

        # Write back if something changed
        if processed:
             with open(yaml_file, "w") as f:
                f.writelines(new_lines)
             print(f"  [!] Re-wrote {os.path.basename(yaml_file)} successfully.")
             updated_any = True

    return updated_any


if __name__ == "__main__":
    if len(sys.argv) < 2:
        sys.exit(1)

    target_dir = os.path.abspath(sys.argv[1])
    if "--update" in sys.argv:
        update_entity_yaml(target_dir)
    else:
        p = generate_skills_prompt(target_dir)
        print(p if p else "No skills found.")
