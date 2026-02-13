import glob
import os
import re
import sys

import yaml


def generate_skills_prompt(entity_dir):
    prompt = "## Available Agent Skills\n"
    prompt += "You have access to the following specialized skills. To use a skill, first call `load_skill(name)` to read its full instructions.\n\n"

    # Scan the entity's own skills directory (where skills are symlinked)
    skills_path = os.path.join(entity_dir, "skills")
    if not os.path.exists(skills_path):
        return None

    skill_files = glob.glob(os.path.join(skills_path, "**/SKILL.md"), recursive=True)

    found = False
    for file_path in skill_files:
        try:
            with open(file_path, "r") as f:
                content = f.read()
                if content.startswith("---"):
                    parts = content.split("---")
                    if len(parts) >= 2:
                        frontmatter = parts[1]
                        data = yaml.safe_load(frontmatter)
                        name = data.get("name")
                        desc = data.get("description")
                        prompt += f"- **{name}**: {desc}\n"
                        found = True
        except Exception as e:
            sys.stderr.write(f"Warning: Error parsing {file_path}: {e}\n")
            continue

    if not found:
        return None

    prompt += "\nIf a user request matches a skill description, you SHOULD call `load_skill` to get detailed behavior instructions."
    return prompt


def update_entity_yaml(entity_dir):
    yaml_file = os.path.join(entity_dir, "llm.yaml")
    if not os.path.exists(yaml_file):
        sys.stderr.write(f"Error: {yaml_file} not found.\n")
        return False

    prompt = generate_skills_prompt(entity_dir)
    if not prompt:
        print(f"[*] No skills found for entity at {entity_dir}. Skipping prompt update.")
        return False

    # Escape for YAML
    escaped_prompt = prompt.replace("'", "''")
    escaped_prompt = escaped_prompt.replace("\n", "\\n")

    with open(yaml_file, "r") as f:
        content = f.read()

    # Pattern for system_prompt:='...'
    pattern = r'("system_prompt:=\')(.*?)(\'")'

    if re.search(pattern, content):
        new_content = re.sub(
            pattern, lambda m: f"{m.group(1)}{escaped_prompt}{m.group(3)}", content
        )
        with open(yaml_file, "w") as f:
            f.write(new_content)
        print(f"Successfully updated system prompt in {yaml_file}")
        return True
    else:
        # Check for placeholder
        placeholder = "<ADD_HERE_SKILLS_SYSTEM_PROMPT>"
        if placeholder in content:
            new_content = content.replace(placeholder, escaped_prompt)
            with open(yaml_file, "w") as f:
                f.write(new_content)
            print(f"Successfully filled placeholder in {yaml_file}")
            return True
        else:
            sys.stderr.write(
                f"Error: Could not find system_prompt line or placeholder in {yaml_file}\n"
            )
            return False


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 generate_prompt.py <entity_dir> [--update]")
        sys.exit(1)

    target_dir = os.path.abspath(sys.argv[1])
    if "--update" in sys.argv:
        update_entity_yaml(target_dir)
    else:
        p = generate_skills_prompt(target_dir)
        print(p if p else "No skills found.")
