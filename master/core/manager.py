import os
import shutil

import yaml
from template_engine import TemplateEngine


class EntityManager:
    def __init__(self, root_dir):
        self.root_dir = os.path.abspath(root_dir)
        self.master_dir = os.path.join(self.root_dir, "master")
        self.entities_dir = os.path.join(self.root_dir, "entities")
        self.templates_dir = os.path.join(self.root_dir, "templates")
        self.skills_dir = os.path.join(self.root_dir, "skills")
        self.env_path = os.path.join(self.master_dir, "config", ".env")
        self.conf_path = os.path.join(self.master_dir, "config", "conf.yaml")

        # Ensure base directories exist
        os.makedirs(self.entities_dir, exist_ok=True)
        os.makedirs(self.skills_dir, exist_ok=True)

        self.engine = TemplateEngine(self.env_path)
        self.conf = self._load_conf()

    def _load_conf(self):
        if os.path.exists(self.conf_path):
            with open(self.conf_path, "r") as f:
                return yaml.safe_load(f) or {}
        return {}

    def spawn_entity(self, category, entity_name, template_category):
        """
        Creates a new entity from a template category.
        """
        dest_dir = os.path.join(self.entities_dir, category, entity_name)
        src_template_dir = os.path.join(self.templates_dir, template_category)

        if os.path.exists(dest_dir):
            raise FileExistsError(f"Entity '{entity_name}' already exists in '{category}'")

        if not os.path.exists(src_template_dir):
            raise FileNotFoundError(f"Template category '{template_category}' not found")

        if not os.path.isdir(src_template_dir):
            raise ValueError(
                f"Template path '{template_category}' is not a directory. "
                "Spawning requires a directory template."
            )

        os.makedirs(dest_dir, exist_ok=True)

        # Update engine with local context for this spawn
        self.engine.env_vars.update(
            {"NAME": entity_name, "CATEGORY": category, "ENTITY_DIR": dest_dir}
        )

        # Process all templates in the template category
        for root, dirs, files in os.walk(src_template_dir):
            rel_path = os.path.relpath(root, src_template_dir)
            target_subdir = os.path.join(dest_dir, rel_path)
            os.makedirs(target_subdir, exist_ok=True)

            for file in files:
                src_file = os.path.join(root, file)
                dest_file = os.path.join(target_subdir, file)

                # Check if it's a file we should process (YAML/JSON/Conf)
                if file.endswith((".yaml", ".yml", ".json", ".conf", ".env")):
                    self.engine.process_file(src_file, dest_file)
                else:
                    shutil.copy2(src_file, dest_file)

        print(f"Entity '{entity_name}' spawned successfully at {dest_dir}")

        # Post-spawn: Linking Default Skills from config
        skill_conf = self.conf.get("skills", {})
        # Prioritize category specifically, fall back to 'defaults'
        defaults = skill_conf.get(category, skill_conf.get("defaults", []))

        if defaults:
            print(f"[*] Post-spawn: Linking default skills for category '{category}'...")
            for s_cat, s_name in defaults:
                try:
                    self.link_skill(category, entity_name, s_cat, s_name)
                except Exception as e:
                    print(f"    [!] Failed to link default skill {s_name}: {e}")

        # Finally, refresh the prompt so the entity knows its skills
        self._refresh_entity_prompt(category, entity_name)

        return dest_dir

    def _refresh_entity_prompt(self, category, entity_name):
        """
        Runs generate_prompt.py for the specified entity.
        """
        entity_dir = os.path.join(self.entities_dir, category, entity_name)
        if not os.path.exists(entity_dir):
            return

        import subprocess

        script_path = os.path.join(self.master_dir, "core", "generate_prompt.py")
        if os.path.exists(script_path):
            # Only refresh if llm.yaml exists (not every entity is an LLM agent)
            if os.path.exists(os.path.join(entity_dir, "llm.yaml")):
                print(f"[*] auto-refresh: Updating system prompt for {entity_name}...")
                subprocess.run(["python3", script_path, entity_dir, "--update"])

    def find_entity(self, entity_name):
        """
        Scans all categories to find an entity by name.
        Returns (category, entity_dir) or (None, None).
        """
        if not os.path.exists(self.entities_dir):
            return None, None

        for category in os.listdir(self.entities_dir):
            cat_path = os.path.join(self.entities_dir, category)
            if not os.path.isdir(cat_path):
                continue

            ent_path = os.path.join(cat_path, entity_name)
            if os.path.exists(ent_path) and os.path.isdir(ent_path):
                return category, ent_path

        return None, None

    def link_skill(self, category, entity_name, skill_category, skill_name):
        """
        Symlinks a skill into the entity's skill folder.
        """
        entity_dir = os.path.join(self.entities_dir, category, entity_name)
        skill_src = os.path.join(self.skills_dir, skill_category, skill_name)

        if not os.path.exists(entity_dir):
            raise FileNotFoundError(f"Entity '{entity_name}' not found")

        if not os.path.exists(skill_src):
            raise FileNotFoundError(f"Skill '{skill_name}' not found in '{skill_category}'")

        entity_skills_dir = os.path.join(entity_dir, "skills")
        os.makedirs(entity_skills_dir, exist_ok=True)

        link_name = os.path.join(entity_skills_dir, skill_name)

        if os.path.exists(link_name):
            os.remove(link_name)

        os.symlink(skill_src, link_name)
        print(f"Linked skill '{skill_name}' to entity '{entity_name}'")

        # Auto-refresh prompt after linking
        self._refresh_entity_prompt(category, entity_name)


if __name__ == "__main__":
    # Test stub
    import sys

    # Example usage: python3 manager.py spawn assistant bob bob_launch
    # Determine root relative to this file
    this_dir = os.path.dirname(os.path.abspath(__file__))
    root = os.path.abspath(os.path.join(this_dir, "..", ".."))
    manager = EntityManager(root)
    if len(sys.argv) > 1:
        if sys.argv[1] == "spawn":
            manager.spawn_entity(sys.argv[2], sys.argv[3], sys.argv[4])
        elif sys.argv[1] == "link":
            manager.link_skill(sys.argv[2], sys.argv[3], sys.argv[4], sys.argv[5])
