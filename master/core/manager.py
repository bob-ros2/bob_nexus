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
        
        # Check if template_category is a path or a name in templates/
        if os.path.exists(template_category) and os.path.isdir(template_category):
            src_template_dir = os.path.abspath(template_category)
        else:
            src_template_dir = os.path.join(self.templates_dir, template_category)

        if os.path.exists(dest_dir):
            raise FileExistsError(f"Entity '{entity_name}' already exists in '{category}'")

        if not os.path.exists(src_template_dir):
            raise FileNotFoundError(f"Template source '{src_template_dir}' not found")

        os.makedirs(dest_dir, exist_ok=True)

        # Update engine with local context for this spawn
        host_nexus = os.getenv("HOST_NEXUS_DIR", self.root_dir)
        self.engine.env_vars.update(
            {
                "NAME": entity_name,
                "ENTITY_CATEGORY": category,
                "ENTITY_DIR": dest_dir,
                "HOST_NEXUS_DIR": host_nexus,
                "BOB_NEXUS_DIR": host_nexus,
            }
        )

        # Process all templates in the template category
        for root, dirs, files in os.walk(src_template_dir):
            rel_path = os.path.relpath(root, src_template_dir)
            target_subdir = os.path.join(dest_dir, rel_path)
            os.makedirs(target_subdir, exist_ok=True)

            for file in files:
                src_file = os.path.join(root, file)
                dest_file = os.path.join(target_subdir, file)

                # Check if it's a file we should process (YAML/JSON/Conf/Env)
                if file.endswith((".yaml", ".yml", ".json", ".conf", ".env", ".template")):
                    self.engine.process_file(src_file, dest_file)
                else:
                    shutil.copy2(src_file, dest_file)

        # Ensure .env exists (mandatory for blueprint.yaml in Docker mode)
        final_env = os.path.join(dest_dir, ".env")
        if not os.path.exists(final_env):
            # Check for .env.template
            template_env = os.path.join(dest_dir, ".env.template")
            if os.path.exists(template_env):
                print(f"[*] auto-env: Initializing {entity_name}/.env from template...")
                shutil.copy2(template_env, final_env)
            else:
                print(f"[*] auto-env: Creating default {entity_name}/.env...")
                with open(final_env, "w") as f:
                    # Basic identity variables
                    f.write(f"# Entity Identity\n")
                    f.write(f"NAME={entity_name}\n")
                    f.write(f"ENTITY_CATEGORY={category}\n")
                    f.write(f"HOST_NEXUS_DIR={os.getenv('HOST_NEXUS_DIR', self.root_dir)}\n")

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
                    print(f"    [!] Error linking skill {s_cat}/{s_name}: {e}")

        # Finally, refresh the prompt so the entity knows its skills
        print(f"Entity '{entity_name}' spawned successfully at {dest_dir}")
        self._refresh_entity_prompt(category, entity_name)

        return dest_dir

    def import_repository(self, name, url=None):
        """
        Clones a repository into the global ros2_ws/src directory.
        """
        src_dir = os.path.join(self.root_dir, "ros2_ws", "src")
        os.makedirs(src_dir, exist_ok=True)
        
        target_dir = os.path.join(src_dir, name)
        if os.path.exists(target_dir):
            print(f"[*] Repository '{name}' already exists in global workspace.")
            return True
            
        if not url:
            # Default to bob-ros2 organization
            url = f"https://github.com/bob-ros2/{name}"
            
        print(f"[*] Importing repository '{name}' from {url}...")
        import subprocess
        result = subprocess.run(["git", "clone", url, target_dir])
        return result.returncode == 0

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
            # Refresh if any valid config exists
            has_config = os.path.exists(os.path.join(entity_dir, "llm.yaml")) or os.path.exists(
                os.path.join(entity_dir, "agent.yaml")
            )

            if has_config:
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
            if os.path.islink(link_name):
                os.remove(link_name)
            else:
                shutil.rmtree(link_name)

        # SWARM 3.0: Copy instead of symlink for strict isolation
        shutil.copytree(skill_src, link_name)
        print(f"[*] Bundled skill '{skill_name}' to entity '{entity_name}'")

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
