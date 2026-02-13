#!/usr/bin/env python3
import argparse
import json
import os
import sys

# Ensure we can import from core/
sys.path.append(os.path.join(os.path.dirname(__file__), "..", "core"))

from deployer import Deployer
from manager import EntityManager


def main():
    root_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
    manager = EntityManager(root_dir)
    deployer = Deployer(root_dir)

    parser = argparse.ArgumentParser(description="Mastermind CLI")
    subparsers = parser.add_subparsers(dest="command", help="Commands")

    # Spawn command
    spawn_parser = subparsers.add_parser("spawn", help="Spawn a new entity")
    spawn_parser.add_argument("category", help="Entity category (e.g., assistant)")
    spawn_parser.add_argument("name", help="Entity name (e.g., bob)")
    spawn_parser.add_argument("template", help="Template category (e.g., bob_launch)")

    # Link command
    link_parser = subparsers.add_parser("link", help="Link a skill to an entity")
    link_parser.add_argument("name", help="Entity name (or category/name)")
    link_parser.add_argument("skill", help="Skill name (or category/name)")
    link_parser.add_argument("--category", help="Explicit entity category")
    link_parser.add_argument("--skill-category", help="Explicit skill category")

    # Up command
    up_parser = subparsers.add_parser("up", help="Start an entity")
    up_parser.add_argument("name", help="Entity name (category inferred if omitted)")
    up_parser.add_argument("category", nargs="?", help="Optional entity category")

    # Down command
    down_parser = subparsers.add_parser("down", help="Stop an entity")
    down_parser.add_argument("name", help="Entity name (category inferred if omitted)")
    down_parser.add_argument("category", nargs="?", help="Optional entity category")

    # Status command
    subparsers.add_parser("status", help="Show all entities status")

    # Refresh command
    refresh_parser = subparsers.add_parser(
        "refresh", help="Refresh entity system prompt based on skills"
    )
    refresh_parser.add_argument("name", help="Entity name")
    refresh_parser.add_argument("category", nargs="?", help="Optional entity category")

    args = parser.parse_args()

    if not args.command:
        parser.print_help()
        return

    try:
        if args.command == "spawn":
            manager.spawn_entity(args.category, args.name, args.template)
        elif args.command == "link":
            category = args.category
            entity_name = args.name
            if not category:
                category, _ = manager.find_entity(entity_name)

            if not category:
                raise ValueError(f"Could not find entity '{entity_name}'. Please specify category.")

            # Skill lookup logic could be added to manager too, but for now we keep it simple
            skill_category = args.skill_category or "core"  # Default to core
            skill_name = args.skill

            manager.link_skill(category, entity_name, skill_category, skill_name)
        elif args.command == "up":
            category = args.category
            entity_name = args.name
            if not category:
                category, _ = manager.find_entity(entity_name)

            if not category:
                raise ValueError(f"Could not find entity '{entity_name}'. Please specify category.")

            entity_dir = os.path.join(manager.entities_dir, category, entity_name)
            res = deployer.up_local(entity_dir)
            print(res)
        elif args.command == "down":
            category = args.category
            entity_name = args.name
            if not category:
                category, _ = manager.find_entity(entity_name)

            if not category:
                raise ValueError(f"Could not find entity '{entity_name}'. Please specify category.")

            entity_dir = os.path.join(manager.entities_dir, category, entity_name)
            res = deployer.down_local(entity_dir)
            print(res)
        elif args.command == "refresh":
            category = args.category
            entity_name = args.name
            if not category:
                category, _ = manager.find_entity(entity_name)

            if not category:
                raise ValueError(f"Could not find entity '{entity_name}'. Please specify category.")

            entity_dir = os.path.join(manager.entities_dir, category, entity_name)
            # Call generate_prompt.py --update
            import subprocess

            cmd = [
                "python3",
                os.path.join(manager.master_dir, "core", "generate_prompt.py"),
                entity_dir,
                "--update",
            ]
            subprocess.run(cmd)
        elif args.command == "status":
            print(f"{'CATEGORY':<15} {'NAME':<15} {'STATUS':<10} {'IDENTIFIER':<15}")
            print("-" * 65)
            for cat in os.listdir(manager.entities_dir):
                cat_path = os.path.join(manager.entities_dir, cat)
                if not os.path.isdir(cat_path):
                    continue
                for ent in os.listdir(cat_path):
                    ent_path = os.path.join(cat_path, ent)
                    manifest_path = os.path.join(ent_path, "manifest.json")
                    status = "stopped"
                    identifier = "-"
                    if os.path.exists(manifest_path):
                        with open(manifest_path, "r") as f:
                            m = json.load(f)
                            # Get actual status from driver
                            status = deployer.driver.get_status(m)

                            # Identifier is PID or Container ID
                            if m.get("pid"):
                                identifier = f"PID:{m['pid']}"
                            elif m.get("container_id"):
                                identifier = f"CID:{m['container_id'][:12]}"  # Short CID

                    print(f"{cat:<15} {ent:<15} {status:<10} {identifier:<15}")
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
