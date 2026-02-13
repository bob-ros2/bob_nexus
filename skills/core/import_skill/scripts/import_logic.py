import json
import logging
import os
import shutil
import subprocess
import sys

# Configure logging to stdout for visibility
logging.basicConfig(
    level=logging.DEBUG,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
    stream=sys.stdout,
)
logger = logging.getLogger("import_logic")

# List of skills found in anthropics/skills/skills/
ANTHROPIC_SKILLS = [
    "algorithmic-art",
    "brand-guidelines",
    "canvas-design",
    "doc-coauthoring",
    "docx",
    "frontend-design",
    "internal-comms",
    "mcp-builder",
    "pdf",
    "pptx",
    "skill-creator",
    "slack-gif-creator",
    "theme-factory",
    "web-artifacts-builder",
    "webapp-testing",
    "xlsx",
]

# Determine project root relative to this file
# This script usually sits in skills/core/import_skill/scripts/import_logic.py
ROOT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
SKILLS_DIR = ROOT_DIR
REPO_URL = "https://github.com/anthropics/skills.git"


def search_skills(query):
    query = query.lower().strip()
    logger.debug(f"Searching for skills with query: '{query}'")

    if not query:
        logger.debug("Empty query, returning all available skills.")
        return {"status": "success", "results": ANTHROPIC_SKILLS}

    matches = [s for s in ANTHROPIC_SKILLS if query in s.lower()]
    logger.debug(f"Found {len(matches)} matches: {matches}")
    return {"status": "success", "results": matches}


def import_skill(skill_name):
    logger.debug(f"Starting import for skill: '{skill_name}'")

    if skill_name not in ANTHROPIC_SKILLS:
        logger.error(f"Skill '{skill_name}' not in official repository list.")
        return {
            "status": "error",
            "message": f"Skill '{skill_name}' not found in official repository.",
        }

    target_dir = os.path.join(SKILLS_DIR, skill_name)
    if os.path.exists(target_dir):
        logger.warning(f"Target directory {target_dir} already exists.")
        return {"status": "error", "message": f"Skill '{skill_name}' already exists locally."}

    tmp_dir = "/tmp/anthropic_skills_clone"
    try:
        if os.path.exists(tmp_dir):
            logger.debug(f"Cleaning up existing temp directory: {tmp_dir}")
            shutil.rmtree(tmp_dir)

        logger.debug(f"Cloning {REPO_URL} into {tmp_dir}...")
        result = subprocess.run(
            ["git", "clone", "--depth", "1", REPO_URL, tmp_dir],
            check=True,
            capture_output=True,
            text=True,
        )
        logger.debug(f"Git clone stdout: {result.stdout}")

        source_dir = os.path.join(tmp_dir, "skills", skill_name)
        if not os.path.exists(source_dir):
            logger.error(f"Source directory {source_dir} not found after clone.")
            return {
                "status": "error",
                "message": f"Folder 'skills/{skill_name}' not found in repo.",
            }

        logger.debug(f"Copying {source_dir} to {target_dir}...")
        shutil.copytree(source_dir, target_dir)
        logger.debug("Copy successful.")

        # Automatically update the system prompt
        promo_script = os.path.join(SKILLS_DIR, "generate_prompt.py")
        if os.path.exists(promo_script):
            logger.debug(f"Running prompt update script: {promo_script}")
            update_result = subprocess.run(
                [sys.executable, promo_script, "--update"],
                check=True,
                capture_output=True,
                text=True,
            )
            logger.debug(f"Update script stdout: {update_result.stdout}")
        else:
            logger.warning(f"Prompt update script not found at {promo_script}")

        logger.info(f"Skill '{skill_name}' successfully imported.")
        return {
            "status": "success",
            "message": f"Skill '{skill_name}' imported and system prompt updated.",
        }

    except subprocess.CalledProcessError as e:
        logger.exception("Subprocess command failed.")
        return {"status": "error", "message": f"Command failed: {e.stderr}"}
    except Exception as e:
        logger.exception("An unexpected error occurred during import.")
        return {"status": "error", "message": str(e)}
    finally:
        if os.path.exists(tmp_dir):
            logger.debug(f"Final cleanup of temp directory: {tmp_dir}")
            shutil.rmtree(tmp_dir)


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(
            json.dumps(
                {"status": "error", "message": "No command provided. Use 'search' or 'import'."}
            )
        )
        sys.exit(1)

    cmd = sys.argv[1]
    arg = sys.argv[2] if len(sys.argv) > 2 else ""

    if cmd == "search":
        result = search_skills(arg)
    elif cmd == "import":
        result = import_skill(arg)
    else:
        result = {"status": "error", "message": f"Unknown command: {cmd}"}

    # Print the JSON result as the very last thing for the agent to parse
    print(json.dumps(result))
