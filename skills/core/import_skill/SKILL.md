---
name: import_skill
description: Use this skill to search and download new Agent Skills from the official Anthropic repository (https://github.com/anthropics/skills). It can list available skills, handle fuzzy name matching, and download skill folders directly into your workspace.
---

# Import Skill

This skill allows you to extend your capabilities by downloading new skills from the community and Anthropic.

## Instructions
1. **Listing/Searching**: To see what's available or search for a name, call `run_skill_script('import_skill', 'scripts/import_logic.py', ['search', '<query_or_empty>'])`.
2. **Importing**: To download a skill, call `run_skill_script('import_skill', 'scripts/import_logic.py', ['import', '<exact_skill_name>'])`.
3. **Fuzzy Matching**: If the user provides a name that isn't an exact match, use the `search` command first. If it returns similar names, present them to the user.
4. **Final Step**: After importing a skill, you MUST call `generate_prompt.py --update` to make the new skill visible in your system prompt. Use `subprocess` or a similar tool to run the update script.

## Example Flow
User: "Download the pdf skill"
Agent: (Checks search results)
Agent: "I found 'pdf'. Should I import it?"
User: "Yes"
Agent: (Calls import logic, then updates prompt)
