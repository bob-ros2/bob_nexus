---
name: search_web
description: Use this skill to perform web searches via your own SearXNG instance (combines results from Google, Bing, etc.). Provides titles, snippets, and links.
---

# Web Search Skill (SearXNG)

This skill allows the agent to find information on the web using a SearXNG instance.

## Instructions
1. Call `run_skill_script('search_web', 'scripts/search.py', ['your search query'])` to execute a search.
2. The script returns a JSON object with search results from your SearXNG instance.
3. Summarize the results for the user and provide links if relevant.

## Example
User: "Who won the Super Bowl last night?"
Agent: (Calls search_web)
Agent: "According to my search, [Team Name] won the Super Bowl..."
