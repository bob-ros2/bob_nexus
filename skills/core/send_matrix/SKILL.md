---
name: send_matrix
description: Use this skill to send text messages to a Matrix chat room. This is useful for status updates, notifications, or direct communication from an entity.
---

# Matrix Message Skill

This skill allows an entity to send messages to a configured Matrix room.

## Instructions
1. Call `run_skill_script('send_matrix', 'scripts/send_matrix.sh', ['your message here'])` to send it.
2. **Message Prefix**: All messages sent via this skill MUST have a prefix indicating the sender, for example: `Entity <Name>: <message>`.

## Example
User: "Tell the team Alice is online."
Agent: (Calls send_matrix with "Entity Alice: I am now online and ready to assist.")
