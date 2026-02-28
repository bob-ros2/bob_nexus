---
name: director
description: Specialized skill for managing the visual dashboard layout and director events.
---

# Director Skill

This skill allows the agent to act as a "Director" (Regie-Raum) for the `bob_sdlviz` dashboard. You can add, update, and remove visual layers to create dynamic scenes.

## Tools

### `update_director_layout`
Sends a list of layer definitions to the dashboard to update the scene.

**Arguments**:
- `layers` (list): A list of JSON objects defining the layers.

**Layer Schema**:
- `id` (string, optional): Unique identifier for the layer.
- `type` (string): Type of the layer (`String`, `MarkerLayer`, `VideoStream`).
- `action` (string, optional): `add` (default) or `remove`.
- `area` (list): `[x, y, width, height]` in pixels.
- `topic` (string): The ROS topic or FIFO path providing data for this layer.
- `text_color` (list, optional): `[R, G, B, A]` for text layers.
- `bg_color` (list, optional): `[R, G, B, A]` for background.

### `send_director_message`
Sends a text message to a specific topic that a dashboard layer is watching.

**Arguments**:
- `topic` (string): The topic name (e.g., `/bob/log`).
- `text` (string): The content to display.

## Examples

**Add a log terminal**:
```json
{
  "layers": [
    {
      "id": "status_log",
      "type": "String",
      "topic": "/bob/log",
      "area": [10, 10, 400, 200]
    }
  ]
}
```

**Remove a layer**:
```json
{
  "layers": [
    {"id": "status_log", "action": "remove"}
  ]
}
```
