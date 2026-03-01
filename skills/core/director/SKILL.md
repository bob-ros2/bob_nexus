---
name: director
description: Specialized skill for managing the visual dashboard layout and director events.
---

# Director Skill

This skill allows the agent to act as a "Director" (Regie-Raum) for the `bob_sdlviz` dashboard. You can add, update, and remove visual layers to create dynamic scenes.

## Strict Usage Rules
1. **FUNCTION CALLS ONLY**: You MUST use the `update_director_layout` or `send_director_message` functions. NEVER output raw JSON in your chat response.
2. **NO SCRIPTS/TIMERS**: You do not have a "timer" or "scripting" tool. If you need a layer to disappear automatically, use the `expire` parameter.
3. **LAYER IDS**: Reuse IDs to update existing layers. Use `action: 'remove'` to delete them.

## Common Layer Fields
All layer types support:
- `id` (string): Unique identifier.
- `action` (string): `add` (default) or `remove`.
- `title` (string): Optional title bar displayed above the layer.
- `expire` (float): Auto-remove the layer after N seconds (0 for infinite).

## Layer Types

### 1. `String` (Text Terminal)
Renders a rolling text terminal or a one-off alert.
- `topic` (string): ROS topic for incoming strings.
- `text` (string): Optional static text to display immediately.
- `area` (list): `[x, y, width, height]`.
- `text_color`: `[R, G, B, A]` (Default: `[200, 200, 200, 255]`).
- `bg_color`: `[R, G, B, A]` (Default: `[30, 30, 30, 180]`).
- `align`: `left`, `center`, or `right`.
- `line_limit`: Max lines to keep in history.
- `clear_on_new` (bool): If true, clears the terminal on every new message.
- `append_newline` (bool): Automatically add `\n` to messages.

### 2. `Image`
Renders a `sensor_msgs/msg/Image`.
- `topic` (string): ROS topic for incoming images.
- `area` (list): `[x, y, width, height]`.
  - If width/height is 0, it scales proportionally based on the other dimension.
  - If omitted, uses original image dimensions at `[0, 0]`.

### 3. `VideoStream` (FIFO Input)
Displays raw video buffers from a pipe.
- `topic`: Path to the FIFO pipe (e.g., `/tmp/overlay_video`).
- `area`: `[x, y, width, height]`.
- `source_width` / `source_height`: Input dimensions (Default: 640x480).

### 4. `MarkerLayer` (2D Projection)
Projects 3D ROS markers onto a 2D plane.
- `topic`: ROS topic for `visualization_msgs/MarkerArray`.
- `area`: `[x, y, width, height]`.
- `scale` (float): Pixels per meter (Default: 1000.0).
- `exclude_ns` (string): Comma-separated list of marker namespaces to hide.

## Tools

### `update_director_layout`
Sends a list of layer definitions to the dashboard.

### `send_director_message`
Sends a text message to a specific topic that a dashboard layer is watching.

## Examples

**Add an image layer (e.g., Camera or Map)**:
```json
{
  "layers": [
    {
      "id": "main_cam",
      "type": "Image",
      "topic": "/image",
      "area": [10, 10, 400, 0],
      "title": "LIVE FEED"
    }
  ]
}
```

**Add a temporary notification with title**:
```json
{
  "layers": [
    {
      "id": "alert",
      "type": "String",
      "text": "CRITICAL SYSTEM REBOOT",
      "area": [227, 10, 400, 50],
      "bg_color": [255, 0, 0, 180],
      "title": "SECURITY",
      "expire": 5.0
    }
  ]
}
```
