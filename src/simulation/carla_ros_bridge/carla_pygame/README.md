# carla_pygame

Web-based map visualization for CARLA simulation.

## Nodes

### pygame_hud

Renders a bird's-eye view of the CARLA map showing roads, lane markings, and actor positions. The visualization is streamed to a web browser via WebSocket, allowing remote monitoring without requiring a display on the simulation host.

The view is centered on the ego vehicle and supports pan/zoom via mouse interaction in the browser. Optionally shows spawn points, trigger volumes, and lane connections for debugging.

Access the visualization at `http://localhost:<web_port>` (default port 5000).

```bash
ros2 run carla_pygame pygame_hud
```

**Parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `carla_host` | string | `localhost` | CARLA server hostname |
| `carla_port` | int | `2000` | CARLA server port |
| `carla_timeout` | double | `10.0` | Connection timeout in seconds |
| `web_port` | int | `5000` | Web server port for map visualization |
| `width` | int | `1280` | Render width in pixels |
| `height` | int | `720` | Render height in pixels |
| `role_name` | string | `ego_vehicle` | Role name of the ego vehicle to track |
| `show_triggers` | bool | `false` | Show trigger volumes on map |
| `show_connections` | bool | `false` | Show lane connections on map |
| `show_spawn_points` | bool | `false` | Show spawn points on map |
| `frame_rate` | double | `30.0` | Frame rate for web streaming in Hz |
