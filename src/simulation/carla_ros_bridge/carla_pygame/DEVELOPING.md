# Developing carla_pygame

## Design Rationale

The HUD runs headless (SDL dummy driver) and streams to a web browser via WebSocket. This allows:
1. Visualization on remote/headless simulation servers
2. Multiple viewers without additional CARLA load
3. No X11/display dependencies

Pygame is used for rendering because CARLA's example code uses it and it handles map drawing well.

## Architecture

1. On configure: connect to CARLA, set up pygame with dummy driver
2. On activate: render map background, start web server, create render timer
3. Timer renders actors to surface, encodes as JPEG, broadcasts via WebSocket
4. Web clients receive frames and display in browser

## Web Streaming

The node runs a simple HTTP server for the viewer page and a WebSocket server for frame data. Frames are JPEG-encoded to reduce bandwidth.

Frame rate is controlled by `frame_rate` parameter to balance responsiveness vs CPU usage.

## Map Rendering

The map is rendered once on activation (roads, lane markings, etc.) to a background surface. Each frame composites actors (vehicles, pedestrians, ego) on top of this background.

The view is centered on the ego vehicle and can be panned/zoomed via mouse interaction in the browser.

## Debug Overlays

Optional overlays (`show_triggers`, `show_connections`, `show_spawn_points`) help debug scenario setup and traffic flow.
