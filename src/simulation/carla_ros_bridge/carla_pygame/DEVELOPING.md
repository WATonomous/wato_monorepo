# Developing carla_pygame

## Architecture

The package is split into focused modules:

```
carla_pygame/
├── pygame_hud_node.py       # Lifecycle node, Flask server
├── world.py                 # Rendering logic, actor management
├── map_image.py             # Road map generation and caching
├── traffic_light_surfaces.py # Traffic light sprites
├── constants.py             # Colors, scaling constants
├── utils.py                 # Helper functions
└── web_template.py          # HTML/JS for web interface
```

## Headless Rendering

Pygame runs in headless mode via SDL dummy driver:

```python
os.environ['SDL_VIDEODRIVER'] = 'dummy'
os.environ['SDL_AUDIODRIVER'] = 'dummy'
```

## Web Streaming

1. Flask serves the HTML page
2. Flask-SocketIO provides WebSocket connection
3. Background thread renders frames and emits base64-encoded PNGs
4. Client JavaScript draws frames to canvas

## Map Caching

Map images are cached to disk based on OpenDRIVE content hash:

```python
filename = f"{map_name}_{opendrive_hash}.tga"
```

## Actor Rendering

The `World` class tracks actors and renders them each frame:

```python
def tick(self):
    actors = self.world.get_actors()
    self.actors_with_transforms = [(a, a.get_transform()) for a in actors]

def render(self):
    # Split actors by type
    # Render each type with appropriate styling
```

## Color Scheme

Actors are color-coded via constants:
- Ego vehicle: Green (COLOR_CHAMELEON_0)
- Vehicles: Blue (COLOR_SKY_BLUE_0)
- Motorcycles: Brown (COLOR_CHOCOLATE_1)
- Pedestrians: Magenta (COLOR_MAGENTA)

## Pan/Zoom

Mouse events are sent via WebSocket and applied to view transform:

```python
def handle_pan(self, dx, dy):
    self.mouse_offset[0] += dx
    self.mouse_offset[1] += dy

def handle_zoom(self, delta, x, y):
    # Zoom centered on cursor position
```
