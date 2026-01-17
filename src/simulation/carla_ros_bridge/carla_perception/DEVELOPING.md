# Developing carla_perception

## Architecture

Sensor nodes spawn CARLA sensors on the ego vehicle and convert sensor data to ROS messages via callbacks.

## Sensor Lifecycle

```python
def on_configure(self, state):
    # Connect to CARLA, find ego vehicle
    # Parse sensor configs from parameters
    # Create ROS publishers

def on_activate(self, state):
    # Spawn sensors in CARLA
    # Register sensor callbacks

def on_deactivate(self, state):
    # Destroy sensors

def on_cleanup(self, state):
    # Destroy publishers
```

## Multi-Sensor Configuration

Sensors are configured via lists in parameters:
```yaml
camera_names:
  - front_camera
  - rear_camera
front_camera.frame_id: "front_camera_optical"
front_camera.image_width: 800
```

The node iterates over `*_names` and reads prefixed parameters for each.

## CARLA Sensor Callbacks

Sensors use async callbacks:
```python
sensor.listen(lambda data: self._sensor_callback(data, sensor_name))
```

Callbacks run in CARLA's thread - keep processing minimal and use queues if needed.

## Coordinate Systems

- **Camera**: CARLA uses X-forward, Y-right, Z-up. Apply optical frame rotation if `optical_frame: true`.
- **LiDAR**: Points are in sensor frame. Convert to ROS coordinates (flip Y axis).

## Adding a New Sensor Type

1. Create new node class extending `LifecycleNode`
2. Define sensor-specific parameters
3. Spawn sensor with appropriate blueprint
4. Convert CARLA data format to ROS message
5. Add entry point in `setup.py`
