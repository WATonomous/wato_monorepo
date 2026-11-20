# Developing camera_trigger_msgs

## Build

```bash
colcon build --packages-select camera_trigger_msgs
```

## Changing CameraTrigger.msg

`CameraTrigger.msg` is a contract with every consumer of `/camera_trigger` (the camera driver's
frame stamping, bags, Foxglove). Changing or removing a field breaks them and makes old bags
unreadable with the new definition, so prefer adding fields.

1. Edit `msg/CameraTrigger.msg`.
2. Update the field table in `README.md`.
3. Rebuild and update `camera_trigger` (and any other user of the message).

Containers other than perception that need to decode `/camera_trigger` (Foxglove in
infrastructure, the topic healthchecker in interfacing) must copy this package into their image.
