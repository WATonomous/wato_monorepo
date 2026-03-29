# WorldObjectArray.msg
# A list of tracked 3D objects with predictions, for multi-object scenes.
#
# Published by the tracking/prediction pipeline, consumed by World Model
# as the single unified input for all detected entities.

std_msgs/Header header

# All tracked objects in the current frame
WorldObject[] objects
