# AreaOccupancy.msg
# Area-based dynamic object detection results.
#
# Contains occupancy status for all configured detection areas.

std_msgs/Header header           # frame_id is the reference frame (e.g., "base_link")

AreaOccupancyInfo[] areas
