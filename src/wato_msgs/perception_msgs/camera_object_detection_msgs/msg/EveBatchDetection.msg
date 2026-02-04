# EveBatchDetection message for Eve vehicle batched camera detections
# Contains detections from multiple Eve cameras in a single message

std_msgs/Header header
vision_msgs/Detection2DArray[] detections
