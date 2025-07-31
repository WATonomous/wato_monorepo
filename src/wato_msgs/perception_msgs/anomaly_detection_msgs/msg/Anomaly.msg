std_msgs/Header  header

# Anomaly Types:
# 0: Camera Image
# 1: Lidar Point Cloud
string sensor_id          # which sensor
string anomaly_type

# Anomaly Sensor Name (topic where the anomalous data was published)
# string anomaly_topic

float32 anomaly_score
BoundingBox2D[] boxes     # all anomalous regions with labels & scores

# Anomalib also provides per-pixel anomaly scores.
# This can be implemented in the future as needed for more complex
# anomaly response.

# test