std_msgs/Header  header

# Anomaly Types:
# 0: Camera Image
# 1: Lidar Point Cloud
uint8 anomaly_type

# Anomaly Sensor Name (topic where the anomalous data was published)
string anomaly_topic

float32 anomaly_score

# Anomalib also provides per-pixel anomaly scores.
# This can be implemented in the future as needed for more complex
# anomaly response.

# test