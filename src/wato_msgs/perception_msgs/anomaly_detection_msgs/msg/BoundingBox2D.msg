# A simple 2D bounding box with a semantic label and a score for how anomalous this region is

int32   x_min          # top-left corner x (pixels)
int32   y_min          # top-left corner y (pixels)
uint32  width          # box width (pixels)
uint32  height         # box height (pixels)
string  label          # "person", "road_obstacle"
float32 score_region   # anomaly score for this box
