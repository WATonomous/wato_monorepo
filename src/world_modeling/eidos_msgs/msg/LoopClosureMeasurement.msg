# One relative-pose measurement between two keyframes, as produced by a loop closure plugin.
# Published for every measurement, whether or not it became a factor, so that plugins can be
# scored against each other and against the optimized graph.

std_msgs/Header header              # stamp: the newer keyframe; frame_id: the base frame

string source                       # plugin instance that measured it
string candidate_source             # how the pair was chosen: distance | place_recognition | consecutive
uint64 from_index                   # state index of the keyframe the pose is expressed in
uint64 to_index                     # state index of the other keyframe
float64 from_stamp                  # s
float64 to_stamp                    # s

geometry_msgs/Pose measured         # T_from^-1 * T_to as measured
geometry_msgs/Pose graph            # the same from the graph's estimates at the time of measuring

bool accepted                       # passed the plugin's checks
bool added                          # became a factor (false in shadow mode)
string reject_reason                # empty when accepted

# Quality. Fields a plugin does not have stay at zero.
float64 trans_spread                # m, camera-to-camera disagreement (visual)
float64 rot_spread                  # rad
float32 confidence                  # model confidence (visual)
uint32 cameras                      # cameras averaged over (visual)
float64 inlier_ratio                # registration inliers / source points (lidar)
float64 similarity                  # place-recognition similarity of the pair, when computed
float64 duration                    # s spent measuring
