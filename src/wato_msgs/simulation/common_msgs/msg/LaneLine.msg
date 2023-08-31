# Lane line may be represented by a polyline or polynomial coefficients.

std_msgs/Header header

# List of 2D points representing image coordinates of detected lane lines
# Confidence values are included for each point
float32[] confidence
geometry_msgs/Point[] points

# Should probably be defined wrt vehicle POV coordinate system?
float32[] coeffs

# Type of roadline found.
string TYPE_CROSSABLE=CROSSABLE # any road line which can be driven through
string TYPE_CENTERLINE=CENTERLINE  #  road line dividing the middle of the road. This assumes that the centerline is uncrossable.
string TYPE_UNCROSSABLE=UNCROSSABLE  # any road line which cannot be driven through
string TYPE_UNKNOWN=UNKNOWN   # if any uncertainity of the type
string type

# Numbering of Lanes, with 0 being the centerline and positive values being to the right of it
int32 lanenumber

# Turn directions for lanes (used by HD Map Processing)
string NOT_INTERSECTION=NONE # lane is not within intersection, we assume that it is straight
string INTERSECTION_LEFT=LEFT # lane is within intersection, corresponds to left turn
string INTERSECTION_RIGHT=RIGHT # lane is within intersection, corresponds to right turn
string INTERSECTION_STRAIGHT=STRAIGHT # lane is within intersection, no turn
string intersection
