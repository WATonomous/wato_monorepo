std_msgs/Header header
float32 Speed
string Direction

# A speed and a direction (one of 'NE' 'NW' 'SE' 'SW)
# It's done this way instead of one message with 4 wheels
# for freshness (the car publishes one can frame per wheel, it may
# or may not be synchronous with the other wheels)

# obviously i.e. NE / north-east is the front-right wheel
