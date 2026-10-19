# Combined stereo frame: left colour image + depth map from the same source frame.
# Eliminates the need for TimeSynchronizer on the subscriber side.

std_msgs/Header header
sensor_msgs/Image left_image
sensor_msgs/Image depth_image
