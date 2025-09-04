# msg/HumanPose24.msg
std_msgs/Header header
int32 bounding_box_id  # ID from the bounding box detection, dummy value -1 if not given
geometry_msgs/Point[24] keypoints
geometry_msgs/Point[2] chest_orientation # the start and the end point of the chest orientation
geometry_msgs/Point[2] head_orientation # the start and the end point of the head orientation