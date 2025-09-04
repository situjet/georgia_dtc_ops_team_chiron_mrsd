# msg/HumanPose17.msg
std_msgs/Header header
int32 bounding_box_id  # ID from the bounding box detection, dummy value -1 if not given
geometry_msgs/Point[17] keypoints
