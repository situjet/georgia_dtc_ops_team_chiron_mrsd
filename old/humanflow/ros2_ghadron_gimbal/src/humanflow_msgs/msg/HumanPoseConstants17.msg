# msg/HumanPoseConstants17.msg
# Constants for joint indices
uint8 NOSE = 0
uint8 LEFT_EYE = 1
uint8 RIGHT_EYE = 2
uint8 LEFT_EAR = 3
uint8 RIGHT_EAR = 4
uint8 LEFT_SHOULDER = 5
uint8 RIGHT_SHOULDER = 6
uint8 LEFT_ELBOW = 7
uint8 RIGHT_ELBOW = 8
uint8 LEFT_WRIST = 9
uint8 RIGHT_WRIST = 10
uint8 LEFT_HIP = 11
uint8 RIGHT_HIP = 12
uint8 LEFT_KNEE = 13
uint8 RIGHT_KNEE = 14
uint8 LEFT_ANKLE = 15
uint8 RIGHT_ANKLE = 16


# Joint connections reference (as comments for documentation)
# Torso:     (5,6) -> (5,11) -> (6,12) -> (11,12)
# Left arm:  (5,7) -> (7,9)
# Right arm: (6,8) -> (8,10)
# Left leg:  (11,13) -> (13,15)
# Right leg: (12,14) -> (14,16)
# Face:      (0,1) -> (0,2) -> (1,3) -> (2,4)
