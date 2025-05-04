# msg/HumanPoseConstants24.msg
# Constants for joint indices
uint8 PELVIS = 0
uint8 SPINE1 = 3
uint8 SPINE2 = 6
uint8 SPINE3 = 9
uint8 LEFT_HIP = 1
uint8 LEFT_KNEE = 4
uint8 LEFT_ANKLE = 7
uint8 LEFT_FOOT = 10
uint8 RIGHT_HIP = 2
uint8 RIGHT_KNEE = 5
uint8 RIGHT_ANKLE = 8
uint8 RIGHT_FOOT = 11
uint8 NECK = 12
uint8 HEAD = 15
uint8 LEFT_COLLAR = 13
uint8 LEFT_SHOULDER = 16
uint8 LEFT_ELBOW = 18
uint8 LEFT_WRIST = 20
uint8 LEFT_HAND = 22
uint8 RIGHT_COLLAR = 14
uint8 RIGHT_SHOULDER = 17
uint8 RIGHT_ELBOW = 19
uint8 RIGHT_WRIST = 21
uint8 RIGHT_HAND = 23


# Joint connections reference (as comments for documentation)
# Left leg: (0,1) -> (1,4) -> (4,7) -> (7,10)
# Right leg:  (0,2) -> (2,5) -> (5,8) -> (8,11)
# Spine:     (0,3) -> (3,6) -> (6,9) -> (9,12)
# Left arm:  (9,13) -> (13,16) -> (16,18) -> (18,20) -> (20,22)
# Right arm: (9,14) -> (14,17) -> (17,19) -> (19,21) -> (21,23)
# Head:      (12,15)
