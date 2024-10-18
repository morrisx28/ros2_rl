import numpy as np
from dataclasses import dataclass
from typing import List
from sensor_msgs.msg import JointState

@dataclass
class Orientation:
    x = 0
    y = 0
    z = 0
    w = 0

@dataclass
class KinematicState:
    linear_vel = 0
    angular_vel = 0
    orientation = Orientation() 

@dataclass
class RobotState:
    kinematic_state = KinematicState()
    joint_states = JointState()

# isaac_robot_joint_names = [
#     "front_left_hip_x",
#     "front_left_hip_y",
#     "front_left_knee",
#     "front_right_hip_x",
#     "front_right_hip_y",
#     "front_right_knee",
#     "rear_left_hip_x",
#     "rear_left_hip_y",
#     "rear_left_knee",
#     "rear_right_hip_x",
#     "rear_right_hip_y",
#     "rear_right_knee",
# ]
isaac_robot_joint_names = [
    "fl_hx",
    "fr_hx",
    "hl_hx",
    "hr_hx",
    "fl_hy",
    "fr_hy",
    "hl_hy",
    "hr_hy",
    "fl_kn",
    "fr_kn",
    "hl_kn",
    "hr_kn",
]