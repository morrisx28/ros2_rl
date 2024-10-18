# Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

from operator import sub
try: 
    from bosdyn.api import robot_state_pb2
except ModuleNotFoundError:
    print("BD api is not available")
from orbit.orbit_configuration import OrbitConfig
from orbit.orbit_constants import ordered_joint_names_orbit
from spatialmath import UnitQuaternion
from spot.constants import ordered_joint_names_bosdyn, ordered_joint_names_isaac
from utils.dict_tools import dict_to_list, find_ordering, reorder
from utils.robot import RobotState

def get_base_linear_velocity(state: RobotState):
    """calculate linear velocity of spots base in the base frame from data
    available in spots state update.  note spot gives velocity in odom frame
    so we need to rotate it to current estimated pose of the base

    arguments
    state -- proto msg from spot containing data on the robots state
    """
    msg = state.kinematic_state.linear_vel

    odom_r_base_msg = state.kinematic_state.orientation
    scalar = odom_r_base_msg.w
    vector = [odom_r_base_msg.x, odom_r_base_msg.y, odom_r_base_msg.z]
    odom_r_base = UnitQuaternion(scalar, vector)

    velocity_odom = [msg.x, msg.y, msg.z]
    velocity_base = odom_r_base.inv() * velocity_odom

    return velocity_base.tolist()

def get_base_angular_velocity(state: RobotState):
    """calculate angular velocity of spots base in the base frame from data
    available in spots state update.  note spot gives velocity in odom frame
    so we need to rotate it to current estimated pose of the base

    arguments
    state -- proto msg from spot containing data on the robots state
    """
    msg = state.kinematic_state.angular_vel

    odom_r_base_msg = state.kinematic_state.orientation
    scalar = odom_r_base_msg.w
    vector = [odom_r_base_msg.x, odom_r_base_msg.y, odom_r_base_msg.z]
    odom_r_base = UnitQuaternion(scalar, vector)

    angular_velocity_odom = [msg.x, msg.y, msg.z]
    angular_velocity_base = odom_r_base.inv() * angular_velocity_odom

    return angular_velocity_base.tolist()

def get_projected_gravity(state: RobotState):
    """calculate direction of gravity in spots base frame
        the assumption here is that the odom frame Z axis is opposite gravity
        this is the case if spots body is parallel to the floor when turned on

    arguments
    state -- proto msg from spot containing data on the robots state
    """
    odom_r_base_msg = state.kinematic_state.orientation

    scalar = odom_r_base_msg.w
    vector = [odom_r_base_msg.x, odom_r_base_msg.y, odom_r_base_msg.z]
    odom_r_base = UnitQuaternion(scalar, vector)

    gravity_odom = [0, 0, -1]
    gravity_base = odom_r_base.inv() * gravity_odom
    return gravity_base.tolist()

def get_joint_positions(state: RobotState, config: OrbitConfig):
    """get joint position from spots state update a reformat for orbit by
    reordering to match orbits expectation and shifting so 0 position is the
    same as was used in training

    arguments
    state -- proto msg from spot containing data on the robots state
    config -- dataclass with values loaded from orbits training data
    """

    spot_to_orbit = find_ordering(ordered_joint_names_isaac, ordered_joint_names_orbit)
    pos = reorder(state.joint_states.position, spot_to_orbit)
    default_joints = dict_to_list(config.default_joints, ordered_joint_names_orbit)
    pos = list(map(sub, pos, default_joints))
    return pos

def get_joint_velocity(state: RobotState):
    """get joint velocity from spots state update a reformat for orbit by
    reordering to match orbits expectation

    arguments
    state -- proto msg from spot containing data on the robots state
    """
    spot_to_orbit = find_ordering(ordered_joint_names_isaac, ordered_joint_names_orbit)
    vel = reorder(state.joint_states.velocity, spot_to_orbit)
    return vel

