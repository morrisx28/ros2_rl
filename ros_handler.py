import rclpy
import threading
import traceback
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from orbit.orbit_configuration import OrbitConfig
from orbit.orbit_constants import ordered_joint_names_orbit
from spatialmath import UnitQuaternion
from spot.constants import ordered_joint_names_isaac
from utils.dict_tools import dict_to_list, find_ordering, reorder
from utils.robot import KinematicState, isaac_robot_joint_names, RobotState
from typing import List
import numpy as np


class ROSManager(Node):

    def __init__(self):
        rclpy.init()
        super().__init__('ros_handler')
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odomCallback, 1)
        self.joint_state_sub = self.create_subscription(JointState, 'isaac_joint_states', self.jointStateCallback, 1)
        self.joint_cmd_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.joint_state = JointState()
        self.joint_cmd = JointState()
        self.odom_state = KinematicState()

        self.offset_pose = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        self.startManager()

    def odomCallback(self, msg: Odometry):
        self.odom_state.linear_vel = msg.twist.twist.linear
        self.odom_state.angular_vel = msg.twist.twist.angular
        self.odom_state.orientation.x = msg.pose.pose.orientation.x
        self.odom_state.orientation.y = msg.pose.pose.orientation.y
        self.odom_state.orientation.z = msg.pose.pose.orientation.z
        self.odom_state.orientation.w = msg.pose.pose.orientation.w

    def jointStateCallback(self, msg: JointState):
        self.joint_state.name = msg.name
        pos = np.array(msg.position) - self.offset_pose
        self.joint_state.position = pos.tolist()
        self.joint_state.velocity = msg.velocity
        self.joint_state.effort = msg.effort
    
    def setOffestPose(self, pos: List[float]):
        self.offset_pose = np.array(pos)

    def sendRobotCmd(self, cmd: JointState):
        self.joint_cmd_pub.publish(cmd)
    
    def getJointState(self):
        return self.joint_state
    
    def getOdomState(self):
        return self.odom_state

    def getRobotState(self):
        robot_state = RobotState()
        robot_state.joint_states = self.getJointState()
        robot_state.kinematic_state = self.getOdomState()
        return robot_state
    
    def startManager(self):
        self.ros_handler_thread = threading.Thread(target=rclpy.spin, args=(self, ), daemon=True)
        self.ros_handler_thread.start()

if __name__ == '__main__':
    ros_manager = ROSManager()
    command_dict = {
        'st': ros_manager.getJointState,
    }

    while True:
        try:
            cmd = input("CMD :")
            if cmd in command_dict:
                command_dict[cmd]()
            elif cmd == "exit":
                break

        except Exception as e:
            traceback.print_exc()
            break
    



