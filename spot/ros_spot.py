import time
from threading import Thread
from typing import Callable
from ros_handler import ROSManager
from utils.robot import isaac_robot_joint_names, RobotState
from sensor_msgs.msg import JointState
from typing import List

class Spot():
    """wrapper around ROS API"""

    def __init__(self) -> None:
        self.ros_manager = ROSManager()

        self.default_pose = [0.1, -0.1, 0.1, -0.1, 0.9, 0.9, 1.1, 1.1, -1.5, -1.5, -1.5, -1.5]

    def sendPoseCmd(self, position: List[float]):
        joint_cmd = JointState()
        joint_cmd.name = isaac_robot_joint_names
        joint_cmd.position = position
        self.ros_manager.sendRobotCmd(joint_cmd)
    
    def sendRobotCmd(self, cmd: JointState):
        self.ros_manager.sendRobotCmd(cmd)
    
    def stand(self):
        # pose = [0.0, 1.25, -1.98, 0.0, 1.25, -1.98, 0.0, 1.25, -1.98, 0.0, 1.25, -1.98]
        self.sendPoseCmd(self.default_pose)
        print("Spot stand")

    def getSpotState(self) -> RobotState:
        return self.ros_manager.getRobotState()
    





    

