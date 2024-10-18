# Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

import os
from dataclasses import dataclass
from operator import add, mul
from threading import Event
from typing import List

import numpy as np
import onnxruntime as ort
import torch
import io
import orbit.observations as ob
try:
    from bosdyn.api import robot_command_pb2
    from bosdyn.api.robot_command_pb2 import JointControlStreamRequest
    from bosdyn.api.robot_state_pb2 import RobotStateStreamResponse
    from bosdyn.util import seconds_to_timestamp, set_timestamp_from_now, timestamp_to_sec
except ModuleNotFoundError:
    print("BD api is not available")
from orbit.orbit_configuration import OrbitConfig
from orbit.orbit_constants import ordered_joint_names_orbit
from spot.constants import DEFAULT_K_Q_P, DEFAULT_K_QD_P, ordered_joint_names_bosdyn, ordered_joint_names_isaac
from utils.dict_tools import dict_to_list, find_ordering, reorder
from utils.robot import RobotState, isaac_robot_joint_names
from sensor_msgs.msg import JointState

class PtCommandGenerator:

    def __init__(self, config: OrbitConfig, policy_file_name: os.PathLike, verbose: bool):
        self._config = config
        
        self._inference_session = self.loadPtModel(policy_file_name)
        self._last_action = [0] * 12
        self._init_pos = None
        self._init_load = None
        self.verbose = verbose

    def processPolicyAction(self, vel_cmd: List[float], cur_state: RobotState)->JointState:
        """Input vel_cmd = [x ,y ,z] cur_state = RobotState()"""
        if self._init_pos is None:
            self._init_pos = cur_state.joint_states.position
            self._init_load = cur_state.joint_states.effort

        # extract observation data from latest spot state data
        input_list = self.collect_inputs(cur_state, self._config, vel_cmd)
        # print("observations", input_list)

        # execute model from onnx file
        input = [np.array(input_list).astype("float32")]
        with torch.no_grad():
                obs = torch.from_numpy(input).view(1, -1).float()
                output = self._inference_session(obs).detach().view(-1).numpy()

        scaled_output = list(map(mul, [self._config.action_scale] * 12, output))
        # test_scaled = list(map(mul, [test_scale] * 12, scaled_output))
    
        default_joints = dict_to_list(self._config.default_joints, ordered_joint_names_orbit)
        shifted_output = list(map(add, scaled_output, default_joints))

        # generate proto message from target joint positions
        cmd = self.generateROSCmd(shifted_output)

        # cache data for history and logging
        self._last_action = output

        return cmd

    def collect_inputs(self, state: RobotState, config: OrbitConfig, vel_cmd: List[float]) -> RobotState:
        """extract observation data from spots current state and format for onnx

        arguments
        state -- proto msg with spots latest state
        config -- model configuration data from orbit

        return list of float values ready to be passed into the model
        """
        observations = []
        observations += ob.get_base_linear_velocity(state)
        observations += ob.get_base_angular_velocity(state)
        observations += ob.get_projected_gravity(state)
        observations += vel_cmd
        if self.verbose:
            print("[INFO] cmd", vel_cmd)
        observations += ob.get_joint_positions(state, config)
        observations += ob.get_joint_velocity(state)
        observations += self._last_action
        return observations

    def generateROSCmd(self, pos_command: List[float]) -> RobotState:
        """generate a cmd msg for ROS manager with a given pos_command

        arguments
        pos_command -- list of joint positions see spot.constants for order

        return robot cmd for ros manager
        """
        robot_cmd = JointState()
        robot_cmd.position = pos_command
        robot_cmd.name = isaac_robot_joint_names
        return robot_cmd

class OnnxCommandGenerator:

    def __init__(self, config: OrbitConfig, policy_file_name: os.PathLike, verbose: bool):
        
        self._config = config
        self._inference_session = ort.InferenceSession(policy_file_name)
        self._last_action = [0] * 12
        self._count = 0
        self._init_pos = None
        self._init_load = None
        self.verbose = verbose
    
    def processPolicyAction(self, vel_cmd: List[float], cur_state: RobotState)->JointState:
        """Input vel_cmd = [x ,y ,z] cur_state = RobotState()"""
        if self._init_pos is None:
            self._init_pos = cur_state.joint_states.position
            self._init_load = cur_state.joint_states.effort

        # extract observation data from latest spot state data
        input_list = self.collect_inputs(cur_state, self._config, vel_cmd)
        # print("observations", input_list)

        # execute model from onnx file
        input = [np.array(input_list).astype("float32")]
        output = self._inference_session.run(None, {"obs": input})[0].tolist()[0]

        # post process model output apply action scaling and return to spots
        # joint order and offset

        # test_scale = min(0.1 * self._count, 1)

        scaled_output = list(map(mul, [self._config.action_scale] * 12, output))
        # test_scaled = list(map(mul, [test_scale] * 12, scaled_output))
    
        default_joints = dict_to_list(self._config.default_joints, ordered_joint_names_orbit)
        shifted_output = list(map(add, scaled_output, default_joints))

        # orbit_to_spot = find_ordering(ordered_joint_names_orbit, ordered_joint_names_isaac)
        # reordered_output = reorder(shifted_output, orbit_to_spot)

        if self.verbose:
            print(f'action output: {shifted_output}')

        # generate proto message from target joint positions
        cmd = self.generateROSCmd(shifted_output)

        # cache data for history and logging
        self._last_action = output
        self._count += 1

        return cmd

    def collect_inputs(self, state: RobotState, config: OrbitConfig, vel_cmd: List[float]) -> RobotState:
        """extract observation data from spots current state and format for onnx

        arguments
        state -- proto msg with spots latest state
        config -- model configuration data from orbit

        return list of float values ready to be passed into the model
        """
        observations = []
        observations += ob.get_base_linear_velocity(state)
        observations += ob.get_base_angular_velocity(state)
        observations += ob.get_projected_gravity(state)
        observations += vel_cmd
        if self.verbose:
            print("[INFO] cmd", vel_cmd)
        observations += ob.get_joint_positions(state, config)
        observations += ob.get_joint_velocity(state)
        observations += self._last_action
        if self.verbose:
            print("base_linear_velocity:", observations[0:3])
            print("base_angular_velocity:", observations[3:6])
            print("projected_gravity:", observations[6:9])
            print("joint_positions", observations[12:24])
            print("joint_velocity", observations[24:36])
            print("last_action", observations[36:48])
        return observations

    def generateROSCmd(self, pos_command: List[float]) -> RobotState:
        """generate a cmd msg for ROS manager with a given pos_command

        arguments
        pos_command -- list of joint positions see spot.constants for order

        return robot cmd for ros manager
        """
        robot_cmd = JointState()
        robot_cmd.position = pos_command
        robot_cmd.name = isaac_robot_joint_names
        return robot_cmd





