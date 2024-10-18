import argparse
import sys
from pathlib import Path
import traceback
import os
import threading
import time

# import bosdyn.client.util
import orbit.orbit_configuration
from hid.gamepad import (
    Gamepad,
    GamepadConfig,
    joystick_connected,
    load_gamepad_configuration,
)
from orbit.onnx_command_generator import (
    OnnxCommandGenerator,
    PtCommandGenerator
)
from utils.event_divider import EventDivider
from spot.ros_spot import Spot

class RLController:

    def __init__(self, path: os.PathLike, dt: float, verbose: bool) -> None:

        conf_file =  orbit.orbit_configuration.detect_config_file(path)
        policy_file = orbit.orbit_configuration.detect_policy_file(path)
        config = orbit.orbit_configuration.load_configuration(conf_file)
        print(f'model configuration : {config}')
        self.commad_generator = OnnxCommandGenerator(config, policy_file, verbose)
        self.spot = Spot()

        self.policy_count = 0
        self.decimation = 10
        self.dt = dt
        self.control_thread = None
        self.controller_is_running = False
    
    def startController(self):
        if self.control_thread is None:
            self.spot.stand()
            time.sleep(0.5)
            self.controller_is_running = True
            self.control_thread = threading.Thread(target=self.controlLoop)
            self.control_thread.start()
    
    def controlLoop(self):
        vel_cmd = [0.0, 0.0, 0.0]
        while self.controller_is_running:
            if self.policy_count % self.decimation == 0:
                state = self.spot.getSpotState()
                cmd = self.commad_generator.processPolicyAction(vel_cmd, state)
            self.spot.sendRobotCmd(cmd)
            self.policy_count += 1
            time.sleep(self.dt)
    
    def stopController(self):
        if self.control_thread is not None:
            self.controller_is_running = False
            self.control_thread.join()
        

def main():
    # """Command line interface. change that is ok"""
    parser = argparse.ArgumentParser()
    parser.add_argument("policy_file_path", type=Path)
    options = parser.parse_args()

    rl_controller = RLController(options.policy_file_path,dt=1/500 ,verbose=True)

    
    command_dict = {
        'st': rl_controller.startController,
    }
    while True:
        try:
            cmd = input("CMD :")
            if cmd in command_dict:
                command_dict[cmd]()
            elif cmd == "exit":
                rl_controller.stopController()
                break

        except Exception as e:
            traceback.print_exc()
            break



if __name__ == "__main__":
    if not main():
        sys.exit(1)