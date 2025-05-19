#!/usr/bin/env python3
import rclpy
from RoboticsUtilities.Transformations import rotxyz
from .PIDController import PID_controller

class StandController(object):
    def __init__(self, default_stance):
        self.def_stance = default_stance
        self.pid_controller = PID_controller(0.75, 2.29, 0.0)
        self.use_imu    = False
        self.use_button = True
        self.pid_controller.reset()

    def updateStateCommand(self, msg, state, command):
        # ensure axes[0..7]
        while len(msg.axes) < 8:
            msg.axes.append(0.0)

        # inject high-level motion
        msg.axes[7] = command.forward_vel
        msg.axes[6] = command.yaw_rate

        # existing stand mapping
        state.body_local_position[0] = msg.axes[7] * 0.08
        state.body_local_position[1] = msg.axes[6] * 0.08
        state.body_local_position[2] = msg.axes[1] * 0.08
        state.body_local_orientation[0] = msg.axes[0] * 0.8
        state.body_local_orientation[1] = msg.axes[4] * 0.6
        state.body_local_orientation[2] = msg.axes[3] * 0.6

        if self.use_button and msg.buttons[7]:
            self.use_imu = not self.use_imu
            self.use_button = False
            rclpy.logging._root_logger.info(f"StandController – IMU: {self.use_imu}")
        if not self.use_button and not msg.buttons[7]:
            self.use_button = True

    @property
    def default_stance(self):
        return self.def_stance

    def step(self, state, command):
        temp = self.default_stance.copy()
        temp[2,:] = [command.robot_height]*4
        if self.use_imu:
            comp = self.pid_controller.run(state.imu_roll, state.imu_pitch)
            rot  = rotxyz(comp[0], comp[1], 0)
            temp = rot.dot(temp)
        return temp

    def run(self, state, command):
        state.foot_locations = self.step(state, command)
        return state.foot_locations
