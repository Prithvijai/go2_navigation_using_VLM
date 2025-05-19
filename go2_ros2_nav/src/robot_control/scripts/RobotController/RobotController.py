#!/usr/bin/env python3
import numpy as np
from tf_transformations import euler_from_quaternion

from .StateCommand        import State, Command, BehaviorState
from .TrotGaitController  import TrotGaitController
from .CrawlGaitController import CrawlGaitController
from .StandController     import StandController

class Robot:
    def __init__(self, body, legs, use_imu):
        # geometry
        self.body, self.legs = body, legs
        self.delta_x  = body[0] * 0.5
        self.delta_y  = body[1] * 0.5 + legs[1]
        self.default_height  = 0.72
        self.x_shift_front   = -0.1
        self.x_shift_back    = -0.02

        # stance matrix
        stance = self.default_stance

        # gait controllers
        self.trotGaitController  = TrotGaitController(stance, 0.1, 0.24, 0.02, use_imu)
        self.crawlGaitController = CrawlGaitController(stance, 0.55, 0.45, 0.02)
        self.standController     = StandController(stance)

        # state & command
        self.currentController = self.standController
        self.state   = State(self.default_height)
        self.state.foot_locations = stance.copy()
        self.command = Command(self.default_height)

        # high-level goals
        self.desired_forward = 0.0
        self.start_x         = None
        self.desired_strafe  = 0.0
        self.start_y         = None

    @property
    def default_stance(self):
        return np.array([
            [ self.delta_x + self.x_shift_front,
              self.delta_x + self.x_shift_front,
             -self.delta_x + self.x_shift_back,
             -self.delta_x + self.x_shift_back],
            [-self.delta_y,
              self.delta_y,
             -self.delta_y,
              self.delta_y],
            [0, 0, 0, 0]
        ])

    # — High-level API — #
    def set_move_goal(self, distance_m: float):
        self.start_x         = self.state.body_local_position[0]
        self.desired_forward = distance_m
        self.command.trot_event = True

    def set_strafe_goal(self, distance_m: float):
        self.start_y         = self.state.body_local_position[1]
        self.desired_strafe  = distance_m
        self.command.trot_event = True

    # — Joystick & IMU — #
    def joystick_command(self, msg):
        # buttons: 0=stand,1=trot,2=crawl
        if msg.buttons[1]:
            self.command.trot_event = True
        elif msg.buttons[2]:
            self.command.crawl_event = True
        elif msg.buttons[0]:
            self.command.stand_event = True

        # height adjust
        if msg.buttons[6]:
            h = 0.52
            self.state.robot_height   = -h
            self.command.robot_height = -h

        self.currentController.updateStateCommand(msg, self.state, self.command)

    def imu_orientation(self, msg):
        q = msg.orientation
        r, p, _ = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.state.imu_roll  = r
        self.state.imu_pitch = p

    # — Main foothold generator — #
    def run(self):
        # **Linear forward/backward motion** (max 0.1 m/s)
        if abs(self.desired_forward) > 1e-3:
            cx = self.state.body_local_position[0]
            traveled = cx - self.start_x
            if abs(traveled) >= abs(self.desired_forward):
                self.desired_forward     = 0.0
                self.command.stand_event = True
                self.command.forward_vel = 0.0
            else:
                self.command.forward_vel = 0.1 * np.sign(self.desired_forward)
        else:
            self.command.forward_vel = 0.0

        # **Lateral strafe motion** (max 0.1 m/s)
        if abs(self.desired_strafe) > 1e-3:
            cy = self.state.body_local_position[1]
            shifted = cy - self.start_y
            if abs(shifted) >= abs(self.desired_strafe):
                self.desired_strafe      = 0.0
                self.command.stand_event = True
                self.command.strafe_vel  = 0.0
            else:
                self.command.strafe_vel  = 0.1 * np.sign(self.desired_strafe)
        else:
            self.command.strafe_vel = 0.0

        # delegate to the active gait controller
        return self.currentController.run(self.state, self.command)

    # — Gait switching — #
    def change_controller(self):
        if self.command.trot_event and self.state.behavior_state == BehaviorState.STAND:
            self.state.behavior_state = BehaviorState.TROT
            self.currentController    = self.trotGaitController
            self.currentController.pid_controller.reset()
            self.state.ticks          = 0
            self.command.trot_event   = False

        elif self.command.crawl_event and self.state.behavior_state == BehaviorState.STAND:
            self.state.behavior_state = BehaviorState.CRAWL
            self.currentController    = self.crawlGaitController
            self.currentController.first_cycle = True
            self.state.ticks          = 0
            self.command.crawl_event  = False

        elif self.command.stand_event:
            self.state.behavior_state = BehaviorState.STAND
            self.currentController    = self.standController
            self.currentController.pid_controller.reset()
            self.command.stand_event  = False
