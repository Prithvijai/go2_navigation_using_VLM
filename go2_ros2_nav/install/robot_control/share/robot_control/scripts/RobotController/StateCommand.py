#!/usr/bin/env python3
import numpy as np
from enum import Enum

class BehaviorState(Enum):
    REST  = 0
    TROT  = 1
    CRAWL = 2
    STAND = 3

class State:
    def __init__(self, default_height):
        self.velocity = np.zeros(2)
        self.yaw_rate = 0.0
        self.robot_height = -default_height
        self.foot_locations = np.zeros((3,4))
        self.body_local_position    = np.zeros(3)
        self.body_local_orientation = np.zeros(3)
        self.imu_roll  = 0.0
        self.imu_pitch = 0.0
        self.ticks         = 0
        self.behavior_state = BehaviorState.REST

class Command:
    def __init__(self, default_height):
        self.velocity   = np.zeros(2)
        self.yaw_rate   = 0.0
        self.robot_height = -default_height
        self.trot_event  = False
        self.crawl_event = False
        self.stand_event = False

        # — HIGH-LEVEL FIELDS — #
        self.forward_vel = 0.0  # m/s
        self.yaw_rate    = 0.0  # rad/s
