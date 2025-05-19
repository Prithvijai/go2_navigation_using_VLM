#!/usr/bin/env python3
import numpy as np, math, rclpy
from .GaitController import GaitController
from .PIDController import PID_controller
from RoboticsUtilities.Transformations import rotxyz, rotz

class TrotSwingController(object):
    def __init__(self, stance_ticks, swing_ticks, time_step, phase_length, z_leg_lift, default_stance):
        self.stance_ticks = stance_ticks
        self.swing_ticks  = swing_ticks
        self.time_step    = time_step
        self.phase_length = phase_length
        self.z_leg_lift   = z_leg_lift
        self.default_stance = default_stance

    def raibert_touchdown_location(self, leg_index, command):
        dp2d     = command.velocity * self.phase_length * self.time_step
        delta    = np.array([dp2d[0], dp2d[1], 0])
        theta    = self.stance_ticks * self.time_step * command.yaw_rate
        rotated  = rotz(theta).dot(self.default_stance[:,leg_index])
        return rotated + delta

    def swing_height(self, phase):
        return self.z_leg_lift * math.sqrt(1 - (phase - 0.5)**2 / 0.25)

    def next_foot_location(self, swing_prop, leg_index, state, command):
        foot = state.foot_locations[:, leg_index]
        height = self.swing_height(swing_prop)
        touchdown = self.raibert_touchdown_location(leg_index, command)
        time_left = self.time_step * self.swing_ticks * (1.0 - swing_prop)
        vel       = (touchdown - foot)/time_left * np.array([1,1,0])
        delta     = vel * self.time_step
        z_vec     = np.array([0,0, height + command.robot_height])
        return foot * np.array([1,1,0]) + delta + z_vec

class TrotStanceController(object):
    def __init__(self, phase_length, stance_ticks, swing_ticks, time_step, z_error_constant):
        self.phase_length     = phase_length
        self.stance_ticks     = stance_ticks
        self.swing_ticks      = swing_ticks
        self.time_step        = time_step
        self.z_error_constant = z_error_constant

    def position_delta(self, leg_index, state, command):
        z = state.foot_locations[2, leg_index]
        sx = command.velocity[0] * (self.phase_length/self.swing_ticks)
        sy = command.velocity[1] * (self.phase_length/self.swing_ticks)
        vel = np.array([
            -(sx/4)/(self.time_step*self.stance_ticks),
            -(sy/4)/(self.time_step*self.stance_ticks),
            (state.robot_height - z)/self.z_error_constant
        ])
        delta = vel * self.time_step
        ori   = rotz(-command.yaw_rate * self.time_step)
        return delta, ori

    def next_foot_location(self, leg_index, state, command):
        foot = state.foot_locations[:, leg_index]
        delta, ori = self.position_delta(leg_index, state, command)
        return ori.dot(foot) + delta

class TrotGaitController(GaitController):
    def __init__(self, default_stance, stance_time, swing_time, time_step, use_imu):
        contact_phases = np.array([
            [1,1,1,0],
            [1,0,1,1],
            [1,0,1,1],
            [1,1,1,0]
        ])
        super().__init__(stance_time, swing_time, time_step, contact_phases, default_stance)

        self.use_imu    = use_imu
        self.use_button = True
        self.autoRest   = True
        self.trotNeeded = True

        z_error_constant = 0.02
        z_leg_lift       = -0.12

        self.max_x_velocity = 0.035
        self.max_y_velocity = 0.02
        self.max_yaw_rate   = 1.3

        self.swingController  = TrotSwingController(
            self.stance_ticks, self.swing_ticks, self.time_step,
            self.phase_length, z_leg_lift, self.default_stance
        )
        self.stanceController = TrotStanceController(
            self.phase_length, self.stance_ticks, self.swing_ticks,
            self.time_step, z_error_constant
        )
        self.pid_controller = PID_controller(0.2, 0.025, 0.0025)

    def updateStateCommand(self, msg, state, command):
        while len(msg.axes) < 8:
            msg.axes.append(0.0)
        msg.axes[7] = command.forward_vel
        msg.axes[6] = command.yaw_rate

        if msg.axes[4] > 0:
            command.velocity[0] = -msg.axes[4]*self.max_x_velocity
        else:
            command.velocity[0] = -msg.axes[4]*self.max_x_velocity*3
        command.velocity[1] = -msg.axes[3]*self.max_y_velocity
        command.yaw_rate    = -msg.axes[0]*self.max_yaw_rate

        if self.use_button:
            if msg.buttons[7]:
                self.use_imu    = not self.use_imu
                self.use_button = False
                rclpy.logging._root_logger.info(f"TrotController – IMU: {self.use_imu}")
            elif msg.buttons[6]:
                self.autoRest   = not self.autoRest
                if not self.autoRest:
                    self.trotNeeded = True
                self.use_button = False
                rclpy.logging._root_logger.info(f"TrotController – AutoRest: {self.autoRest}")
        else:
            if not (msg.buttons[6] or msg.buttons[7]):
                self.use_button = True

    def step(self, state, command):
        if self.autoRest:
            vel = command.velocity
            if abs(vel[0])<=0.005 and abs(vel[1])<=0.005 and abs(command.yaw_rate)<=0.002:
                if state.ticks % (2*self.phase_length)==0:
                    self.trotNeeded = False
            else:
                self.trotNeeded = True

        if self.trotNeeded:
            modes = self.contacts(state.ticks)
            new_locs = np.zeros((3,4))
            for i in range(4):
                if modes[i]==1:
                    loc = self.stanceController.next_foot_location(i, state, command)
                else:
                    prop = self.subphase_ticks(state.ticks)/float(self.swing_ticks)
                    loc  = self.swingController.next_foot_location(prop, i, state, command)
                new_locs[:,i] = loc

            if self.use_imu:
                comp = self.pid_controller.run(state.imu_roll, state.imu_pitch)
                new_locs = rotxyz(comp[0], comp[1], 0).dot(new_locs)

            state.ticks += 1
            return new_locs
        else:
            temp = self.default_stance.copy()
            temp[2,:] = [command.robot_height]*4
            return temp

    def run(self, state, command):
        state.foot_locations = self.step(state, command)
        state.robot_height   = command.robot_height
        return state.foot_locations
