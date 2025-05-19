#!/usr/bin/env python3
import numpy as np
from .GaitController import GaitController
from RoboticsUtilities.Transformations import rotz

class CrawlGaitController(GaitController):
    def __init__(self, default_stance, stance_time, swing_time, time_step):
        contact_phases = np.array([
            [1,1,1,0,1,1,1,1],
            [1,1,1,1,1,1,1,0],
            [1,0,1,1,1,1,1,1],
            [1,1,1,1,1,0,1,1]
        ])
        super().__init__(stance_time, swing_time, time_step, contact_phases, default_stance)

        self.max_x_velocity = 0.011
        self.max_yaw_rate   = 0.15
        self.body_shift_y   = 0.04

        self.swingController = CrawlSwingController(
            self.stance_ticks, self.swing_ticks, self.time_step,
            self.phase_length, -0.1, self.default_stance, self.body_shift_y
        )
        self.stanceController = CrawlStanceController(
            self.phase_length, self.stance_ticks, self.swing_ticks,
            self.time_step, 0.02, self.body_shift_y
        )
        self.first_cycle = True

    def updateStateCommand(self, msg, state, command):
        while len(msg.axes) < 8:
            msg.axes.append(0.0)
        msg.axes[7] = command.forward_vel
        msg.axes[6] = command.yaw_rate

        command.velocity[0] = -msg.axes[4] * self.max_x_velocity
        command.yaw_rate     = -msg.axes[0] * self.max_yaw_rate

    def step(self, state, command):
        modes = self.contacts(state.ticks)
        new_locs = np.zeros((3,4))
        phase = self.phase_index(state.ticks)

        for i in range(4):
            if modes[i]==1:
                move_side = phase in (0,4)
                move_left = (phase==0)
                loc = self.stanceController.next_foot_location(
                    i, state, command, self.first_cycle, move_side, move_left
                )
            else:
                prop = self.subphase_ticks(state.ticks)/float(self.swing_ticks)
                left = phase in (1,3)
                loc = self.swingController.next_foot_location(
                    prop, i, state, command, left
                )
            new_locs[:,i] = loc

        state.ticks += 1
        if self.phase_index(state.ticks)>0:
            self.first_cycle = False
        return new_locs

    def run(self, state, command):
        state.foot_locations = self.step(state, command)
        state.robot_height   = command.robot_height
        return state.foot_locations

class CrawlSwingController(object):
    def __init__(self, stance_ticks, swing_ticks, time_step,
                 phase_length, z_lift, default_stance, body_shift_y):
        self.stance_ticks   = stance_ticks
        self.swing_ticks    = swing_ticks
        self.time_step      = time_step
        self.phase_length   = phase_length
        self.z_lift         = z_lift
        self.default_stance = default_stance
        self.body_shift_y   = body_shift_y

    def raibert_touchdown_location(self, idx, command, shift_left):
        dp = command.velocity * self.phase_length * self.time_step
        delta = np.array([dp[0], dp[1], 0])
        theta = self.stance_ticks*self.time_step*command.yaw_rate
        base = rotz(theta).dot(self.default_stance[:,idx])
        shift_vec = np.array([0, self.body_shift_y if shift_left else -self.body_shift_y, 0])
        return base + delta + shift_vec

    def swing_height(self, p):
        return p/0.5*self.z_lift if p<0.5 else self.z_lift*(1-(p-0.5)/0.5)

    def next_foot_location(self, p, idx, state, command, shift_left):
        foot = state.foot_locations[:,idx]
        h = 0 if abs(command.velocity[0])<=0.002 and abs(command.yaw_rate)<=0.002 else self.swing_height(p)
        td = self.raibert_touchdown_location(idx, command, shift_left)
        tleft = self.time_step*self.swing_ticks*(1.0-p)
        vel = (td - foot)/tleft * np.array([1,1,0])
        delta = vel*self.time_step
        zvec = np.array([0,0,h+command.robot_height])
        return foot * np.array([1,1,0]) + delta + zvec

class CrawlStanceController(object):
    def __init__(self, phase_length, stance_ticks, swing_ticks,
                 time_step, z_err, body_shift_y):
        self.phase_length     = phase_length
        self.stance_ticks     = stance_ticks
        self.swing_ticks      = swing_ticks
        self.time_step        = time_step
        self.z_err_constant   = z_err
        self.body_shift_y     = body_shift_y

    def position_delta(self, idx, state, cmd, first, side, left):
        z = state.foot_locations[2,idx]
        stepx = cmd.velocity[0]*(self.phase_length/self.swing_ticks)
        sv = 0.0
        if side and (abs(cmd.velocity[0])>=0.005 or abs(cmd.yaw_rate)>=0.002):
            factor = 1 if first else 2
            sv = (self.body_shift_y*factor)/(self.time_step*self.stance_ticks)*(1 if left else -1)
        vel = np.array([
            -stepx/3/(self.time_step*self.stance_ticks),
            sv,
            (state.robot_height - z)/self.z_err_constant
        ]) * self.time_step
        ori = rotz(-cmd.yaw_rate*self.time_step)
        return vel, ori

    def next_foot_location(self, idx, state, cmd, first, side, left):
        foot = state.foot_locations[:,idx]
        delta, ori = self.position_delta(idx, state, cmd, first, side, left)
        return ori.dot(foot) + delta
