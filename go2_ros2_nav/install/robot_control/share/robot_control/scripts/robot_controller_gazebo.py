#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, Imu
from std_msgs.msg import Float64MultiArray
import numpy as np

# Absolute imports from your ROS2 package “robot_control”
from robot_control.RobotController import Robot
from robot_control.InverseKinematics import robot_IK
from robot_control.StateCommand import BehaviorState

# geometry parameters — must match your Robot __init__ signature
BODY    = [0.559, 0.12]
LEGS    = [0.0, 0.1425, 0.426, 0.345]
USE_IMU = True

spot_robot = Robot(BODY, LEGS, USE_IMU)
ik         = robot_IK.InverseKinematics(BODY, LEGS)

class GazeboControllerNode(Node):
    def __init__(self):
        super().__init__('robot_controller_gazebo')
        self.get_logger().info("Starting Gazebo Controller Node…")

        # Subs: joystick and IMU
        self.create_subscription(
            Joy, '/spot_joy/joy_ramped',
            spot_robot.joystick_command, 10
        )
        self.create_subscription(
            Imu, '/imu',
            spot_robot.imu_orientation, 10
        )

        # Pub: joint angles
        self.pub = self.create_publisher(
            Float64MultiArray,
            '/forward_position_controller/commands',
            10
        )

        # Main loop ~66 Hz
        self.create_timer(0.015, self.timer_cb)
        np.set_printoptions(formatter={'float_kind':"{:.2f}".format})

    def timer_cb(self):
        # 1) Possibly switch gait
        spot_robot.change_controller()

        # 2) Compute footholds
        feet = spot_robot.run()

        # 3) IK → joint angles
        x, y, z       = spot_robot.state.body_local_position
        r, p, yaw     = spot_robot.state.body_local_orientation
        try:
            angles = ik.inverse_kinematics(feet, x, y, z, r, p, yaw)
        except Exception as e:
            self.get_logger().warning(f"IK failure: {e}")
            return

        # 4) Publish
        self.pub.publish(Float64MultiArray(data=angles))

def main(args=None):
    rclpy.init(args=args)
    node = GazeboControllerNode()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
