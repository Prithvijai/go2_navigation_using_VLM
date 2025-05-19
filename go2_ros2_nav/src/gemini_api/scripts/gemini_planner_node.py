#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import os
import tempfile

from google import genai
from google.genai import types

class GeminiPlannerNode(Node):
    def __init__(self):
        super().__init__('gemini_planner_node')
        # Subscribe to your camera feed
        self.subscription = self.create_subscription(
            Image, '/rgb', self.image_callback, 10)
        self.bridge = CvBridge()

        # Publisher for the comma-delimited action plan
        self.plan_pub = self.create_publisher(String, '/gemini_action_plan', 10)

        self.image_received = False
        self.temp_image_path = os.path.join(
            tempfile.gettempdir(), 'gemini_rgb_image.png')

        self.get_logger().info("Gemini Planner Node is waiting for image...")
        self.prompt = input("Enter the prompt for Gemini (e.g., 'Move to red cube at (2,1)'): ")

        # Init Gemini client
        self.client = genai.Client(api_key="Enter your API key here")

    def image_callback(self, msg: Image):
        if not self.image_received:
            # Convert ROS Image → OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv2.imwrite(self.temp_image_path, cv_image)
            self.get_logger().info(f"Image received and saved to {self.temp_image_path}")
            self.image_received = True
            self.run_gemini()

    def run_gemini(self):
        try:
            uploaded_file = self.client.files.upload(file=self.temp_image_path)

            response = self.client.models.generate_content(
                model="gemini-2.0-flash",
                config=types.GenerateContentConfig(
                    system_instruction=(
                        "You are a High level planner of the robot. "
                        "(JUST GIVE THE ACTION LIST) Based on the user's request, "
                        "you will provide a detailed plan to achieve the goal. "
                        "You will also provide just list of action needed to achieve the goal. "
                        "The Types of actions are: "
                        "1. Turn 90 degrees left, "
                        "2. Turn 90 degrees right, "
                        "3. Move forward 1 m, "
                        "4. Move backward 1 m. "
                        "The User will say color of block with its location (x,y) (Only Include 1 m in Forward and Backward). "
                        "and assume the robot is in (0,0) facing the x-axis. "
                        "(JUST GIVE THE ACTION LIST) (DON'T GIVE ANY EXPLANATION). "
                        "(NOTE: EACH GRID IS 1m x 1m)"
                    )
                ),
                contents=[uploaded_file, self.prompt]
            )

            plan_text = response.text.strip()
            self.get_logger().info("🔁 Gemini's Action Plan:\n" + plan_text)

            # Parse into individual steps and publish
            steps = []
            for line in plan_text.splitlines():
                line = line.strip()
                if not line: continue
                # strip leading numbering “1.”, “2.” etc.
                if '.' in line:
                    _, rest = line.split('.', 1)
                    steps.append(rest.strip())
                else:
                    steps.append(line)
            # join with commas
            plan_msg = String()
            plan_msg.data = ",".join(steps)
            self.plan_pub.publish(plan_msg)
            self.get_logger().info(f"Published plan: {plan_msg.data}")

        except Exception as e:
            self.get_logger().error(f"Gemini call failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = GeminiPlannerNode()
    # spin until we get image + plan
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

