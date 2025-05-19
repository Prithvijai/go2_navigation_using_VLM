#!/usr/bin/env python3
import rclpy, time, re, numpy as np
from rclpy.node import Node
from std_msgs.msg import String

class PlanExecutor(Node):
    def __init__(self):
        super().__init__('plan_executor')
        self.create_subscription(String, '/gemini_action_plan',
                                 self.cb, 10)
        self.pub = self.create_publisher(String, '/high_level_cmd', 10)
        self.fs = 0.2  # m/s
        self.ys = 0.5  # rad/s
        self.get_logger().info("PlanExecutor ready.")

    def cb(self, msg: String):
        steps = [s.strip() for s in msg.data.split(',') if s.strip()]
        for i, step in enumerate(steps):
            sl = step.lower()
            self.get_logger().info(f"→ {sl}")
            self.pub.publish(String(data=sl))

            # estimate time
            wt = 2.0
            m = re.match(r"move forward\s+([\d.]+)", sl)
            if m:
                wt = float(m.group(1))/self.fs + 1.0
            elif "turn" in sl:
                deg = re.search(r"(\d+)", sl)
                if deg:
                    ang = float(deg.group(1))*np.pi/180.0
                    wt = abs(ang)/self.ys + 0.5

            time.sleep(wt)

            # final stand
            if i == len(steps)-1:
                self.get_logger().info("→ stand")
                self.pub.publish(String(data="stand"))

def main():
    rclpy.init()
    node = PlanExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':
    main()
