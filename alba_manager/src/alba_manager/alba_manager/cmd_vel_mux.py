#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelMux(Node):
    def __init__(self):
        super().__init__('cmd_vel_mux')

        # 우선순위: 낮을수록 우선
        self.sources = ['cmd_vel_teleop', 'cmd_vel_obstacle_dynamic', 'cmd_vel_obstacle_static', 'cmd_vel_nav2']
        self.priority = {name: i for i, name in enumerate(self.sources)}
        self.last_msgs = {}
        self.last_times = {}

        self.timeout_sec = 0.5
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        for topic in self.sources:
            self.create_subscription(Twist, f'/{topic}', self.make_callback(topic), 10)

        self.create_timer(0.05, self.publish_muxed_cmd_vel)

    def make_callback(self, topic_name):
        def callback(msg):
            self.last_msgs[topic_name] = msg
            self.last_times[topic_name] = self.get_clock().now()
        return callback

    def publish_muxed_cmd_vel(self):
        now = self.get_clock().now()
        selected_topic = None
        selected_msg = None

        for topic in self.sources:
            if topic in self.last_msgs:
                age = (now - self.last_times[topic]).nanoseconds * 1e-9
                if age < self.timeout_sec:
                    selected_topic = topic
                    selected_msg = self.last_msgs[topic]
                    break

        if selected_msg:
            self.publisher.publish(selected_msg)
            self.get_logger().info(f"[MUX] Publishing from: /{selected_topic} (age={age:.2f}s)")
        else:
            #self.publisher.publish(Twist())
            self.get_logger().info("[MUX] No valid input → sending stop command.")

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("[MUX] Keyboard interrupt")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()