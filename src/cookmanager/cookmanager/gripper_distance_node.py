import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
import numpy as np

class GripperDistanceNode(Node):
    def __init__(self):
        super().__init__('gripper_distance_node')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.0, self.timer_callback)

        # 파라미터 선언 및 읽기
        self.declare_parameter('robot1_gripper_frame', 'robot1_jiazhua_Link')
        self.declare_parameter('robot2_gripper_frame', 'robot2_jiazhua_Link')
        self.frame1 = self.get_parameter('robot1_gripper_frame').get_parameter_value().string_value
        self.frame2 = self.get_parameter('robot2_gripper_frame').get_parameter_value().string_value

    def timer_callback(self):
        try:
            tf1 = self.tf_buffer.lookup_transform('world', self.frame1, rclpy.time.Time())
            tf2 = self.tf_buffer.lookup_transform('world', self.frame2, rclpy.time.Time())

            p1 = np.array([tf1.transform.translation.x,
                           tf1.transform.translation.y,
                           tf1.transform.translation.z])
            p2 = np.array([tf2.transform.translation.x,
                           tf2.transform.translation.y,
                           tf2.transform.translation.z])

            dist = np.linalg.norm(p1 - p2)
            self.get_logger().info(f"Gripper-to-Gripper Distance: {dist:.3f} m")
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")

def main():
    rclpy.init()
    node = GripperDistanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
