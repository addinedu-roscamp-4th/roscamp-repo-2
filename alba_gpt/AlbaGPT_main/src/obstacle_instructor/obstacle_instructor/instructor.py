import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy

from std_msgs.msg import String

class Instructor(Node):

    def __init__(self):
        super().__init__('instructor')
        self.declare_parameter('qos_depth', 10)
        qos_depth = self.get_parameter('qos_depth').value

        QOS_RKL10V = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=qos_depth)

        self.obstacle_information_publisher = self.create_publisher(
            String,
            'discriminated_obstacle',
            QOS_RKL10V)

        self.timer = self.create_timer(1.0, self.publish_obstacle_information)

    def publish_obstacle_information(self):
        msg = String()
        msg.data = "Hello World"
        self.get_logger().info(f"Published message : {msg.data}")
        self.obstacle_information_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        instructor = Instructor()
        try:
            rclpy.spin(instructor)
        except KeyboardInterrupt:
            instructor.get_logger().info('Keyboard Interrupt (SIGINT)')
        finally:
            instructor.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()