import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy

from std_msgs.msg import String

class Receiver(Node):

    def __init__(self):
        super().__init__('receiver')
        self.declare_parameter('qos_depth', 10)
        qos_depth = self.get_parameter('qos_depth').value
        
        QOS_RKL10V = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=qos_depth)

        self.obstacle_information_receiver = self.create_subscription(
            String,
            'discriminated_obstacle',
            self.subscribe_obstacle_information,
            QOS_RKL10V)

    def subscribe_obstacle_information(self, msg):
        self.get_logger().info(f"Received message : {msg.data}")
def main(args=None):
    rclpy.init(args=args)
    node=Receiver()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()