import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
class ScanReader(Node):
    def __init__(self):
        super().__init__('scan_reader')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            10)
    def angle_to_index(self, angle_deg):
        return int((angle_deg + 180) / 0.5)
    def listener_callback(self, msg):
        # 기본 방향
        idx_front = self.angle_to_index(180) % len(msg.ranges)
        idx_back = self.angle_to_index(0) % len(msg.ranges)
        idx_left = self.angle_to_index(-90) % len(msg.ranges)
        idx_right = self.angle_to_index(90) % len(msg.ranges)
        # 대각선 방향
        idx_front_left = self.angle_to_index(225) % len(msg.ranges)
        idx_front_right = self.angle_to_index(135) % len(msg.ranges)
        idx_back_left = self.angle_to_index(315) % len(msg.ranges)
        idx_back_right = self.angle_to_index(45) % len(msg.ranges)
        # 거리 (m → cm)
        front = msg.ranges[idx_front] * 100
        back = msg.ranges[idx_back] * 100
        left = msg.ranges[idx_left] * 100
        right = msg.ranges[idx_right] * 100
        front_left = msg.ranges[idx_front_left] * 100
        front_right = msg.ranges[idx_front_right] * 100
        back_left = msg.ranges[idx_back_left] * 100
        back_right = msg.ranges[idx_back_right] * 100
        # 로그 출력
        self.get_logger().info(
            f'\nFront: {front:.1f} cm, Back: {back:.1f} cm\n'
            f'Left: {left:.1f} cm, Right: {right:.1f} cm\n'
            f'Front-Left: {front_left:.1f} cm, Front-Right: {front_right:.1f} cm\n'
            f'Back-Left: {back_left:.1f} cm, Back-Right: {back_right:.1f} cm\n\n'
            f'\t{front_left:.1f}cm  {front:.1f}cm  {front_right:.1f}cm\n'
            f'\t{left:.1f}cm \t\t {right:.1f}cm\n'
            f'\t{back_left:.1f}cm  {back:.1f}cm  {back_right:.1f}cm\n\n\n'
        )
def main(args=None):
    rclpy.init(args=args)
    node = ScanReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()