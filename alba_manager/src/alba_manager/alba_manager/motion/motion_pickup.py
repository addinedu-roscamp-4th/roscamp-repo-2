#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import LaserScan
import math
import tf_transformations
import time 

class SimplePID:
    def __init__(self, P=1.5, I=0.0, D=0.1, output_limit=0.1): #(self, P=1.5, I=0.0, D=0.1, output_limit=0.05)
        self.P = P
        self.I = I
        self.D = D
        self.output_limit = output_limit
        self.prev_error = 0.0
        self.integral = 0.0
    def reset(self):
        self.prev_error = 0.0
        self.integral = 0.0
    def compute(self, error):
        self.integral += error
        derivative = error - self.prev_error
        output = self.P * error + self.I * self.integral + self.D * derivative
        # Saturate output
        output = max(min(output, self.output_limit), -self.output_limit)
        self.prev_error = error
        return output
    
class RotateToYawPID(Node):
    def __init__(self, rad):
        super().__init__('rotate_to_yaw_pid')
        # 목표 yaw (라디안) 설정
        self.target_yaw = rad  # 기준 라디안
        # PID 컨트롤러 생성
        self.pid = SimplePID(P=0.1, I=0.0, D=0.1, output_limit=0.1) # 기본값 (P=0.1, I=0.0, D=0.1, output_limit=1.5) 
        # 서브스크라이버
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/tracked_pose',
            self.pose_callback,
            10)
        # 퍼블리셔
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # 완료 플래그
        self.reached = False
        self.tolerance_rad = math.radians(1)  # 허용 오차
    def pose_callback(self, msg):
        if self.reached:
            return  # 이미 완료된 경우 무시
        # 현재 yaw 계산
        q = msg.pose.orientation
        _, _, yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        '''
        # 에러 계산 (-pi ~ pi)
        error = self.target_yaw - yaw
        error = math.atan2(math.sin(error), math.cos(error))
        # PID 계산
        twist = Twist()
        if abs(error) < self.tolerance_rad:
            twist.angular.z = 0.0
            self.reached = True
            self.get_logger().info("목표 방향 도달. 정지.")
        else:
            twist.angular.z = self.pid.compute(error)
        # 직진 X
        twist.angular.z = self.target_yaw
        twist.linear.x = 0.0
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info(
            f"[현재 Yaw] {math.degrees(yaw):.2f}° | [목표] {math.degrees(self.target_yaw):.2f}° | "
            f"[오차] {math.degrees(error):.2f}° | [PID출력] {twist.angular.z:.3f} rad/s"
        )'''

class MoveBack(Node):
    def __init__(self):
        super().__init__('move_back')
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.target_distance_cm = 51
        self.reverse_speed = -0.1       # 후진 속도 (m/s)
        self.reached = False
    def angle_to_index(self, angle_deg, msg):
        angle_rad = math.radians(angle_deg)
        return int((angle_rad - msg.angle_min) / msg.angle_increment) % len(msg.ranges)
    def scan_callback(self, msg):
        if self.reached:
            return
        idx_back = self.angle_to_index(0, msg)  # 0도 (뒤쪽)
        distance_m = msg.ranges[idx_back]
        if not math.isfinite(distance_m) or distance_m > 5.0:
            self.get_logger().warn("유효하지 않은 라이다 거리. 대기합니다.")
            return
        distance_cm = distance_m * 100.0
        self.get_logger().info(f"[후방 거리] {distance_cm:.1f} cm")
        if distance_cm > self.target_distance_cm:
            # 계속 후진
            twist = Twist()
            twist.linear.x = self.reverse_speed
            self.cmd_vel_pub.publish(twist)
        else:
            # 목표 거리 도달: 정지
            twist = Twist()
            twist.linear.x = 0.0
            self.cmd_vel_pub.publish(twist)
            self.reached = True
            self.get_logger().info("후진 완료. 정지")


def run_pickup_motion(pickup_deg):
    # 각도를 라디안으로
    pickup_yaw = math.radians(pickup_deg) 
    # 방향 정렬
    rotate_node = RotateToYawPID(pickup_yaw)
    while rclpy.ok() and not rotate_node.reached:
        rclpy.spin_once(rotate_node, timeout_sec=0.1)
    time.sleep(2)
    rotate_node.get_logger().info("Align Completed. Stard Moving Backword.")
    # 후진 시작
    move_back_node = MoveBack()
    while rclpy.ok() and not move_back_node.reached:
        rclpy.spin_once(move_back_node, timeout_sec=0.1)
    move_back_node.get_logger().info("Back up Completed. Pickup Motion Completed")
    # 종료
    rotate_node.destroy_node()
    move_back_node.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    # 픽업 지점 근처에 왔다고 가정. 이후 픽업 지점까지 이동
    # 1. 방향 회전
    rotate_node = RotateToYawPID(3.141592)
    # 2. 방향 정렬 완료까지 spin
    while rclpy.ok() and not rotate_node.reached:
        rclpy.spin_once(rotate_node, timeout_sec=0.1)
    time.sleep(2)
    rotate_node.get_logger().info("방향 정렬 완료. 후진 시작")
    # 3. 후진
    move_back_node = MoveBack()
    while rclpy.ok() and not move_back_node.reached:
        rclpy.spin_once(move_back_node, timeout_sec=0.1)
    move_back_node.get_logger().info("후진 완료. 종료합니다.")
    rotate_node.destroy_node()
    move_back_node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()