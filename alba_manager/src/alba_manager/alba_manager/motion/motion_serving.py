#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import LaserScan
import math
import tf_transformations
import sys
import time
import os
from datetime import datetime

class SimplePID:
    def __init__(self, P=1.5, I=0.0, D=0.1, output_limit=0.1):
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
        output = max(min(output, self.output_limit), -self.output_limit)
        self.prev_error = error
        return output
    
class RotateToYawPID(Node):
    def __init__(self, target_yaw, mode=None, fix_one_side=False):
        super().__init__('rotate_to_yaw_pid')
        self.pid = SimplePID(P=0.2, I=0.0, D=0.1, output_limit=0.01)
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/tracked_pose',
            self.pose_callback,
            10
        )
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.target_yaw = target_yaw  # 목표 yaw 설정
        self.reached = False
        self.tolerance_rad = math.radians(2.0)  
        self.mode = mode                # 0 또는 1
        self.fix_one_side = fix_one_side  # True: 회전 방향 고정
    def pose_callback(self, msg):
        q = msg.pose.orientation
        _, _, yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        error = self.normalize_angle(self.target_yaw - yaw)
        twist = Twist()
        if abs(error) < self.tolerance_rad:
            twist.angular.z = 0.0
            self.reached = True
            self.get_logger().info(f"✅ 목표 Yaw {math.degrees(self.target_yaw):.2f}° 도달. 정지.")
        else:
            output = self.pid.compute(error)
            if self.fix_one_side and self.mode is not None:
                twist.linear.x = abs(output)
                if self.mode == 0:
                    # 오른쪽 고정: 시계방향으로만 회전
                    twist.angular.z = -min(abs(output), 0.01)
                    print(f"rotation speed = {twist.angular.z}") 
                elif self.mode == 1:
                    # 왼쪽 고정: 반시계방향으로만 회전
                    twist.angular.z = min(abs(output), 0.01) 
                    print(f"rotation speed = {twist.angular.z}") 
            else:
                # 일반 PID 회전(방향 상관 없이 가까운 쪽으로 회전))
                max_speed = 0.01  # 최대 회전 속도(rad/s)
                limited_output = max(min(output, max_speed), -max_speed)
                twist.angular.z = limited_output
                print(f"rotation speed = {twist.angular.z}") 
        twist.linear.x = 0.0
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info(
            f"[현재 Yaw] {math.degrees(yaw):.2f}° | [목표] {math.degrees(self.target_yaw):.2f}° | "
            f"[오차] {math.degrees(error):.2f}° | [PID출력] {twist.angular.z:.3f} rad/s"
        )
    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))
        
    
class MoveForword(Node):
    def __init__(self, distance):
        super().__init__('move_front')
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.target_distance_cm = distance
        self.speed = 0.4       # 속도 (m/s)
        self.reached = False
    def angle_to_index(self, angle_deg, msg):
        angle_rad = math.radians(angle_deg)
        return int((angle_rad - msg.angle_min) / msg.angle_increment) % len(msg.ranges)
    def scan_callback(self, msg):
        if self.reached:
            return
        idx_front = self.angle_to_index(180, msg)  # 라이다 전방 180도
        distance_m = msg.ranges[idx_front]
        if not math.isfinite(distance_m) or distance_m > 5.0:
            self.get_logger().warn("유효하지 않은 라이다 거리. 대기합니다.")
            return
        distance_cm = distance_m * 100.0
        self.get_logger().info(f"[전방 거리] {distance_cm:.1f} cm")
        if distance_cm > self.target_distance_cm:
            # 전진
            twist = Twist()
            twist.linear.x = self.speed
            self.cmd_vel_pub.publish(twist)
        else:
            # 목표 거리 도달: 정지
            twist = Twist()
            twist.linear.x = 0.0
            self.cmd_vel_pub.publish(twist)
            self.reached = True
            self.get_logger().info("전진 완료. 정지")

class MoveBackword(Node):
    def __init__(self, distance):
        super().__init__('move_front')
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.target_distance_cm = distance
        self.speed = -0.4  # 속도 (m/s)
        self.reached = False
    def angle_to_index(self, angle_deg, msg):
        angle_rad = math.radians(angle_deg)
        return int((angle_rad - msg.angle_min) / msg.angle_increment) % len(msg.ranges)
    def scan_callback(self, msg):
        if self.reached:
            return
        idx_front = self.angle_to_index(0, msg)  # 라이다 후방 0도
        distance_m = msg.ranges[idx_front]
        if not math.isfinite(distance_m) or distance_m > 5.0:
            self.get_logger().warn("유효하지 않은 라이다 거리. 대기합니다.")
            return
        distance_cm = distance_m * 100.0
        self.get_logger().info(f"[후방 거리] {distance_cm:.1f} cm")
        if distance_cm > self.target_distance_cm:
            # 계속 전진
            twist = Twist()
            twist.linear.x = self.speed
            self.cmd_vel_pub.publish(twist)
        else:
            # 목표 거리 도달: 정지
            twist = Twist()
            twist.linear.x = 0.0
            self.cmd_vel_pub.publish(twist)
            self.reached = True
            self.get_logger().info("후진 완료. 정지")


def run_serving_motion(serving_deg: float, table_deg: float, mode: int):
    if mode not in (0, 1):
        print("mode는 0 또는 1")
        return
    #각도(서빙, 테이블)를 라디안으로
    serving_yaw = math.radians(serving_deg)
    table_yaw = math.radians(table_deg)
    #정렬
    align_node = RotateToYawPID(serving_yaw, None, False)
    while rclpy.ok() and not align_node.reached:
        rclpy.spin_once(align_node, timeout_sec=0.1)
    time.sleep(2)
    align_node.get_logger().info("Align Completed. Start Moving Forword.")
    #전진
    move_forword_node = MoveForword(20)
    while rclpy.ok() and not move_forword_node.reached:
        rclpy.spin_once(move_forword_node, timeout_sec=0.1)
    move_forword_node.get_logger().info("Move Forword Completed. Start Rotate to Table.")
    #회전
    rotate_node = RotateToYawPID(table_yaw, mode, True)
    while rclpy.ok() and not rotate_node.reached:
        rclpy.spin_once(rotate_node, timeout_sec=0.1)
    time.sleep(2)
    align_node.get_logger().info("Rotation Completed. Start Moving Backword.")
    #후진
    move_backword_node = MoveBackword(50)
    while rclpy.ok() and not move_backword_node.reached:
        rclpy.spin_once(move_backword_node, timeout_sec=0.1)
    move_forword_node.get_logger().info("Move Backword Completed. Serving motion Completed.")
    #종료
    align_node.destroy_node()
    move_forword_node.destroy_node()
    rotate_node.destroy_node()
    move_backword_node.destroy_node()



def main(args=None):
    rclpy.init(args=args)
    '''
    log_dir = "~/pinky_logs/serving_copy/"
    os.makedirs(log_dir, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_filename = os.path.join(log_dir, f"servinf_copy_g_{timestamp}.txt")
    sys.stdout = open(log_filename, "w")
    sys.stderr = sys.stdout
    print(f"로그 저장 위치: {log_dir}\n")
    '''
    if len(sys.argv) != 2:
        print("사용법: python3 your_script.py <mode>")
        print("mode=0: 시계방향 회전")
        print("mode=1: 반시계방향 회전")
        sys.exit(1)
    mode = int(sys.argv[1])
    if mode not in [0, 1]:
        print("mode는 0 또는 1만 허용")
        sys.exit(1)
    # === 1.  초기 정렬 (yaw 0도로 정렬) === #
    target_yaw = math.radians(0)
    rotate_node = RotateToYawPID(
        target_yaw=target_yaw,
        mode = None,
        fix_one_side=False
    )
    rotate_node.get_logger().info("[1단계] Yaw 0도로 정렬 시작")
    while rclpy.ok() and not rotate_node.reached:
        rclpy.spin_once(rotate_node, timeout_sec=0.1)
    rotate_node.get_logger().info("정렬 완료. 1초 대기 후 90도 회전 시작")
    time.sleep(1)
    rotate_node.destroy_node()
    # === 2.  벽 앞 18cm까지 이동 === #
    move_node = MoveForword(distance=18)
    move_node.get_logger().info("[2단계] 벽 앞 {distance}cm 까지 전진")
    while rclpy.ok() and not move_node.reached:
        rclpy.spin_once(move_node, timeout_sec=1)
    move_node.get_logger().info("앞으로 이동 완료")
    move_node.destroy_node()
    # === 3. 90도 회전 ===
    target_yaw = math.radians(95) if mode == 1 else math.radians(-87)
    rotate_node2 = RotateToYawPID(
        target_yaw=target_yaw,
        mode=mode,
        fix_one_side=True
    )
    rotate_node2.get_logger().info(f"[3단계] 90도 회전 시작 (mode={mode})")
    while rclpy.ok() and not rotate_node2.reached:
        rclpy.spin_once(rotate_node2, timeout_sec=0.1)
    rotate_node2.get_logger().info("90도 회전 완료.")
    rotate_node2.destroy_node()
    # === 2.  테이블 앞 까지 후진 이동 === #
    back_node = MoveBackword(distance=37)
    back_node.get_logger().info("[4단계] 뒤로 이동 시작(벽과의 거리 35cm)")
    while rclpy.ok() and not back_node.reached:
        rclpy.spin_once(back_node, timeout_sec=0.1)
    back_node.get_logger().info("뒤로 이동 완료. 종료합니다.")
    back_node.destroy_node()  
    rclpy.shutdown()
if __name__ == '__main__':
    main()