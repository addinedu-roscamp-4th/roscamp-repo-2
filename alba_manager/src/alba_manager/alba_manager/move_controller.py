import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
import tf_transformations
from std_msgs.msg import Float64
import sys
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from alba_manager.mapping import *

class PIDController:
    def __init__(self, kp, ki, kd, output_limit=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0.0
        self.last_error = 0.0
        self.max_output = 0.5  # 최대 각속도 제한 (예시)
        self.min_output = -0.5
        self.output_limit = output_limit
        self.window_size = 10
        self._buffer = [0.0] * self.window_size
        self._prev_filtered_value = 0.0
        self.alpha = 0.3  # Low-pass filter 계수 (0 < alpha < 1)

    def low_pass_filter(self, new_value):
        filtered_value = self.alpha * new_value + (1 - self.alpha) * self._prev_filtered_value
        self._prev_filtered_value = filtered_value
        return filtered_value

    def update(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.last_error) / dt if dt > 0 else 0.0
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        if self.output_limit is not None:
            output = max(min(output, self.output_limit[1]), self.output_limit[0])
        low_pass_output = self.low_pass_filter(output)
        if low_pass_output > self.max_output:
            output = self.max_output
        elif low_pass_output < self.min_output:
            output = self.min_output
        else:
            output = low_pass_output
        output = max(min(output, self.max_output), self.min_output)
        self.last_error = error
        return output

class MoveToGoal(Node):
    def __init__(self, map_locate, goal_x, goal_y):
        super().__init__('move_to_goal')
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/tracked_pose',
            self.pose_callback,
            10
        )
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        #self.yaw_error_pub = self.create_publisher(Float64, '/yaw_error', 10)
        #self.pid_output_pub = self.create_publisher(Float64, '/pid_output', 10)
        self.map_locate = map_locate
        self.goal_x = goal_x
        self.goal_y = goal_y
        self.speed = 0.05  # m/s
        self.angle_tolerance = math.radians(10)  # 10도 허용
        self.dist_tolerance = 0.020  # 2cm 허용
        self.reached = False

        # PID 제어기 계수 (환경에 따라 조정 필요)
        self.pid = PIDController(kp=1.0, ki=0.0, kd=0.1)
        self.last_time = self.get_clock().now()

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def pose_callback(self, msg):
        if self.reached:
            return

        x = msg.pose.position.x
        y = msg.pose.position.y
        orientation = msg.pose.orientation
        quaternion = (
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
        )
        _, _, yaw = tf_transformations.euler_from_quaternion(quaternion)

        dx = self.goal_x - x
        dy = self.goal_y - y
        target_yaw = math.atan2(dy, dx)
        yaw_error = self.normalize_angle(target_yaw - yaw)
        tx, ty = real_to_map(self.goal_x, self.goal_y)
        cx, cy = real_to_map(x, y)

        dist = math.hypot(dx, dy)
        if dist < 0.10: # 10 cm
            self.angle_tolerance = math.radians(20)
        else:
            self.angle_tolerance = math.radians(30)

        self.get_logger().info(f"{self.map_locate} ({tx:>3.2f},{ty:>3.2f}) <-> ({cx:>3.2f},{cy:>3.2f}) [거리] {dist*100:.2f}cm({self.dist_tolerance*100}), [각도오차] {math.degrees(yaw_error):.1f}deg({math.degrees(self.angle_tolerance):3.2f})")

        twist = Twist()
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        pid_output = self.pid.update(yaw_error, dt)
        #self.yaw_error_pub.publish(Float64(data=yaw_error))
        #self.pid_output_pub.publish(Float64(data=pid_output))

        if abs(yaw_error) > self.angle_tolerance:
            # PID 제어로 각속도 계산
            if dist <= self.dist_tolerance:
                self.reached = True
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.get_logger().info("목적지 도달! 각도 안 맞지만 정지합니다.")
            else:
                twist.angular.z = pid_output
                twist.linear.x = 0.0
        elif dist > self.dist_tolerance:
            twist.linear.x = self.speed
            twist.angular.z = 0.0
        else:
            self.get_logger().info("목적지 도달! 정지합니다.")
            self.reached = True

        self.cmd_vel_pub.publish(twist)


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
        self.pid = SimplePID(P=0.1, I=0.0, D=0.1, output_limit=0.01)

        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/tracked_pose',
            self.pose_callback,
            10
        )

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.target_yaw = target_yaw  # 목표 yaw 설정
        self.reached = False
        self.tolerance_rad = math.radians(1.0)

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


def main(args=None):
    rclpy.init(args=args)

    # 예시: (35, 69) 픽셀 좌표를 실좌표로 변환
    # TABLE 2   TABLE 4
    #    HOME           PICKUP
    # TABLE 1   TABLE 3
    map_locate = {
        0: "HOME",
        1: "TABLE1",
        2: "TABLE2",
        3: "TABLE3",
        4: "TABLE4",
        9: "PICKUP",
    }
    if len(sys.argv) != 2:
        print("Usage: pid_move_to_goal.py <0: HOME, 9: PICKUP, 1 ~ 4: TABLE>")
        return

    try:
        option = int(sys.argv[1])  # argument를 정수로 변환합니다.
        print(option)
    except ValueError:
        print("Please provide a valid integer (1 or 2).")
        return

    if option == 0:
        map_x, map_y = (28, 65) # home
    elif option == 1:
        map_x, map_y = (33.50, 65) # TABLE 1
    elif option == 2:
        map_x, map_y = (33.50, 57) # TABLE 2
    elif option == 3:
        map_x, map_y = (40.00, 65) # TABLE 3
    elif option == 4:
        map_x, map_y = (40.00, 57) # TABLE 4
    elif option == 9:
        map_x, map_y = (47, 61.70) # pick-up
    else:
        option = 0
        map_x, map_y = (27.0, 67.0) # home

    goal_x, goal_y = pixel_to_real_coordinates(map_x, map_y)
    move_node = MoveToGoal(map_locate[option], goal_x, goal_y)
    move_node.get_logger().info(f"PID 제어로 대각선 목적지 ({goal_x:.2f}, {goal_y:.2f})로 이동 시작")
    while rclpy.ok() and not move_node.reached:
        rclpy.spin_once(move_node, timeout_sec=0.1)
    move_node.get_logger().info("이동 완료")
    move_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

