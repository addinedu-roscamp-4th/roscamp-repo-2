# 하나의 알바봇만 동작을 실행시키는 코드 (픽업, 서빙 동작)
# 좌표 고정, 커맨드 받지 않음
import os
import multiprocessing as mp
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import json
import inspect
#from alba_manager.albabot_controller import MoveToGoal
#from albabot_controller import *
#from move_controller import *
from mapping import *
from motion.motion_pickup import *
from motion.motion_seving import *
from move.nav2_goal_send import send_goal_and_wait
import math
import time


pickup_deg = 180.0
serving_deg = -90.0
table_deg = 0.0


# --------- 로봇 제어 노드 (각 도메인 별 프로세스) ---------
class RobotMoverNode(Node):

    def __init__(self):
        super().__init__('RobotMoverNode')

        #self.publisher = self.create_publisher(PoseStamped, '/move_goal', qos)
        #self.publisher = self.create_publisher(PoseStamped, '/goal_pose')
        #self.timer = self.create_timer(0.1, self.process_queue)
        #self.pub_command = self.create_publisher(String, '/command', qos)
        # RobotMoverNode.__init__ 내부 마지막 줄에 추가
        self.move_to_pickup()
        self.pick_up()
        self.move_to_serving()
        self.serving()

    def move_to_pickup(self):
        self.log_with_info("Move to Pickup Zone")
        # 픽업 위치 설정
        #map_x, map_y = (51.64, 61.15) # pick-up
        #goal_x, goal_y = pixel_to_real_coordinates(map_x, map_y)
        goal_x, goal_y = (1.120, -0.325)
        pickup_yaw = pickup_deg
        # goal(픽업 좌표) 전송, 이동 시작
        result = send_goal_and_wait(goal_x, goal_y, pickup_yaw)
        self.log_with_info(f"이동 시작: {goal_x:.2f}, {goal_y:.2f}")
        #결과 확인
        if result == 1:
            self.log_with_info("Reached to Pickup Zone")
        else:
            self.log_with_info("Move Failed or Goal Send Rejected")

    def pick_up(self):
        self.log_with_info("Pickup Motion")
        run_pickup_motion(pickup_deg)
        self.log_with_info("Pickup motion complete")

    def move_to_serving(self):
        self.log_with_info("Move to Serving Table")
        #서빙 테이블 위치 설정
        #map_x, map_y = (34.00, 67.00) # pick-up
        #goal_x, goal_y = pixel_to_real_coordinates(map_x, map_y)
        goal_x, goal_y = (0.500, -0.500)
        serving_yaw = table_deg
        # goal(테이블 서빙 좌표) 전송, 이동 시작
        result = send_goal_and_wait(goal_x, goal_y, serving_yaw)
        self.log_with_info(f"이동 시작: {goal_x:.2f}, {goal_y:.2f}")
        #결과 확인
        if result == 1:
            self.log_with_info("Reached to Serving Spot")
        else:
            self.log_with_info("Move Failed or Goal Send Rejected")
    
    def serving(self):
        self.log_with_info("Serving Motion")
        run_serving_motion(serving_deg, table_deg, 1)
        self.log_with_info("Serving motion complete")


    def log_with_info(self, message):
        frame = inspect.currentframe()
        caller_frame = frame.f_back
        function_name = caller_frame.f_code.co_name
        line_number = caller_frame.f_lineno
        self.get_logger().info(f"[{function_name}({line_number})] {message}")


def main(args=None):
    rclpy.init(args=args)
    node = RobotMoverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()