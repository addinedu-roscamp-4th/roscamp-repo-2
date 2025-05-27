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
from motion.move.nav2_goal_send import send_goal_and_wait



tables = {
    1: (34.00, 67.00),
    2: (34.00, 56.50),
    3: (38.00, 67.00),
    4: (38.00, 56.50)
    }
emergency_exit = {
    1: (26.00, 56.00),
    2: (26.00, 67.00),
    3: (51.00, 70.00),
    }
security_area = {
    1: (39.00, 56.00),
    2: (32.00, 67.00),
    3: (46.00, 67.00),
    }
cleaning_area = {
    1: (27.00, 56.00),
    2: (37.00, 56.00),
    3: (47.00, 56.00),
}
maintenance = (52.00, 54.50)
pickup_deg = 180.0
serving_deg = -90.0
serving_deg2 = 90.0
table_deg = 0.0
temp_deg = 0.0



# --------- 로봇 제어 노드 (각 도메인 별 프로세스) ---------
class RobotMoverNode(Node):
    def __init__(self, domain_id, pose_queue):
        super().__init__(f'robot_mover_{domain_id}')
        self.domain_id = domain_id
        self.pose_queue = pose_queue
        self.domain_to_robot_id = { 
            74: 1,
            62: 2,
            58: 3
        }
        self.robot_id = self.domain_to_robot_id.get(domain_id)

        # QoS 설정 (로봇 제어용)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )

        self.publisher = self.create_publisher(PoseStamped, '/goal_pose', qos)
        self.timer = self.create_timer(0.1, self.process_queue)
        self.pub_command = self.create_publisher(String, '/command', qos)
        # RobotMoverNode.__init__ 내부 마지막 줄에 추가


    def handle_pickup_task(self):
        self.log_with_info("Move to Pickup Zone")
        # 픽업 위치 설정
        map_x, map_y = (51.64, 61.15) # pick-up
        goal_x, goal_y = pixel_to_real_coordinates(map_x, map_y) #goal_x, goal_y = (1.120, -0.325)
        pickup_yaw = pickup_deg
        # goal(픽업 좌표) 전송, 이동 시작
        result = send_goal_and_wait(goal_x, goal_y, pickup_yaw)
        self.log_with_info(f"이동 시작: {goal_x:.2f}, {goal_y:.2f}")
        # 이동 결과 확인
        if result == 1:
            self.log_with_info("Reached to Pickup Zone")
            # 픽업 정밀 모션 시작
            self.log_with_info("Pickup Motion Start")
            run_pickup_motion(pickup_deg)
            self.log_with_info("Pickup motion complete")
        else:
            self.log_with_info("Move Failed or Goal Send Rejected")


    def handle_serving_task(self, table):
        self.log_with_info(f"Serving to Table {table}")
        self.log_with_info("Move to Serving Table")
        #서빙 테이블 위치 설정
        map_x, map_y = tables[table][0], tables[table][1]
        goal_x, goal_y = pixel_to_real_coordinates(map_x, map_y) #goal_x, goal_y = (0.500, -0.500)
        serving_yaw = serving_deg
        # goal(테이블 서빙 좌표) 전송, 이동 시작
        result = send_goal_and_wait(goal_x, goal_y, serving_yaw)
        self.log_with_info(f"이동 시작: {goal_x:.2f}, {goal_y:.2f}")
        #결과 확인
        if result == 1:
            self.log_with_info("Reached to Serving Spot")
            #서빙 정밀 모션 시작
            self.log_with_info("Serving Motion")
            run_serving_motion(serving_deg, table_deg, 1)
            self.log_with_info("Serving motion complete")
        else:
            self.log_with_info("Move Failed or Goal Send Rejected")


    def handle_birthday_task(self, table):
        self.get_logger().info(f"BIRTHDAY TO TABLE({table})")
        self.log_with_info(f"Move to Brithday Table: {table}")
        #생일 테이블 위치 설정
        map_x, map_y = tables[table][0], tables[table][1]
        goal_x, goal_y = pixel_to_real_coordinates(map_x, map_y)
        if table in (1, 3):
            table_yaw = temp_deg
        else:
            table_yaw = temp_deg
        #goal(테이블 좌표) 전송, 이동 시작
        result = send_goal_and_wait(goal_x, goal_y, table_yaw)
        self.log_with_info(f'Moving to Table{table}: ({goal_x:.2f}, {goal_y:.2f}) ...')
        #이동 결과 확인
        if result == 1:
            self.log_with_info(f"Reached to Table{table}")
            #생일 축하 모션 시작
            self.log_with_info("생일 축하중(모션 추가 예정)...") 
                # 추가예정
            self.log_with_info("Birthday Completed")
        else:
            self.log_with_info("Move Failed or Goal Send Rejected")


    def handle_emergency_task(self):
        self.get_logger().info("EMERGENCY")
        self.log_with_info(f"Move to Emergency Exit {self.robot_id}")
        # 비상시 이동 위치 설정
        map_x, map_y = emergency_exit[self.robot_id][0], emergency_exit[self.robot_id][1]
        goal_x, goal_y = pixel_to_real_coordinates(map_x, map_y)
        if self.robot_id == 3:
            rotate_yaw = temp_deg
        else:
            rotate_yaw = temp_deg
        # goal(비상시 위치) 전송, 이동 시작
        result = send_goal_and_wait(goal_x, goal_y, rotate_yaw)
        self.log_with_info(f"Moving to Emergency Exit {self.robot_id}")
        # 이동 결과 확인
        if result == 1:
            self.log_with_info(f"Reached to Emergency Exit {self.robot_id}")
            # 비상 동작 시작
            self.log_with_info("비상구 안내중(모션 추가 예정)...")
                # 추가예정
        else: 
            self.log_with_info("Move Failed or Goal Send Rejected")


    def handle_maintenance_task(self):
        self.get_logger().info(f"MAINTENANCE")
        self.log_with_info("Move to Maintenance Area")
        # 정비 구역 위치 설정
        map_x, map_y = maintenance
        goal_x, goal_y = pixel_to_real_coordinates(map_x, map_y)
        maintenance_yaw = temp_deg
        # goal(정비구역 위치) 전송, 이동 시작
        result = send_goal_and_wait(goal_x, goal_y, maintenance_yaw)
        self.log_with_info("Moving to Maintenance Area")
        # 이동 결과 확인
        if result == 1:
            self.log_with_info(f"Reached to Maintenance Area")
            # 정비 시작
            self.log_with_info("정비모션중(모션 추가 예정)...")
                # 추가예정
            self.log_with_info("Maintenance Completed")
        else: 
            self.log_with_info("Move Failed or Goal Send Rejected")


    def handle_cleaning_task(self):
        self.get_logger().info("CLEANING")
        self.log_with_info("Move to Maintenance Area")
        # 청소 구역 위치 설정
        map_x, map_y = cleaning_area[self.robot_id][0], cleaning_area[self.robot_id][1]
        goal_x, goal_y = pixel_to_real_coordinates(map_x, map_y)
        cleaning_yaw = temp_deg
        # goal(청소구역 위치) 전송, 이동 시작
        result = send_goal_and_wait(goal_x, goal_y, cleaning_yaw)
        self.log_with_info("Moving to Cleaning Area")
        # 이동 결과 확인
        if result == 1:
            self.log_with_info(f"Reached to Cleaning Area")
            # 정비 시작
            self.log_with_info("청소중(모션 추가 예정)...")
                # 추가예정
            self.log_with_info("Cleaning Completed")
        else: 
            self.log_with_info("Move Failed or Goal Send Rejected")


    def handle_security_task(self):
        self.get_logger().info("SECURITY")
        self.log_with_info("Move to Security Area")
        # 경비 구역 위치 설정
        map_x, map_y = security_area[self.robot_id][0], security_area[self.robot_id][1]
        goal_x, goal_y = pixel_to_real_coordinates(map_x, map_y)
        security_yaw = temp_deg
        # goal(경비 구역) 전송, 이동 시작
        result = send_goal_and_wait(goal_x, goal_y, security_yaw)
        self.log_with_info("Moving to Security Area")
        # 이동 결과 확인
        if result == 1:
            self.log_with_info(f"Reached to Security Area")
            # 경비 모드 시작
            self.log_with_info("경비 모션중...")
                # 추가 예정
        else: 
            self.log_with_info("Move Failed or Goal Send Rejected")


    #------------------------------------------------------------
    def send_response(self, command_msg):
        status_msg = String()
        status_msg.data = json.dumps(command_msg)
        self.get_logger().info(f'📤 도메인 {self.domain_id} 명령 전송: {status_msg.data}')
        self.pub_command.publish(status_msg)

    def process_queue(self):
        try:
            if not self.pose_queue.empty():
                command_data = self.pose_queue.get_nowait()
                command = command_data['command']

                command_msg = {
                    'domain_id': self.domain_id,
                    'command': command,
                }
                command_msg['command_status'] = "ACKED"
                self.send_response(command_msg)

                if command == 'PICKUP':
                    if self.handle_pickup_task():
                        command_msg['command_status'] = "EXECUTED"
                    else:
                        command_msg['command_status'] = "FAILED"
                elif command == 'SERVING':
                    table = command_msg['param1'] = command_data['param1']
                    order_id = command_msg['param2'] = command_data['param2']
                    print(f"order_id: {order_id}, command_msg: {command_msg}, command_data: {command_data}") 
                    if self.handle_serving_task(table):
                        print("serving_success")
                        command_msg['command_status'] = "EXECUTED"
                    else:
                        print("serving_failed")
                        command_msg['command_status'] = "FAILED"
                    command_msg['order_id'] = order_id
                elif command == 'BIRTHDAY':
                    table = command_msg['param1'] = command_data['param1']
                    if self.handle_birthday_task(table):
                        command_msg['command_status'] = "EXECUTED"
                    else:
                        command_msg['command_status'] = "FAILED"
                elif command == 'EMERGENCY':
                    if self.handle_emergency_task():
                        command_msg['command_status'] = "EXECUTED"
                    else:
                        command_msg['command_status'] = "FAILED"
                elif command == 'MAINTENANCE' or command == 'CHARGE':
                    if self.handle_maintenance_task():
                        command_msg['command_status'] = "EXECUTED"
                    else:
                        command_msg['command_status'] = "FAILED"
                elif command == 'CLEANING':
                    if self.handle_cleaning_task():
                        command_msg['command_status'] = "EXECUTED"
                    else:
                        command_msg['command_status'] = "FAILED"
                elif command == 'SECURITY':
                    if self.handle_security_task():
                        command_msg['command_status'] = "EXECUTED"
                    else:
                        command_msg['command_status'] = "FAILED"
                else:
                    command_msg['command_status'] = "FAILED"
                    self.send_response(command_msg)
                    self.get_logger().warn(f'⚠️ 알 수 없는 명령: {command}')
                    return

                self.send_response(command_msg)

                if command_msg['command_status'] == "EXECUTED" and command != 'PICKUP':
                    # PICKUP은 음식을 실은 상태에서 SERVING해야 하므로 완료 처리 되어 새 TASK 받으면 안됨
                    # 그 외의 명령은 완료 후 대기 위치로 이동
                    self.handle_return_task()
                    command_msg['command_status'] = "COMPLETED"
                    self.send_response(command_msg)

        except KeyError as e:
            self.get_logger().error(f'❌ 누락된 키: {e}')
        except Exception as e:
            self.get_logger().error(f'❌ 큐 처리 중 예외 발생: {e}')

    def log_with_info(self, message):
        frame = inspect.currentframe()
        caller_frame = frame.f_back
        function_name = caller_frame.f_code.co_name
        line_number = caller_frame.f_lineno
        self.get_logger().info(f"[{function_name}({line_number})] {message}")



def robot_worker(domain_id, queue):
    os.environ['ROS_DOMAIN_ID'] = str(domain_id)
    rclpy.init()
    node = RobotMoverNode(domain_id, queue)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()




# --------- 중앙 제어 노드 (메인 프로세스) ---------
class CentralController(Node):
    def __init__(self, domain_queues):
        super().__init__('central_controller')
        self.domain_queues = domain_queues

        # 로봇 status 저장
        self.robot_status = {did: {'command': None, 'param1': None, 'param2': None} for did in domain_queues}
        
        # 명령 수신 토픽 설정
        self.subscription = self.create_subscription(
            String,
            '/robot_command',
            self.command_callback,
            10
        )
        self.get_logger().info('🔧 중앙 제어기 준비 완료')

    def command_callback(self, msg):
        try:
            print("\n\n")
            self.get_logger().info('=============== new command updated ===============')
            data = json.loads(msg.data)
            domain_id = int(data['domain_id'])
            command = data['command']

            if command == 'PICKUP':
                param1 = None
                param2 = None
                command_data = {'command': command, 'param1': param1, 'param2': param2}
            elif command == 'SERVING':
                table = int(data['param1'])
                order_id = int(data['param2'])
                if not self.is_existing_table(table):
                    return
                param1 = table
                param2 = order_id
                command_data = {'command': command, 'param1': param1, 'param2': param2}
            elif command == 'BIRTHDAY':
                table = int(data['param1'])
                if not self.is_existing_table(table):
                    return
                param1 = table
                param2 = None
                command_data = {'command': command, 'param1': param1, 'param2': param2}
            else:
                self.get_logger().warn(f"Unknown Command: {command}")
                param1, param2 = None, None
                command_data = {'command': command, 'param1': param1, 'param2': param2}


            # ✅ 상태 저장
            if domain_id in self.robot_status:
                self.robot_status[domain_id] = {'command': command, 'param1': param1, 'param2': param2}
                self.get_logger().info(f'📌 상태 갱신: {self.robot_status[domain_id]}')

            if domain_id in self.domain_queues:
                self.domain_queues[domain_id].put(command_data)
                self.get_logger().info(f'📨 도메인 {domain_id} 명령 수신: {command}')
            else:
                self.get_logger().error(f'❌ 잘못된 도메인 ID: {domain_id}')

        except Exception as e:
            self.get_logger().error(f'⚠️  명령 처리 오류: {e}')
                
        except ValueError:
            self.get_logger().error('⚠️  명령 형식 오류: "domain_id command param1 param2" 입력 필요')

        # ✅ 상태 출력
        self.get_logger().info(f'📊 로봇 상태 요약: {json.dumps(self.robot_status, indent=2)}')


    def is_existing_table(self, table):
        table_list = [1, 2, 3, 4]
        if table in table_list:
            return True
        else:
            self.get_logger().error(f'❌ 존재하지 않는 테이블: {table} (테이블: {table_list})')
            return False




# --------- 실행부 ---------
def main():
    mp.set_start_method('spawn', force=True)  # 필수!
    domain_ids = [58, 62, 74]
    domain_queues = {did: mp.Queue() for did in domain_ids}

    # 로봇 프로세스 시작
    processes = []
    for did in domain_ids:
        p = mp.Process(target=robot_worker, args=(did, domain_queues[did]))
        p.start()
        processes.append(p)
        print(f'🤖 도메인 {did} 로봇 프로세스 시작')

    # 중앙 제어 노드 실행
    rclpy.init()
    controller = CentralController(domain_queues)
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        print("\n🔴 시스템 종료 중...")
    finally:
        controller.destroy_node()
        rclpy.shutdown()
        for p in processes:
            p.terminate()

if __name__ == '__main__':
    main()
