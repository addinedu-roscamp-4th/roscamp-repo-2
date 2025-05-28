import os
import multiprocessing as mp
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import json
#from alba_manager.albabot_controller import MoveToGoal
#from albabot_controller import *
#from move_controller import *
from mapping import *
import math
import time

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
        self.get_logger().info("PICKUP")
        self.get_logger().info("픽업 구역 이동 시작")
        map_x, map_y = (51.64, 61.15) # pick-up
        goal_x, goal_y = pixel_to_real_coordinates(map_x, map_y)
        move_node = MoveToGoal("PICKUP", goal_x, goal_y)
        while rclpy.ok() and not move_node.reached:
            rclpy.spin_once(move_node, timeout_sec=0.1)
        move_node.destroy_node()
        self.get_logger().info("픽업 구역 이동 완료")
        time.sleep(1)

        self.get_logger().info("픽업 정밀 제어 시작")
        init_yaw = 180 # degree
        target_yaw = math.radians(init_yaw)
        rotate_node = RotateToYawPID(
            target_yaw=target_yaw,
            mode = None,
            fix_one_side=False
        )
        while rclpy.ok() and not rotate_node.reached:
            rclpy.spin_once(rotate_node, timeout_sec=0.1)
        rotate_node.destroy_node()
        time.sleep(1)

        ## 뒤의 벽으로부터 약 50까지 후진
        back_node = MoveBackword(distance=50)
        while rclpy.ok() and not back_node.reached:
            rclpy.spin_once(back_node, timeout_sec=0.1)
        back_node.destroy_node()  
        self.get_logger().info("픽업 정밀 제어 완료")
        time.sleep(1)
        self.get_logger().info("완료했습니다.")
        return True


    def handle_serving_task(self, table):
        self.get_logger().info(f"SERVING TO TABLE({table})")
        tables = {
            1: (34.00, 67.00),
            2: (34.00, 56.50),
            3: (38.00, 67.00),
            4: (38.00, 56.50)
        }
        self.get_logger().info("테이블 이동 시작")
        map_x, map_y = tables[table][0], tables[table][1]
        goal_x, goal_y = pixel_to_real_coordinates(map_x, map_y)
        move_node = MoveToGoal(f"TABLE{table}", goal_x, goal_y)
        while rclpy.ok() and not move_node.reached:
            rclpy.spin_once(move_node, timeout_sec=0.1)
        move_node.destroy_node()
        self.get_logger().info("테이블 이동 완료")
        time.sleep(1)

        self.get_logger().info("서빙 정밀 제어 시작")
        if table == 1 or table == 3:
            init_yaw = -90
        else:
            init_yaw = 90
        target_yaw = math.radians(init_yaw)
        rotate_node = RotateToYawPID(
            target_yaw=target_yaw,
            mode = None,
            fix_one_side=False
        )
        while rclpy.ok() and not rotate_node.reached:
            rclpy.spin_once(rotate_node, timeout_sec=0.1)
        rotate_node.destroy_node()
        time.sleep(1)

        ## 테이블 벽과 거리까지 전진
        distance = 20   # cm
        move_node = MoveForword(distance)
        while rclpy.ok() and not move_node.reached:
            rclpy.spin_once(move_node, timeout_sec=1)
        move_node.destroy_node()
        time.sleep(1)

        if table == 1 or table == 2:
            last_yaw = 0
        else:
            last_yaw = 180
        target_yaw = math.radians(last_yaw)
        rotate_node = RotateToYawPID(
            target_yaw=target_yaw,
            mode = None,
            fix_one_side=False
        )
        while rclpy.ok() and not rotate_node.reached:
            rclpy.spin_once(rotate_node, timeout_sec=0.1)
        rotate_node.destroy_node()
        self.get_logger().info("서빙 정밀 제어 완료")
        self.get_logger().info("손님이 음식 가져가기 대기 중")
        time.sleep(10)
        self.get_logger().info("완료했습니다.")
        return True

    def handle_birthday_task(self, table):
        self.get_logger().info(f"BIRTHDAY TO TABLE({table})")
        tables = {
            1: (34.00, 67.00),
            2: (34.00, 56.50),
            3: (38.00, 67.00),
            4: (38.00, 56.50)
        }
        self.get_logger().info("테이블 이동 시작")
        map_x, map_y = tables[table][0], tables[table][1]
        goal_x, goal_y = pixel_to_real_coordinates(map_x, map_y)
        move_node = MoveToGoal(f"TABLE{table}", goal_x, goal_y)
        while rclpy.ok() and not move_node.reached:
            rclpy.spin_once(move_node, timeout_sec=0.1)
        move_node.destroy_node()
        self.get_logger().info("테이블 이동 완료")
        time.sleep(1)

        self.get_logger().info("축하 정밀 제어 시작")
        if table == 1 or table == 3:
            init_yaw = -90
        else:
            init_yaw = 90
        target_yaw = math.radians(init_yaw)
        rotate_node = RotateToYawPID(
            target_yaw=target_yaw,
            mode = None,
            fix_one_side=False
        )
        while rclpy.ok() and not rotate_node.reached:
            rclpy.spin_once(rotate_node, timeout_sec=0.1)
        rotate_node.destroy_node()
        time.sleep(1)

        ## 테이블 벽과 거리까지 전진
        distance = 20   # cm
        move_node = MoveForword(distance)
        while rclpy.ok() and not move_node.reached:
            rclpy.spin_once(move_node, timeout_sec=1)
        move_node.destroy_node()
        time.sleep(1)

        if table == 1 or table == 2:
            last_yaw = 0
        else:
            last_yaw = 180
        target_yaw = math.radians(last_yaw)
        rotate_node = RotateToYawPID(
            target_yaw=target_yaw,
            mode = None,
            fix_one_side=False
        )
        while rclpy.ok() and not rotate_node.reached:
            rclpy.spin_once(rotate_node, timeout_sec=0.1)
        rotate_node.destroy_node()
        self.get_logger().info("축하 정밀 제어 완료")
        self.get_logger().info("생일축하 모션 동작 중")
        # TODO: 생일 축하 모션 동작
        time.sleep(10)
        self.get_logger().info("완료했습니다.")
        return True

    def handle_emergency_task(self):
        self.get_logger().info("EMERGENCY")
        emergency_exit = {
            1: (26.00, 56.00),
            2: (26.00, 67.00),
            3: (51.00, 70.00),
        }
        self.get_logger().info("비상구 이동 시작")
        map_x, map_y = emergency_exit[self.robot_id][0], emergency_exit[self.robot_id][1]
        goal_x, goal_y = pixel_to_real_coordinates(map_x, map_y)
        move_node = MoveToGoal("EXIT{self.robot_id}", goal_x, goal_y)
        while rclpy.ok() and not move_node.reached:
            rclpy.spin_once(move_node, timeout_sec=0.1)
        move_node.destroy_node()
        self.get_logger().info("비상구 이동 완료")
        time.sleep(1)

        self.get_logger().info("비상 정밀 제어 시작") # 벽 앞에 서서 안내하기 위해 회전
        if table == 1 or table == 2:
            init_yaw = 0
        else:
            init_yaw = 90
        target_yaw = math.radians(init_yaw)
        rotate_node = RotateToYawPID(
            target_yaw=target_yaw,
            mode = None,
            fix_one_side=False
        )
        while rclpy.ok() and not rotate_node.reached:
            rclpy.spin_once(rotate_node, timeout_sec=0.1)
        rotate_node.destroy_node()
        time.sleep(1)

        self.get_logger().info("비상 정밀 제어 완료") 
        time.sleep(1)
        self.get_logger().info("비상구 안내 중") 
        time.sleep(10)
        self.get_logger().info("완료했습니다.")
        return True


    def handle_maintenance_task(self):
        self.get_logger().info(f"MAINTENANCE")
        self.get_logger().info("정비 구역 이동 시작")
        map_x, map_y = (52.00, 54.50) # maintenance
        goal_x, goal_y = pixel_to_real_coordinates(map_x, map_y)
        move_node = MoveToGoal("MAINTENANCE", goal_x, goal_y)
        while rclpy.ok() and not move_node.reached:
            rclpy.spin_once(move_node, timeout_sec=0.1)
        move_node.destroy_node()
        self.get_logger().info("정비 구역 이동 완료")
        time.sleep(1)

        self.get_logger().info("정비 구역 정밀 제어 시작")
        init_yaw = -90 # degree
        target_yaw = math.radians(init_yaw)
        rotate_node = RotateToYawPID(
            target_yaw=target_yaw,
            mode = None,
            fix_one_side=False
        )
        while rclpy.ok() and not rotate_node.reached:
            rclpy.spin_once(rotate_node, timeout_sec=0.1)
        rotate_node.destroy_node()
        self.get_logger().info("정비 구역 정밀 제어 완료")
        time.sleep(1)

        ## 뒤의 벽으로부터 약 50까지 후진
        back_node = MoveBackword(distance=50)
        while rclpy.ok() and not back_node.reached:
            rclpy.spin_once(back_node, timeout_sec=0.1)
        back_node.destroy_node()  
        self.get_logger().info("픽업 정밀 제어 완료")
        time.sleep(1)
        self.get_logger().info("정비 중") 
        time.sleep(10)
        self.get_logger().info("완료했습니다.")
        return True

    def handle_cleaning_task(self):
        self.get_logger().info(f"CLEANING")
        cleaning_area = {
            1: (27.00, 56.00),
            2: (37.00, 56.00),
            3: (47.00, 56.00),
        }
        self.get_logger().info("청소 구역 이동 시작")
        map_x, map_y = cleaning_area[self.robot_id][0], cleaning_area[self.robot_id][1]
        goal_x, goal_y = pixel_to_real_coordinates(map_x, map_y)
        move_node = MoveToGoal("CLEAN{self.robot_id}", goal_x, goal_y)
        while rclpy.ok() and not move_node.reached:
            rclpy.spin_once(move_node, timeout_sec=0.1)
        move_node.destroy_node()
        self.get_logger().info("청소 구역 이동 완료")
        time.sleep(1)

        self.get_logger().info("청소 정밀 제어 시작") # ㄹ자로 청소 모션
        init_yaw = -90
        target_yaw = math.radians(init_yaw)
        rotate_node = RotateToYawPID(
            target_yaw=target_yaw,
            mode = None,
            fix_one_side=False
        )
        while rclpy.ok() and not rotate_node.reached:
            rclpy.spin_once(rotate_node, timeout_sec=0.1)
        rotate_node.destroy_node()
        self.get_logger().info("청소 정밀 제어 종료") # ㄹ자로 청소 구역 이동
        time.sleep(1)
        self.get_logger().info("청소 중")
        # 전진 -> 좌회전 -> 전진 -> 좌회전 -> 전진 -> 우회전 -> 전진 -> 우회전 <= 과정 반복
        time.sleep(10)
        self.get_logger().info("완료했습니다.")
        return True

    def handle_security_task(self):
        self.get_logger().info(f"SECURITY")
        security_area = {
            1: (39.00, 56.00),
            2: (32.00, 67.00),
            3: (46.00, 67.00),
        }
        self.get_logger().info("경비 구역 이동 시작")
        map_x, map_y = security_area[self.robot_id][0], security_area[self.robot_id][1]
        goal_x, goal_y = pixel_to_real_coordinates(map_x, map_y)
        move_node = MoveToGoal("SECURITY{self.robot_id}", goal_x, goal_y)
        while rclpy.ok() and not move_node.reached:
            rclpy.spin_once(move_node, timeout_sec=0.1)
        move_node.destroy_node()
        self.get_logger().info("경비 구역 이동 완료")
        time.sleep(1)

        self.get_logger().info("경비 정밀 제어 시작") # 삼각 편대로 위치 해서 < ^ > 경비
        if self.robot_id == 1:
            init_yaw = 90
        elif self.robot_id == 2:
            init_yaw = 180
        elif self.robot_id == 3:
            init_yaw = 0
        target_yaw = math.radians(init_yaw)
        rotate_node = RotateToYawPID(
            target_yaw=target_yaw,
            mode = None,
            fix_one_side=False
        )
        while rclpy.ok() and not rotate_node.reached:
            rclpy.spin_once(rotate_node, timeout_sec=0.1)
        rotate_node.destroy_node()
        self.get_logger().info("경비 정밀 제어 종료") # ㄹ자로 청소 구역 이동
        time.sleep(1)
        self.get_logger().info("경비 중")
        time.sleep(10)
        self.get_logger().info("완료했습니다.")
        return True

    def handle_return_task(self):
        self.get_logger().info(f"MOVE TO WAITING LOCATION")
        waiting_area = {
            1: (39.00, 56.00),
            2: (32.00, 67.00),
            3: (46.00, 67.00),
        }
        self.get_logger().info("복귀 시작")
        map_x, map_y = waiting_area[self.robot_id][0], waiting_area[self.robot_id][1]
        goal_x, goal_y = pixel_to_real_coordinates(map_x, map_y)
        move_node = MoveToGoal("RETURN{self.robot_id}", goal_x, goal_y)
        while rclpy.ok() and not move_node.reached:
            rclpy.spin_once(move_node, timeout_sec=0.1)
        move_node.destroy_node()
        self.get_logger().info("복귀 구역 이동 완료")
        time.sleep(1)

        self.get_logger().info("정밀 제어 시작") # 테이블을 향해서 위치
        if self.robot_id == 1:
            init_yaw = 90
        elif self.robot_id == 2:
            init_yaw = -90
        elif self.robot_id == 3:
            init_yaw = -90
        target_yaw = math.radians(init_yaw)
        rotate_node = RotateToYawPID(
            target_yaw=target_yaw,
            mode = None,
            fix_one_side=False
        )
        while rclpy.ok() and not rotate_node.reached:
            rclpy.spin_once(rotate_node, timeout_sec=0.1)
        rotate_node.destroy_node()
        self.get_logger().info("정밀 제어 종료")
        time.sleep(1)
        self.get_logger().info("대기 중")
        return True

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
            elif command == 'CLEAN':
                param1 = None
                param2 = None
                command_data = {'command': command, 'param1': param1, 'param2': param2}

            else:
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



    def pixel_to_real_coordinates(self, pixel_x=None, pixel_y=None):
        width = 99
        height = 100
        origin_x = -1.26
        origin_y = -2.13
        resolution = 0.05

        pixel_x = int(pixel_x)
        pixel_y = int(pixel_y)

        # x_real 및 y_real 계산
        x_real = origin_x + (pixel_x * resolution)
        y_real = origin_y + ((height - pixel_y - 1) * resolution)

        return x_real, y_real

    def is_within_map_bounds(self, x, y):
        # 범위 정하기
        min_x, min_y = 21, 51
        max_x, max_y = 62, 75

        # 조건문을 통해 좌표가 범위 내인지 확인
        if min_x <= x <= max_x and min_y <= y <= max_y:
            return True
        else:
            self.get_logger().error(f'❌ 잘못된 좌표 : x: {x}, y: {y} ( x: {min_x} ~ {max_x}, y: {min_y} ~ {max_y} )')
            return False

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
