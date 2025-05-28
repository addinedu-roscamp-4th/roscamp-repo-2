import os
import multiprocessing as mp
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import json
from alba_manager.move_controller import *
from alba_manager.mapping import *
import math
import time

# --------- 로봇 제어 노드 (각 도메인 별 프로세스) ---------
class RobotMoverNode(Node):
    def __init__(self, domain_id, pose_queue):
        super().__init__(f'robot_mover_{domain_id}')
        self.domain_id = domain_id
        self.pose_queue = pose_queue

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
        self.get_logger().info("픽업존 이동 시작")
        time.sleep(3)
        self.get_logger().info("픽업존 이동 완료")
        time.sleep(3)
        self.get_logger().info("픽업 정밀 제어 시작")
        time.sleep(3)
        self.get_logger().info("픽업 정밀 제어 완료")
        time.sleep(3)
        self.get_logger().info("완료했습니다.")
        return True


    def handle_serving_task(self, table):
        self.get_logger().info(f"SERVING TO TABLE({table})")
        self.get_logger().info("테이블 이동 시작")
        time.sleep(3)
        self.get_logger().info("테이블 이동 완료")
        time.sleep(3)
        self.get_logger().info("서빙 정밀 제어 시작")
        time.sleep(3)
        self.get_logger().info("서빙 정밀 제어 완료")
        time.sleep(3)
        self.get_logger().info("완료했습니다.")
        return True

    def handle_birthday_task(self, table):
        self.get_logger().info(f"BIRTHDAY TO TABLE({table})")
        self.get_logger().info("테이블 이동 시작")
        time.sleep(3)
        self.get_logger().info("테이블 이동 완료")
        time.sleep(3)
        self.get_logger().info("정밀 제어 시작") # 테이블을 향해 회전
        time.sleep(3)
        self.get_logger().info("정밀 제어 완료")
        time.sleep(3)
        self.get_logger().info("완료했습니다.")
        return True

    def handle_emergency_task(self):
        self.get_logger().info(f"EMERGENCY")
        self.get_logger().info("비상구 이동 시작")
        time.sleep(3)
        self.get_logger().info("비상구 이동 완료")
        time.sleep(3)
        self.get_logger().info("정밀 제어 시작") # 벽 앞에 서서 안내하기 위해 회전
        time.sleep(3)
        self.get_logger().info("정밀 제어 완료")
        time.sleep(3)
        self.get_logger().info("완료했습니다.")
        return True

    def handle_maintenance_task(self):
        self.get_logger().info(f"MAINTENANCE")
        self.get_logger().info("비상구 이동 시작")
        time.sleep(3)
        self.get_logger().info("비상구 이동 완료")
        time.sleep(3)
        self.get_logger().info("정밀 제어 시작") # 벽 앞에 서서 안내하기 위해 회전
        time.sleep(3)
        self.get_logger().info("정밀 제어 완료")
        time.sleep(3)
        self.get_logger().info("완료했습니다.")
        return True

    def handle_cleaning_task(self):
        self.get_logger().info(f"CLEANING")
        self.get_logger().info("청소구역 이동 시작")
        time.sleep(3)
        self.get_logger().info("청소구역 이동 완료")
        time.sleep(3)
        self.get_logger().info("청소 제어 시작") # 청소 모션 플래닝
        time.sleep(3)
        self.get_logger().info("청소 제어 완료")
        time.sleep(3)
        self.get_logger().info("완료했습니다.")
        return True

    def handle_security_task(self):
        self.get_logger().info(f"SECURITY")
        self.get_logger().info("청소구역 이동 시작")
        time.sleep(3)
        self.get_logger().info("청소구역 이동 완료")
        time.sleep(3)
        self.get_logger().info("청소 제어 시작") # 청소 모션 플래닝
        time.sleep(3)
        self.get_logger().info("청소 제어 완료")
        time.sleep(3)
        self.get_logger().info("완료했습니다.")
        return True

    def handle_waiting_task(self):
        self.get_logger().info(f"MOVE TO WAITING LOCATION")
        self.get_logger().info("복귀 시작")
        time.sleep(3)
        self.get_logger().info("복귀 완료")
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
                    self.handle_waiting_task()
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

    def simplify_path(self, path, angle_threshold_deg=30):
        """
        path: [[x0, y0], [x1, y1], ..., [xn, yn]] (리스트 또는 numpy array)
        angle_threshold_deg: 각도 임계값(도 단위)
        """
        if len(path) < 3:
            return path

        angle_threshold_rad = np.deg2rad(angle_threshold_deg)
        simplified = [path[0]]

        for i in range(1, len(path)-1):
            prev = np.array(path[i-1])
            curr = np.array(path[i])
            next = np.array(path[i+1])

            v1 = curr - prev
            v2 = next - curr

            # 벡터 정규화
            if np.linalg.norm(v1) == 0 or np.linalg.norm(v2) == 0:
                continue
            v1 = v1 / np.linalg.norm(v1)
            v2 = v2 / np.linalg.norm(v2)

            # 두 벡터 사이 각도 계산
            dot = np.clip(np.dot(v1, v2), -1.0, 1.0)
            angle = np.arccos(dot)

            if angle >= angle_threshold_rad:
                simplified.append(path[i])

        simplified.append(path[-1])
        return np.array(simplified).tolist()

    def command_callback(self, msg):
        try:
            print("\n\n")
            self.get_logger().info('=============== new command updated ===============')
            data = json.loads(msg.data)
            domain_id = int(data['domain_id'])
            command = data['command']
            path = data['path']

            """
            if path:
                for idx, (x, y) in enumerate(path):
                    self.get_logger().info(f'Point {idx}: x={x}, y={y}')
            """

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
        #self.get_logger().info(f'📊 로봇 상태 요약: {json.dumps(self.robot_status, indent=2)}')


    def is_within_map_bounds(self, x, y):
        # 범위 정하기
        min_x, min_y = 23, 51
        max_x, max_y = 62, 72

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
