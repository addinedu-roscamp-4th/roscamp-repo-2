import os
import multiprocessing as mp
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import json


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
        
        #self.publisher = self.create_publisher(PoseStamped, '/move_goal', qos)
        self.publisher = self.create_publisher(PoseStamped, '/goal_pose', qos)
        self.timer = self.create_timer(0.1, self.process_queue)
        self.pub_command = self.create_publisher(String, '/command', qos)
        # RobotMoverNode.__init__ 내부 마지막 줄에 추가



    def process_queue(self):
        try:
            if not self.pose_queue.empty():
                command_data = self.pose_queue.get_nowait()
                command = command_data['command']

                command_msg = {
                    'domain_id': self.domain_id,
                    'command': command,
                }

                if command == 'move':
                    msg = PoseStamped()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = 'map'
                    command_msg['param1'] = command_data['param1']
                    command_msg['param2'] = command_data['param2']
                    msg.pose.position.x = command_data['param1']
                    msg.pose.position.y = command_data['param2']
                    msg.pose.orientation.w = 1.0
                    self.publisher.publish(msg)
                    self.get_logger().info(
                        f'🚀 도메인 {self.domain_id} {command} 명령 : X: {command_data["param1"]:.2f}, Y: {command_data["param2"]:.2f}'
                    )
                elif command == 'serving':
                    command_msg['param1'] = command_data['param1']
                    self.get_logger().info(f"Domain {self.domain_id}: Serving to table {command_data['param1']}")
                elif command in ['pickup', 'serving', 'clean', 'birthday']:
                    self.get_logger().info(f'🛠️ 도메인 {self.domain_id} {command} 명령 실행')
                else:
                    self.get_logger().warn(f'⚠️ 알 수 없는 명령: {command}')
                    return
                
                status_msg = String()
                status_msg.data = json.dumps(command_msg)
                self.pub_command.publish(status_msg)

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

            if command == 'move':
                x = float(data['param1'])
                y = float(data['param2'])

                if not self.is_within_map_bounds(x, y):
                    return
                x, y = self.pixel_to_real_coordinates(x, y)
                param1, param2 = x, y
                command_data = {'command': command, 'param1': param1, 'param2': param2}
            elif command == 'serving':
                table = int(data['param1'])
                if not self.is_existing_table(table):
                    return
                param1 = table
                param2 = None
                command_data = {'command': command, 'param1': param1}
            else:
                param1, param2 = None, None 
                command_data = {'command': command}

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
            self.get_logger().error(f'⚠️ 명령 처리 오류: {e}')
                
        except ValueError:
            self.get_logger().error('⚠️ 명령 형식 오류: "domain_id command param1 param2" 입력 필요')

        
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

