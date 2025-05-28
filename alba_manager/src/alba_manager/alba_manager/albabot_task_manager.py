import rclpy
from rclpy.node import Node
import math  # math 모듈을 임포트합니다.
from alba_manager.mapping import *
import requests
from datetime import datetime, timezone
from alba_msgs.msg import AlbabotCoordinate
from std_msgs.msg import String
import json
from enum import Enum
import time
from alba_manager.path_planner import AStar

class AlbaBotStatus(str, Enum):
    IDLE = "IDLE"
    SETTING = "SETTING"
    COOKING = "COOKING"
    PICKUP = "PICKUP"
    SERVING = "SERVING"
    CLEANING = "CLEANING"
    EMERGENCY = "EMERGENCY"
    MAINTENANCE = "MAINTENANCE"
    SECURITY = "SECURITY"
    BIRTHDAY = "BIRTHDAY"
    CALLING = "CALLING"
    ERROR = "ERROR"

class CommandStatus(str, Enum):
    PENDING = "PENDING"
    SENT = "SENT"
    ACKED = "ACKED"
    EXECUTED = "EXECUTED"
    FAILED = "FAILED"
    COMPLETED = "COMPLETED" # only albabot internal use


pickup_x, pickup_y = 51.70, 61.00
serving_tables = {
    1: (33.50, 67.00),
    2: (33.50, 54.00),
    3: (38.00, 67.00),
    4: (39.50, 54.00)
    }
birthday_tables = {
    1: (29.50, 67.00),
    2: (29.50, 54.00),
    3: (44.00, 67.00),
    4: (44.00, 54.00)
    }
emergency_exit = {
    1: (26.00, 56.00),
    2: (38.00, 56.00),
    3: (38.00, 68.00),
    }
security_area = {
    1: (39.00, 55.00),
    2: (32.00, 66.00),
    3: (47.00, 66.00),
    }
cleaning_area = {
    1: (27.00, 55.00), # -> 46, 55
    2: (49.00, 61.00), # -> 26, 62
    3: (27.00, 67.00), # -> 46, 69
}
cleaning_area2 = {
    1: (49.00, 57.00),
    2: (27.00, 62.00),
    3: (49.00, 68.00),
}
cleaning_area3 = {
    1: (27.00, 59.00),
    2: (49.00, 63.00),
    3: (27.00, 69.00),
}
cleaning_deg = {
    1: (0.00, 180.00),
    2: (180.00, 0.00),
    3: (0.00, 180.00),
}

maintenance = (23.00, 64.00)
temp_deg = 0.0
pickup_deg = 180.0
serving_deg = -90.0
serving_deg2 = 90.0
table_deg = 0.0


class AlbaBotTaskManager(Node):

    def __init__(self, path_planning=False):
        super().__init__('albabot_task_manager')

        self.robot_ids = [1, 2, 3]
        self.robot_pos = {robot_id: (0.0, 0.0) for robot_id in self.robot_ids}
        self.robot_pos[1] = (47, 65)
        self.robot_pos[2] = (36, 54)
        self.robot_pos[3] = (24, 60)
        self.path_planning = path_planning

        self.robot_status = {
            1: {"domain": 74, "command_id": 0, "status": "IDLE", "command": None, "command_status": None},
            2: {"domain": 62, "command_id": 0, "status": "IDLE", "command": None, "command_status": None},
            3: {"domain": 58, "command_id": 0, "status": "IDLE", "command": None, "command_status": None},
        }

        self.domain_to_robot_id = {
            74: 1,
            62: 2,
            58: 3
        }
        self.robot_to_domain_id = {
            1: 74,
            2: 62,
            3: 58
        }

        self.subscription = self.create_subscription(
                String,
                '/command',
                self.response_callback,
                10)

        self.create_subscription(
            AlbabotCoordinate,
            '/albabot_pos',
            self.albabot_pos_callback,
            10
        )

        self.timer = self.create_timer(1.0, self.timer_callback)

        self.publisher_ = self.create_publisher(String, '/robot_command', 10)

        if self.path_planning:
            self.path_planner = AStar(False)
            self.path_planner.initialize_map()

    def albabot_pos_callback(self, msg: AlbabotCoordinate):
        # 입력된 값이 robot_ids에 없는 경우
        if msg.robot_id not in self.robot_ids:
            return

        robot_id = msg.robot_id
        self.robot_pos[robot_id] = (msg.world_pose.position.x, msg.world_pose.position.y)

    def timer_callback(self):
        idle_albabots = self.check_idle_albabot()
        if idle_albabots:
            self.get_one_work_and_publish(idle_albabots)
        else:
            self.get_logger().warn("모든 알바봇이 작업 중입니다.")
        self.print_robot_status()

    def response_callback(self, msg):
        #self.get_logger().info(f'Received message: {msg.data}')
        try:
            # 수신한 JSON 문자열을 파이썬 객체로 변환
            json_data = json.loads(msg.data)
            self.get_logger().info(f'Parsed JSON: {json_data}')
            domain_id = json_data['domain_id']
            command = json_data['command']
            command_status = json_data['command_status']
            robot_id = self.domain_to_robot_id.get(domain_id)
            if robot_id is not None:
                if command_status == "COMPLETED":
                    self.clear_robot_status(robot_id)
                elif command_status == "EXECUTED":
                    if command == "SERVING":
                        order_id = json_data['order_id']
                        self.update_order_status(order_id, "SERVED")
                    self.set_command_status(robot_id, command_status)
                    command_id = self.robot_status[robot_id]["command_id"]
                    self.update_albabot_cmd(robot_id, command_id, command, command_status)
                    if self.path_planning:
                        self.path_planner.release_assigned_path(robot_id)
                else:
                    self.set_command_status(robot_id, command_status)
                #self.print_robot_status()


        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to decode JSON: {e}')

    def print_robot_status(self):
        for robot_id, status in self.robot_status.items():
            print(f'    AlbaBot[{robot_id}] Status: {status}')

    def set_command_status(self, robot_id, value):
        self.robot_status[robot_id]["command_status"] = value

    def get_command_status(self, robot_id):
        return self.robot_status[robot_id]["command_status"]  # 현재 상태를 반환

    def set_status(self, robot_id, value):
        self.robot_status[robot_id]["status"] = value

    def get_status(self, robot_id):
        return self.robot_status[robot_id]["status"]  # 현재 상태를 반환

    def clear_robot_status(self, robot_id):
        domain_id = self.robot_status[robot_id]["domain"]
        self.robot_status[robot_id] = {
            "domain": domain_id,
            "command_id": 0,
            "status": "IDLE",
            "command": None,
            "command_status": None
        }

    def update_albabot_cmd(self, robot_id, command_id, command, status):
        # 요청할 URL 설정
        url = f"http://192.168.0.156:8000/api/robots/commands/{command_id}"

        # 현재 시간 가져오기
        current_time = datetime.now()
        # 현재 시간을 ISO 8601 형식으로 변환
        timestamp = current_time.isoformat(timespec='microseconds')


        try:
            if robot_id is None:
                self.get_logger().warn("✅ robot_id가 None입니다. 명령을 처리할 수 없습니다.")
                robot_id = 1
                #return  # robot_id가 None일 경우 함수를 종료

        except Exception as e:
            self.get_logger().warn(f":경고: 명령 업데이트 실패: {e}")

        # 업데이트할 로봇 정보 (예시)
        payload = {
            "robot_id": int(robot_id),
            "command": command,
            "parameters": {},
            "status": status
        }

        """
        ############################################
        # TEST TODO REMOVE
        if command == "SERVING":
            payload["parameters"]["table_id"] = 1
        elif command == "BIRTHDAY":
            payload["parameters"]["table_id"] = 2
        ############################################
        """


        # PUT 요청 보내기
        try:
            response = requests.put(url, json=payload)
            response.raise_for_status()  # HTTP 오류가 발생하면 예외를 발생시킴

            # 요청에 대한 응답 처리
            if response.status_code == 200:
                print("로봇 정보가 성공적으로 업데이트되었습니다.")
                print(response.json())  # 응답 데이터 출력
            else:
                print(f"요청 실패: 상태 코드 {response.status_code}")

        except requests.exceptions.HTTPError as http_err:
            print(f"HTTP 오류 발생: {http_err}")
        except requests.exceptions.ConnectionError:
            print("서버에 연결할 수 없습니다.")
        except requests.exceptions.Timeout:
            print("요청 시간이 초과되었습니다.")
        except requests.exceptions.RequestException as err:
            print(f"요청 중 오류가 발생했습니다: {err}")

    def update_order_status(self, order_id, status):
        # 요청할 URL 설정
        url = f"http://192.168.0.156:8000/api/orders/{order_id}/status"

        # 현재 시간 가져오기
        current_time = datetime.now()
        # 현재 시간을 ISO 8601 형식으로 변환
        timestamp = current_time.isoformat(timespec='microseconds')

        # 업데이트할 로봇 정보 (예시)
        payload = {
            "status": status
        }

        # PUT 요청 보내기
        try:
            response = requests.put(url, json=payload)
            response.raise_for_status()  # HTTP 오류가 발생하면 예외를 발생시킴

            # 요청에 대한 응답 처리
            if response.status_code == 200:
                print("로봇 정보가 성공적으로 업데이트되었습니다.")
                print(response.json())  # 응답 데이터 출력
            else:
                print(f"요청 실패: 상태 코드 {response.status_code}")

        except requests.exceptions.HTTPError as http_err:
            print(f"HTTP 오류 발생: {http_err}")
        except requests.exceptions.ConnectionError:
            print("서버에 연결할 수 없습니다.")
        except requests.exceptions.Timeout:
            print("요청 시간이 초과되었습니다.")
        except requests.exceptions.RequestException as err:
            print(f"요청 중 오류가 발생했습니다: {err}")

    def assign_path(self, robot_id, command, param1, parma2):

        start = self.robot_pos[robot_id]
        if start is None:
            self.get_logger().warn(f"robot[{robot_id}] pos is None")
            return None

        if command == 'PICKUP':
            goal = pickup_area
        elif command == 'SERVING':
            table = param1
            goal = serving_tables[table]
        elif command == 'BIRTHDAY':
            table = param1
            goal = birthday_tables[table]
        elif command == 'EMERGENCY':
            goal = emergency_exit[robot_id]
        elif command == 'MAINTENANCE':
            goal = mainenance_area
        elif command == 'CLEANING':
            goal = cleaning_area[robot_id]
        elif command == 'SECURITY':
            goal = security_area[robot_id]
        else:
            self.get_logger().warn(f"Unknown command {command}")
            return None

        start = int(start[0]), int(start[1])
        goal = int(goal[0]), int(goal[1])

        if self.path_planning:
            path = self.path_planner.assign_path_for_robot(robot_id, start, goal)
        else:
            path = None
        return path

    def publish_command(self, robot_id, domain_id, command, param1, param2, path):
        # 메시지 딕셔너리 구성
        msg_dict = {
            "domain_id": domain_id,
            "command": command,
            "param1": param1,
            "param2": param2,
            "path": path.tolist() if path is not None else None
        }

        # JSON 문자열로 변환
        msg_str = json.dumps(msg_dict)

        # ROS 메시지 생성 및 데이터 설정
        ros_msg = String()
        ros_msg.data = msg_str

        # 명령 상태 업데이트
        self.set_command_status(robot_id, CommandStatus.SENT.value)
        self.update_albabot_cmd(robot_id, self.robot_status[robot_id]["command_id"], command, CommandStatus.SENT.value)

        # 로그 출력 및 메시지 발행
        self.get_logger().info(f"Published: {msg_str}")
        self.publisher_.publish(ros_msg)

    def get_one_work_and_publish(self, robot_ids):
        try:
            url = f"http://192.168.0.156:8000/api/robots/commands"
            response = requests.get(url)
            if response.status_code == 200:
                data = response.json()
                pending_list = [
                        item for item in data
                        if item['status'] == 'PENDING' and item['command'] not in ['MENU', 'HELP', 'PAYMENT']
                    ]

                if pending_list:
                    sorted_pending_list = sorted(pending_list, key=lambda x: x['id'])
                    while sorted_pending_list:
                        work_item = sorted_pending_list[0]
                        print("======", work_item)
                        command = work_item['command']
                        robot_id = robot_ids[0]
                        domain_id = self.robot_status[robot_id]["domain"]
                        param1 = param2 = 0
                        if command == "PICKUP":
                            self.robot_status[robot_id]["command_id"] = work_item['id']
                            self.set_status(robot_id, work_item['command'])
                            if self.path_planning:
                                path = self.assign_path(robot_id, command, param1, param2)
                            else:
                                path = None
                            self.publish_command(robot_id, domain_id, command, param1, param2, path)

                        elif command == "SERVING":
                            robot_id = None
                            for id in robot_ids:
                                if self.robot_status[id]["status"] == "PICKUP":
                                    robot_id = id
                                    break
                            if robot_id == None:  # 기본 robot_id가 변하지 않았다면 (PICKUP 중인 로봇이 없다면)
                                #self.get_logger().warn("✅ PICKUP 완료된 로봇이 없습니다. 다음 커맨드로 진행합니다.")
                                sorted_pending_list.pop(0)
                                if not sorted_pending_list:
                                    #self.get_logger().warn("✅ 더 이상 처리할 수 있는 PENDING 항목이 없습니다. 종료합니다.")
                                    return
                                continue
                            domain_id = self.robot_status[robot_id]["domain"]
                            param1 = work_item.get('parameters', {}).get('table_id', 0)
                            param2 = work_item.get('parameters', {}).get('order_id', 0)
                            self.robot_status[robot_id]["command_id"] = work_item['id']
                            self.set_status(robot_id, work_item['command'])
                            if self.path_planning:
                                path = self.assign_path(robot_id, command, param1, param2)
                            else:
                                path = None
                            self.publish_command(robot_id, domain_id, command, param1, param2, path)

                        elif command == "BIRTHDAY":
                            param1 = work_item.get('parameters', {}).get('table_id', 0)
                            self.robot_status[robot_id]["command_id"] = work_item['id']
                            self.set_status(robot_id, work_item['command'])
                            if self.path_planning:
                                path = self.assign_path(robot_id, command, param1, param2)
                            else:
                                path = None
                            self.publish_command(robot_id, domain_id, command, param1, param2, path)

                        elif command == "MAINTENANCE":
                            self.robot_status[robot_id]["command_id"] = work_item['id']
                            self.set_status(robot_id, work_item['command'])
                            if self.path_planning:
                                path = self.assign_path(robot_id, command, param1, param2)
                            else:
                                path = None
                            self.publish_command(robot_id, domain_id, command, param1, param2, path)

                        elif command == "CLEANING":
                            for robot_id in self.robot_ids:
                                self.robot_status[robot_id]["command_id"] = work_item['id']
                                self.set_status(robot_id, work_item['command'])
                                domain_id = self.robot_to_domain_id.get(robot_id)
                                if self.path_planning:
                                    path = self.assign_path(robot_id, command, param1, param2)
                                else:
                                    path = None
                                self.publish_command(robot_id, domain_id, command, param1, param2, path)

                        elif command == "SECURITY":
                            for robot_id in self.robot_ids:
                                self.robot_status[robot_id]["command_id"] = work_item['id']
                                self.set_status(robot_id, work_item['command'])
                                domain_id = self.robot_to_domain_id.get(robot_id)
                                if self.path_planning:
                                    path = self.assign_path(robot_id, command, param1, param2)
                                else:
                                    path = None
                                self.publish_command(robot_id, domain_id, command, param1, param2, path)

                        break
                #else: self.get_logger().warn("✅ PENDING 항목이 없어 실행을 건너뜁니다.")
            else:
                self.get_logger().warn(f":x: 명령 요청 실패 - 상태 코드 {response.status_code}")
        except requests.exceptions.RequestException as e:
            self.get_logger().warn(f":경고: REST API 통신 실패: {e}")

    def check_idle_albabot(self):
        idle_robots = [
            robot_id for robot_id, status_info in self.robot_status.items()
            if (status_info["status"] == "IDLE" and status_info["command_status"] is None)
            or (status_info["status"] == "PICKUP" and status_info["command_status"] == "EXECUTED")
        ]
        idle_robots = [robot_id for robot_id in idle_robots if (robot_id != 3)]
        return idle_robots


def main(args=None):
    rclpy.init(args=args)
    node = AlbaBotTaskManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

