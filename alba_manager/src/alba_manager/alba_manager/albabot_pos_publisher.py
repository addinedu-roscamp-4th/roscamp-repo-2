import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32, String
from alba_msgs.msg import AlbabotCoordinate
import json
import socket
from alba_manager.tcp_client import AlbaTCPClient

class AlbabotPosPublisher(Node):
    def __init__(self):
        super().__init__('albabot_pos_publisher')
        self.tcp_client = AlbaTCPClient(host='192.168.0.156', port=8001)

        self.robot_ids = [1, 2, 3]
        self.robot_data = {robot_id: {} for robot_id in self.robot_ids}
        #print(self.robot_data)

        # 각 로봇의 구독 생성
        for robot_id in self.robot_ids:
            self.create_subscription(
                Float32,
                f'/albabot_{robot_id}/pinky_battery_present',
                lambda msg, robot_id=robot_id: self.battery_callback(robot_id, msg),
                10
            )
            self.create_subscription(
                PoseStamped,
                f'/albabot_{robot_id}/tracked_pose',
                lambda msg, robot_id=robot_id: self.tracked_pos_callback(robot_id, msg),
                10
            )


        self.create_subscription(
            AlbabotCoordinate,
            '/albabot_pos',
            self.albabot_pos_callback,
            10
        )
        # 0.5초마다 publish_data 호출
        self.timer = self.create_timer(0.5, self.publish_data)

    def battery_callback(self, robot_id, msg: Float32):
        self.robot_data[robot_id]['battery_level'] = msg.data
        #self.get_logger().info(f'Robot {robot_id} battery level updated: {self.robot_data[robot_id]["battery_level"]}')

    def tracked_pos_callback(self, robot_id, msg: PoseStamped):
        self.robot_data[robot_id]['tracked_pos'] = {
            'pinky_x': msg.pose.position.x,
            'pinky_y': msg.pose.position.y,
            'pinky_z': msg.pose.position.z,
            'pinky_roll': msg.pose.orientation.x,
            'pinky_pitch': msg.pose.orientation.y,
            'pinky_yaw': msg.pose.orientation.z,
        }
        #self.get_logger().info(f'Robot {robot_id} tracked_pos updated: {self.robot_data[robot_id]["tracked_pos"]}')

    def albabot_pos_callback(self, msg: AlbabotCoordinate):
        # 입력된 값이 robot_ids에 없는 경우
        if msg.robot_id not in self.robot_ids:
            return

        robot_id = msg.robot_id
        self.robot_data[robot_id]['albabot_pos'] = {
            'global_x': msg.global_pose.position.x,
            'global_y': msg.global_pose.position.y,
            'global_z': msg.global_pose.position.z,
            'global_roll': msg.global_pose.orientation.x,
            'global_pitch': msg.global_pose.orientation.y,
            'global_yaw': msg.global_pose.orientation.z,
            'world_x': msg.world_pose.position.x,
            'world_y': msg.world_pose.position.y,
            'world_z': msg.world_pose.position.z,
            'world_roll': msg.world_pose.orientation.x,
            'world_pitch': msg.world_pose.orientation.y,
            'world_yaw': msg.world_pose.orientation.z,
        }
        #self.get_logger().info(f'Robot {robot_id} albabot_pos updated: {self.robot_data[robot_id]["albabot_pos"]}')

    def publish_data(self):
        for robot_id in sorted(self.robot_data.keys()):
            data_dict = self.robot_data[robot_id]
            # 데이터가 모두 들어왔는지 체크
            if 'tracked_pos' not in data_dict or 'albabot_pos' not in data_dict or 'battery_level' not in data_dict:
                #missing_keys = [key for key in ['tracked_pos', 'albabot_pos', 'battery_level'] if key not in data_dict]

                #if missing_keys:
                    #self.get_logger().error(f"[{robot_id}] Missing keys in data_dict: {', '.join(missing_keys)}")
                continue  # 데이터가 부족하면 skip

            data = {
                'msg_type': "Albabot",
                'robot_id': robot_id,
                'status': "IDLE",
                'battery_level': data_dict['battery_level'],
                # tracked_pos
                'pinky_x': data_dict['tracked_pos']['pinky_x'],
                'pinky_y': data_dict['tracked_pos']['pinky_y'],
                'pinky_z': data_dict['tracked_pos']['pinky_z'],
                'pinky_roll': data_dict['tracked_pos']['pinky_roll'],
                'pinky_pitch': data_dict['tracked_pos']['pinky_pitch'],
                'pinky_yaw': data_dict['tracked_pos']['pinky_yaw'],
                # albabot_pos - global
                'global_x': data_dict['albabot_pos']['global_x'],
                'global_y': data_dict['albabot_pos']['global_y'],
                'global_z': data_dict['albabot_pos']['global_z'],
                'global_roll': data_dict['albabot_pos']['global_roll'],
                'global_pitch': data_dict['albabot_pos']['global_pitch'],
                'global_yaw': data_dict['albabot_pos']['global_yaw'],
                # albabot_pos - world
                'world_x': data_dict['albabot_pos']['world_x'],
                'world_y': data_dict['albabot_pos']['world_y'],
                'world_z': data_dict['albabot_pos']['world_z'],
                'world_roll': data_dict['albabot_pos']['world_roll'],
                'world_pitch': data_dict['albabot_pos']['world_pitch'],
                'world_yaw': data_dict['albabot_pos']['world_yaw'],
            }
            # 데이터 전송
            # Send via TCP
            json_data = json.dumps(data)
            #self.get_logger().info(f'📢 Published: {json_data}')
            try:
                response = self.tcp_client.send_data(data)
                self.get_logger().info(f'✅ 서버 응답: {response}')
            except Exception as e:
                self.get_logger().error(f'❌ TCP 전송 실패: {e}')

            # 전송 후 tracked_pos와 albabot_pos 초기화
            data_dict.pop('tracked_pos', None)
            data_dict.pop('albabot_pos', None)

        """
        aggregated_data = {
            'robot_data': self.robot_data
        }

        # JSON 형식으로 변환
        json_data = json.dumps(aggregated_data)

        # 주요 정보를 게시
        msg = String(data=json_data)
        #self.create_publisher(String, '/aggregated_robot_data', 10).publish(msg)
        self.get_logger().info(f'Published aggregated data: {json_data}')
        """

def main(args=None):
    rclpy.init(args=args)
    albabot_pos_publisher = AlbabotPosPublisher()
    rclpy.spin(albabot_pos_publisher)
    albabot_pos_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
