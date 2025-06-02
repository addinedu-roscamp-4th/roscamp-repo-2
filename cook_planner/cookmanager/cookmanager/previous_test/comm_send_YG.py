import gradio as gr
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Int32
from mycobot_interfaces.msg import MycobotAngles, MycobotCoords  # 메시지 타입 맞게 수정
import time
import threading

# 상태 저장용 전역 변수
status = {"coords": "", "angles": ""}

class MyCobotTeleop(Node):
    def __init__(self):
        super().__init__('mycobot_teleop_node')

        # Subscriber 생성
        self.create_subscription(MycobotAngles, 'mycobot/angles_real48', self.angles_callback, 10)
        self.create_subscription(MycobotCoords, 'mycobot/coords_real48', self.coords_callback, 10)

        # Publisher 생성
        self.angles_pub = self.create_publisher(MycobotAngles, 'mycobot/angles_targ48', 10)
        self.coords_pub = self.create_publisher(MycobotCoords, 'mycobot/coords_targ48', 10)
        self.gripper_pub = self.create_publisher(Int32, 'mycobot/gripper_targ48', 10)


    # Subscriber 콜백 함수
    def angles_callback(self, msg):
        #print(f"[📩 angles 수신됨] → {[msg.joint_1, msg.joint_2, msg.joint_3, msg.joint_4, msg.joint_5, msg.joint_6]}")
        status["angles"] = f"[{msg.joint_1:.2f}, {msg.joint_2:.2f}, {msg.joint_3:.2f}, " \
                           f"{msg.joint_4:.2f}, {msg.joint_5:.2f}, {msg.joint_6:.2f}]"

    def coords_callback(self, msg):
        #print(f"[📩 coords 수신됨] → {[msg.x, msg.y, msg.z, msg.rx, msg.ry, msg.rz]}")
        status["coords"] = f"[{msg.x:.2f}, {msg.y:.2f}, {msg.z:.2f}, " \
                           f"{msg.rx:.2f}, {msg.ry:.2f}, {msg.rz:.2f}]"

    # Publisher 함수
    def send_com_angles(self, joint_angles):
        msg = MycobotAngles()
        msg.joint_1, msg.joint_2, msg.joint_3, msg.joint_4, msg.joint_5, msg.joint_6 = joint_angles
        print(f"[📤 angles 전송] → {joint_angles}")  # 추가
        self.angles_pub.publish(msg)

    def send_com_coords(self, coords):
        msg = MycobotCoords()
        msg.x, msg.y, msg.z, msg.rx, msg.ry, msg.rz = coords
        self.coords_pub.publish(msg)


# ROS 노드 전역 인스턴스
teleop_node = None


def update_status():
    return status["coords"], status["angles"]


def control_robot(j1, j2, j3, j4, j5, j6,
                  x, y, z, rx, ry, rz,
                  move_type, action):

    joint_angles = [float(j1), float(j2), float(j3), float(j4), float(j5), float(j6)]
    coords = list(map(float, [x, y, z, rx, ry, rz]))
    move_mode = 0 if move_type == "LMove (선형)" else 1

    if action == "초기화":
        teleop_node.send_com_angles([0, 0, 0, 0, 0, 0])
        teleop_node.send_com_coords([0, 0, 0, 0, 0, 0])
        teleop_node.send_gripper(100)
    elif action == "그리퍼 열기":
        # 그리퍼 퍼블리셔 만들었다면 여기서 publish
        teleop_node.send_gripper(100)
        pass
    elif action == "그리퍼 닫기":
        # 그리퍼 퍼블리셔 만들었다면 여기서 publish
        teleop_node.send_gripper(0)
        pass
    else:
        if move_mode == 0:
            print(f"→ send_com_coords: {coords}")
            teleop_node.send_com_coords(coords)
        else:
            print(f"→ send_com_angles: {joint_angles}")
            teleop_node.send_com_angles(joint_angles)

    return update_status()

# Gradio UI
def launch_ui():
    with gr.Blocks(title="MyCobot 웹 제어") as demo:
        gr.Markdown("## 🤖 MyCobot 실시간 제어 패널")

        with gr.Tab("관절 제어"):
            joint_sliders = [
                gr.Slider(-180, 180, value=0, label=f"Joint {i+1}") for i in range(6)
            ]

        with gr.Tab("좌표 제어"):
            coord_sliders = [
                gr.Slider(-300, 300, value=0, label=label) for label in ["X", "Y", "Z", "Rx", "Ry", "Rz"]
            ]

        move_selector = gr.Radio(["LMove (선형)", "JMove (관절)"], value="LMove (선형)", label="이동 방식")
        action_selector = gr.Radio(["없음", "초기화", "그리퍼 열기", "그리퍼 닫기"], value="없음", label="동작 선택")

        run_button = gr.Button("🟢 실행")
        update_button = gr.Button("🔄 상태 업데이트")

        coord_out = gr.Textbox(label="📍 현재 좌표")
        angle_out = gr.Textbox(label="🔩 현재 각도")

        run_button.click(
            control_robot,
            inputs=joint_sliders + coord_sliders + [move_selector, action_selector],
            outputs=[coord_out, angle_out]
        )

        update_button.click(
            update_status,
            outputs=[coord_out, angle_out]
        )

    demo.launch(inbrowser=True)  # ✅ 브라우저 자동 열기 설정

# ROS2 노드용 스레드
def ros_spin_thread():
    rclpy.spin(teleop_node)


# main 함수
def main():
    global teleop_node

    rclpy.init()
    teleop_node = MyCobotTeleop()

    ros_thread = threading.Thread(target=ros_spin_thread, daemon=True)
    ros_thread.start()

    launch_ui()

    # 종료 처리
    teleop_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()