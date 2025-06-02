import gradio as gr
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Int32
from mycobot_interfaces.msg import MycobotAngles, MycobotCoords  # 메시지 타입 맞게 수정
import time
import threading

# 상태 저장용 전역 변수
status_48 = {"coords_48": "", "angles_48": ""}
status_b4 = {"coords_b4": "", "angles_b4": ""}

class MyCobotTeleop(Node):
    def __init__(self):
        super().__init__('mycobot_teleop_node')

        # Subscriber 생성
        self.create_subscription(MycobotAngles, 'mycobot/angles_real48', self.angles_callback_48, 10)
        self.create_subscription(MycobotCoords, 'mycobot/coords_real48', self.coords_callback_48, 10)
        self.create_subscription(MycobotAngles, 'mycobot/angles_realb4', self.angles_callback_b4, 10)
        self.create_subscription(MycobotCoords, 'mycobot/coords_realb4', self.coords_callback_b4, 10)

        # Publisher 생성
        self.angles_pub_48 = self.create_publisher(MycobotAngles, 'mycobot/angles_targ48', 10)
        self.coords_pub_48 = self.create_publisher(MycobotCoords, 'mycobot/coords_targ48', 10)
        self.gripper_pub_48 = self.create_publisher(Int32, 'mycobot/gripper_targ48', 10)
        self.angles_pub_b4 = self.create_publisher(MycobotAngles, 'mycobot/angles_targb4', 10)
        self.coords_pub_b4 = self.create_publisher(MycobotCoords, 'mycobot/coords_targb4', 10)
        self.gripper_pub_b4 = self.create_publisher(Int32, 'mycobot/gripper_targb4', 10)


    # Subscriber 콜백 함수
    def angles_callback_48(self, msg):
        # print(f"[📩 angles 수신됨] → {[msg.joint_1, msg.joint_2, msg.joint_3, msg.joint_4, msg.joint_5, msg.joint_6]}")
        status_48["angles_48"] = f"[{msg.joint_1:.2f}, {msg.joint_2:.2f}, {msg.joint_3:.2f}, " \
                           f"{msg.joint_4:.2f}, {msg.joint_5:.2f}, {msg.joint_6:.2f}]"

    def coords_callback_48(self, msg):
        #print(f"[📩 coords 수신됨] → {[msg.x, msg.y, msg.z, msg.rx, msg.ry, msg.rz]}")
        status_48["coords_48"] = f"[{msg.x:.2f}, {msg.y:.2f}, {msg.z:.2f}, " \
                           f"{msg.rx:.2f}, {msg.ry:.2f}, {msg.rz:.2f}]"
        
    def angles_callback_b4(self, msg):
        #print(f"[📩 angles 수신됨] → {[msg.joint_1, msg.joint_2, msg.joint_3, msg.joint_4, msg.joint_5, msg.joint_6]}")
        status_b4["angles_b4"] = f"[{msg.joint_1:.2f}, {msg.joint_2:.2f}, {msg.joint_3:.2f}, " \
                           f"{msg.joint_4:.2f}, {msg.joint_5:.2f}, {msg.joint_6:.2f}]"

    def coords_callback_b4(self, msg):
        #print(f"[📩 coords 수신됨] → {[msg.x, msg.y, msg.z, msg.rx, msg.ry, msg.rz]}")
        status_b4["coords_b4"] = f"[{msg.x:.2f}, {msg.y:.2f}, {msg.z:.2f}, " \
                           f"{msg.rx:.2f}, {msg.ry:.2f}, {msg.rz:.2f}]"

    # Publisher 함수
    def send_com_angles_48(self, joint_angles):
        msg = MycobotAngles()
        msg.joint_1, msg.joint_2, msg.joint_3, msg.joint_4, msg.joint_5, msg.joint_6 = joint_angles
        print(f"[📤 angles 전송] → {joint_angles}")  # 추가
        self.angles_pub_48.publish(msg)

    def send_com_coords_48(self, coords):
        msg = MycobotCoords()
        msg.x, msg.y, msg.z, msg.rx, msg.ry, msg.rz = coords
        self.coords_pub_48.publish(msg)

    def send_com_angles_b4(self, joint_angles):
        msg = MycobotAngles()
        msg.joint_1, msg.joint_2, msg.joint_3, msg.joint_4, msg.joint_5, msg.joint_6 = joint_angles
        print(f"[📤 angles 전송] → {joint_angles}")  # 추가
        self.angles_pub_b4.publish(msg)

    def send_com_coords_b4(self, coords):
        msg = MycobotCoords()
        msg.x, msg.y, msg.z, msg.rx, msg.ry, msg.rz = coords
        self.coords_pub_b4.publish(msg)


# ROS 노드 전역 인스턴스
teleop_node = None


def update_status_48():
    return status_48["coords_48"], status_48["angles_48"]
def update_status_b4():
    return status_b4["coords_b4"], status_b4["angles_b4"]


def control_robot_48(j1, j2, j3, j4, j5, j6,
                  x, y, z, rx, ry, rz,
                  move_type, action):

    joint_angles = [float(j1), float(j2), float(j3), float(j4), float(j5), float(j6)]
    coords = list(map(float, [x, y, z, rx, ry, rz]))
    move_mode = 0 if move_type == "LMove (선형)" else 1

    if action == "초기화":
        teleop_node.send_com_angles_48([0, 0, 0, 0, 0, 0])
        teleop_node.send_com_coords_48([0, 0, 0, 0, 0, 0])
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
            teleop_node.send_com_coords_48(coords)
        else:
            print(f"→ send_com_angles: {joint_angles}")
            teleop_node.send_com_angles_48(joint_angles)

    return update_status_48()

def control_robot_b4(j1, j2, j3, j4, j5, j6,
                  x, y, z, rx, ry, rz,
                  move_type, action):

    joint_angles = [float(j1), float(j2), float(j3), float(j4), float(j5), float(j6)]
    coords = list(map(float, [x, y, z, rx, ry, rz]))
    move_mode = 0 if move_type == "LMove (선형)" else 1

    if action == "초기화":
        teleop_node.send_com_angles_b4([0, 0, 0, 0, 0, 0])
        teleop_node.send_com_coords_b4([0, 0, 0, 0, 0, 0])
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
            teleop_node.send_com_coords_b4(coords)
        else:
            print(f"→ send_com_angles: {joint_angles}")
            teleop_node.send_com_angles_b4(joint_angles)

    return update_status_b4()

# Gradio UI
def launch_ui():
    with gr.Blocks(title="MyCobot 듀얼 제어 패널") as demo:
        gr.Markdown("## 🤖 MyCobot 듀얼 제어 패널 (48, B4)")

        with gr.Tab("관절 제어"):
            gr.Markdown("### 🔩 관절 제어")
            joint_sliders_48 = []
            joint_sliders_b4 = []
            for i in range(6):
                with gr.Row():
                    joint_sliders_48.append(gr.Slider(-180, 180, value=0, label=f"Joint {i+1} (48)"))
                    joint_sliders_b4.append(gr.Slider(-180, 180, value=0, label=f"Joint {i+1} (B4)"))

        with gr.Tab("좌표 제어"):
            gr.Markdown("### 📍 좌표 제어")
            coord_labels = ["X", "Y", "Z", "Rx", "Ry", "Rz"]
            coord_sliders_48 = []
            coord_sliders_b4 = []
            for label in coord_labels:
                with gr.Row():
                    coord_sliders_48.append(gr.Slider(-300, 300, value=0, label=f"{label} (48)"))
                    coord_sliders_b4.append(gr.Slider(-300, 300, value=0, label=f"{label} (B4)"))

        move_selector_48 = gr.Radio(["LMove (선형)", "JMove (관절)"], value="LMove (선형)", label="이동 방식")
        action_selector_48 = gr.Radio(["없음", "초기화", "그리퍼 열기", "그리퍼 닫기"], value="없음", label="동작 선택")

        move_selector_b4 = gr.Radio(["LMove (선형)", "JMove (관절)"], value="LMove (선형)", label="이동 방식")
        action_selector_b4 = gr.Radio(["없음", "초기화", "그리퍼 열기", "그리퍼 닫기"], value="없음", label="동작 선택")


        run_button_48 = gr.Button("🟢 48 실행")
        update_button_48 = gr.Button("🔄 48 상태 업데이트")
        run_button_b4 = gr.Button("🟢 b4 실행")
        update_button_b4 = gr.Button("🔄 b4 상태 업데이트")

        coord_out_48 = gr.Textbox(label="📍 48 현재 좌표")
        angle_out_48 = gr.Textbox(label="🔩 48 현재 각도")
        coord_out_b4 = gr.Textbox(label="📍 b4 현재 좌표")
        angle_out_b4 = gr.Textbox(label="🔩 b4 현재 각도")

        # 실행 시 두 로봇 데이터 모두 전달
        run_button_48.click(
            control_robot_48,
            inputs=joint_sliders_48 + coord_sliders_48 + [move_selector_48, action_selector_48],
            outputs=[coord_out_48, angle_out_48]
        )

        update_button_48.click(
            update_status_48,
            outputs=[coord_out_48, angle_out_48]
        )
        run_button_b4.click(
            control_robot_b4,
            inputs=joint_sliders_b4 + coord_sliders_b4 + [move_selector_b4, action_selector_b4],
            outputs=[coord_out_b4, angle_out_b4]
        )

        update_button_b4.click(
            update_status_b4,
            outputs=[coord_out_b4, angle_out_b4]
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