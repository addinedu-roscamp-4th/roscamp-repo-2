#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import signal
import os
import time
import json


def stop_process_safely(process, logger=None):
    if process is not None:
        if logger:
            logger.info("Stopping running process:...")
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()


def kill_matching_processes(keywords, logger=None):
    try:
        output = subprocess.check_output(["ps", "-eo", "pid,cmd"], text=True)
        for line in output.splitlines():
            for keyword in keywords:
                if keyword in line and "python3" in line:
                    pid = int(line.strip().split()[0])
                    if logger:
                        logger.info(f"Killing leftover process [{pid}]: {line.strip()}")
                    os.kill(pid, signal.SIGINT)
    except Exception as e:
        if logger:
            logger.error(f"Error killing processes: {e}")


class AvoidanceMode(Node):
    def __init__(self):
        super().__init__('avoidance_mode_dispatcher')
        self.subscription = self.create_subscription(
            String,
            '/discriminated_obstacle',
            self.mode_callback,
            10
        )
        self.current_mode = None
        self.running_process = None
        self.get_logger().info("Avoidance Mode Dispatcher Node Started")

    def mode_callback(self, msg):

        data = json.loads(msg.data)

        robot_id = data.get('robot_id')
        new_mode = data.get('type')

        if robot_id != 'AlbaBot_3':
            return

        if new_mode == self.current_mode:
            self.get_logger().info(f"Same mode '{new_mode}', ignoring...")
            return



        self.get_logger().info(f"Switching mode to: {new_mode}")

        # 현재 프로세스 종료
        stop_process_safely(self.running_process, self.get_logger())
        self.running_process = None
        time.sleep(0.5)


        # 새로운 모드 실행
        if new_mode == "static":
            self.get_logger().info("Launching static node")
            self.running_process = subprocess.Popen([
                "python3", "/home/addinedu/workspace/roscamp-repo-2/alba_manager/src/alba_manager/alba_manager/obstacle_handler/obstacle_static.py"
            ])
            # rclpy.sleep(5)
            #time.sleep(3) 
        elif new_mode == "dynamic":
            self.get_logger().info("Launching dynamic node")
            self.running_process = subprocess.Popen([
                "python3", "/home/addinedu/workspace/roscamp-repo-2/alba_manager/src/alba_manager/alba_manager/obstacle_handler/obstacle_dynamic.py"
            ])
        elif new_mode == "None":
            self.get_logger().info("Received 'clear' mode. Stopping current node...")
            stop_process_safely(self.running_process, self.get_logger())
            self.running_process = None
            return
        else:
            self.get_logger().warn(f"Unknown mode: {new_mode}")
            self.running_process = None

        self.current_mode = new_mode


def main(args=None):
    rclpy.init(args=args)
    node = AvoidanceMode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop_process_safely(node.running_process, node.get_logger())
        kill_matching_processes(
            keywords=["obstacle_dynamic.py", "obstacle_static.py"],
            logger=node.get_logger()
        )
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()