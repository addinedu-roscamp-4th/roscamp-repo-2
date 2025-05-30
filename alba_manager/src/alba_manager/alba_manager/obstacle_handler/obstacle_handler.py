#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import signal
import os
import time
import json
import threading
from obstacle_static import *
from obstacle_dynamic import *


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
        self.running_thread = None
        self.stop_event = threading.Event() 
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


        # 이전 스레드 종료
        if self.running_thread and self.running_thread.is_alive():
            self.get_logger().info("Stopping previous node...")
            self.stop_event.set()
            self.running_thread.join()
            self.stop_event.clear()

        # 새로운 모드 실행
        if new_mode == "static":
            self.running_thread = threading.Thread(target=run_static_obstacle_handler, args=(self.stop_event,))
        elif new_mode == "dynamic":
            self.running_thread = threading.Thread(target=run_dynamic_obstacle_handler, args=(self.stop_event,))
        elif new_mode == "None":
            self.get_logger().info("Received 'clear' mode. No node will be run.")
            self.running_thread = None
            self.current_mode = None
            return
        else:
            self.get_logger().warn(f"Unknown mode: {new_mode}")
            return

        self.running_thread.start()
        self.current_mode = new_mode


def main(args=None):
    rclpy.init(args=args)
    node = AvoidanceMode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.running_thread and node.running_thread.is_alive():
            node.stop_event.set()
            node.running_thread.join()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()