"""
A* algorithm
Author: Weicent
randomly generate obstacles, start and goal point
searching path from start and end simultaneously
"""

import numpy as np
import matplotlib.pyplot as plt
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
from ament_index_python.packages import get_package_share_directory
import os

pkg_path = get_package_share_directory('alba_manager')
map_file = os.path.join(pkg_path, 'my_map3.pgm')

class AStar_Node:
    """node with properties of g, h, coordinate and parent node"""
    def __init__(self, G=0, H=0, coordinate=None, parent=None):
        self.G = G
        self.H = H
        self.F = G + H
        self.parent = parent
        self.coordinate = coordinate

    def reset_f(self):
        self.F = self.G + self.H

    def __str__(self):
        return f"Node(Coordinate: {self.coordinate}, G: {self.G}, H: {self.H}, F: {self.F})"


class AStar(Node):
    def __init__(self, show_animation=True, robot_size=1):
        super().__init__('a_star_node')
        self.show_animation = show_animation
        self.map_data = None
        self.obstacle = None
        self.bound = None
        self.start = None
        self.goal = None
        self.fig = None
        self.ax = None
        self.click_count = 0
        self.robot_size = robot_size
        self.assigned_paths = {}        # 할당된 경로 저장
        self.path_lines = {}            # plot된 Line2D 객체
        self.path_colors = {1: 'r', 2: 'g', 3: 'b'}
        self.robot_ids = {1, 2, 3}      # 로봇 아이디

    def load_pgm_map(self, file_path):
        """PGM 파일에서 맵 데이터를 로드합니다."""
        with open(file_path, 'rb') as f:
            header = f.readline().decode('utf-8').strip()
            if header != 'P5':
                raise ValueError('Not a valid PGM file')

            while True:
                line = f.readline().decode('utf-8').strip()
                if not line.startswith('#'):
                    break

            width, height = map(int, line.split())
            max_val = int(f.readline().decode('utf-8').strip())

            img_data = np.frombuffer(f.read(), dtype=np.uint8)
            img_data = img_data.reshape((height, width)).copy()
            #img_data[img_data == 205] = 0

            return img_data

    def create_map_data(self):
        """맵 데이터를 생성합니다."""
        self.map_data = self.load_pgm_map(map_file)

        map_list = []
        for y in range(self.map_data.shape[0]):
            row = []
            for x in range(self.map_data.shape[1]):
                is_in_range = (23 <= x <= 61) and (51 <= y <= 73)
                if self.map_data[y, x] == 0 or not is_in_range:
                    row.append(0)
                else:
                    row.append(1)
            map_list.append(row)

        return map_list

    def expand_obstacles(self, obstacle_list, robot_size, map_width, map_height):
        expanded = set()
        for ox, oy in obstacle_list:
            for dx in range(-robot_size, robot_size + 1):
                for dy in range(-robot_size, robot_size + 1):
                    nx, ny = ox + dx, oy + dy
                    # 원 안(유클리드 거리)만 확장
                    if 0 <= nx < map_width and 0 <= ny < map_height:
                        if dx**2 + dy**2 <= robot_size**2:
                            expanded.add((nx, ny))
        return [list(coord) for coord in expanded]

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


    def hcost(self, node_coordinate, goal):
        dx = abs(node_coordinate[0] - goal[0])
        dy = abs(node_coordinate[1] - goal[1])
        #return dx + dy
        return math.hypot(dx, dy)  # 유클리드 거리로 통일

    def gcost(self, fixed_node, update_node_coordinate):
        dx = abs(fixed_node.coordinate[0] - update_node_coordinate[0])
        dy = abs(fixed_node.coordinate[1] - update_node_coordinate[1])
        gc = math.hypot(dx, dy)
        return fixed_node.G + gc

    def find_neighbor(self, node, obstacles, closed):
        obstacle_set = set(tuple(o) for o in obstacles)
        closed_set = set(tuple(c) for c in closed)
        neighbor_set = set()

        for x in range(node.coordinate[0] - 1, node.coordinate[0] + 2):
            for y in range(node.coordinate[1] - 1, node.coordinate[1] + 2):
                coord = (x, y)
                # 장애물 또는 이미 closed set에 있으면 제외
                if coord == tuple(node.coordinate):
                    continue
                if coord in obstacle_set:
                    continue
                if coord in closed_set:
                    continue
                neighbor_set.add(coord)

        top_nei = (node.coordinate[0], node.coordinate[1] + 1)
        bottom_nei = (node.coordinate[0], node.coordinate[1] - 1)
        left_nei = (node.coordinate[0] - 1, node.coordinate[1])
        right_nei = (node.coordinate[0] + 1, node.coordinate[1])
        lt_nei = (node.coordinate[0] - 1, node.coordinate[1] + 1)
        rt_nei = (node.coordinate[0] + 1, node.coordinate[1] + 1)
        lb_nei = (node.coordinate[0] - 1, node.coordinate[1] - 1)
        rb_nei = (node.coordinate[0] + 1, node.coordinate[1] - 1)

        if top_nei in obstacle_set and left_nei in obstacle_set:
            neighbor_set.discard(lt_nei)
        if top_nei in obstacle_set and right_nei in obstacle_set:
            neighbor_set.discard(rt_nei)
        if bottom_nei in obstacle_set and left_nei in obstacle_set:
            neighbor_set.discard(lb_nei)
        if bottom_nei in obstacle_set and right_nei in obstacle_set:
            neighbor_set.discard(rb_nei)

        closed_set = set(map(tuple, closed))
        neighbor_set -= closed_set

        return list(neighbor_set)

    def find_node_index(self, coordinate, node_list):
        for node in node_list:
            if node.coordinate == coordinate:
                return node_list.index(node)
        return 0

    def find_path(self, open_list, closed_list, goal, obstacles):
        obstacle_set = set(tuple(o) for o in obstacles)
        flag = len(open_list)
        for i in range(flag):
            node = open_list[0]
            open_coordinate_list = [node.coordinate for node in open_list]
            closed_coordinate_list = [node.coordinate for node in closed_list]
            temp = self.find_neighbor(node, obstacles, closed_coordinate_list)

            for element in temp:
                # 장애물 좌표는 무조건 건너뜀
                if tuple(element) in obstacle_set:
                    continue
                # 이미 closed에 있으면 건너뜀
                if element in closed_coordinate_list:
                    continue

                elif element in open_coordinate_list:
                    ind = open_coordinate_list.index(element)
                    new_g = self.gcost(node, element)
                    if new_g <= open_list[ind].G:
                        open_list[ind].G = new_g
                        open_list[ind].reset_f()
                        open_list[ind].parent = node
                else:
                    ele_node = AStar_Node(coordinate=element, parent=node,
                                  G=self.gcost(node, element), H=self.hcost(element, goal))
                    open_list.append(ele_node)

            open_list.remove(node)
            closed_list.append(node)
            open_list.sort(key=lambda x: x.F)

        return open_list, closed_list

    def node_to_coordinate(self, node_list):
        return [node.coordinate for node in node_list]

    def check_node_coincide(self, close_ls1, closed_ls2):
        cl1 = self.node_to_coordinate(close_ls1)
        cl2 = self.node_to_coordinate(closed_ls2)
        return [node for node in cl1 if node in cl2]

    def find_surrounding(self, coordinate, obstacle):
        boundary = []
        for x in range(coordinate[0] - 1, coordinate[0] + 2):
            for y in range(coordinate[1] - 1, coordinate[1] + 2):
                if [x, y] in obstacle:
                    boundary.append([x, y])
        return boundary

    def get_border_line(self, node_closed_ls, obstacle):
        border = []
        coordinate_closed_ls = self.node_to_coordinate(node_closed_ls)
        for coordinate in coordinate_closed_ls:
            temp = self.find_surrounding(coordinate, obstacle)
            border = border + temp
        return np.array(border)

    def get_path(self, org_list, goal_list, coordinate):
        path_org = []
        path_goal = []
        ind = self.find_node_index(coordinate, org_list)
        node = org_list[ind]
        while node != org_list[0]:
            path_org.append(node.coordinate)
            node = node.parent
        path_org.append(org_list[0].coordinate)
        ind = self.find_node_index(coordinate, goal_list)
        node = goal_list[ind]
        while node != goal_list[0]:
            path_goal.append(node.coordinate)
            node = node.parent
        path_goal.append(goal_list[0].coordinate)
        path_org.reverse()
        path = path_org + path_goal
        return np.array(path)

    def draw(self, close_origin, close_goal, start, end, bound, obstacle):
        if not close_goal.tolist():
            close_goal = np.array([end])
        #plt.cla()
        plt.gcf().set_size_inches(8, 6, forward=True)

        obstacle_np = np.array(obstacle)
        if obstacle_np.size > 0:
            plt.plot(obstacle_np[:, 0], obstacle_np[:, 1], 'sy', markersize=3, label='Obstacle')

        #plt.plot(close_origin[:, 0], close_origin[:, 1], 'oy', markersize=3, label='Origin Search')
        #plt.plot(close_goal[:, 0], close_goal[:, 1], 'og', markersize=3, label='Goal Search')
        plt.plot(bound[:, 0], bound[:, 1], 'sk', markersize=3, label='Boundary')
        plt.plot(end[0], end[1], '*b', markersize=3, label='Goal')
        plt.plot(start[0], start[1], 'ob', markersize=3, label='Origin')
        #plt.legend()
        plt.grid(True)
        plt.pause(0.0001)

    def redraw(self):
        plt.cla()
        combined_obstacle = self.get_combined_obstacle()
        # 최신 장애물로 시각화
        if combined_obstacle.size > 0 and combined_obstacle.shape[1] == 2:
            # 나머지 경로, 점 등도 다시 그림
            plt.plot(combined_obstacle[:, 0], combined_obstacle[:, 1], 'sy', markersize=3, label='Obstacle')
            plt.plot(self.bound[:, 0], self.bound[:, 1], 'sk', markersize=3, label='Boundary')
            for robot_id, path in self.assigned_paths.items():
                color = self.path_colors[robot_id]
                if path is not None and len(path) > 0:
                    path = np.array(path)
                    if path.ndim == 1:
                        path = path.reshape(-1, 2)
                    plt.plot(path[:, 0], path[:, 1], color, linewidth=2, label=f'Robot{robot_id} Path')
        plt.gca().set_aspect('equal')
        plt.grid(True)
        plt.legend()
        plt.gcf().canvas.draw()

    def draw_control(self, org_closed, goal_closed, flag, start, end, bound, obstacle):
        stop_loop = 0
        org_closed_ls = self.node_to_coordinate(org_closed)
        org_array = np.array(org_closed_ls)
        goal_closed_ls = self.node_to_coordinate(goal_closed)
        goal_array = np.array(goal_closed_ls)
        path = None

        if self.show_animation:
            self.draw(org_array, goal_array, start, end, bound, obstacle)

        if flag == 0:
            node_intersect = self.check_node_coincide(org_closed, goal_closed)
            if node_intersect:
                path = self.get_path(org_closed, goal_closed, node_intersect[0])
                path_list = path.tolist()  # numpy array를 파이썬 리스트로 변환
                stop_loop = 1
                print('Path found!')
                if self.show_animation:
                    plt.title('Robot Arrived', size=20, loc='center')
                    plt.pause(0.01)
                    plt.show()
        elif flag == 1:
            stop_loop = 1
            print('There is no path to the goal! Start point is blocked!')
        elif flag == 2:
            stop_loop = 1
            print('There is no path to the goal! End point is blocked!')

        if self.show_animation:
            info = 'There is no path to the goal! Robot&Goal are split by border shown in red \'x\'!'
            if flag == 1:
                border = self.get_border_line(org_closed, obstacle)
                plt.plot(border[:, 0], border[:, 1], 'xr', markersize=3, label='Border')
                plt.title(info, size=14, loc='center')
                plt.pause(0.01)
                plt.show()
            elif flag == 2:
                border = self.get_border_line(goal_closed, obstacle)
                border = np.array(border)
                # border가 비어있을 때 예외처리
                if border.size == 0:
                    return stop_loop, path

                # 1차원(예: [[x, y]])이 아닌 [x, y] 형태로 들어올 때 reshape
                if border.ndim == 1:
                    border = border.reshape(-1, 2)
                plt.plot(border[:, 0], border[:, 1], 'xr', markersize=3, label='Border')
                plt.title(info, size=14, loc='center')
                plt.pause(0.01)
                plt.show()

        return stop_loop, path

    def searching_control(self, start, end, bound, obstacle):
        origin = AStar_Node(coordinate=start, H=self.hcost(start, end))
        goal = AStar_Node(coordinate=end, H=self.hcost(end, start))
        origin_open = [origin]
        origin_close = []
        goal_open = [goal]
        goal_close = []
        target_goal = end
        flag = 0
        path = None

        while True:
            origin_open, origin_close = self.find_path(origin_open, origin_close, target_goal, obstacle)
            if not origin_open:
                flag = 1
                self.draw_control(origin_close, goal_close, flag, start, end, bound, obstacle)
                break

            target_origin = min(origin_open, key=lambda x: x.F).coordinate
            goal_open, goal_close = self.find_path(goal_open, goal_close, target_origin, obstacle)

            if not goal_open:
                flag = 2
                self.draw_control(origin_close, goal_close, flag, start, end, bound, obstacle)
                break

            target_goal = min(goal_open, key=lambda x: x.F).coordinate
            stop_sign, path = self.draw_control(origin_close, goal_close, flag, start, end, bound, obstacle)

            if stop_sign:
                path_list = path.tolist() if path is not None else []
                break

        return path

    def boundary_and_obstacles(self, top_vertex, bottom_vertex, map_list):
        """
        :param start: start coordinate
        :param goal: goal coordinate
        :param top_vertex: top right vertex coordinate of boundary
        :param bottom_vertex: bottom left vertex coordinate of boundary
        :param obs_number: number of obstacles generated in the map
        :return: boundary_obstacle array, obstacle list
        """
        # below can be merged into a rectangle boundary
        ay = list(range(bottom_vertex[1], top_vertex[1]))
        ax = [bottom_vertex[0]] * len(ay)
        cy = ay
        cx = [top_vertex[0]] * len(cy)
        bx = list(range(bottom_vertex[0] + 1, top_vertex[0]))
        by = [bottom_vertex[1]] * len(bx)
        dx = [bottom_vertex[0]] + bx + [top_vertex[0]]
        dy = [top_vertex[1]] * len(dx)

        height = len(map_list)
        width = len(map_list[0])

        ob = []
        for y in range(height):
            for x in range(width):
                if map_list[y][x] == 0:
                    ob.append([x, y])

        # x y coordinate in certain order for boundary
        x = ax + bx + cx + dx
        y = ay + by + cy + dy

        obstacle = ob
        #obstacle = [coor for coor in obstacle if coor != start and coor != goal]
        obs_array = np.array(obstacle)

        bound = np.vstack((x, y)).T
        bound_obs = np.vstack((bound, obs_array))
        return bound_obs, obstacle

    def initialize_map(self):
        map_list = self.create_map_data()
        height = len(map_list)
        width = len(map_list[0])
        top_vertex = [width, height]
        bottom_vertex = [0, 0]

        # 장애물과 경계 설정
        bound, obstacle = self.boundary_and_obstacles(top_vertex, bottom_vertex, map_list)
        #self.obstacle = np.array(obstacle)

        expanded_obstacle = self.expand_obstacles(obstacle, self.robot_size, width, height)
        self.obstacle = [tuple(coord) for coord in expanded_obstacle]
        self.bound = bound

        obstacle_np = np.array(self.obstacle)
        if self.show_animation:
            plt.figure(figsize=(8, 6))
            plt.gca().set_aspect('equal')
            plt.grid(True)

            if obstacle_np.size > 0:
                plt.plot(obstacle_np[:, 0], obstacle_np[:, 1], 'sy', markersize=3, label='Obstacle')
            plt.plot(bound[:, 0], bound[:, 1], 'sk', markersize=3, label='Boundary')
            plt.legend()

            plt.gcf().canvas.mpl_connect('button_press_event', self.on_click)
            plt.gcf().canvas.mpl_connect('key_press_event', self.on_key_press)

    def assign_path_for_robot(self, robot_id, start, goal):
        if robot_id not in self.robot_ids:
            self.get_logger().warn(f"Invalid robot ID: {robot_id}. Must be one of {self.robot_ids}.")
            return None

        if robot_id in self.assigned_paths:
            self.get_logger().warn(f"로봇 {robot_id}의 경로가 이미 할당되어 있습니다.")
            return None

        print(f"로봇 {robot_id} 경로 생성 중...")
        print(f"로봇 {robot_id} 시작점: {start}, 목표점: {goal}")
        path = self.searching_control(start, goal, self.bound, self.get_combined_obstacle())
        print(f"로봇 {robot_id} 경로: {path}")
        if path is not None:
            color = self.path_colors[robot_id]
            line, = plt.plot(path[:, 0], path[:, 1], color, linewidth=2, label=f'Robot{robot_id} Path')
            self.path_lines[robot_id] = line
            self.assigned_paths[robot_id] = path
            # 경로 단순화 및 시각화
            simplified_path = self.simplify_path(path, angle_threshold_deg=60)
            simplified_path = np.array(simplified_path)
            print(f"로봇 {robot_id} 심플 경로: {simplified_path}")
            plt.scatter(simplified_path[:, 0], simplified_path[:, 1], color='hotpink', s=30, label='Simple Path')
            if self.show_animation:
                plt.gcf().canvas.draw()
            print(f"로봇 {robot_id} 경로 생성 완료!")
            return path
        else:
            print(f"로봇 {robot_id} 경로 찾기 실패!")
            return None

    def release_assigned_path(self, robot_id):
        if robot_id in self.robot_ids:
            if robot_id in self.assigned_paths:
                del self.assigned_paths[robot_id]
                print(f"로봇 {robot_id}의 경로가 해제되었습니다.")
            else:
                print(f"로봇 {robot_id}의 경로가 없습니다.")

    def check_assigned_path(self):
        print(f"assigned_paths: {self.assigned_paths}")

    def on_click(self, event):
        if event.inaxes != plt.gca():
            return

        x = int(round(event.xdata))
        y = int(round(event.ydata))

        # 장애물 위에 클릭했는지 확인
        for obs in self.obstacle:
            if obs[0] == x and obs[1] == y:
                print(f"장애물 위에 클릭했습니다: ({x}, {y})")
                return

        available_ids = [i for i in [1, 2, 3] if i not in self.assigned_paths]
        if not available_ids:
            print("모든 로봇의 경로가 이미 할당됨")
            return
        robot_id = available_ids[0]

        if self.click_count % 2 == 0:
            self.start = [x, y]
            plt.plot(x, y, '^b', markersize=5, label=f'Start {self.click_count//2 + 1}')
            print(f"로봇 {robot_id} 시작점: ({x}, {y})")
        else:
            self.goal = [x, y]
            plt.plot(x, y, '*r', markersize=5, label=f'Goal {self.click_count//2 + 1}')
            print(f"로봇 {robot_id} 목표점: ({x}, {y})")

            # 경로 생성
            path = self.searching_control(self.start, self.goal, self.bound, self.get_combined_obstacle())

            if path is not None:
                color = self.path_colors[robot_id]
                line, = plt.plot(path[:, 0], path[:, 1], color, linewidth=2, label=f'Robot{robot_id} Path')
                self.path_lines[robot_id] = line
                self.assigned_paths[robot_id] = path
                if path is not None:
                    path = np.array(path)
                simplified_path = self.simplify_path(path, angle_threshold_deg=60)
                simplified_path = np.array(simplified_path)
                plt.scatter(simplified_path[:, 0], simplified_path[:, 1], color='hotpink', s=30, label='Simple Path')
                print(f"로봇 {robot_id} 경로 생성 완료!")
            else:
                print(f"로봇 {robot_id} 경로 찾기 실패! (대기 필요)")
        if self.show_animation:
            plt.gcf().canvas.draw()

        self.click_count += 1

    def on_key_press(self, event):
        if self.show_animation is False:
            return
        # '1', '2', '3' 키로 각 로봇 경로 삭제
        if event.key in ['1', '2', '3']:
            robot_id = int(event.key)
            if robot_id in self.assigned_paths:
                del self.assigned_paths[robot_id]
                if robot_id in self.path_lines:
                    del self.path_lines[robot_id]
                plt.cla()
                self.redraw()
                print(f"로봇 {robot_id}의 경로가 삭제되었습니다.")
            else:
                print(f"로봇 {robot_id}의 경로가 없습니다.")

    def get_combined_obstacle(self):
        obs = np.array(self.obstacle)
        if obs.ndim == 1:
            obs = obs.reshape(-1, 2)
        reserved = None
        if self.assigned_paths:
            reserved_list = [np.array(p) for p in self.assigned_paths.values() if len(p) > 0]
            if reserved_list:
                reserved = np.vstack(reserved_list)
                if reserved.ndim == 1:
                    reserved = reserved.reshape(-1, 2)
        if reserved is not None and reserved.size > 0:
            combined_obstacle = np.vstack((obs, reserved))
        else:
            combined_obstacle = obs
        return combined_obstacle

    def run(self):
        self.initialize_map()
        plt.show()

def main(args=None):

    rclpy.init(args=args)
    astar_node = AStar()
    astar_node.run()
    rclpy.spin(astar_node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
