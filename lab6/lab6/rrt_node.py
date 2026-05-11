import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid, Odometry
from visualization_msgs.msg import Marker, MarkerArray
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np
import math
import random
import time
import signal
import os
import csv

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError


# =====================================================================
# 시작 전 가장 먼저 확인할 플래그
# =====================================================================
# False  : 차량은 정지(speed=0)하고 RRT* 트리만 RViz에 시각화.
#          Stage 1 (RRT*만 단독 검증)을 진행할 때 사용 권장.
# True   : pure pursuit baseline으로 waypoint를 따라 주행.
#          (Stage 1까지만 구현된 상태에서는 RRT*는 시각화만 되고
#           실제 주행에는 사용되지 않음. 따라서 장애물에 부딪힘.)
#          Stage 2 (follow_path 모드 분기)를 구현한 뒤에는 장애물을
#          회피하며 주행하게 됨.
ENABLE_DRIVE = True
# =====================================================================


# Pure pursuit 파라미터 (lab5와 동일한 의미)
KP = 0.50
LOOKAHEAD_DISTANCE = 1.20      # pure pursuit이 바라보는 lookahead 거리 (m)

WAYPOINTS_FILENAME = 'waypoints.csv'
WAYPOINTS_INTERVAL = 50        # CSV에서 매 N번째 row만 사용

# RRT*가 도달하려는 목표점의 차량 기준 거리 (m).
# _find_rrt_goal_global이 이 거리에 가장 가까운 forward waypoint를 고름.
RRT_GOAL_TARGET_RADIUS = 5.0


PACKAGE_NAME = 'lab6'
try:
    waypoint_dir = get_package_share_directory(PACKAGE_NAME)
    waypoint_filepath = os.path.join(waypoint_dir, WAYPOINTS_FILENAME)
except (PackageNotFoundError, Exception):
    # ament_index가 초기화되지 않은 환경(예: IDE 디버거에서 직접 실행)
    # 에서는 패키지 source 폴더 옆의 CSV를 fallback으로 사용.
    waypoint_filepath = os.path.normpath(
        os.path.join(os.path.dirname(os.path.abspath(__file__)),
                     '..', WAYPOINTS_FILENAME))


class RRTStarNode:
    """RRT* 트리의 한 노드. (x, y)는 local(차량 base_link) frame 기준."""

    def __init__(self, x, y, parent=None, cost=0.0):
        self.x = x
        self.y = y
        self.parent = parent
        self.cost = cost


class RRTStar(Node):
    def __init__(self):
        super().__init__('rrt_star_node')

        # ---- Occupancy grid 파라미터 ----
        self.grid_length_x = 7              # grid의 x축 길이 (m)
        self.grid_length_y = 7              # grid의 y축 길이 (m)
        self.grid_resolution = 0.05         # 한 셀의 크기 (m)
        self.grid_width = int(self.grid_length_x / self.grid_resolution)
        self.grid_height = int(self.grid_length_y / self.grid_resolution)
        # grid 원점 offset (차량 base_link 기준).
        # 차량이 grid의 (x_offset, y_offset) 위치에 있도록 배치.
        self.x_offset = 1.0
        self.y_offset = self.grid_length_y / 2
        # 장애물 한 셀당 inflation(팽창) 반경 (셀 단위).
        # 차량 폭을 고려해 안전 마진을 두기 위함.
        self.occupancy_thickness = 5        # 약 20cm

        # ---- RRT* 파라미터 ----
        # 용어 정리:
        #   rrt_goal     = RRT planner의 목적지 (local frame)
        #   pp_target    = pure pursuit이 steering 계산에 쓰는 점 (local frame)
        self.rrt_goal_x = 0.0
        self.rrt_goal_y = 0.0
        self.pp_target_x = 0.0
        self.pp_target_y = 0.0
        self.max_iterations = 500
        self.step_size = 0.3                # 한 번에 트리를 늘리는 거리 (m)
        self.neighborhood_radius = 2.0      # rewire 후보 노드 검색 반경 (m)
        self.goal_threshold = self.step_size  # 목표 도달 판정 거리 (m)

        # ---- per-cycle planner 상태 ----
        # Stage 2에서 PURE_PURSUIT / RRT* 모드를 분기할 때 활용 예정.
        self.trajectory_clear = True
        self.rrt_path_found = False
        self.final_node = None              # RRT*가 rrt_goal에 도달한 노드

        # ---- 차량 자세 ----
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        # Scan이 들어온 순간의 차량 자세 snapshot. occupancy_grid는
        # 이 자세 기준으로 build되므로, RRT*가 collision check 할 때도
        # 같은 snapshot으로 좌표를 변환해야 정합이 맞음 (convert_to_grid 참고).
        self.grid_pose_x = 0.0
        self.grid_pose_y = 0.0
        self.grid_pose_yaw = 0.0

        # ---- frame_id 설정 ----
        self.rrt_tree_markers_frame_id = 'ego_racecar/base_link'
        self.occupancy_grid_markers_frame_id = 'ego_racecar/laser'

        # ---- ROS publisher / subscriber ----
        self.path_marker_pub = self.create_publisher(MarkerArray, '/path', 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.target_marker_pub = self.create_publisher(Marker, '/target_marker', 10)
        self.waypoints_marker_pub = self.create_publisher(Marker, '/waypoints_marker', 10)
        self.goal_marker_pub = self.create_publisher(Marker, '/goal_marker', 10)

        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.grid_pub = self.create_publisher(OccupancyGrid, '/occupancy_grid', 10)
        self.rrt_tree_marker_pub = self.create_publisher(MarkerArray, '/rrt_star_tree', 10)
        self.odom_sub = self.create_subscription(Odometry, 'ego_racecar/odom', self.pose_callback, 10)

        # ---- 초기 occupancy grid (-1 = unknown) ----
        self.occupancy_grid = np.ones((self.grid_width, self.grid_height), dtype=np.int8) * -1
        # 트리의 루트는 항상 차량 위치(local 원점).
        self.nodes = [RRTStarNode(0, 0)]

        self.marker_lifetime = rclpy.duration.Duration(seconds=10.1).to_msg()

        # ---- waypoints 로드 ----
        self.waypoints_x = []
        self.waypoints_y = []
        self.load_waypoints(waypoint_filepath, WAYPOINTS_INTERVAL)

    # =================================================================
    # Waypoint 로드 & 좌표 변환 (제공)
    # =================================================================

    def load_waypoints(self, filepath, interval=100):
        """CSV의 (x, y, heading, speed) 중 (x, y)만 interval 간격으로 sub-sample하여 로드."""
        try:
            with open(filepath, 'r') as f:
                reader = csv.reader(f)
                for i, row in enumerate(reader):
                    if i % interval == 0:
                        x, y, _, _ = map(float, row)
                        self.waypoints_x.append(x)
                        self.waypoints_y.append(y)
            self.get_logger().info(
                f"Loaded {len(self.waypoints_x)} waypoints from {filepath}")
        except (FileNotFoundError, OSError) as e:
            self.get_logger().error(
                f"Failed to load waypoints from {filepath}: {e}")

    def local_to_global(self, x_local, y_local):
        """local(차량 base_link) -> global(map) 좌표 변환."""
        x_global = self.current_x + (x_local * math.cos(self.current_yaw)
                                     - y_local * math.sin(self.current_yaw))
        y_global = self.current_y + (x_local * math.sin(self.current_yaw)
                                     + y_local * math.cos(self.current_yaw))
        return x_global, y_global

    def global_to_local(self, x_global, y_global):
        """global(map) -> local(차량 base_link) 좌표 변환."""
        dx = x_global - self.current_x
        dy = y_global - self.current_y
        x_local = dx * math.cos(-self.current_yaw) - dy * math.sin(-self.current_yaw)
        y_local = dx * math.sin(-self.current_yaw) + dy * math.cos(-self.current_yaw)
        return x_local, y_local

    def convert_to_grid(self, x_local, y_local):
        """현재 local frame의 (x, y)를 grid 셀 인덱스로 변환.

        주의: scan이 들어온 시점과 RRT*가 collision check 하는 시점 사이에
        차량이 움직였을 수 있으므로, 항상 scan-time pose(grid_pose_*) 기준의
        local frame으로 다시 환원한 뒤 셀 인덱스를 계산한다.
        """
        # 1) 현재 local -> map
        cos_p = math.cos(self.current_yaw)
        sin_p = math.sin(self.current_yaw)
        x_map = cos_p * x_local - sin_p * y_local + self.current_x
        y_map = sin_p * x_local + cos_p * y_local + self.current_y
        # 2) map -> scan-time local
        dx = x_map - self.grid_pose_x
        dy = y_map - self.grid_pose_y
        cos_s = math.cos(self.grid_pose_yaw)
        sin_s = math.sin(self.grid_pose_yaw)
        x_grid_local = cos_s * dx + sin_s * dy
        y_grid_local = -sin_s * dx + cos_s * dy
        # 3) scan-time local -> 셀 인덱스
        x_grid = int(round((x_grid_local + self.x_offset) / self.grid_resolution))
        y_grid = int(round((y_grid_local + self.y_offset) / self.grid_resolution))
        return x_grid, y_grid

    # =================================================================
    # Pure pursuit baseline (제공)
    # =================================================================

    def _lookahead_on_polyline(self, points_local, lookahead):
        """local frame polyline이 원점 중심 반지름 lookahead인 원과 처음 만나는 점.

        못 만나면 None.
        """
        L2 = lookahead * lookahead
        for i in range(len(points_local) - 1):
            ax, ay = points_local[i]
            bx, by = points_local[i + 1]
            dx = bx - ax
            dy = by - ay
            a = dx * dx + dy * dy
            if a < 1e-12:
                continue  # 거의 0 길이 segment는 건너뜀
            b = 2.0 * (ax * dx + ay * dy)
            c = ax * ax + ay * ay - L2
            disc = b * b - 4.0 * a * c
            if disc < 0.0:
                continue  # 원과 만나지 않음
            sqrt_disc = math.sqrt(disc)
            t1 = (-b - sqrt_disc) / (2.0 * a)
            t2 = (-b + sqrt_disc) / (2.0 * a)
            for t in (t1, t2):
                if 0.0 <= t <= 1.0:
                    return ax + t * dx, ay + t * dy
        return None

    def _find_pp_target_global(self):
        """Pure pursuit이 따라갈 점(global frame).

        가장 가까운 waypoint부터 시작해서 전방으로 polyline을 만든 뒤,
        lookahead 거리에 있는 점을 보간하여 반환. waypoint를 찾지 못하면
        가장 가까운 waypoint 자체를 반환.
        """
        if not self.waypoints_x:
            return self.current_x, self.current_y

        n = len(self.waypoints_x)
        # 가장 가까운 waypoint
        min_d2 = float('inf')
        closest_idx = 0
        for i in range(n):
            dx = self.waypoints_x[i] - self.current_x
            dy = self.waypoints_y[i] - self.current_y
            d2 = dx * dx + dy * dy
            if d2 < min_d2:
                min_d2 = d2
                closest_idx = i

        # closest부터 전방으로 polyline 누적, lookahead 너머까지 도달하면 정지.
        polyline_local = []
        idx = closest_idx
        steps = 0
        while True:
            wx, wy = self.waypoints_x[idx], self.waypoints_y[idx]
            lx, ly = self.global_to_local(wx, wy)
            polyline_local.append((lx, ly))
            if (math.hypot(lx, ly) >= LOOKAHEAD_DISTANCE
                    and len(polyline_local) >= 2):
                break
            idx = (idx + 1) % n
            steps += 1
            if steps >= n:
                break

        intersect_local = self._lookahead_on_polyline(
            polyline_local, LOOKAHEAD_DISTANCE)
        if intersect_local is None:
            return (self.waypoints_x[closest_idx],
                    self.waypoints_y[closest_idx])
        return self.local_to_global(*intersect_local)

    def _find_rrt_goal_global(self):
        """RRT*가 도달하려는 목표점(global frame).

        Baseline: 차량 진행방향(+x local)으로 RRT_GOAL_TARGET_RADIUS 거리에
        가장 가까운 waypoint를 선택. occupancy grid는 고려하지 않음.

        Stage 2 단계에서는 README의 '`rrt_goal` 선택 방법' 절을 참고해
        더 정교한 방식(grid 내 free cell만, forward half-plane만 등)으로
        개선할 수 있다.
        """
        if not self.waypoints_x:
            return self.current_x, self.current_y

        best = None
        best_diff = float('inf')
        for i in range(len(self.waypoints_x)):
            wp_x, wp_y = self.waypoints_x[i], self.waypoints_y[i]
            lx, ly = self.global_to_local(wp_x, wp_y)
            if lx <= 0:
                continue  # 후방 waypoint는 무시
            dist = math.hypot(lx, ly)
            diff = abs(dist - RRT_GOAL_TARGET_RADIUS)
            if diff < best_diff:
                best_diff = diff
                best = (wp_x, wp_y)

        if best is not None:
            return best
        return self.current_x, self.current_y

    def _compute_steering(self, steering_target_x, steering_target_y):
        """Pure pursuit 조향각 계산 (local frame, +x = 차량 진행방향)."""
        d2 = steering_target_x ** 2 + steering_target_y ** 2
        if d2 < 1e-6:
            return 0.0
        return KP * 2.0 * steering_target_y / d2

    def _publish_drive(self, angle):
        """주행 명령 publish. ENABLE_DRIVE=False면 speed=0으로 정지."""
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = angle
        if not ENABLE_DRIVE:
            # Stage 1 검증 모드: 차량을 정지시키고 RViz에서 RRT*만 관찰.
            drive_msg.drive.speed = 0.0
            self.drive_pub.publish(drive_msg)
            return
        # 조향각이 클수록 속도를 줄여 안정성 확보.
        if abs(angle) > 20.0 * np.pi / 180.0:
            drive_msg.drive.speed = 0.3
        elif abs(angle) > 10.0 * np.pi / 180.0:
            drive_msg.drive.speed = 1.5
        else:
            drive_msg.drive.speed = 3.5
        self.drive_pub.publish(drive_msg)

    # =================================================================
    # Marker 시각화 (제공)
    # =================================================================

    def _publish_goal_marker(self):
        """rrt_goal을 마젠타색 sphere로 map frame에 publish."""
        gx, gy = self.local_to_global(self.rrt_goal_x, self.rrt_goal_y)
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.scale.x = 0.30
        marker.scale.y = 0.30
        marker.scale.z = 0.30
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.pose.position.x = gx
        marker.pose.position.y = gy
        marker.pose.position.z = 0.0
        marker.lifetime = self.marker_lifetime
        self.goal_marker_pub.publish(marker)

    def _publish_waypoints_marker(self):
        """waypoints 전체를 파란 POINTS marker로 map frame에 publish."""
        if not self.waypoints_x:
            return
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.color.a = 1.0
        marker.color.b = 1.0
        marker.points = [Point(x=x, y=y, z=0.0)
                         for x, y in zip(self.waypoints_x, self.waypoints_y)]
        self.waypoints_marker_pub.publish(marker)

    def _publish_target_marker(self, steering_target_x, steering_target_y):
        """현재 steering이 바라보는 점을 노란 sphere로 map frame에 publish."""
        gx, gy = self.local_to_global(steering_target_x, steering_target_y)
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.scale.x = 0.25
        marker.scale.y = 0.25
        marker.scale.z = 0.25
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.pose.position.x = gx
        marker.pose.position.y = gy
        marker.pose.position.z = 0.0
        marker.lifetime = self.marker_lifetime
        self.target_marker_pub.publish(marker)

    def add_marker(self, marker_array, node, idx, frame_id, is_rrt_goal=False):
        """RRT* 트리의 노드 하나를 sphere marker로 추가."""
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.lifetime = self.marker_lifetime
        if is_rrt_goal is True:
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 1.0
        else:
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
        marker.pose.position.x = node.x
        marker.pose.position.y = node.y
        marker.pose.position.z = 0.0
        marker.id = idx
        marker_array.markers.append(marker)

    def add_edge_marker(self, marker_array, start_node, end_node, idx, frame_id,
                        is_final=False):
        """RRT* 트리의 edge 하나를 line_strip marker로 추가.

        is_final=True이면 굵은 초록 선(최종 path), 아니면 얇은 빨간 선(탐색용).
        """
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.lifetime = self.marker_lifetime

        marker.color.a = 1.0
        if is_final is True:
            marker.scale.x = 0.1
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
        else:
            marker.scale.x = 0.05
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
        start_point = Point(x=start_node.x, y=start_node.y, z=0.0)
        end_point = Point(x=end_node.x, y=end_node.y, z=0.0)
        marker.points = [start_point, end_point]
        # 노드 id와 충돌하지 않도록 max_iterations 만큼 offset.
        marker.id = idx + self.max_iterations
        marker_array.markers.append(marker)

    def _publish_tree_markers(self):
        """현재 self.nodes의 모든 노드와 edge를 RViz로 publish.

        매번 DELETEALL을 먼저 보내서 rewire로 무효해진 edge marker가
        화면에 남지 않도록 한다.
        """
        marker_array = MarkerArray()

        delete_all = Marker()
        delete_all.header.frame_id = self.rrt_tree_markers_frame_id
        delete_all.action = Marker.DELETEALL
        delete_all.id = -1  # 노드 id(0~)와 겹치지 않도록 음수
        marker_array.markers.append(delete_all)

        for idx, node in enumerate(self.nodes):
            self.add_marker(marker_array, node, idx, self.rrt_tree_markers_frame_id)
            if node.parent is not None:
                self.add_edge_marker(marker_array, node.parent, node, idx,
                                     self.rrt_tree_markers_frame_id)

        self.rrt_tree_marker_pub.publish(marker_array)

    def _publish_rrt_path_markers(self):
        """RRT*가 goal에 도달했을 때 final_node 체인을 굵은 초록 선으로 표시.

        final_node가 None이면 마커만 DELETEALL해서 이전 cycle 잔여 표시 제거.
        """
        path_marker_array = MarkerArray()

        delete_all = Marker()
        delete_all.header.frame_id = 'map'
        delete_all.action = Marker.DELETEALL
        delete_all.id = -1
        path_marker_array.markers.append(delete_all)

        if self.final_node is not None:
            rrt_goal_global_x, rrt_goal_global_y = self.local_to_global(
                self.rrt_goal_x, self.rrt_goal_y)
            rrt_goal_node_global = RRTStarNode(rrt_goal_global_x, rrt_goal_global_y)

            marker_id = 0
            self.add_marker(path_marker_array, rrt_goal_node_global, marker_id,
                            'map', is_rrt_goal=True)

            current_node_local = self.final_node
            current_node_global = rrt_goal_node_global
            while current_node_local.parent is not None:
                marker_id += 1
                global_x, global_y = self.local_to_global(current_node_local.x,
                                                         current_node_local.y)
                new_node_global = RRTStarNode(global_x, global_y)
                new_node_global.parent = current_node_global
                self.add_marker(path_marker_array, new_node_global, marker_id, 'map')
                self.add_edge_marker(path_marker_array, new_node_global.parent,
                                     new_node_global, marker_id, 'map', is_final=True)
                current_node_local = current_node_local.parent
                current_node_global = new_node_global

        self.path_marker_pub.publish(path_marker_array)

    # =================================================================
    # Occupancy grid (Stage 1 TODO)
    # =================================================================

    def scan_callback(self, scan_msg):
        """LaserScan을 받아 self.occupancy_grid를 새로 빌드한다.

        주의: 함수의 가장 첫 줄에서 scan-time pose snapshot을 저장해야 한다
        (self.grid_pose_x/y/yaw <- self.current_x/y/yaw). 이걸 빠뜨리면 차량이
        움직였을 때 RRT* collision check가 grid와 어긋난다.

        빌드가 끝나면 self.publish_occupancy_grid(scan_msg.header.stamp) 호출.
        """
        # TODO (Stage 1)
        pass

    def publish_occupancy_grid(self, stamp):
        """Occupancy grid를 map frame에 publish.

        grid_pose_*(scan-time 자세)를 origin으로 anchor하여, RViz의 TF
        interpolation으로 인한 jitter를 방지한다.
        """
        occupancy_msg = OccupancyGrid()
        occupancy_msg.header.stamp = stamp
        occupancy_msg.header.frame_id = 'map'

        occupancy_msg.info.resolution = self.grid_resolution
        occupancy_msg.info.width = self.grid_width
        occupancy_msg.info.height = self.grid_height

        # grid의 (0, 0) 셀은 scan-time local frame에서 (-x_offset, -y_offset).
        # 이를 map frame으로 변환한 값을 origin.position에 넣는다.
        cos_y = math.cos(self.grid_pose_yaw)
        sin_y = math.sin(self.grid_pose_yaw)
        occupancy_msg.info.origin.position.x = (
            self.grid_pose_x
            + cos_y * (-self.x_offset)
            - sin_y * (-self.y_offset))
        occupancy_msg.info.origin.position.y = (
            self.grid_pose_y
            + sin_y * (-self.x_offset)
            + cos_y * (-self.y_offset))
        occupancy_msg.info.origin.position.z = 0.0
        # grid 축을 scan-time local frame과 같은 방향으로 회전.
        occupancy_msg.info.origin.orientation.z = math.sin(self.grid_pose_yaw / 2)
        occupancy_msg.info.origin.orientation.w = math.cos(self.grid_pose_yaw / 2)

        occupancy_msg.data = np.transpose(self.occupancy_grid).flatten().tolist()
        self.grid_pub.publish(occupancy_msg)

    # =================================================================
    # Pose / RRT* / follow_path 루프
    # =================================================================

    def pose_callback(self, odom_msg):
        """매 odometry tick마다 호출. RRT*를 한 cycle 돌리고 follow_path 실행."""
        # 차량 자세 업데이트
        self.current_x = odom_msg.pose.pose.position.x
        self.current_y = odom_msg.pose.pose.position.y
        q = odom_msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

        # ---- RRT* 단계 ----
        # 매 cycle마다 트리를 차량 위치(local 원점)에서 다시 시작.
        self.nodes = [RRTStarNode(0.0, 0.0)]

        # 이번 cycle의 RRT goal (local frame).
        rrt_goal_xg, rrt_goal_yg = self._find_rrt_goal_global()
        self.rrt_goal_x, self.rrt_goal_y = self.global_to_local(
            rrt_goal_xg, rrt_goal_yg)

        # RRT* 실행. Stage 1 단계에서는 매 cycle 항상 실행되고, RViz로
        # 시각화만 됨 (실제 주행은 아래 pure pursuit baseline).
        # Stage 2에서는 trajectory_clear 여부에 따라 분기 가능.
        self.perform_rrt_star()
        self._publish_rrt_path_markers()

        # ---- Pure pursuit 단계 (기구현 완료)----
        # 이번 cycle의 PP target (local frame).
        pp_xg, pp_yg = self._find_pp_target_global()
        self.pp_target_x, self.pp_target_y = self.global_to_local(pp_xg, pp_yg)

        # ---- 주행 명령 publish ----
        self.follow_path()

    def follow_path(self):
        """차량 주행 명령을 publish.

        Stage 1 baseline: 항상 pure pursuit(pp_target)로만 주행.
                          → 장애물에 부딪힐 수 있음.

        TODO (Stage 2): occupancy_grid 기반으로 전방 trajectory가 막혔는지
        판별해서, 막혔으면 RRT*가 찾은 path 위의 lookahead 점을 steering
        target으로 사용하도록 모드 분기를 구현. 구체적인 설계는 README의
        'Stage 2: Pure Pursuit과 통합' 절을 참고하라.
        """
        # ---- Stage 1 baseline: pure pursuit ----
        steering_target_local = (self.pp_target_x, self.pp_target_y)

        # ---- 주행 명령 & marker publish ----
        tx, ty = steering_target_local
        angle = self._compute_steering(tx, ty)
        self._publish_drive(angle)
        self._publish_target_marker(tx, ty)
        self._publish_waypoints_marker()
        self._publish_goal_marker()

    # =================================================================
    # RRT* 핵심 알고리즘 (Stage 1 TODO)
    # =================================================================

    def perform_rrt_star(self):
        """RRT* 메인 루프 (Algorithm 6).

        Pseudocode는 README의 rrt_star.png를 그대로 따른다. README §1.2의
        pseudocode ↔ 코드 대응표 참고.

        성공 시 self.final_node에 goal에 도달한 노드를, self.rrt_path_found에
        True를 세팅해야 한다 (실패면 None / False 유지). 두 값은 트리/path 시각화
        및 후속 follow_path에서 사용된다.
        """
        rrt_goal_node = RRTStarNode(self.rrt_goal_x, self.rrt_goal_y)
        self.final_node = None
        self.rrt_path_found = False

        # ---- TODO (Stage 1) ----

        # ---- 구현 끝 ----

        # 트리 시각화 (제공)
        self._publish_tree_markers()

        if not self.rrt_path_found:
            self.get_logger().warn(
                f"RRT* could not reach rrt_goal "
                f"({self.rrt_goal_x:.2f}, {self.rrt_goal_y:.2f}) in "
                f"{self.max_iterations} iterations.",
                throttle_duration_sec=2.0,
            )

    def get_random_node(self):
        """grid 영역 내 (local frame) uniform 무작위 RRTStarNode 반환."""
        # TODO (Stage 1)
        return RRTStarNode(0.0, 0.0)

    def get_nearest_node(self, random_node):
        """self.nodes 중 random_node와 가장 가까운 노드 반환."""
        # TODO (Stage 1)
        return self.nodes[0]

    def get_neighbors(self, node):
        """self.nodes 중 node로부터 self.neighborhood_radius 이내의 노드 리스트."""
        # TODO (Stage 1)
        return []

    def rewire(self, new_node, neighbors):
        """new_node를 거쳐 가는 것이 더 싸면 neighbor의 parent를 new_node로 재배선."""
        # TODO (Stage 1)
        pass

    def is_collision_free(self, nearest_node, new_node):
        """두 노드를 잇는 직선이 occupancy_grid의 점유 셀을 통과하지 않으면 True.

        주의: grid 밖으로 나가는 점도 충돌로 처리해야 한다 (RRT*가 grid 바깥으로
        뻗어나가는 걸 막기 위함). 셀 변환은 self.convert_to_grid를 사용.
        """
        # TODO (Stage 1)
        return True

    def distance(self, node1, node2):
        """두 노드 사이 유클리드 거리."""
        return math.sqrt((node1.x - node2.x) ** 2 + (node1.y - node2.y) ** 2)

    # =================================================================
    # Shutdown 정리 (제공)
    # =================================================================

    def clear_visualization(self):
        """종료 시 RViz의 모든 marker를 지운다."""
        tree_clear = MarkerArray()
        tree_delete = Marker()
        tree_delete.header.frame_id = self.rrt_tree_markers_frame_id
        tree_delete.action = Marker.DELETEALL
        tree_delete.id = -1
        tree_clear.markers.append(tree_delete)
        self.rrt_tree_marker_pub.publish(tree_clear)

        path_clear = MarkerArray()
        path_delete = Marker()
        path_delete.header.frame_id = 'map'
        path_delete.action = Marker.DELETEALL
        path_delete.id = -1
        path_clear.markers.append(path_delete)
        self.path_marker_pub.publish(path_clear)

        # 단일 marker 토픽들은 id=0으로 DELETE.
        for pub, frame in (
            (self.waypoints_marker_pub, 'map'),
            (self.target_marker_pub, 'map'),
            (self.goal_marker_pub, 'map'),
        ):
            m = Marker()
            m.header.frame_id = frame
            m.action = Marker.DELETE
            m.id = 0
            pub.publish(m)

    def stop_vehicle(self):
        """종료 시 speed=0을 publish해서 시뮬레이터에서 차량을 즉시 정지시킴."""
        msg = AckermannDriveStamped()
        msg.drive.steering_angle = 0.0
        msg.drive.speed = 0.0
        self.drive_pub.publish(msg)


def main(args=None):
    # rclpy 기본 SIGINT 핸들러를 끄고 finally 블록에서 안전하게 정리.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)

    # SIGTERM도 KeyboardInterrupt로 라우팅 → kill <pid> 도 cleanup 경로 탐.
    def _sigterm_handler(_signum, _frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, _sigterm_handler)

    rrt_star_node = RRTStar()
    try:
        rclpy.spin(rrt_star_node)
    except KeyboardInterrupt:
        pass
    finally:
        rrt_star_node.stop_vehicle()
        rrt_star_node.clear_visualization()
        # DDS가 stop + DELETE marker를 flush할 시간 확보.
        time.sleep(0.2)
        rrt_star_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
