#!/usr/bin/env python3
"""
Optimised mission: TSP route ordering + 3D A* obstacle avoidance + metrics logging.
"""

import argparse
import math
import time
import json
import os
from time import sleep

import yaml
import numpy as np
from scipy.spatial.distance import euclidean
from python_tsp.exact import solve_tsp_dynamic_programming
from python_tsp.heuristics import solve_tsp_simulated_annealing

import rclpy
from as2_python_api.drone_interface import DroneInterface
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

try:
    from cv_bridge import CvBridge
    import cv2
    HAS_CV = True
except ImportError:
    HAS_CV = False

# ======================== CONSTANTS ========================
TAKE_OFF_HEIGHT = 1.0
TAKE_OFF_SPEED = 1.0
SPEED = 2.0
LAND_SPEED = 0.5
SLEEP_TIME = 2.0
GRID_RESOLUTION = 0.5
OBSTACLE_PADDING = 1.0

import heapq


# ======================== A* PATH PLANNER ========================
class AStarPlanner3D:
    def __init__(self, obstacles, resolution=GRID_RESOLUTION, padding=OBSTACLE_PADDING):
        self.obstacles = obstacles
        self.resolution = resolution
        self.padding = padding
        self.collision_avoidances = 0

    def is_collision(self, point):
        px, py, pz = point
        for obs in self.obstacles:
            ox, oy, oz = obs['x'], obs['y'], obs['z']
            hw = obs['w'] / 2 + self.padding
            hd = obs['d'] / 2 + self.padding
            hh = obs['h'] / 2 + self.padding
            if (ox - hw <= px <= ox + hw and
                oy - hd <= py <= oy + hd and
                oz - hh <= pz <= oz + hh):
                return True
        return False

    def line_collision(self, p1, p2):
        dist = math.sqrt(sum((a-b)**2 for a, b in zip(p1, p2)))
        steps = max(20, int(dist / 0.2))
        for i in range(steps + 1):
            t = i / steps
            point = (
                p1[0] + t * (p2[0] - p1[0]),
                p1[1] + t * (p2[1] - p1[1]),
                p1[2] + t * (p2[2] - p1[2])
            )
            if self.is_collision(point):
                return True
        return False

    def _snap(self, val):
        return round(val / self.resolution) * self.resolution

    def _heuristic(self, a, b):
        return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2 + (a[2]-b[2])**2)

    def _get_neighbors(self, node):
        x, y, z = node
        r = self.resolution
        neighbors = []
        for dx in (-r, 0, r):
            for dy in (-r, 0, r):
                for dz in (-r, 0, r):
                    if dx == 0 and dy == 0 and dz == 0:
                        continue
                    nx, ny, nz = round(x+dx, 4), round(y+dy, 4), round(z+dz, 4)
                    if -15 <= nx <= 15 and -15 <= ny <= 15 and 0.5 <= nz <= 10:
                        neighbors.append((nx, ny, nz))
        return neighbors

    def _astar_search(self, start, goal, max_iterations=30000):
        start_g = (self._snap(start[0]), self._snap(start[1]), self._snap(start[2]))
        goal_g = (self._snap(goal[0]), self._snap(goal[1]), self._snap(goal[2]))

        open_set = []
        heapq.heappush(open_set, (0, start_g))
        came_from = {}
        g_score = {start_g: 0}
        closed = set()
        iterations = 0

        while open_set and iterations < max_iterations:
            iterations += 1
            _, current = heapq.heappop(open_set)
            if current in closed:
                continue
            closed.add(current)

            if self._heuristic(current, goal_g) <= self.resolution * 2.0:
                node = current
                waypoints = []
                while node in came_from:
                    waypoints.append(node)
                    node = came_from[node]
                waypoints.reverse()
                simplified = self._simplify_path([start] + waypoints + [goal])
                print(f"[A*] Found path in {iterations} iters, {len(simplified)-1} waypoints")
                return simplified[1:]

            for neighbor in self._get_neighbors(current):
                if neighbor in closed:
                    continue
                if self.is_collision(neighbor):
                    continue
                move_cost = self._heuristic(current, neighbor)
                tentative_g = g_score[current] + move_cost
                if tentative_g < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f = tentative_g + self._heuristic(neighbor, goal_g)
                    heapq.heappush(open_set, (f, neighbor))

        print(f"[A*] Exhausted after {iterations} iters")
        return None

    def _simplify_path(self, path):
        if len(path) <= 2:
            return path
        simplified = [path[0]]
        i = 0
        while i < len(path) - 1:
            furthest = i + 1
            for j in range(len(path) - 1, i, -1):
                if not self.line_collision(path[i], path[j]):
                    furthest = j
                    break
            simplified.append(path[furthest])
            i = furthest
        return simplified

    def plan_path(self, start, goal):
        if not self.line_collision(start, goal):
            return [goal], False

        self.collision_avoidances += 1
        print(f"[A*] Obstacle on path, planning around...")

        for dz in [2.0, 3.0, 4.0, 5.0]:
            mid_z = max(start[2], goal[2]) + dz
            if mid_z > 9.5:
                continue
            up_s = (start[0], start[1], mid_z)
            up_e = (goal[0], goal[1], mid_z)
            if (not self.is_collision(up_s) and not self.is_collision(up_e) and
                not self.line_collision(start, up_s) and
                not self.line_collision(up_s, up_e) and
                not self.line_collision(up_e, goal)):
                print(f"[A*] Elevation bypass z={mid_z:.1f}")
                return [up_s, up_e, goal], True

        result = self._astar_search(start, goal)
        if result:
            return result, True

        for hz in [9.0, 9.5]:
            hs = (start[0], start[1], hz)
            he = (goal[0], goal[1], hz)
            if (not self.line_collision(start, hs) and
                not self.line_collision(hs, he) and
                not self.line_collision(he, goal)):
                print(f"[A*] High altitude z={hz}")
                return [hs, he, goal], True

        print(f"[A*] WARNING: no safe path, direct flight")
        return [goal], True


# ======================== TSP SOLVER ========================
def solve_tour(start_pos, viewpoints):
    vp_list = list(viewpoints.items())
    n = len(vp_list)

    positions = [start_pos]
    for vpid, vp in vp_list:
        positions.append((vp['x'], vp['y'], vp['z']))

    size = len(positions)
    dist_matrix = np.zeros((size, size))
    for i in range(size):
        for j in range(size):
            if i != j:
                dist_matrix[i][j] = euclidean(positions[i], positions[j])

    dist_matrix[:, 0] = 0

    if n <= 15:
        try:
            permutation, distance = solve_tsp_dynamic_programming(dist_matrix)
        except Exception:
            permutation, distance = solve_tsp_simulated_annealing(dist_matrix)
    else:
        permutation, distance = solve_tsp_simulated_annealing(dist_matrix)

    ordered = []
    for idx in permutation:
        if idx == 0:
            continue
        ordered.append(vp_list[idx - 1])

    print(f"[TSP] Tour distance: {distance:.2f}m, order: {[vpid for vpid, _ in ordered]}")
    return ordered, distance


# ======================== CAMERA (screenshot only) ========================
class SimpleCamera:
    def __init__(self, drone_interface):
        self.frame = None
        self.bridge = CvBridge() if HAS_CV else None
        if HAS_CV:
            drone_interface.create_subscription(
                Image,
                "/drone0/sensor_measurements/hd_camera/image_raw",
                self._cb,
                qos_profile_sensor_data
            )

    def _cb(self, msg):
        if not HAS_CV:
            return
        try:
            self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:
            self.frame = self.bridge.imgmsg_to_cv2(msg)

    def save(self, name, out_dir="screenshots"):
        if not HAS_CV or self.frame is None:
            return
        os.makedirs(out_dir, exist_ok=True)
        path = f"{out_dir}/{name}.png"
        cv2.imwrite(path, self.frame)
        print(f"[CAM] Saved: {path}")


# ======================== MISSION ========================
def read_scenario(file_path):
    with open(file_path, 'r') as f:
        return yaml.safe_load(f)


def drone_start(drone_interface):
    print('[MISSION] Arm + Offboard + Takeoff')
    drone_interface.arm()
    drone_interface.offboard()
    success = drone_interface.takeoff(height=TAKE_OFF_HEIGHT, speed=TAKE_OFF_SPEED)
    print(f'[MISSION] Takeoff: {success}')
    return success


def drone_run(drone_interface, scenario, scenario_name):
    start_pos = (
        scenario['drone_start_pose']['x'],
        scenario['drone_start_pose']['y'],
        TAKE_OFF_HEIGHT
    )

    obstacles = []
    if 'obstacles' in scenario and scenario['obstacles']:
        for oid, obs in scenario['obstacles'].items():
            obstacles.append(obs)
    print(f"[INFO] {len(obstacles)} obstacles")

    planner = AStarPlanner3D(obstacles)
    ordered_vps, tsp_dist = solve_tour(start_pos, scenario['viewpoint_poses'])

    current_pos = start_pos
    total_distance = 0.0
    segment_times = []
    segment_details = []
    visit_order = []

    for vpid, vp in ordered_vps:
        target = (vp['x'], vp['y'], vp['z'])
        yaw_deg = math.degrees(vp['w'])
        print(f"\n[NAV] >>> VP {vpid}: ({target[0]:.2f}, {target[1]:.2f}, {target[2]:.2f}) yaw={yaw_deg:.1f}deg")

        visit_order.append(int(vpid) if isinstance(vpid, (int, str)) else vpid)
        seg_start = time.time()

        waypoints, avoided = planner.plan_path(current_pos, target)

        for i, wp in enumerate(waypoints):
            is_last = (i == len(waypoints) - 1)
            if is_last:
                print(f"[NAV]   -> final: [{wp[0]:.2f}, {wp[1]:.2f}, {wp[2]:.2f}]")
                drone_interface.go_to.go_to_point_with_yaw(
                    list(wp), angle=yaw_deg, speed=SPEED)
            else:
                print(f"[NAV]   -> wp {i+1}: [{wp[0]:.2f}, {wp[1]:.2f}, {wp[2]:.2f}]")
                drone_interface.go_to.go_to_point(list(wp), speed=SPEED)

        sleep(SLEEP_TIME)
        if cam:
            cam.save(f"{scenario_name}_vp{vpid}")

        seg_time = time.time() - seg_start
        dist = euclidean(current_pos, target)
        total_distance += dist

        segment_times.append(round(seg_time, 2))
        # Record A* intermediate waypoints for visualisation
        wp_list = [{"x": round(w[0], 2), "y": round(w[1], 2), "z": round(w[2], 2)} for w in waypoints[:-1]] if len(waypoints) > 1 else []
        segment_details.append({
            "viewpoint_id": str(vpid),
            "position": {"x": round(vp['x'], 2), "y": round(vp['y'], 2), "z": round(vp['z'], 2)},
            "intermediate_waypoints": wp_list,
            "distance_from_prev_m": round(dist, 2),
            "time_s": round(seg_time, 2),
            "obstacle_avoided": avoided,
        })

        current_pos = target
        print(f"[NAV] VP {vpid} done ({seg_time:.1f}s)")

    return {
        "total_distance": total_distance,
        "segment_times": segment_times,
        "segment_details": segment_details,
        "visited": len(visit_order),
        "visit_order": visit_order,
        "tsp_distance": tsp_dist,
        "num_obstacles": len(obstacles),
        "collision_avoidances": planner.collision_avoidances,
    }


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Optimised drone mission')
    parser.add_argument('-s', '--scenario', type=str, required=True)
    parser.add_argument('-n', '--namespace', type=str, default='drone0')
    parser.add_argument('-v', '--verbose', action='store_true', default=False)
    parser.add_argument('--use_sim_time', action='store_true', default=True)
    args = parser.parse_args()

    scenario = read_scenario(args.scenario)
    scenario_name = os.path.splitext(os.path.basename(args.scenario))[0]
    total_vps = len(scenario['viewpoint_poses'])

    rclpy.init()
    uav = DroneInterface(drone_id=args.namespace, use_sim_time=args.use_sim_time, verbose=args.verbose)

    cam = SimpleCamera(uav) if HAS_CV else None
    success = drone_start(uav)
    try:
        start_time = time.time()
        run_data = drone_run(uav, scenario, scenario_name) if success else {}
        total_time = time.time() - start_time

        report = {
            "scenario": scenario_name,
            "method": "optimised",
            "algorithm": {
                "tsp_solver": "dynamic_programming" if total_vps <= 15 else "simulated_annealing",
                "path_planner": "A*_3D",
                "obstacle_padding_m": OBSTACLE_PADDING,
                "speed_ms": SPEED,
            },
            "total_time_s": round(total_time, 2),
            "total_distance_m": round(run_data.get("total_distance", 0), 2),
            "tsp_optimal_distance_m": round(run_data.get("tsp_distance", 0), 2),
            "avg_speed_ms": round(run_data.get("total_distance", 0) / total_time, 2) if total_time > 0 else 0,
            "waypoints_visited": run_data.get("visited", 0),
            "total_waypoints": total_vps,
            "success_rate_pct": round(run_data.get("visited", 0) / total_vps * 100, 1),
            "visit_order": run_data.get("visit_order", []),
            "num_obstacles": run_data.get("num_obstacles", 0),
            "collision_avoidances": run_data.get("collision_avoidances", 0),
            "segment_times_s": run_data.get("segment_times", []),
            "segment_details": run_data.get("segment_details", []),
        }

        print("\n" + "=" * 50)
        print("MISSION REPORT")
        print("=" * 50)
        for k, v in report.items():
            if k != "segment_details":
                print(f"  {k}: {v}")
        print("=" * 50)

        fname = f"metrics_{scenario_name}_optimised.json"
        with open(fname, 'w') as f:
            json.dump(report, f, indent=2)
        print(f"Saved: {fname}")

    except KeyboardInterrupt:
        print("\n[INTERRUPTED]")

    print('[MISSION] Landing...')
    uav.land(speed=LAND_SPEED)
    uav.manual()
    uav.shutdown()
    rclpy.shutdown()
    print('Clean exit')
