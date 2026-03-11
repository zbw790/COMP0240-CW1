#!/usr/bin/env python3
"""Baseline mission with metrics logging."""

import argparse
import json
import os
import time
from time import sleep

import yaml
import numpy as np
from scipy.spatial.distance import euclidean

import rclpy
from as2_python_api.drone_interface import DroneInterface

TAKE_OFF_HEIGHT = 1.0
TAKE_OFF_SPEED = 1.0
SLEEP_TIME = 0.5
SPEED = 1.0
LAND_SPEED = 0.5


def read_scenario(file_path):
    with open(file_path, 'r') as file:
        return yaml.safe_load(file)


def drone_start(drone_interface):
    print('Start mission')
    print('Arm')
    success = drone_interface.arm()
    print(f'Arm success: {success}')
    print('Offboard')
    success = drone_interface.offboard()
    print(f'Offboard success: {success}')
    print('Take Off')
    success = drone_interface.takeoff(height=TAKE_OFF_HEIGHT, speed=TAKE_OFF_SPEED)
    print(f'Take Off success: {success}')
    return success


def drone_run(drone_interface, scenario):
    start_pos = (scenario['drone_start_pose']['x'], scenario['drone_start_pose']['y'], TAKE_OFF_HEIGHT)
    current_pos = start_pos
    total_distance = 0.0
    segment_times = []
    segment_details = []
    visited = 0
    visit_order = []

    for vpid, vp in scenario["viewpoint_poses"].items():
        target = (vp['x'], vp['y'], vp['z'])
        print(f'Go to {vpid} with path facing {vp}')
        visit_order.append(int(vpid) if isinstance(vpid, (int, str)) else vpid)

        seg_start = time.time()
        goal = [vp["x"], vp["y"], vp["z"]]
        success = drone_interface.go_to.go_to_point_with_yaw(goal, angle=vp["w"], speed=SPEED)
        seg_time = time.time() - seg_start

        dist = euclidean(current_pos, target)
        total_distance += dist
        segment_times.append(round(seg_time, 2))
        segment_details.append({
            "viewpoint_id": str(vpid),
            "position": {"x": round(vp['x'], 2), "y": round(vp['y'], 2), "z": round(vp['z'], 2)},
            "distance_from_prev_m": round(dist, 2),
            "time_s": round(seg_time, 2),
        })

        current_pos = target
        visited += 1
        print(f'Go to success: {success}')
        sleep(SLEEP_TIME)

    return {
        "total_distance": total_distance,
        "segment_times": segment_times,
        "segment_details": segment_details,
        "visited": visited,
        "visit_order": visit_order,
    }


def drone_end(drone_interface):
    print('End mission')
    print('Land')
    success = drone_interface.land(speed=LAND_SPEED)
    print(f'Land success: {success}')
    print('Manual')
    success = drone_interface.manual()
    print(f'Manual success: {success}')
    return success


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Baseline drone mission with metrics')
    parser.add_argument('-s', '--scenario', type=str, required=True)
    parser.add_argument('-n', '--namespace', type=str, default='drone0')
    parser.add_argument('-v', '--verbose', action='store_true', default=False)
    parser.add_argument('--use_sim_time', action='store_true', default=True)

    args = parser.parse_args()
    scenario = read_scenario(args.scenario)
    scenario_name = os.path.splitext(os.path.basename(args.scenario))[0]
    total_vps = len(scenario['viewpoint_poses'])
    num_obstacles = len(scenario.get('obstacles', {}) or {})

    rclpy.init()
    uav = DroneInterface(drone_id=args.namespace, use_sim_time=args.use_sim_time, verbose=args.verbose)

    success = drone_start(uav)
    try:
        start_time = time.time()
        if success:
            run_data = drone_run(uav, scenario)
        total_time = time.time() - start_time

        report = {
            "scenario": scenario_name,
            "method": "baseline",
            "algorithm": {
                "tsp_solver": "none (sequential order)",
                "path_planner": "none (direct point-to-point)",
                "speed_ms": SPEED,
            },
            "total_time_s": round(total_time, 2),
            "total_distance_m": round(run_data["total_distance"], 2),
            "avg_speed_ms": round(run_data["total_distance"] / total_time, 2) if total_time > 0 else 0,
            "waypoints_visited": run_data["visited"],
            "total_waypoints": total_vps,
            "success_rate_pct": round(run_data["visited"] / total_vps * 100, 1),
            "visit_order": run_data["visit_order"],
            "num_obstacles": num_obstacles,
            "collision_avoidances": 0,
            "segment_times_s": run_data["segment_times"],
            "segment_details": run_data["segment_details"],
        }

        print("\n" + "=" * 50)
        print("BASELINE METRICS REPORT")
        print("=" * 50)
        for k, v in report.items():
            if k != "segment_details":
                print(f"  {k}: {v}")
        print("=" * 50)

        fname = f"metrics_{scenario_name}_baseline.json"
        with open(fname, 'w') as f:
            json.dump(report, f, indent=2)
        print(f"Metrics saved to {fname}")

    except KeyboardInterrupt:
        pass

    drone_end(uav)
    uav.shutdown()
    rclpy.shutdown()
    print('Clean exit')
