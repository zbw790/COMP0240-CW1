#!/usr/bin/env python3
"""Planned (straight-line TSP) vs Actual (with A* avoidance waypoints) flight path comparison."""

import json, os, yaml
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


def load_scenario(path):
    with open(path) as f:
        return yaml.safe_load(f)

def load_metrics(path):
    if not os.path.exists(path):
        return None
    with open(path) as f:
        return json.load(f)

def draw_cuboid(ax, obs, color="red", alpha=0.15):
    cx, cy, cz = obs["x"], obs["y"], obs["z"]
    hw, hd, hh = obs["w"]/2, obs["d"]/2, obs["h"]/2
    x0, x1 = cx-hw, cx+hw
    y0, y1 = cy-hd, cy+hd
    z0, z1 = cz-hh, cz+hh
    verts = [
        [[x0,y0,z0],[x1,y0,z0],[x1,y1,z0],[x0,y1,z0]],
        [[x0,y0,z1],[x1,y0,z1],[x1,y1,z1],[x0,y1,z1]],
        [[x0,y0,z0],[x1,y0,z0],[x1,y0,z1],[x0,y0,z1]],
        [[x0,y1,z0],[x1,y1,z0],[x1,y1,z1],[x0,y1,z1]],
        [[x0,y0,z0],[x0,y1,z0],[x0,y1,z1],[x0,y0,z1]],
        [[x1,y0,z0],[x1,y1,z0],[x1,y1,z1],[x1,y0,z1]],
    ]
    ax.add_collection3d(Poly3DCollection(verts, alpha=alpha, facecolors=color, edgecolors="darkred", linewidths=0.5))

def setup_ax(ax, scenario, start):
    vps = scenario["viewpoint_poses"]
    obstacles = scenario.get("obstacles", {}) or {}
    for oid, obs in obstacles.items():
        draw_cuboid(ax, obs)
    vp_xs = [vp["x"] for vp in vps.values()]
    vp_ys = [vp["y"] for vp in vps.values()]
    vp_zs = [vp["z"] for vp in vps.values()]
    ax.scatter(vp_xs, vp_ys, vp_zs, c="orange", s=60, marker="^", label="Viewpoints", zorder=5)
    for vpid, vp in vps.items():
        ax.text(vp["x"], vp["y"], vp["z"]+0.3, str(vpid), fontsize=7, ha="center")
    ax.scatter([start["x"]], [start["y"]], [1.0], c="blue", s=100, marker="o", label="Start", zorder=5)
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.set_xlim(-12, 12)
    ax.set_ylim(-12, 12)
    ax.set_zlim(0, 8)

def plot_planned_vs_actual(scenario_name, scenario, metrics, out_dir="."):
    if not metrics or "segment_details" not in metrics:
        print(f"Skipping {scenario_name}: no segment_details")
        return
    start = scenario["drone_start_pose"]

    fig = plt.figure(figsize=(18, 7))

    # --- Left: Planned (straight-line TSP) ---
    ax1 = fig.add_subplot(121, projection="3d")
    setup_ax(ax1, scenario, start)

    # Draw straight lines between viewpoints in TSP order
    px = [start["x"]]
    py = [start["y"]]
    pz = [1.0]
    for seg in metrics["segment_details"]:
        pos = seg["position"]
        px.append(pos["x"])
        py.append(pos["y"])
        pz.append(pos["z"])
    ax1.plot(px, py, pz, c="#2ca02c", linewidth=1.8, alpha=0.8, label="Planned path")

    # Highlight segments that would collide
    idx = 0
    first_avoid = True
    for seg in metrics["segment_details"]:
        pos = seg["position"]
        if seg.get("obstacle_avoided", False):
            lbl = "Collision segment" if first_avoid else ""
            ax1.plot([px[idx], px[idx+1]], [py[idx], py[idx+1]], [pz[idx], pz[idx+1]],
                     c="orange", linewidth=2.5, linestyle=":", alpha=0.9, label=lbl)
            first_avoid = False
        idx += 1

    # Compute planned (straight-line) total distance
    planned_dist = sum(np.sqrt((px[i+1]-px[i])**2 + (py[i+1]-py[i])**2 + (pz[i+1]-pz[i])**2) for i in range(len(px)-1))
    ax1.set_title(f"{scenario_name} — Planned (TSP straight-line)\nDist: {planned_dist:.1f}m | {len(metrics['segment_details'])} VP", fontsize=10)
    ax1.legend(fontsize=7, loc="upper left")

    # --- Right: Actual (with A* waypoints) ---
    ax2 = fig.add_subplot(122, projection="3d")
    setup_ax(ax2, scenario, start)

    ax_ = [start["x"]]
    ay_ = [start["y"]]
    az_ = [1.0]
    seg_boundaries = [0]  # track where each segment starts in the path arrays
    for seg in metrics["segment_details"]:
        for iwp in seg.get("intermediate_waypoints", []):
            ax_.append(iwp["x"])
            ay_.append(iwp["y"])
            az_.append(iwp["z"])
        pos = seg["position"]
        ax_.append(pos["x"])
        ay_.append(pos["y"])
        az_.append(pos["z"])
        seg_boundaries.append(len(ax_) - 1)

    # Draw full path in green first
    ax2.plot(ax_, ay_, az_, c="#2ca02c", linewidth=1.8, alpha=0.8, label="Actual path")

    # Overdraw avoidance segments in red
    first_avoid = True
    for si, seg in enumerate(metrics["segment_details"]):
        if seg.get("obstacle_avoided", False):
            s_start = seg_boundaries[si]
            s_end = seg_boundaries[si + 1]
            for k in range(s_start, s_end):
                lbl = "Avoidance path" if first_avoid else ""
                ax2.plot([ax_[k], ax_[k+1]], [ay_[k], ay_[k+1]], [az_[k], az_[k+1]],
                         c="red", linewidth=2.5, linestyle="--", alpha=0.9, label=lbl)
                first_avoid = False

    # Compute actual total distance
    actual_dist = sum(np.sqrt((ax_[i+1]-ax_[i])**2 + (ay_[i+1]-ay_[i])**2 + (az_[i+1]-az_[i])**2) for i in range(len(ax_)-1))
    n_avoid = sum(1 for s in metrics["segment_details"] if s.get("obstacle_avoided"))
    ax2.set_title(f"{scenario_name} — Actual (with A* avoidance)\nDist: {actual_dist:.1f}m | Avoidances: {n_avoid}", fontsize=10)
    ax2.legend(fontsize=7, loc="upper left")

    # Bottom text: distance overhead
    if planned_dist > 0:
        overhead = (actual_dist - planned_dist) / planned_dist * 100
        fig.text(0.5, 0.01, f"Distance overhead from obstacle avoidance: {actual_dist - planned_dist:.1f}m (+{overhead:.1f}%)",
                 ha="center", fontsize=12, fontweight="bold", color="#d62728" if overhead > 5 else "#2ca02c")

    plt.tight_layout(rect=[0, 0.04, 1, 1])
    out_path = os.path.join(out_dir, f"vis_{scenario_name}_planned_vs_actual.png")
    plt.savefig(out_path, dpi=150, bbox_inches="tight")
    print(f"Saved: {out_path}")
    plt.close()


if __name__ == "__main__":
    base_dir = os.path.dirname(os.path.abspath(__file__))
    out_dir = os.path.join(base_dir, "visualisations")
    os.makedirs(out_dir, exist_ok=True)

    for i in range(1, 5):
        sc_file = os.path.join(base_dir, f"scenarios/scenario{i}.yaml")
        opt_file = os.path.join(base_dir, f"metrics_scenario{i}_optimised.json")
        scenario = load_scenario(sc_file)
        metrics = load_metrics(opt_file)
        plot_planned_vs_actual(f"scenario{i}", scenario, metrics, out_dir)

    print(f"\nAll planned-vs-actual visualisations saved to {out_dir}/")
