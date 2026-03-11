#!/usr/bin/env python3
"""Visualise baseline vs optimised flight paths side by side for each scenario."""

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

def draw_path(ax, start, metrics, color, label, dash=False, scenario=None):
    if not metrics:
        return
    path_x = [start["x"]]
    path_y = [start["y"]]
    path_z = [1.0]

    if "segment_details" in metrics:
        for seg in metrics["segment_details"]:
            # Include intermediate A* waypoints if present
            for iwp in seg.get("intermediate_waypoints", []):
                path_x.append(iwp["x"])
                path_y.append(iwp["y"])
                path_z.append(iwp["z"])
            pos = seg["position"]
            path_x.append(pos["x"])
            path_y.append(pos["y"])
            path_z.append(pos["z"])
    elif scenario:
        vps = scenario["viewpoint_poses"]
        for vpid in sorted(vps.keys(), key=lambda k: int(k)):
            vp = vps[vpid]
            path_x.append(vp["x"])
            path_y.append(vp["y"])
            path_z.append(vp["z"])

    if len(path_x) <= 1:
        return

    ls = "--" if dash else "-"
    ax.plot(path_x, path_y, path_z, c=color, linewidth=1.8, alpha=0.8, linestyle=ls, label=label)

    if not dash and "segment_details" in metrics:
        # Re-draw avoidance segments in red over the green path
        idx = 0  # track position in path arrays (0=start)
        for seg in metrics["segment_details"]:
            n_intermediate = len(seg.get("intermediate_waypoints", []))
            n_points = n_intermediate + 1  # intermediates + final viewpoint
            if seg.get("obstacle_avoided", False):
                start_idx = idx
                end_idx = idx + n_points
                for k in range(start_idx, end_idx):
                    lbl = "Avoidance" if k == start_idx else ""
                    ax.plot([path_x[k], path_x[k+1]], [path_y[k], path_y[k+1]],
                            [path_z[k], path_z[k+1]], c="red", linewidth=2.5, linestyle="--", alpha=0.9,
                            label=lbl if not any(s.get("obstacle_avoided") for s in metrics["segment_details"][:metrics["segment_details"].index(seg)]) and k == start_idx else "")
            idx += n_points

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

def metrics_str(m, label):
    if not m:
        return f"{label}: No data"
    return f"{label}: {m['total_time_s']}s | {m['total_distance_m']}m | {m['waypoints_visited']}/{m['total_waypoints']} VP | Avoid: {m.get('collision_avoidances',0)}"

def plot_comparison(scenario_name, scenario, baseline, optimised, out_dir="."):
    start = scenario["drone_start_pose"]

    fig = plt.figure(figsize=(18, 7))

    # Left: baseline
    ax1 = fig.add_subplot(121, projection="3d")
    setup_ax(ax1, scenario, start)
    draw_path(ax1, start, baseline, "#d62728", "Baseline path", scenario=scenario)
    ax1.set_title(f"{scenario_name} — Baseline\n{metrics_str(baseline, 'Baseline')}", fontsize=10)
    ax1.legend(fontsize=7, loc="upper left")

    # Right: optimised
    ax2 = fig.add_subplot(122, projection="3d")
    setup_ax(ax2, scenario, start)
    draw_path(ax2, start, optimised, "#2ca02c", "Optimised path", scenario=scenario)
    ax2.set_title(f"{scenario_name} — Optimised\n{metrics_str(optimised, 'Optimised')}", fontsize=10)
    ax2.legend(fontsize=7, loc="upper left")

    # Add improvement text if both exist
    if baseline and optimised:
        bt, ot = baseline["total_time_s"], optimised["total_time_s"]
        bd, od = baseline["total_distance_m"], optimised["total_distance_m"]
        time_imp = (bt - ot) / bt * 100
        dist_imp = (bd - od) / bd * 100
        fig.text(0.5, 0.01, f"Improvement: Time {time_imp:.1f}% | Distance {dist_imp:.1f}%",
                 ha="center", fontsize=12, fontweight="bold", color="#2ca02c")

    plt.tight_layout(rect=[0, 0.04, 1, 1])
    out_path = os.path.join(out_dir, f"vis_{scenario_name}_comparison.png")
    plt.savefig(out_path, dpi=150, bbox_inches="tight")
    print(f"Saved: {out_path}")
    plt.close()

def plot_summary_table(all_data, out_dir="."):
    fig, ax = plt.subplots(figsize=(14, 4))
    ax.axis("off")

    headers = ["Scenario", "Method", "Viewpoints", "Obstacles", "Time (s)", "Distance (m)", "Avoidances", "Success"]
    rows = []
    for sc_name, bl, opt in all_data:
        for label, m in [("Baseline", bl), ("Optimised", opt)]:
            if m:
                rows.append([
                    sc_name if label == "Baseline" else "",
                    label, str(m["total_waypoints"]), str(m.get("num_obstacles", 0)),
                    str(m["total_time_s"]), str(m["total_distance_m"]),
                    str(m.get("collision_avoidances", 0)),
                    f'{m["success_rate_pct"]}%',
                ])
            else:
                rows.append([sc_name if label == "Baseline" else "", label, "-", "-", "-", "-", "-", "-"])

    table = ax.table(cellText=rows, colLabels=headers, loc="center", cellLoc="center")
    table.auto_set_font_size(False)
    table.set_fontsize(9)
    table.scale(1, 1.4)

    for j in range(len(headers)):
        table[0, j].set_facecolor("#4472C4")
        table[0, j].set_text_props(color="white", fontweight="bold")
    for i in range(1, len(rows)+1):
        # Alternate color per scenario pair
        group = (i - 1) // 2
        if group % 2 == 0:
            c = "#D6E4F0" if (i-1) % 2 == 1 else "#FFFFFF"
        else:
            c = "#FFF2CC" if (i-1) % 2 == 1 else "#FCE4D6"
        for j in range(len(headers)):
            table[i, j].set_facecolor(c)

    fig.suptitle("Baseline vs Optimised — Performance Comparison", fontsize=13, fontweight="bold")
    plt.tight_layout()
    out_path = os.path.join(out_dir, "vis_summary.png")
    plt.savefig(out_path, dpi=150, bbox_inches="tight")
    print(f"Saved: {out_path}")
    plt.close()

if __name__ == "__main__":
    base_dir = os.path.dirname(os.path.abspath(__file__))
    out_dir = os.path.join(base_dir, "visualisations")
    os.makedirs(out_dir, exist_ok=True)

    all_data = []
    for i in range(1, 5):
        sc_file = os.path.join(base_dir, f"scenarios/scenario{i}.yaml")
        bl_file = os.path.join(base_dir, f"metrics_scenario{i}_baseline.json")
        opt_file = os.path.join(base_dir, f"metrics_scenario{i}_optimised.json")
        scenario = load_scenario(sc_file)
        bl = load_metrics(bl_file)
        opt = load_metrics(opt_file)
        plot_comparison(f"scenario{i}", scenario, bl, opt, out_dir)
        all_data.append((f"scenario{i}", bl, opt))

    plot_summary_table(all_data, out_dir)
    print(f"\nAll visualisations saved to {out_dir}/")
