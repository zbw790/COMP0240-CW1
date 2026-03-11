#!/usr/bin/env python3
"""Generate COMP0240 CW1 report PDF using WeasyPrint. Arial 11pt, 2cm margins."""

import os, shutil
from weasyprint import HTML

DIR = os.path.dirname(os.path.abspath(__file__))
IMG_DIR = os.path.join(os.path.expanduser("~"), ".openclaw/workspace/comp0240_report")

for i in range(1, 5):
    src = os.path.join(IMG_DIR, f"vis_scenario{i}_planned_vs_actual.png")
    dst = os.path.join(DIR, f"vis_scenario{i}_planned_vs_actual.png")
    if os.path.exists(src):
        shutil.copy2(src, dst)

html_content = """<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<style>
@page {
    size: A4;
    margin: 2cm 2cm 2cm 2cm;
    @bottom-center {
        content: counter(page);
        font-family: Arial, Helvetica, sans-serif;
        font-size: 9pt;
        color: #666;
    }
}

body {
    font-family: Arial, Helvetica, sans-serif;
    font-size: 11pt;
    line-height: 1.45;
    color: #1a1a1a;
}

.title-block {
    text-align: center;
    margin-bottom: 25px;
    padding-bottom: 15px;
    border-bottom: 2px solid #2c3e50;
}

.title-block h1 {
    font-size: 16pt;
    color: #2c3e50;
    margin: 0 0 4px 0;
}

.title-block h2 {
    font-size: 13pt;
    color: #34495e;
    margin: 0 0 8px 0;
    font-weight: normal;
}

.title-block .author {
    font-size: 11pt;
    color: #555;
}

h2 {
    font-size: 13pt;
    color: #2c3e50;
    border-bottom: 1px solid #bdc3c7;
    padding-bottom: 3px;
    margin-top: 18px;
    margin-bottom: 6px;
}

h3 {
    font-size: 11.5pt;
    color: #34495e;
    margin-top: 12px;
    margin-bottom: 5px;
}

p {
    text-align: justify;
    margin: 0 0 7px 0;
}

table {
    width: 100%;
    border-collapse: collapse;
    margin: 10px 0;
    font-size: 9.5pt;
}

th {
    background-color: #2c3e50;
    color: white;
    padding: 6px 6px;
    text-align: center;
    font-weight: bold;
}

td {
    padding: 4px 6px;
    text-align: center;
    border-bottom: 1px solid #ddd;
}

tr:nth-child(even) {
    background-color: #f8f9fa;
}

.caption {
    font-size: 9pt;
    color: #555;
    text-align: center;
    font-style: italic;
    margin: 3px 0 12px 0;
}

.figure-full img {
    width: 100%;
    margin: 5px 0;
}

ul, ol {
    margin: 3px 0 7px 0;
    padding-left: 20px;
}

li {
    margin-bottom: 3px;
    text-align: justify;
}

.improvement {
    color: #27ae60;
    font-weight: bold;
}

.overhead {
    color: #c0392b;
    font-weight: bold;
}
</style>
</head>
<body>

<div class="title-block">
    <h1>COMP0240 Coursework 1</h1>
    <h2>Structural Inspection Path Planning Challenge</h2>
    <div class="author">Bowen Zhu &nbsp;|&nbsp; March 2026</div>
</div>

<h2>1. Problem Statement and Objectives</h2>

<p>Autonomous drone-based structural inspection requires visiting a set of camera viewpoints to photograph markers on structures, while navigating around obstacles in the environment. This mirrors real-world inspection scenarios where drones must efficiently survey buildings, bridges, or infrastructure while maintaining safe clearance from the structure and surrounding obstacles.</p>

<p>The challenge decomposes into two coupled optimisation problems: (1) <strong>tour optimisation</strong> -- determining the most efficient visitation order across all viewpoints to minimise total flight time and distance; and (2) <strong>collision-free motion planning</strong> -- routing the drone between consecutive viewpoints while avoiding cuboid obstacles. Four scenarios of increasing complexity evaluate the system:</p>

<table>
    <tr><th>Scenario</th><th>Viewpoints</th><th>Obstacles</th><th>Key Challenge</th></tr>
    <tr><td>1</td><td>10</td><td>1</td><td>Basic route optimisation</td></tr>
    <tr><td>2</td><td>10</td><td>5</td><td>Moderate obstacle density</td></tr>
    <tr><td>3</td><td>20</td><td>1</td><td>Scalability to large viewpoint sets</td></tr>
    <tr><td>4</td><td>5</td><td>10</td><td>Dense obstacle field navigation</td></tr>
</table>

<p>The objectives are to: achieve 100% viewpoint visitation across all scenarios, minimise total mission time and flight distance compared to a sequential baseline, avoid all obstacle collisions, and verify task completion by capturing photographs at each viewpoint showing the ArUco markers.</p>

<h2>2. Methodology</h2>

<p>The solution decouples the problem into two stages: a global route optimiser determines the visitation order, then a local path planner handles obstacle avoidance for each segment. This decomposition simplifies implementation while achieving strong performance, as the TSP and motion planning sub-problems can each be solved with well-established algorithms.</p>

<h3>2.1 Tour Optimisation (TSP)</h3>

<p>The viewpoint visitation order is formulated as an open-loop Travelling Salesman Problem (TSP). Starting from the drone's initial position, the solver finds the shortest tour visiting all viewpoints exactly once. The 3D Euclidean distance between all position pairs forms the cost matrix, with the return-to-start column zeroed since the drone lands at the final viewpoint rather than returning to its origin.</p>

<p>A <strong>hybrid solver</strong> was chosen to balance optimality with computational feasibility:</p>

<ul>
    <li><strong>Exact dynamic programming</strong> (O(n<sup>2</sup> * 2<sup>n</sup>)) for scenarios with n &le; 15 viewpoints (Scenarios 1, 2, and 4), guaranteeing the globally optimal tour. With n=10, this computes in under 1 second.</li>
    <li><strong>Simulated annealing</strong> for Scenario 3 (n=20), where DP would require 20<sup>2</sup> * 2<sup>20</sup> &asymp; 419 million states. SA provides near-optimal solutions through stochastic neighbourhood search with temperature-based acceptance of worse solutions to escape local minima.</li>
</ul>

<p>The <code>python_tsp</code> library provides both solvers. This hybrid approach was chosen over pure heuristics (which sacrifice optimality for small n) or pure exact methods (which become infeasible for n > 15).</p>

<h3>2.2 Collision-Free Motion Planning (3D A*)</h3>

<p>Before executing each flight segment, a collision check samples points along the straight-line path at 0.2m intervals. If any sample falls within an obstacle's bounding box (expanded by a 1.0m safety padding to account for drone size and positional uncertainty), the path planner activates.</p>

<p>A* was chosen over alternatives for the following reasons: unlike Dijkstra's algorithm, A*'s heuristic (Euclidean distance to goal) significantly reduces search space in sparse 3D environments; unlike RRT, A* produces deterministic, optimal paths on the discretised grid; and unlike Dubins paths, A* naturally handles 3D obstacle avoidance without requiring separate 2D planning and altitude management.</p>

<p>The avoidance strategy uses a <strong>three-tier approach</strong> to prioritise computational efficiency:</p>

<ol>
    <li><strong>Elevation bypass:</strong> Attempts to fly over obstacles by raising altitude incrementally (+2m, +3m, +4m, +5m above the higher endpoint), verifying that ascent, traverse, and descent segments are all collision-free. This requires only 3 waypoints and avoids the computational cost of grid search.</li>
    <li><strong>Full 3D A* search:</strong> If elevation bypass fails (e.g., very tall obstacles), a grid-based search (resolution 0.5m, 26-connected neighbourhood) finds a path through 3D space. Path simplification iteratively removes intermediate waypoints where direct line-of-sight exists, reducing flight segments.</li>
    <li><strong>High-altitude fallback:</strong> As a last resort, the drone ascends to 9.0-9.5m to overfly all obstacles.</li>
</ol>

<h3>2.3 Integration with Gazebo Simulation</h3>

<p>The solution integrates with AeroStack2's ROS2 interface through the <code>DroneInterface</code> API. The drone executes blocking <code>go_to_point_with_yaw</code> commands for final viewpoint waypoints (ensuring correct camera orientation) and <code>go_to_point</code> for intermediate avoidance waypoints. The yaw angle, specified in radians in the scenario file, is converted to degrees for the flight controller.</p>

<p>At each viewpoint, the drone pauses for 2 seconds to allow camera stabilisation, then captures a screenshot via ROS2 camera topic subscription. These screenshots serve as proof of task completion, showing the ArUco markers at each inspection point. All metrics (time, distance, visit order, avoidance details, intermediate waypoints) are logged to JSON files for post-flight analysis and visualisation.</p>

<h2>3. Results and Discussion</h2>

<h3>3.1 Performance Summary</h3>

<table>
    <tr><th>Scenario</th><th>Viewpoints</th><th>Obstacles</th><th>Time (s)</th><th>Distance (m)</th><th>Avoidances</th><th>Success</th></tr>
    <tr><td>1</td><td>10/10</td><td>1</td><td>126.37</td><td>59.00</td><td>1</td><td>100%</td></tr>
    <tr><td>2</td><td>10/10</td><td>5</td><td>143.23</td><td>59.00</td><td>1</td><td>100%</td></tr>
    <tr><td>3</td><td>20/20</td><td>1</td><td>213.50</td><td>81.18</td><td>1</td><td>100%</td></tr>
    <tr><td>4</td><td>5/5</td><td>10</td><td>112.95</td><td>47.78</td><td>3</td><td>100%</td></tr>
</table>
<div class="caption">Table 2: Optimised mission performance across all four scenarios.</div>

<p>All scenarios achieve <span class="improvement">100% viewpoint visitation</span> with zero collisions. The TSP solver produces identical optimal distances for Scenarios 1 and 2 (59.0m) despite different obstacle counts, confirming that the viewpoint layout -- not the obstacles -- determines tour length. Scenario 3 scales sub-linearly: doubling viewpoints from 10 to 20 increases distance by only 37% (59.0m to 81.2m), demonstrating the SA solver's effectiveness.</p>

<h3>3.2 Planned vs Actual Flight Paths</h3>

<p>Figures 1a-1d compare TSP-planned straight-line paths (left) against actual flight paths with A* obstacle avoidance (right). Orange dotted lines highlight segments where the planned path would collide with obstacles; red dashed lines show the elevation bypass trajectories actually flown.</p>

<div class="figure-full">
    <img src="vis_scenario1_planned_vs_actual.png" alt="Scenario 1">
    <div class="caption">Figure 1a: Scenario 1 (10 VP, 1 obstacle) -- planned vs actual flight path.</div>
</div>
<div class="figure-full">
    <img src="vis_scenario2_planned_vs_actual.png" alt="Scenario 2">
    <div class="caption">Figure 1b: Scenario 2 (10 VP, 5 obstacles) -- planned vs actual flight path.</div>
</div>
<div class="figure-full">
    <img src="vis_scenario3_planned_vs_actual.png" alt="Scenario 3">
    <div class="caption">Figure 1c: Scenario 3 (20 VP, 1 obstacle) -- planned vs actual flight path.</div>
</div>
<div class="figure-full">
    <img src="vis_scenario4_planned_vs_actual.png" alt="Scenario 4">
    <div class="caption">Figure 1d: Scenario 4 (5 VP, 10 obstacles) -- planned vs actual flight path.</div>
</div>

<h3>3.3 Obstacle Avoidance Overhead</h3>

<table>
    <tr><th>Scenario</th><th>Planned (m)</th><th>Actual (m)</th><th>Overhead</th><th>Strategy</th></tr>
    <tr><td>1</td><td>59.0</td><td>65.1</td><td class="overhead">+6.1m (+10.4%)</td><td>1 elevation bypass</td></tr>
    <tr><td>2</td><td>59.0</td><td>65.1</td><td class="overhead">+6.1m (+10.4%)</td><td>1 elevation bypass</td></tr>
    <tr><td>3</td><td>81.2</td><td>86.8</td><td class="overhead">+5.6m (+6.9%)</td><td>1 elevation bypass</td></tr>
    <tr><td>4</td><td>47.8</td><td>65.2</td><td class="overhead">+17.4m (+36.5%)</td><td>3 elevation bypasses</td></tr>
</table>
<div class="caption">Table 3: Distance overhead from obstacle avoidance.</div>

<p>The elevation bypass strategy resolved all obstacle conflicts across all scenarios -- neither full A* search nor the high-altitude fallback were required. In Scenario 4, the drone ascended to z &asymp; 5-7m to overfly dense obstacle clusters, with the longest segment (VP3 to VP4) spanning the full arena width. The +36.5% overhead in Scenario 4 highlights the main limitation: the TSP solver plans routes assuming straight-line distances and is unaware of obstacles, potentially routing through obstacle-dense areas.</p>

<p>Comparing Scenarios 1 and 2 is instructive: both have identical optimal tour distances (59.0m) and identical avoidance overhead (+6.1m), despite Scenario 2 having 5x more obstacles. This indicates that obstacle placement relative to the optimal tour matters more than raw obstacle count.</p>

<h2>4. Conclusions</h2>

<p>The combined TSP + 3D A* approach achieves 100% success across all four scenarios with zero collisions. The hybrid TSP solver (exact DP for n &le; 15, simulated annealing for larger sets) scales effectively from 5 to 20 viewpoints, and the three-tier obstacle avoidance strategy provides robust collision-free navigation with 6.9-36.5% distance overhead depending on obstacle density.</p>

<p>The primary limitation is the decoupled planning approach: the TSP solver uses straight-line distances that ignore obstacles, potentially producing tours that require costly detours. Future work could address this through obstacle-aware cost matrices that use actual avoidance distances, or iterative refinement where the TSP is re-solved after computing avoidance paths. Additionally, incorporating Dubins paths could better account for the drone's kinematic constraints, and real-time replanning could handle dynamic obstacles in real-world deployments.</p>

</body>
</html>
"""

out_path = os.path.join(DIR, "report.pdf")
HTML(string=html_content, base_url=DIR).write_pdf(out_path)
print(f"Saved: {out_path}")

import re
text = re.sub(r'<[^>]+>', '', html_content)
words = len(text.split())
print(f"Approximate word count: {words}")
