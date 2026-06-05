#!/usr/bin/env python3
"""
W102 Path Planning Simulation Node
Robot navigates from (0,0) around a chair obstacle at (0,4) to reach John at (0,10)
in a 10ft x 12ft living room using graph-based Dijkstra path planning.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import String

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation, PillowWriter
import csv
import os
import heapq
import time


# ---------------------------------------------------------------------------
# Graph-based path planning utilities
# ---------------------------------------------------------------------------

def euclidean(a, b):
    return float(np.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2))


def dijkstra(nodes, edges, start, goal):
    """Return (cost, path-list) for the shortest path from start to goal."""
    dist = {n: float('inf') for n in nodes}
    prev = {n: None for n in nodes}
    dist[start] = 0.0
    heap = [(0.0, start)]

    while heap:
        d, u = heapq.heappop(heap)
        if d > dist[u]:
            continue
        if u == goal:
            break
        for v, w in edges.get(u, []):
            nd = dist[u] + w
            if nd < dist[v]:
                dist[v] = nd
                prev[v] = u
                heapq.heappush(heap, (nd, v))

    # Reconstruct
    path = []
    cur = goal
    while cur is not None:
        path.append(cur)
        cur = prev[cur]
    path.reverse()
    return dist[goal], path


# ---------------------------------------------------------------------------
# ROS2 Node
# ---------------------------------------------------------------------------

class W102PathSimNode(Node):

    # Room / world constants (feet)
    ROOM_W = 10.0
    ROOM_H = 12.0

    ROBOT_W = 2.25   # 27 in -> ft
    ROBOT_H = 2.67   # 32 in -> ft
    SAFETY  = 1.75   # inflated obstacle margin ft

    START  = (0.0,  0.0)
    CHAIR  = (0.0,  4.0)
    JOHN   = (0.0, 10.0)

    GRAPH_NODES = {
        'S':  (0.0,  0.0),
        'L1': (-3.0, 0.0),
        'L2': (-3.0, 6.0),
        'R1': ( 3.0, 0.0),
        'R2': ( 3.0, 6.0),
        'G':  (0.0, 10.0),
    }

    def __init__(self):
        super().__init__('w102_path_sim')

        # Publishers
        self.wp_pub    = self.create_publisher(Point,  'w102/current_waypoint', 10)
        self.status_pub = self.create_publisher(String, 'w102/status', 10)

        # Resolve output directory relative to this file so it works wherever
        # the workspace is built.
        pkg_dir = os.path.dirname(os.path.abspath(__file__))
        self.results_dir = os.path.join(
            pkg_dir, '..', '..', '..', '..', '..', 'results'
        )
        # Fallback: always write next to the install tree results folder
        explicit = os.environ.get('W102_RESULTS_DIR')
        if explicit:
            self.results_dir = explicit
        os.makedirs(self.results_dir, exist_ok=True)

        self.get_logger().info('W102 Path Planning Node started.')
        self.get_logger().info(f'Results will be saved to: {os.path.abspath(self.results_dir)}')

        self._run_simulation()

    # ------------------------------------------------------------------
    def _build_graph(self):
        """Build weighted adjacency list. Direct S->G path is blocked."""
        N = self.GRAPH_NODES

        def edge(a, b):
            return (b, euclidean(N[a], N[b]))

        # Right path gets a negligible epsilon advantage so Dijkstra chooses it
        # when both paths share the same integer cost (14 ft).
        EPS = 1e-6

        def redge(a, b):
            label, cost = edge(a, b)
            return (label, cost - EPS)

        edges = {
            'S':  [redge('S', 'R1'), edge('S', 'L1')],
            'L1': [edge('L1', 'S'), edge('L1', 'L2')],
            'L2': [edge('L2', 'L1'), edge('L2', 'G')],
            'R1': [redge('R1', 'S'), redge('R1', 'R2')],
            'R2': [redge('R2', 'R1'), redge('R2', 'G')],
            'G':  [edge('G', 'L2'), redge('G', 'R2')],
        }
        return edges

    # ------------------------------------------------------------------
    def _run_simulation(self):
        self.get_logger().info('=' * 55)
        self.get_logger().info('  W102 Living-Room Path Planning Simulation')
        self.get_logger().info('=' * 55)

        # --- World description ---
        self.get_logger().info(f'Room         : {self.ROOM_W} ft x {self.ROOM_H} ft')
        self.get_logger().info(f'Robot (W102) : {self.ROBOT_W} ft x {self.ROBOT_H} ft footprint')
        self.get_logger().info(f'Start        : {self.START}')
        self.get_logger().info(f'Chair (obs)  : {self.CHAIR}  (safety margin {self.SAFETY} ft)')
        self.get_logger().info(f'Goal (John)  : {self.JOHN}')

        # --- Graph ---
        edges = self._build_graph()
        self.get_logger().info('\nGraph nodes:')
        for label, pos in self.GRAPH_NODES.items():
            self.get_logger().info(f'  {label:3s} = {pos}')

        self.get_logger().info('\nDirect path S->G BLOCKED (chair at (0,4) is in the way).')

        # --- Dijkstra ---
        cost, path = dijkstra(self.GRAPH_NODES, edges, 'S', 'G')
        self.get_logger().info(f'\nShortest path  : {" -> ".join(path)}')
        self.get_logger().info(f'Total cost     : {round(cost, 4):.4f} ft')

        waypoints = [self.GRAPH_NODES[n] for n in path]

        # --- Simulate movement ---
        self.get_logger().info('\nSimulating W102 movement:')
        for i, wp in enumerate(waypoints):
            label = path[i]
            self.get_logger().info(f'  Waypoint {i}: {label} = {wp}')

            msg = Point()
            msg.x, msg.y, msg.z = float(wp[0]), float(wp[1]), 0.0
            self.wp_pub.publish(msg)

            status = String()
            status.data = f'W102 reached waypoint {label} at {wp}'
            self.status_pub.publish(status)

            # Brief pause to allow ROS message propagation
            time.sleep(0.05)

        self.get_logger().info('\nW102 has reached John! Mission complete.')

        # --- Save artifacts ---
        self._save_csv(waypoints, path)
        self._save_plot(waypoints, path, cost)
        self._save_animation(waypoints)

        final_status = String()
        final_status.data = 'W102 mission complete. Artifacts saved.'
        self.status_pub.publish(final_status)

    # ------------------------------------------------------------------
    def _save_csv(self, waypoints, path):
        csv_path = os.path.join(self.results_dir, 'trajectory_points.csv')
        with open(csv_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['step', 'label', 'x_ft', 'y_ft'])
            for i, (label, wp) in enumerate(zip(path, waypoints)):
                writer.writerow([i, label, wp[0], wp[1]])
        self.get_logger().info(f'CSV saved : {csv_path}')

    # ------------------------------------------------------------------
    def _save_plot(self, waypoints, path, cost):
        fig, ax = plt.subplots(figsize=(7, 9))

        # Room boundary (origin at bottom-left; robot starts at centre-bottom)
        room_x0 = -self.ROOM_W / 2
        room = patches.Rectangle(
            (room_x0, 0), self.ROOM_W, self.ROOM_H,
            linewidth=2, edgecolor='black', facecolor='#f5f5f5', zorder=1
        )
        ax.add_patch(room)

        # Inflated obstacle (safety region)
        r_inf = self.SAFETY
        chair_inflate = patches.Circle(
            self.CHAIR, r_inf,
            linewidth=1.5, edgecolor='orange', facecolor='#ffe0b2',
            linestyle='--', zorder=2, label=f'Safety margin ({r_inf} ft)'
        )
        ax.add_patch(chair_inflate)

        # Chair obstacle
        chair_r = 0.75   # approximate chair radius ft
        chair_patch = patches.Circle(
            self.CHAIR, chair_r,
            linewidth=1.5, edgecolor='saddlebrown', facecolor='#a0522d',
            zorder=3, label='Chair obstacle'
        )
        ax.add_patch(chair_patch)
        ax.text(self.CHAIR[0], self.CHAIR[1], 'Chair',
                ha='center', va='center', fontsize=8, color='white', fontweight='bold', zorder=4)

        # Graph edges (all, greyed out)
        edges_display = [
            ('S', 'L1'), ('L1', 'L2'), ('L2', 'G'),
            ('S', 'R1'), ('R1', 'R2'), ('R2', 'G'),
        ]
        for a, b in edges_display:
            pa, pb = self.GRAPH_NODES[a], self.GRAPH_NODES[b]
            ax.plot([pa[0], pb[0]], [pa[1], pb[1]],
                    color='lightgray', linewidth=1.2, linestyle='--', zorder=3)

        # Blocked direct path
        ax.plot([self.START[0], self.JOHN[0]], [self.START[1], self.JOHN[1]],
                color='red', linewidth=1.5, linestyle=':', zorder=3,
                label='Blocked direct path')
        mid = ((self.START[0]+self.JOHN[0])/2, (self.START[1]+self.JOHN[1])/2)
        ax.text(mid[0]+0.15, mid[1], '✗ blocked', color='red', fontsize=7.5, zorder=5)

        # Planned path
        px = [w[0] for w in waypoints]
        py = [w[1] for w in waypoints]
        ax.plot(px, py, 'b-o', linewidth=2.5, markersize=7, zorder=6, label=f'Planned path ({cost:.1f} ft)')
        for label, wp in zip(path, waypoints):
            offset = (0.15, 0.15)
            ax.annotate(label, xy=wp, xytext=(wp[0]+offset[0], wp[1]+offset[1]),
                        fontsize=9, color='navy', fontweight='bold', zorder=7)

        # Robot footprint at start
        rw, rh = self.ROBOT_W, self.ROBOT_H
        robot_rect = patches.FancyBboxPatch(
            (self.START[0] - rw/2, self.START[1] - rh/2), rw, rh,
            boxstyle='round,pad=0.05', linewidth=1.5,
            edgecolor='blue', facecolor='#bbdefb', zorder=8, label='W102 robot footprint'
        )
        ax.add_patch(robot_rect)
        ax.text(self.START[0], self.START[1], 'W102',
                ha='center', va='center', fontsize=8, color='navy', fontweight='bold', zorder=9)

        # John (goal)
        ax.plot(*self.JOHN, marker='*', markersize=18, color='green', zorder=8, label='John (goal)')
        ax.text(self.JOHN[0]+0.2, self.JOHN[1]+0.15, 'John', fontsize=9, color='darkgreen',
                fontweight='bold', zorder=9)

        # Decorations
        ax.set_xlim(room_x0 - 0.5, -room_x0 + 0.5)
        ax.set_ylim(-1.0, self.ROOM_H + 0.5)
        ax.set_aspect('equal')
        ax.set_xlabel('x (ft)', fontsize=11)
        ax.set_ylabel('y (ft)', fontsize=11)
        ax.set_title('W102 Path Planning — Living Room\n'
                     f'Path: {" → ".join(path)}  |  Cost: {cost:.2f} ft',
                     fontsize=12, fontweight='bold')
        ax.legend(loc='upper right', fontsize=8, framealpha=0.9)
        ax.grid(True, linestyle=':', alpha=0.4)

        # Scale bar
        ax.annotate('', xy=(4.5, 0.3), xytext=(3.5, 0.3),
                    arrowprops=dict(arrowstyle='<->', color='black'))
        ax.text(4.0, 0.55, '1 ft', ha='center', fontsize=7)

        out = os.path.join(self.results_dir, 'trajectory_result.png')
        fig.tight_layout()
        fig.savefig(out, dpi=150, bbox_inches='tight')
        plt.close(fig)
        self.get_logger().info(f'Plot saved: {out}')

    # ------------------------------------------------------------------
    def _save_animation(self, waypoints):
        try:
            fig, ax = plt.subplots(figsize=(6, 8))

            room_x0 = -self.ROOM_W / 2
            room = patches.Rectangle(
                (room_x0, 0), self.ROOM_W, self.ROOM_H,
                linewidth=2, edgecolor='black', facecolor='#f5f5f5'
            )
            ax.add_patch(room)

            ax.add_patch(patches.Circle(self.CHAIR, self.SAFETY,
                linewidth=1.5, edgecolor='orange', facecolor='#ffe0b2', linestyle='--'))
            ax.add_patch(patches.Circle(self.CHAIR, 0.75,
                linewidth=1.5, edgecolor='saddlebrown', facecolor='#a0522d'))
            ax.text(*self.CHAIR, 'Chair', ha='center', va='center',
                    fontsize=8, color='white', fontweight='bold')

            ax.plot(*self.JOHN, marker='*', markersize=16, color='green')
            ax.text(self.JOHN[0]+0.2, self.JOHN[1]+0.15, 'John', color='darkgreen',
                    fontsize=9, fontweight='bold')

            # Full planned path (faint)
            px = [w[0] for w in waypoints]
            py = [w[1] for w in waypoints]
            ax.plot(px, py, 'b--', alpha=0.3, linewidth=1.5)

            robot_dot, = ax.plot([], [], 'bo', markersize=14, label='W102')
            trail_line, = ax.plot([], [], 'b-', linewidth=2.5, alpha=0.7)
            trail_x, trail_y = [], []

            # Interpolate frames between waypoints
            n_interp = 20
            frames_pts = []
            for i in range(len(waypoints) - 1):
                x0, y0 = waypoints[i]
                x1, y1 = waypoints[i + 1]
                for t in np.linspace(0, 1, n_interp, endpoint=False):
                    frames_pts.append((x0 + t*(x1-x0), y0 + t*(y1-y0)))
            frames_pts.append(waypoints[-1])

            def init():
                robot_dot.set_data([], [])
                trail_line.set_data([], [])
                return robot_dot, trail_line

            def update(frame):
                fx, fy = frames_pts[frame]
                robot_dot.set_data([fx], [fy])
                trail_x.append(fx)
                trail_y.append(fy)
                trail_line.set_data(trail_x, trail_y)
                return robot_dot, trail_line

            ax.set_xlim(room_x0 - 0.5, -room_x0 + 0.5)
            ax.set_ylim(-1.0, self.ROOM_H + 0.5)
            ax.set_aspect('equal')
            ax.set_title('W102 Navigation Simulation', fontweight='bold')
            ax.set_xlabel('x (ft)')
            ax.set_ylabel('y (ft)')
            ax.grid(True, linestyle=':', alpha=0.4)

            anim = FuncAnimation(fig, update, frames=len(frames_pts),
                                 init_func=init, blit=True, interval=60)

            gif_path = os.path.join(self.results_dir, 'w102_simulation.gif')
            anim.save(gif_path, writer=PillowWriter(fps=20))
            plt.close(fig)
            self.get_logger().info(f'GIF saved : {gif_path}')

        except Exception as e:
            self.get_logger().warn(f'GIF generation skipped: {e}')


# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = W102PathSimNode()
    # Spin briefly so publishers can flush, then shut down cleanly
    rclpy.spin_once(node, timeout_sec=0.5)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
