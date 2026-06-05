#!/usr/bin/env python3
"""
W102 RViz2 Visualization Marker Publisher.

Publishes static and live markers so RViz2 can show:
  - Room boundary walls
  - Chair obstacle + safety margin
  - Graph waypoints (S, R1, R2, L1, L2, G)
  - Planned path highlight
  - John (goal) figure
  - Live robot odometry trail

Topics published:
  /w102/markers        (visualization_msgs/MarkerArray)  — room, chair, waypoints
  /w102/odom_trail     (nav_msgs/Path)                   — live driven path
"""

import math
import rclpy
import rclpy.parameter
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import ColorRGBA
from builtin_interfaces.msg import Duration


FT = 0.3048   # feet → metres

# ── World constants (metres) ─────────────────────────────────────────────────
ROOM_W  = 10 * FT      # 3.048 m  east–west
ROOM_H  = 12 * FT      # 3.658 m  north–south
CHAIR   = (0.0, 4 * FT)
SAFETY  = 1.75 * FT    # 0.533 m
JOHN    = (0.0, 10 * FT)

NODES = {
    'S':  ( 0.000,  0.750),
    'R1': ( 0.700,  0.750),
    'R2': ( 0.700,  2.950),
    'G':  ( 0.000,  2.950),
    'L1': (-0.700,  0.750),
    'L2': (-0.700,  2.950),
}

CHOSEN_PATH   = ['S', 'R1', 'R2', 'G']
INACTIVE_LEGS = [('S', 'L1'), ('L1', 'L2'), ('L2', 'G')]


def _color(r, g, b, a=1.0) -> ColorRGBA:
    c = ColorRGBA()
    c.r, c.g, c.b, c.a = float(r), float(g), float(b), float(a)
    return c


def _pt(x, y, z=0.05) -> Point:
    p = Point()
    p.x = float(x)
    p.y = float(y)
    p.z = float(z)
    return p


class W102VizMarkers(Node):

    def __init__(self):
        super().__init__('w102_viz_markers',
                         automatically_declare_parameters_from_overrides=True)
        self.set_parameters([
            rclpy.parameter.Parameter('use_sim_time',
                                      rclpy.parameter.Parameter.Type.BOOL, True)
        ])

        latching = QoSProfile(depth=1,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.marker_pub = self.create_publisher(
            MarkerArray, '/w102/markers', latching)
        self.path_pub   = self.create_publisher(
            Path, '/w102/odom_trail', 10)
        self.odom_sub   = self.create_subscription(
            Odometry, '/odom', self._odom_cb, 10)

        self._trail: Path = Path()
        self._trail.header.frame_id = 'odom'

        # Publish static markers once (latched) and refresh every 2 s
        self._publish_static()
        self.create_timer(2.0, self._publish_static)

        self.get_logger().info('W102 RViz2 marker publisher started.')

    # ── Static markers ───────────────────────────────────────────────────────
    def _publish_static(self):
        ma = MarkerArray()
        uid = 0

        def add(m):
            nonlocal uid
            m.id = uid
            uid += 1
            ma.markers.append(m)

        now = self.get_clock().now().to_msg()
        forever = Duration(sec=0, nanosec=0)   # 0 = never expire

        def base(ns: str, mtype: int) -> Marker:
            m = Marker()
            m.header.frame_id = 'odom'
            m.header.stamp    = now
            m.ns              = ns
            m.action          = Marker.ADD
            m.type            = mtype
            m.lifetime        = forever
            m.pose.orientation.w = 1.0
            return m

        # ── Floor (thin flat box, light wood colour) ─────────────────────
        fl = base('room', Marker.CUBE)
        fl.pose.position.x = 0.0
        fl.pose.position.y = ROOM_H / 2
        fl.pose.position.z = -0.01
        fl.scale.x, fl.scale.y, fl.scale.z = ROOM_W, ROOM_H, 0.01
        fl.color = _color(0.76, 0.60, 0.42, 0.6)
        add(fl)

        # ── Room walls (4 thin planes) ───────────────────────────────────
        wall_h = 0.4   # visible height in RViz
        walls = [
            # (cx, cy, sx, sy) — all float so geometry_msgs C extension is happy
            (0.0,        0.0,        ROOM_W, 0.04),   # south
            (0.0,        ROOM_H,     ROOM_W, 0.04),   # north
            (-ROOM_W/2,  ROOM_H/2,   0.04,  ROOM_H), # west
            ( ROOM_W/2,  ROOM_H/2,   0.04,  ROOM_H), # east
        ]
        for (cx, cy, sx, sy) in walls:
            w = base('room', Marker.CUBE)
            w.pose.position.x = float(cx)
            w.pose.position.y = float(cy)
            w.pose.position.z = float(wall_h / 2)
            w.scale.x, w.scale.y, w.scale.z = sx, sy, wall_h
            w.color = _color(0.90, 0.88, 0.82, 0.9)
            add(w)

        # ── Safety margin (transparent orange cylinder) ──────────────────
        sm = base('chair', Marker.CYLINDER)
        sm.pose.position.x = CHAIR[0]
        sm.pose.position.y = CHAIR[1]
        sm.pose.position.z = 0.3
        sm.scale.x = sm.scale.y = SAFETY * 2
        sm.scale.z = 0.6
        sm.color = _color(1.0, 0.55, 0.0, 0.25)
        add(sm)

        # ── Chair obstacle (brown cylinder) ─────────────────────────────
        ch = base('chair', Marker.CYLINDER)
        ch.pose.position.x = CHAIR[0]
        ch.pose.position.y = CHAIR[1]
        ch.pose.position.z = 0.45
        ch.scale.x = ch.scale.y = 0.55
        ch.scale.z = 0.9
        ch.color = _color(0.40, 0.25, 0.10, 1.0)
        add(ch)

        # Chair label
        cl = base('labels', Marker.TEXT_VIEW_FACING)
        cl.pose.position.x = CHAIR[0]
        cl.pose.position.y = CHAIR[1]
        cl.pose.position.z = 1.1
        cl.scale.z = 0.18
        cl.text  = 'Chair'
        cl.color = _color(1, 1, 1, 1)
        add(cl)

        # ── John (green capsule + label) ─────────────────────────────────
        jn = base('john', Marker.CYLINDER)
        jn.pose.position.x = JOHN[0]
        jn.pose.position.y = JOHN[1]
        jn.pose.position.z = 0.85
        jn.scale.x = jn.scale.y = 0.38
        jn.scale.z = 1.70
        jn.color = _color(0.2, 0.6, 0.2, 1.0)
        add(jn)

        jl = base('labels', Marker.TEXT_VIEW_FACING)
        jl.pose.position.x = JOHN[0]
        jl.pose.position.y = JOHN[1]
        jl.pose.position.z = 1.95
        jl.scale.z = 0.20
        jl.text  = 'John (Goal)'
        jl.color = _color(0.2, 0.9, 0.2, 1)
        add(jl)

        # ── Inactive graph edges (grey dashes) ───────────────────────────
        for (a, b) in INACTIVE_LEGS:
            e = base('graph_inactive', Marker.LINE_STRIP)
            e.scale.x = 0.02
            e.color   = _color(0.6, 0.6, 0.6, 0.5)
            e.points  = [_pt(*NODES[a]), _pt(*NODES[b])]
            add(e)

        # ── Chosen path edges (bright blue) ──────────────────────────────
        for i in range(len(CHOSEN_PATH) - 1):
            a, b = CHOSEN_PATH[i], CHOSEN_PATH[i + 1]
            e = base('graph_chosen', Marker.LINE_STRIP)
            e.scale.x = 0.04
            e.color   = _color(0.0, 0.5, 1.0, 0.9)
            e.points  = [_pt(*NODES[a], 0.06), _pt(*NODES[b], 0.06)]
            add(e)

        # ── Waypoint spheres ─────────────────────────────────────────────
        wp_colors = {
            'S':  _color(0.2, 0.9, 0.2, 1.0),   # green  — start
            'G':  _color(1.0, 0.8, 0.0, 1.0),   # gold   — goal
            'R1': _color(0.0, 0.7, 1.0, 1.0),   # cyan   — chosen path
            'R2': _color(0.0, 0.7, 1.0, 1.0),
            'L1': _color(0.5, 0.5, 0.5, 0.7),   # grey   — unchosen
            'L2': _color(0.5, 0.5, 0.5, 0.7),
        }
        for label, (nx, ny) in NODES.items():
            sp = base('waypoints', Marker.SPHERE)
            sp.pose.position.x = nx
            sp.pose.position.y = ny
            sp.pose.position.z = 0.08
            sp.scale.x = sp.scale.y = sp.scale.z = 0.12
            sp.color = wp_colors.get(label, _color(1, 1, 1))
            add(sp)

            tx = base('labels', Marker.TEXT_VIEW_FACING)
            tx.pose.position.x = nx + 0.10
            tx.pose.position.y = ny + 0.10
            tx.pose.position.z = 0.25
            tx.scale.z = 0.15
            tx.text  = label
            tx.color = _color(1, 1, 1, 1)
            add(tx)

        self.marker_pub.publish(ma)

    # ── Live odometry trail ──────────────────────────────────────────────────
    def _odom_cb(self, msg: Odometry):
        ps = PoseStamped()
        ps.header = msg.header
        ps.header.frame_id = 'odom'
        ps.pose = msg.pose.pose
        self._trail.poses.append(ps)
        self._trail.header.stamp = msg.header.stamp
        self.path_pub.publish(self._trail)


def main(args=None):
    rclpy.init(args=args)
    node = W102VizMarkers()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
