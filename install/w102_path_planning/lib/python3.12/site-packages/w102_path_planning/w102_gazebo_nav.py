#!/usr/bin/env python3
"""
W102 Gazebo Navigation Node
Drives W102 through the planned right-side path in Gazebo using odometry feedback
and a proportional angular+linear controller.

Waypoints (feet → metres, 1 ft = 0.3048 m):
  S  (0,    0)    ft  →  (0.000,  0.300) m  [start, slight offset from south wall]
  R1 (3,    0)    ft  →  (0.914,  0.300) m
  R2 (3,    6)    ft  →  (0.914,  1.829) m
  G  (0,   10)    ft  →  (0.000,  3.048) m  [John]

Topics consumed:
  /odom  (nav_msgs/Odometry)  — provided by ros-gz bridge from Gazebo diff-drive plugin

Topics published:
  /cmd_vel  (geometry_msgs/Twist) — consumed by ros-gz bridge → Gazebo
  /w102/status (std_msgs/String)  — human-readable status
"""

import math
import rclpy
import rclpy.parameter
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String


# ---------------------------------------------------------------------------
# Waypoints in metres  (world frame, matching the SDF)
# Robot is spawned at (0, 0.3) facing +Y  (yaw = π/2)
# ---------------------------------------------------------------------------
FT = 0.3048   # feet → metres

WAYPOINTS = [
    (3 * FT,  0 * FT + 0.3),   # R1 — same y-row as robot start
    (3 * FT,  6 * FT),          # R2
    (0 * FT, 10 * FT),          # G  = John
]
WAYPOINT_LABELS = ['R1', 'R2', 'G (John)']


class W102GazeboNav(Node):

    # Controller gains
    KP_ANG  = 2.0    # rad/s per rad of heading error
    KP_LIN  = 0.6    # m/s per metre of remaining distance
    MAX_LIN = 0.12   # m/s   top forward speed
    MAX_ANG = 1.2    # rad/s top turn speed

    # Tolerances
    ARRIVE_DIST  = 0.12   # m  — consider waypoint reached
    ALIGN_THRESH = 0.08   # rad — start moving forward only when aligned

    def __init__(self):
        super().__init__('w102_gazebo_nav',
                         automatically_declare_parameters_from_overrides=True)
        # Use Gazebo simulation clock so timing matches the physics
        self.set_parameters([
            rclpy.parameter.Parameter('use_sim_time',
                                      rclpy.parameter.Parameter.Type.BOOL, True)
        ])

        self.cmd_pub    = self.create_publisher(Twist,  '/cmd_vel',     10)
        self.status_pub = self.create_publisher(String, '/w102/status', 10)
        self.odom_sub   = self.create_subscription(
            Odometry, '/odom', self._odom_cb, 10)

        # State
        self.x = 0.0
        self.y = 0.3
        self.yaw = math.pi / 2   # facing +Y initially
        self.odom_received = False
        self.wp_idx = 0
        self.mission_done = False

        # 20 Hz control loop
        self.timer = self.create_timer(0.05, self._control_loop)

        self.get_logger().info('W102 Gazebo Nav started — waiting for first /odom …')

    # ------------------------------------------------------------------
    def _odom_cb(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

        if not self.odom_received:
            self.odom_received = True
            self.get_logger().info(
                f'/odom received — robot at ({self.x:.3f}, {self.y:.3f}), '
                f'yaw={math.degrees(self.yaw):.1f}°'
            )

    # ------------------------------------------------------------------
    def _control_loop(self):
        if self.mission_done or not self.odom_received:
            return

        if self.wp_idx >= len(WAYPOINTS):
            self._stop()
            self.mission_done = True
            self.get_logger().info('✓ W102 has reached John!  Mission complete.')
            self._publish_status('Mission complete — W102 reached John.')
            return

        tx, ty = WAYPOINTS[self.wp_idx]
        dx = tx - self.x
        dy = ty - self.y
        dist = math.hypot(dx, dy)

        if dist < self.ARRIVE_DIST:
            label = WAYPOINT_LABELS[self.wp_idx]
            self.get_logger().info(
                f'  ✓ Waypoint {self.wp_idx + 1}/{len(WAYPOINTS)} reached: '
                f'{label}  ({tx:.3f}, {ty:.3f})'
            )
            self._publish_status(f'W102 reached {label} ({tx:.2f}, {ty:.2f}) m')
            self._stop()
            self.wp_idx += 1
            return

        # Heading to target
        target_yaw = math.atan2(dy, dx)
        angle_err  = self._wrap(target_yaw - self.yaw)

        cmd = Twist()

        if abs(angle_err) > self.ALIGN_THRESH:
            # Pure rotation phase
            cmd.angular.z = float(
                max(-self.MAX_ANG, min(self.MAX_ANG, self.KP_ANG * angle_err)))
        else:
            # Drive forward with gentle heading correction
            cmd.linear.x = float(
                min(self.MAX_LIN, self.KP_LIN * dist))
            cmd.angular.z = float(
                max(-self.MAX_ANG * 0.5,
                    min(self.MAX_ANG * 0.5, self.KP_ANG * angle_err)))

        self.cmd_pub.publish(cmd)

    # ------------------------------------------------------------------
    def _stop(self):
        self.cmd_pub.publish(Twist())

    def _publish_status(self, text: str):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    @staticmethod
    def _wrap(angle: float) -> float:
        """Wrap angle to [-π, π]."""
        while angle >  math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = W102GazeboNav()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        try:
            node._stop()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
