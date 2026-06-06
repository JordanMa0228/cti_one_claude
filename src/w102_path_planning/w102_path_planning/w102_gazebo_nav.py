#!/usr/bin/env python3
"""
W102 Gazebo Navigation Node
Drives W102 through the planned right-side path in Gazebo using odometry feedback
and a proportional angular+linear controller.

Waypoints (metres, world frame):
  S  →  (0.000,  0.000) m  [start — room enlarged south so full rotation sweep fits]
  R1 →  (0.750,  0.000) m  [right of chair; safe east-wall clearance during turn]
  R2 →  (0.750,  1.829) m
  G  →  (0.000,  3.048) m  [John]

  R1/R2 x = 0.75 m chosen so the 0.813 m-wide robot body clears both constraints:
    east wall inner face (1.524 m) — 24 cm margin at peak 45-deg rotation
      (half-diagonal 0.532 m; 1.524 - 0.75 - 0.532 = 0.242 m)
    chair right edge (0.25 m)      —  9 cm margin (robot left edge at 0.344 m)

Root-cause note:
  KP_ANG=2.0 caused the 90-deg R1->R2 turn to oscillate ±0.3 rad around π/2.
  Each oscillation cycle briefly entered drive mode and pushed the robot slightly east.
  This positive-feedback drift eventually pinned the robot against the east wall.
  Fix: KP_ANG=0.8, MAX_ANG=0.7 (ramp-down begins earlier, far less overshoot),
  plus a 1-sec settle after each waypoint and stuck-detection+recovery.

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
# Robot is spawned at (0, 0) facing +Y  (yaw = π/2)
# ---------------------------------------------------------------------------
FT = 0.3048   # feet → metres

WAYPOINTS = [
    (0.75,      0.0),    # R1
    (0.75,      6 * FT), # R2
    (0   * FT, 10 * FT), # G  = John
]
WAYPOINT_LABELS = ['R1', 'R2', 'G (John)']


class W102GazeboNav(Node):

    # ---- Controller gains ------------------------------------------------
    # KP_ANG reduced from 2.0 → 0.8:  at ALIGN_THRESH (0.08 rad), commanded
    # angular speed is now 0.064 rad/s instead of 0.16 rad/s.  Combined with
    # the lower MAX_ANG cap, angular momentum at turn-end is small enough that
    # the robot does not overshoot past the REALIGN_THRESH band.
    KP_ANG  = 0.8    # rad/s per rad of heading error
    KP_LIN  = 0.6    # m/s per metre of remaining distance
    MAX_LIN = 0.12   # m/s   top forward speed
    MAX_ANG = 0.7    # rad/s top turn speed (was 1.2 — lower cap reduces inertial overshoot)

    # ---- Heading tolerances -----------------------------------------------
    ARRIVE_DIST    = 0.15   # m   — consider waypoint reached
    ALIGN_THRESH   = 0.08   # rad — exit rotation-only and start driving
    REALIGN_THRESH = 0.25   # rad — re-enter rotation-only while driving

    # ---- Settle / stuck parameters ----------------------------------------
    SETTLE_TICKS  = 20   # zero-vel ticks after each waypoint  (20 × 0.05 s = 1.0 s)
    STUCK_TICKS   = 80   # drive-mode ticks without STUCK_DIST progress → recovery
    STUCK_DIST    = 0.03 # m  — minimum XY progress to reset the stuck counter
    RECOVER_TICKS = 12   # reverse ticks  (12 × 0.05 s = 0.6 s)

    def __init__(self):
        super().__init__('w102_gazebo_nav',
                         automatically_declare_parameters_from_overrides=True)
        self.set_parameters([
            rclpy.parameter.Parameter('use_sim_time',
                                      rclpy.parameter.Parameter.Type.BOOL, True)
        ])

        self.cmd_pub    = self.create_publisher(Twist,  '/cmd_vel',     10)
        self.status_pub = self.create_publisher(String, '/w102/status', 10)
        self.odom_sub   = self.create_subscription(
            Odometry, '/odom', self._odom_cb, 10)

        # Odometry state
        self.x = 0.0
        self.y = 0.0
        self.yaw = math.pi / 2   # facing +Y initially
        self.odom_received = False

        # Mission state
        self.wp_idx      = 0
        self.mission_done = False
        self.aligning    = True   # start each waypoint in rotation-only mode

        # Settle counter: zero-velocity hold ticks after each waypoint arrival
        self._settle_cnt = 0

        # Stuck detection (drive-mode only)
        self._stuck_cnt  = 0
        self._ref_x      = 0.0
        self._ref_y      = 0.0

        # Recovery counter: reverse ticks when stuck detected
        self._recover_cnt = 0

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
            self._ref_x = self.x
            self._ref_y = self.y
            self.get_logger().info(
                f'/odom received — robot at ({self.x:.3f}, {self.y:.3f}), '
                f'yaw={math.degrees(self.yaw):.1f}°'
            )

    # ------------------------------------------------------------------
    def _control_loop(self):
        if self.mission_done or not self.odom_received:
            return

        # ---- 1. Post-arrival settle: hold zero velocity ----------------
        if self._settle_cnt > 0:
            self._settle_cnt -= 1
            self._stop()
            return

        # ---- 2. Stuck recovery: reverse briefly ------------------------
        if self._recover_cnt > 0:
            self._recover_cnt -= 1
            cmd = Twist()
            cmd.linear.x = -0.06   # slow reverse
            self.cmd_pub.publish(cmd)
            if self._recover_cnt == 0:
                self.get_logger().info('Recovery complete — re-aligning')
                self.aligning = True
                self._stuck_cnt = 0
                self._ref_x = self.x
                self._ref_y = self.y
            return

        # ---- 3. Mission complete check ---------------------------------
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

        # ---- 4. Waypoint arrival ---------------------------------------
        if dist < self.ARRIVE_DIST:
            label = WAYPOINT_LABELS[self.wp_idx]
            self.get_logger().info(
                f'  ✓ Waypoint {self.wp_idx + 1}/{len(WAYPOINTS)} reached: '
                f'{label}  ({tx:.3f}, {ty:.3f})'
            )
            self._publish_status(f'W102 reached {label} ({tx:.2f}, {ty:.2f}) m')
            self._stop()
            self.wp_idx      += 1
            self.aligning    = True
            self._settle_cnt = self.SETTLE_TICKS   # 1-sec dwell before next turn
            self._stuck_cnt  = 0
            self._ref_x      = self.x
            self._ref_y      = self.y
            return

        # ---- 5. Heading to target --------------------------------------
        target_yaw = math.atan2(dy, dx)
        angle_err  = self._wrap(target_yaw - self.yaw)

        # Hysteresis: prevents chatter at the mode boundary
        if self.aligning and abs(angle_err) < self.ALIGN_THRESH:
            self.aligning = False
        elif not self.aligning and abs(angle_err) > self.REALIGN_THRESH:
            self.aligning = True

        # ---- 6. Stuck detection (drive mode only) ----------------------
        if not self.aligning:
            moved = math.hypot(self.x - self._ref_x, self.y - self._ref_y)
            if moved >= self.STUCK_DIST:
                # made progress — reset counter and reference
                self._stuck_cnt = 0
                self._ref_x = self.x
                self._ref_y = self.y
            else:
                self._stuck_cnt += 1
                if self._stuck_cnt >= self.STUCK_TICKS:
                    self.get_logger().warn(
                        f'Stuck at ({self.x:.3f},{self.y:.3f}) '
                        f'targeting wp{self.wp_idx} — reversing'
                    )
                    self._recover_cnt = self.RECOVER_TICKS
                    self._stuck_cnt   = 0
                    return
        else:
            # During rotation, reset the XY reference so the clock starts
            # fresh once drive mode begins.
            self._stuck_cnt = 0
            self._ref_x = self.x
            self._ref_y = self.y

        # ---- 7. Velocity command ---------------------------------------
        cmd = Twist()
        if self.aligning:
            cmd.angular.z = float(
                max(-self.MAX_ANG, min(self.MAX_ANG, self.KP_ANG * angle_err)))
        else:
            cmd.linear.x  = float(min(self.MAX_LIN, self.KP_LIN * dist))
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
