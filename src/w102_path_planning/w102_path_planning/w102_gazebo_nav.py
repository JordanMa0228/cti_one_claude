#!/usr/bin/env python3
"""
W102 Gazebo Navigation Node
Drives W102 through the planned right-side path in Gazebo using odometry feedback
and an explicit ALIGNING / DRIVING state machine.

Waypoints (metres, world frame):
  S  →  (0.000,  0.000) m  [start — room enlarged south so full rotation sweep fits]
  R1 →  (0.750,  0.000) m  [right of chair; 24 cm east-wall margin at 45-deg sweep]
  R2 →  (0.750,  1.829) m
  G  →  (0.000,  3.048) m  [John]

State machine per waypoint leg:
  ALIGNING  — pure rotation only; heading must stay within ALIGN_THRESH for
               ALIGN_HOLD_TICKS consecutive ticks before DRIVING is entered.
               Any tick outside the threshold resets the hold counter to zero.
  DRIVING   — forward + corrective angular; if heading error exceeds
               REALIGN_THRESH the robot returns to ALIGNING immediately.

Topics consumed:
  /odom  (nav_msgs/Odometry)
Topics published:
  /cmd_vel  (geometry_msgs/Twist)
  /w102/status (std_msgs/String)
"""

import math
import rclpy
import rclpy.parameter
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String


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
    KP_ANG  = 0.8    # rad/s per rad of heading error
    KP_LIN  = 0.6    # m/s per metre of remaining distance
    MAX_LIN = 0.12   # m/s   top forward speed
    MAX_ANG = 0.7    # rad/s top turn speed

    # ---- Arrival / alignment thresholds ----------------------------------
    ARRIVE_DIST      = 0.15  # m   — consider waypoint reached
    ALIGN_THRESH     = 0.08  # rad — heading must be inside this band …
    ALIGN_HOLD_TICKS = 10    # … for this many consecutive ticks before DRIVING
                             #   (10 × 0.05 s = 0.5 s of stable heading)
    REALIGN_THRESH   = 0.20  # rad — heading error that forces return to ALIGNING

    # ---- Post-arrival settle / stuck recovery ----------------------------
    SETTLE_TICKS  = 20   # zero-vel ticks after each waypoint  (20 × 0.05 s = 1.0 s)
    STUCK_TICKS   = 80   # DRIVING ticks without STUCK_DIST progress → recovery
    STUCK_DIST    = 0.03 # m  — minimum XY progress to reset stuck counter
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
        self.x   = 0.0
        self.y   = 0.0
        self.yaw = math.pi / 2
        self.odom_received = False

        # Mission state
        self.wp_idx       = 0
        self.mission_done = False

        # --- Explicit alignment state ------------------------------------
        # aligning=True  → ALIGNING state (pure rotation)
        # aligning=False → DRIVING state  (forward + correction)
        self.aligning        = True
        self._align_hold_cnt = 0   # consecutive ticks inside ALIGN_THRESH

        # Post-arrival settle
        self._settle_cnt = 0

        # Stuck detection (DRIVING state only)
        self._stuck_cnt  = 0
        self._ref_x      = 0.0
        self._ref_y      = 0.0

        # Recovery
        self._recover_cnt = 0

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
    def _enter_aligning(self):
        """Transition (back) into ALIGNING state and reset the hold counter."""
        self.aligning        = True
        self._align_hold_cnt = 0

    # ------------------------------------------------------------------
    def _control_loop(self):
        if self.mission_done or not self.odom_received:
            return

        # ---- 1. Post-arrival settle ------------------------------------
        if self._settle_cnt > 0:
            self._settle_cnt -= 1
            self._stop()
            return

        # ---- 2. Stuck recovery: reverse briefly ------------------------
        if self._recover_cnt > 0:
            self._recover_cnt -= 1
            cmd = Twist()
            cmd.linear.x = -0.06
            self.cmd_pub.publish(cmd)
            if self._recover_cnt == 0:
                self.get_logger().info('Recovery complete — re-aligning')
                self._enter_aligning()
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
            self.wp_idx     += 1
            self._enter_aligning()          # always start next leg in ALIGNING
            self._settle_cnt = self.SETTLE_TICKS
            self._stuck_cnt  = 0
            self._ref_x      = self.x
            self._ref_y      = self.y
            return

        # ---- 5. Heading error ------------------------------------------
        target_yaw = math.atan2(dy, dx)
        angle_err  = self._wrap(target_yaw - self.yaw)

        # ---- 6. State-machine transition --------------------------------
        #
        #  ALIGNING → DRIVING:
        #    |angle_err| < ALIGN_THRESH  for ALIGN_HOLD_TICKS consecutive ticks.
        #    Any single tick outside the band resets the counter to zero.
        #    This prevents a momentary threshold crossing (due to inertia / noise)
        #    from prematurely unlocking forward motion.
        #
        #  DRIVING → ALIGNING:
        #    |angle_err| > REALIGN_THRESH at any single tick.
        #
        if self.aligning:
            if abs(angle_err) < self.ALIGN_THRESH:
                self._align_hold_cnt += 1
                if self._align_hold_cnt >= self.ALIGN_HOLD_TICKS:
                    self.aligning        = False
                    self._align_hold_cnt = 0
                    self._stuck_cnt      = 0
                    self._ref_x          = self.x
                    self._ref_y          = self.y
                    self.get_logger().info(
                        f'  → DRIVING toward wp{self.wp_idx} '
                        f'({tx:.2f},{ty:.2f})  yaw={math.degrees(self.yaw):.1f}°'
                    )
            else:
                self._align_hold_cnt = 0   # out of band — restart the hold counter
        else:
            if abs(angle_err) > self.REALIGN_THRESH:
                self._enter_aligning()
                self.get_logger().info(
                    f'  → ALIGNING (err={math.degrees(angle_err):.1f}°) '
                    f'wp{self.wp_idx}'
                )

        # ---- 7. Stuck detection (DRIVING state only) -------------------
        if not self.aligning:
            moved = math.hypot(self.x - self._ref_x, self.y - self._ref_y)
            if moved >= self.STUCK_DIST:
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
            self._stuck_cnt = 0
            self._ref_x = self.x
            self._ref_y = self.y

        # ---- 8. Velocity command ---------------------------------------
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
