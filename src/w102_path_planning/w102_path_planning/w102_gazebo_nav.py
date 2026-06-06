#!/usr/bin/env python3
"""
W102 Gazebo Navigation Node — three-state controller (ALIGNING / BRAKING / DRIVING)

Waypoints (metres, world frame):
  S  →  (0.000,  0.000) m
  R1 →  (0.750,  0.000) m
  R2 →  (0.750,  1.829) m
  G  →  (0.000,  3.048) m  [John]

Controller state machine (per waypoint leg):

  ALIGNING  — pure angular only.
               Transitions to BRAKING when |angle_err| < COARSE_THRESH (0.20 rad).
               Hard timeout: after ALIGN_TIMEOUT_TICKS ticks without reaching
               COARSE_THRESH (indicating a stall / wall contact during spin),
               also enters BRAKING to break the spin and re-evaluate.

  BRAKING   — zero velocity for BRAKE_TICKS ticks so angular inertia dissipates.
               After brake:
                 |angle_err| < FINE_THRESH (0.12 rad)  →  DRIVING
                 otherwise                              →  ALIGNING (re-rotate)

  DRIVING   — forward + corrective angular.
               |angle_err| > REALIGN_THRESH (0.25 rad) →  ALIGNING

Why previous fixes failed:
  1. Single-tick threshold (original): transient threshold crossing triggered
     premature forward motion → heading disturbed → oscillation.
  2. Hold counter (previous fix): correct idea but if oscillation amplitude
     exceeds ALIGN_THRESH the counter never saturates → robot spins indefinitely
     → caster drag causes eastward drift → at x≈0.99 the east corner contacts
     the east wall at 45° of rotation → physical stall at 45°.
  The BRAKING state eliminates both problems by killing inertia before committing.

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
FT = 0.3048

WAYPOINTS = [
    (0.75,      0.0),
    (0.75,      6 * FT),
    (0   * FT, 10 * FT),
]
WAYPOINT_LABELS = ['R1', 'R2', 'G (John)']

# Controller states
_ALIGNING = 'ALIGNING'
_BRAKING  = 'BRAKING'
_DRIVING  = 'DRIVING'


class W102GazeboNav(Node):

    # ---- Gains -----------------------------------------------------------
    KP_ANG  = 0.8
    KP_LIN  = 0.6
    MAX_LIN = 0.12   # m/s
    MAX_ANG = 0.6    # rad/s  (kept low to reduce angular momentum)

    # ---- Arrival ---------------------------------------------------------
    ARRIVE_DIST = 0.15   # m

    # ---- Alignment thresholds -------------------------------------------
    COARSE_THRESH        = 0.20   # rad: ALIGNING → BRAKING
    FINE_THRESH          = 0.12   # rad: after brake, enter DRIVING
    REALIGN_THRESH       = 0.25   # rad: DRIVING → ALIGNING
    ALIGN_TIMEOUT_TICKS  = 60     # ticks before forced brake (3 s at 20 Hz)
                                  # prevents infinite spin / wall-contact drift

    # ---- Braking --------------------------------------------------------
    BRAKE_TICKS = 15   # zero-vel ticks (0.75 s at 20 Hz)

    # ---- Post-arrival settle / stuck recovery ---------------------------
    SETTLE_TICKS  = 20
    STUCK_TICKS   = 80
    STUCK_DIST    = 0.03
    RECOVER_TICKS = 12

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

        # Odometry
        self.x   = 0.0
        self.y   = 0.0
        self.yaw = math.pi / 2
        self.odom_received = False

        # Mission
        self.wp_idx       = 0
        self.mission_done = False

        # Nav state machine
        self._nav_state       = _ALIGNING
        self._brake_cnt       = 0
        self._align_tick_cnt  = 0   # ticks spent in current ALIGNING phase (timeout guard)

        # Post-arrival settle
        self._settle_cnt = 0

        # Stuck detection (DRIVING only)
        self._stuck_cnt  = 0
        self._ref_x      = 0.0
        self._ref_y      = 0.0
        self._recover_cnt = 0

        # Debug logging throttle
        self._dbg_cnt = 0

        self.timer = self.create_timer(0.05, self._control_loop)
        self.get_logger().info('W102 Nav started — waiting for /odom …')

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
                f'/odom received — ({self.x:.3f},{self.y:.3f}) '
                f'yaw={math.degrees(self.yaw):.1f}°'
            )

    # ------------------------------------------------------------------
    def _set_state(self, new_state: str, reason: str = ''):
        if self._nav_state != new_state:
            self.get_logger().info(
                f'  [{self._nav_state}→{new_state}] '
                f'x={self.x:.3f} y={self.y:.3f} '
                f'yaw={math.degrees(self.yaw):.1f}° '
                f'wp{self.wp_idx}  {reason}'
            )
            self._nav_state = new_state
            self._align_tick_cnt = 0
            self._dbg_cnt = 0

    # ------------------------------------------------------------------
    def _control_loop(self):
        if self.mission_done or not self.odom_received:
            return

        # ---- 1. Post-arrival settle ------------------------------------
        if self._settle_cnt > 0:
            self._settle_cnt -= 1
            self._stop()
            return

        # ---- 2. Stuck recovery: reverse briefly -----------------------
        if self._recover_cnt > 0:
            self._recover_cnt -= 1
            cmd = Twist()
            cmd.linear.x = -0.06
            self.cmd_pub.publish(cmd)
            if self._recover_cnt == 0:
                self.get_logger().info('Recovery complete — re-aligning')
                self._set_state(_ALIGNING, 'post-recovery')
                self._stuck_cnt = 0
                self._ref_x = self.x
                self._ref_y = self.y
            return

        # ---- 3. Mission complete ---------------------------------------
        if self.wp_idx >= len(WAYPOINTS):
            self._stop()
            self.mission_done = True
            self.get_logger().info('✓ Mission complete — W102 reached John.')
            self._publish_status('Mission complete.')
            return

        tx, ty = WAYPOINTS[self.wp_idx]
        dx = tx - self.x
        dy = ty - self.y
        dist = math.hypot(dx, dy)

        # ---- 4. Waypoint arrival ---------------------------------------
        if dist < self.ARRIVE_DIST:
            label = WAYPOINT_LABELS[self.wp_idx]
            self.get_logger().info(
                f'  ✓ wp{self.wp_idx+1} {label} reached  '
                f'({self.x:.3f},{self.y:.3f})'
            )
            self._publish_status(f'W102 reached {label}')
            self._stop()
            self.wp_idx      += 1
            self._set_state(_ALIGNING, 'new waypoint')
            self._settle_cnt  = self.SETTLE_TICKS
            self._stuck_cnt   = 0
            self._ref_x       = self.x
            self._ref_y       = self.y
            return

        target_yaw = math.atan2(dy, dx)
        angle_err  = self._wrap(target_yaw - self.yaw)

        # ---- 5. Debug logging (every 20 ticks = 1 sec) ----------------
        self._dbg_cnt += 1
        if self._dbg_cnt % 20 == 0:
            self.get_logger().info(
                f'  [DBG/{self._nav_state}] wp{self.wp_idx} '
                f'pos=({self.x:.3f},{self.y:.3f}) '
                f'yaw={math.degrees(self.yaw):.1f}° '
                f'tgt={math.degrees(target_yaw):.1f}° '
                f'err={math.degrees(angle_err):.1f}° '
                f'dist={dist:.3f}'
            )

        # ---- 6. State machine ------------------------------------------
        cmd = Twist()

        if self._nav_state == _ALIGNING:
            self._align_tick_cnt += 1

            if abs(angle_err) < self.COARSE_THRESH:
                # Close enough — stop spinning so inertia can die
                self._set_state(_BRAKING,
                    f'err={math.degrees(angle_err):.1f}° < {math.degrees(self.COARSE_THRESH):.0f}°')
                self._brake_cnt = self.BRAKE_TICKS
                # fall through to BRAKING this tick (zero cmd)

            elif self._align_tick_cnt >= self.ALIGN_TIMEOUT_TICKS:
                # Spinning too long without reaching COARSE_THRESH.
                # Likely stalled against wall or persistent oscillation.
                # Force a brake to let physics settle, then re-evaluate.
                self._set_state(_BRAKING,
                    f'timeout after {self._align_tick_cnt} ticks '
                    f'err={math.degrees(angle_err):.1f}°')
                self._brake_cnt = self.BRAKE_TICKS
                # zero cmd this tick

            else:
                cmd.angular.z = float(
                    max(-self.MAX_ANG,
                        min(self.MAX_ANG, self.KP_ANG * angle_err)))

        if self._nav_state == _BRAKING:
            # Zero velocity — let inertia die
            self._brake_cnt -= 1
            if self._brake_cnt <= 0:
                if abs(angle_err) < self.FINE_THRESH:
                    self._set_state(_DRIVING,
                        f'err={math.degrees(angle_err):.1f}° after brake')
                    self._stuck_cnt = 0
                    self._ref_x = self.x
                    self._ref_y = self.y
                else:
                    self._set_state(_ALIGNING,
                        f'err={math.degrees(angle_err):.1f}° still too large — re-rotate')
            # cmd stays zero

        elif self._nav_state == _DRIVING:
            if abs(angle_err) > self.REALIGN_THRESH:
                self._set_state(_ALIGNING,
                    f'err={math.degrees(angle_err):.1f}° > {math.degrees(self.REALIGN_THRESH):.0f}°')
                cmd.angular.z = float(
                    max(-self.MAX_ANG,
                        min(self.MAX_ANG, self.KP_ANG * angle_err)))
            else:
                cmd.linear.x  = float(min(self.MAX_LIN, self.KP_LIN * dist))
                cmd.angular.z = float(
                    max(-self.MAX_ANG * 0.5,
                        min(self.MAX_ANG * 0.5, self.KP_ANG * angle_err)))

                # ---- Stuck detection (DRIVING only) --------------------
                moved = math.hypot(self.x - self._ref_x, self.y - self._ref_y)
                if moved >= self.STUCK_DIST:
                    self._stuck_cnt = 0
                    self._ref_x = self.x
                    self._ref_y = self.y
                else:
                    self._stuck_cnt += 1
                    if self._stuck_cnt >= self.STUCK_TICKS:
                        self.get_logger().warn(
                            f'Stuck at ({self.x:.3f},{self.y:.3f}) — reversing')
                        self._recover_cnt = self.RECOVER_TICKS
                        self._stuck_cnt   = 0
                        return

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
        while angle >  math.pi: angle -= 2 * math.pi
        while angle < -math.pi: angle += 2 * math.pi
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
