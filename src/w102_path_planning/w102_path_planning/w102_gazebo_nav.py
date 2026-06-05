#!/usr/bin/env python3
"""
W102 Gazebo Navigation Node  —  Reference-Inspired Adaptive Controller
=======================================================================

Control design — adapted from ~/ros2_ws (Ubuntu-22.04) patterns:

1. Coordinated vx-wz coupling  [from gimbal_tracking_controller.base_control_loop]
   -----------------------------------------------------------------------
   Instead of "rotate to align, then drive", vx and wz are published simultaneously
   every cycle.  Forward speed is scaled by cos(K_YAW_VX * heading_error):
     · heading_error = 0  → yaw_scale = 1.0  → full forward speed
     · heading_error = π/2 → yaw_scale ≈ 0   → nearly stopped, pure rotation
   This gives smooth, continuous behaviour and naturally prevents the robot from
   driving sideways into a wall while correcting its heading.

2. IIR low-pass filter on vx  [from base_control_loop, alpha = ALPHA]
   -----------------------------------------------------------------------
   Raw desired vx is smoothed before the slew limiter:
     vx_filt = (1 - ALPHA) * vx_filt + ALPHA * vx_raw
   This suppresses high-frequency jumps caused by heading-error noise.

3. Slew-rate limiter on both vx and wz  [from cmd_safe_vel, ax_max / awz_max]
   -----------------------------------------------------------------------
   The actual output ramps toward the filtered desired command at a fixed
   acceleration cap each timer tick:
     dv = clamp(target - out, -a_max*dt, a_max*dt)
     out += dv
   Prevents the physics engine from receiving discontinuous velocity steps,
   which was the root cause of the robot being pushed into walls on sharp turns.

4. Wall proximity speed reduction  [inspired by cmd_safe_vel._compute_avoidance]
   -----------------------------------------------------------------------
   If the robot centre is within WALL_DANGER of any wall inner face, vx_des
   is linearly scaled to zero at the face.  This acts before the IIR/slew
   chain, so the deceleration itself is also slew-limited (no sudden stop).

5. Stuck detection + recovery state machine  [retained from previous design]
   -----------------------------------------------------------------------
   If distance to the current waypoint does not improve by STUCK_DIST within
   STUCK_TIMEOUT seconds → RECOVER_REVERSE → RECOVER_ROTATE → NAVIGATE.

Waypoints (world frame = odom frame, metres):
  R1 (0.700, 0.750)  R2 (0.700, 2.950)  G (0.000, 2.950)

Topics consumed:  /odom  (nav_msgs/Odometry)
Topics published: /cmd_vel (geometry_msgs/Twist), /w102/status (std_msgs/String)
"""

import math
import time
from enum import IntEnum

import rclpy
import rclpy.parameter
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String


# ── Waypoints ────────────────────────────────────────────────────────────────
WAYPOINTS = [
    (0.700,  0.750),   # R1 — east corridor
    (0.700,  2.950),   # R2 — north of chair
    (0.000,  2.950),   # G  — near John
]
WAYPOINT_LABELS = ['R1', 'R2', 'G (John)']

# ── Room wall inner-face coordinates (metres) ────────────────────────────────
WALL_S =  0.000   # south  (wall centre y=-0.06, thickness 0.12)
WALL_N =  3.658   # north
WALL_W = -1.524   # west
WALL_E =  1.524   # east


class State(IntEnum):
    NAVIGATE        = 0
    RECOVER_REVERSE = 1
    RECOVER_ROTATE  = 2


class W102GazeboNav(Node):

    # ── Speed limits ──────────────────────────────────────────────────────────
    MAX_LIN = 0.12   # m/s   maximum forward speed
    MAX_ANG = 1.2    # rad/s maximum angular speed

    # ── Proportional heading gain ─────────────────────────────────────────────
    # wz_des = clamp(KP_ANG * heading_error, ±MAX_ANG)
    KP_ANG  = 1.5    # rad/s per rad — ref: base_kp_wz = 1.0 (scaled for slower robot)

    # ── Coordinated vx-wz coupling  [from base_control_loop lines 678-680] ───
    # yaw_scale = clip(cos(K_YAW_VX * heading_err), 0, 1)
    # vx_des    = MAX_LIN * yaw_scale * dist_scale
    K_YAW_VX = 0.9   # coupling strength — directly from reference (k_yaw_vx = 0.9)

    # ── IIR low-pass filter on vx  [from base_control_loop alpha = 0.35] ─────
    ALPHA = 0.35     # blend factor — directly from reference (alpha = 0.35)

    # ── Slew-rate limits  [from cmd_safe_vel ax_max / awz_max] ───────────────
    AX_MAX  = 0.30   # m/s²    — ref: ax_max = 0.4 (reduced for tighter clearances)
    AWZ_MAX = 2.00   # rad/s²  — ref: awz_max = 3.0

    # ── Distance-based slowdown ───────────────────────────────────────────────
    SLOW_RADIUS = 0.40   # m — start ramping down vx when closer than this

    # ── Arrival tolerance ─────────────────────────────────────────────────────
    ARRIVE_DIST = 0.12   # m

    # ── Wall proximity vx suppression  [inspired by _compute_avoidance] ──────
    WALL_DANGER = 0.35   # m — linear scale-down begins inside this clearance

    # ── Stuck detection ───────────────────────────────────────────────────────
    STUCK_DIST    = 0.04   # m   — minimum progress per STUCK_TIMEOUT window
    STUCK_TIMEOUT = 3.0    # s   — seconds without progress before recovery

    # ── Recovery ──────────────────────────────────────────────────────────────
    RECOVERY_REVERSE_SPD  = 0.08   # m/s
    RECOVERY_REVERSE_TIME = 1.2    # s
    RECOVERY_TURN_SPD     = 0.8    # rad/s
    RECOVERY_TURN_TIME    = 1.8    # s

    # ── Timer period ──────────────────────────────────────────────────────────
    DT = 0.05   # s  (20 Hz)

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

        # Pose
        self.x   = 0.0
        self.y   = 0.750
        self.yaw = math.pi / 2
        self.odom_received = False

        # Mission
        self.wp_idx       = 0
        self.mission_done = False
        self.state        = State.NAVIGATE

        # ── Filter / slew state (initialised to zero) ─────────────────────────
        # IIR filter state for vx  [ref: _base_vx_filt]
        self._vx_filt = 0.0
        # Slew-rate output state (what was actually sent last cycle)
        self._out_vx  = 0.0   # [ref: out_vx in cmd_safe_vel]
        self._out_wz  = 0.0   # [ref: out_wz in cmd_safe_vel]

        # Stuck detection
        self.best_dist          = float('inf')
        self.last_progress_time = self.get_clock().now()

        # Recovery timing
        self.recovery_start = None

        self.timer = self.create_timer(self.DT, self._control_loop)
        self.get_logger().info(
            'W102 nav (coupled P + slew + IIR) started — awaiting /odom …')

    # ── Odometry callback ────────────────────────────────────────────────────
    def _odom_cb(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny, cosy)
        if not self.odom_received:
            self.odom_received = True
            self.get_logger().info(
                f'/odom ready — robot at ({self.x:.3f}, {self.y:.3f}) '
                f'yaw={math.degrees(self.yaw):.1f}°')

    # ── Main control loop (20 Hz) ────────────────────────────────────────────
    def _control_loop(self):
        if self.mission_done or not self.odom_received:
            return

        if self.wp_idx >= len(WAYPOINTS):
            self._stop()
            self.mission_done = True
            self.get_logger().info('✓ W102 reached John!  Mission complete.')
            self._publish_status('Mission complete — W102 reached John.')
            return

        tx, ty = WAYPOINTS[self.wp_idx]

        # ── Recovery phase 1: reverse ─────────────────────────────────────
        if self.state == State.RECOVER_REVERSE:
            if self._elapsed(self.recovery_start) < self.RECOVERY_REVERSE_TIME:
                self._send(v=-self.RECOVERY_REVERSE_SPD, w=0.0)
                return
            self.state          = State.RECOVER_ROTATE
            self.recovery_start = self.get_clock().now()
            self.get_logger().info('Recovery phase 2: rotate to disengage …')
            self._publish_status('Recovery: rotating …')

        # ── Recovery phase 2: rotate CCW ──────────────────────────────────
        if self.state == State.RECOVER_ROTATE:
            if self._elapsed(self.recovery_start) < self.RECOVERY_TURN_TIME:
                self._send(v=0.0, w=self.RECOVERY_TURN_SPD)
                return
            self.state              = State.NAVIGATE
            self.best_dist          = float('inf')
            self.last_progress_time = self.get_clock().now()
            self._vx_filt = 0.0
            self._out_vx  = 0.0
            self._out_wz  = 0.0
            self.get_logger().info('Recovery complete — resuming navigation.')
            self._publish_status(f'Resuming → {WAYPOINT_LABELS[self.wp_idx]}')

        # ── Normal navigation ─────────────────────────────────────────────
        dist = math.hypot(tx - self.x, ty - self.y)

        # Arrival
        if dist < self.ARRIVE_DIST:
            lbl = WAYPOINT_LABELS[self.wp_idx]
            self.get_logger().info(
                f'  ✓ {self.wp_idx + 1}/{len(WAYPOINTS)} reached: {lbl}')
            self._publish_status(f'W102 reached {lbl}')
            self._stop()
            self.wp_idx            += 1
            self.best_dist          = float('inf')
            self.last_progress_time = self.get_clock().now()
            self._vx_filt = 0.0
            self._out_vx  = 0.0
            self._out_wz  = 0.0
            return

        # Stuck detection
        if dist < self.best_dist - self.STUCK_DIST:
            self.best_dist          = dist
            self.last_progress_time = self.get_clock().now()

        if self._elapsed(self.last_progress_time) > self.STUCK_TIMEOUT:
            self.get_logger().warn(
                f'Stuck at ({self.x:.2f}, {self.y:.2f})! '
                f'No progress for {self.STUCK_TIMEOUT:.1f} s.')
            self._publish_status('Stuck — recovering')
            self.state          = State.RECOVER_REVERSE
            self.recovery_start = self.get_clock().now()
            self._stop()
            return

        # Compute and publish velocity command
        out_vx, out_wz = self._compute_command(tx, ty, dist)
        self._send(out_vx, out_wz)

    # ── Reference-inspired velocity computation ───────────────────────────────
    def _compute_command(self, tx: float, ty: float, dist: float):
        """
        Proportional heading controller with three reference-derived layers.

        Step 1 — Proportional angular control
        ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        wz_des = clamp(KP_ANG * heading_err, ±MAX_ANG)
        Directly proportional to the heading error, same structure as
        base_kp_wz in gimbal_tracking_controller.base_control_loop.

        Step 2 — Coordinated vx-wz coupling  [base_control_loop lines 678-680]
        ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        yaw_scale = clip(cos(K_YAW_VX * heading_err), 0, 1)
        vx_des    = MAX_LIN * yaw_scale * dist_scale

        cos(K_YAW_VX * err) gives a smooth, continuous coupling:
          · err = 0.00 rad → yaw_scale = 1.00 → full speed ahead
          · err = 0.87 rad → yaw_scale = 0.50 → half speed while turning
          · err = π/2  rad → yaw_scale ≈ 0   → essentially stopped
        The robot never hard-stops to rotate; it always drives forward
        proportionally while correcting its heading, matching the reference's
        "avoid stop-go turning" design comment.

        Step 3 — Wall proximity reduction  [inspired by _compute_avoidance]
        ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        If the robot centre is within WALL_DANGER of any wall face, vx_des is
        linearly scaled toward zero.  This enters the IIR and slew pipeline so
        the deceleration is gradual, not an abrupt cutoff.

        Step 4 — IIR low-pass filter on vx  [base_control_loop alpha = 0.35]
        ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        vx_filt = (1 - ALPHA) * vx_filt + ALPHA * vx_des
        Smooths noise-driven jumps in desired forward speed.

        Step 5 — Slew-rate limiter on vx and wz  [cmd_safe_vel ax_max/awz_max]
        ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        dv   = clamp(target - out_prev, ±a_max*dt)
        out += dv
        The actual published command ramps toward the filtered target at a
        bounded acceleration.  Prevents the physics engine receiving abrupt
        velocity steps, which previously pushed the robot into walls.
        """

        # ── Step 1: proportional heading control ──────────────────────────
        angle_to_goal = math.atan2(ty - self.y, tx - self.x)
        heading_err   = self._wrap(angle_to_goal - self.yaw)

        wz_des = self._clamp(self.KP_ANG * heading_err, -self.MAX_ANG, self.MAX_ANG)

        # ── Step 2: coordinated coupling  [ref: base_control_loop L678-680] ─
        yaw_scale  = max(0.0, math.cos(self.K_YAW_VX * heading_err))
        dist_scale = min(1.0, dist / self.SLOW_RADIUS)
        vx_des     = self.MAX_LIN * yaw_scale * dist_scale

        # ── Step 3: wall proximity vx reduction  [ref: _compute_avoidance] ──
        min_clear = min(
            self.x   - WALL_W,   # west wall
            WALL_E   - self.x,   # east wall
            self.y   - WALL_S,   # south wall
            WALL_N   - self.y,   # north wall
        )
        if min_clear < self.WALL_DANGER:
            # linear ramp: full speed at WALL_DANGER, zero at the face
            vx_des *= max(0.0, min_clear / self.WALL_DANGER)

        # ── Step 4: IIR low-pass filter on vx  [ref: alpha = 0.35] ──────────
        self._vx_filt = (1.0 - self.ALPHA) * self._vx_filt + self.ALPHA * vx_des

        # ── Step 5: slew-rate limiter  [ref: cmd_safe_vel ax_max / awz_max] ──
        # vx ramp
        max_dvx   = self.AX_MAX  * self.DT
        dvx       = self._clamp(self._vx_filt - self._out_vx, -max_dvx, max_dvx)
        self._out_vx += dvx

        # wz ramp (angular acceleration limit, ref: _slew_wz + awz_max)
        max_dwz   = self.AWZ_MAX * self.DT
        dwz       = self._clamp(wz_des - self._out_wz, -max_dwz, max_dwz)
        self._out_wz += dwz

        # Final safety clamp
        self._out_vx = self._clamp(self._out_vx, 0.0,        self.MAX_LIN)
        self._out_wz = self._clamp(self._out_wz, -self.MAX_ANG, self.MAX_ANG)

        return self._out_vx, self._out_wz

    # ── Helpers ───────────────────────────────────────────────────────────────
    def _send(self, v: float, w: float):
        cmd = Twist()
        cmd.linear.x  = float(v)
        cmd.angular.z = float(w)
        self.cmd_pub.publish(cmd)

    def _stop(self):
        self._send(0.0, 0.0)
        self._out_vx  = 0.0
        self._out_wz  = 0.0
        self._vx_filt = 0.0

    def _publish_status(self, text: str):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def _elapsed(self, t0) -> float:
        """Seconds since rclpy.Time t0."""
        return (self.get_clock().now() - t0).nanoseconds * 1e-9

    @staticmethod
    def _clamp(x: float, lo: float, hi: float) -> float:
        """clamp() helper — same signature as reference cmd_safe_vel.clamp()."""
        return max(lo, min(hi, x))

    @staticmethod
    def _wrap(angle: float) -> float:
        """Wrap angle to (−π, π] — same as reference _wrap_to_pi()."""
        while angle >  math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


# ── Entry point ───────────────────────────────────────────────────────────────

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
