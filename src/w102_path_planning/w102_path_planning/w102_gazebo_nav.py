#!/usr/bin/env python3
"""
W102 Gazebo Navigation Node  —  Adaptive Velocity-Command Sampling Controller
==============================================================================

Design: Option B — velocity command sampler (lightweight DWA-style)
------------------------------------------------------------------------
Why this approach?
  The previous rotate-then-drive proportional controller is brittle: a single
  wall contact corrupts the heading, and the controller has no recovery logic.
  A sampling-based controller selects its action every cycle by scoring many
  candidate commands on multiple objectives simultaneously, so it naturally
  corrects heading errors, stays away from walls, and adapts online without a
  rigid "rotate first, then drive" state machine.

Control cycle (20 Hz):
  1. Generate a grid of N_LIN × N_ANG candidate (v, ω) commands.
  2. Forward-simulate each candidate for SIM_STEPS × SIM_DT seconds using
     the unicycle kinematic model.
  3. Score each candidate on four weighted cost terms:
       heading  — angle error from predicted pose to goal  (lower = better)
       progress — remaining distance to goal               (lower = closer)
       smooth   — deviation from the previous command      (rewards consistency)
       wall     — soft quadratic penalty near wall faces   (avoids contact)
  4. Publish the minimum-cost command.

Stuck detection + recovery state machine:
  NAVIGATE       — normal sampling loop
  RECOVER_REVERSE — back up at low speed for RECOVERY_REVERSE_TIME s
  RECOVER_ROTATE  — rotate CCW in place for RECOVERY_TURN_TIME s
  → returns to NAVIGATE; stuck clock resets after recovery or waypoint arrival

Waypoints (world frame = odom frame, metres):
  S  (0.000, 0.750) — spawn
  R1 (0.700, 0.750) — east corridor
  R2 (0.700, 2.950) — north of chair
  G  (0.000, 2.950) — near John at y=3.048

Topics consumed:  /odom  (nav_msgs/Odometry)
Topics published: /cmd_vel (geometry_msgs/Twist), /w102/status (std_msgs/String)
"""

import math
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
# South wall centre y=-0.06, thickness 0.12 → inner face y=0.000
# North wall centre y= 3.718, thickness 0.12 → inner face y=3.658
# West wall centre x=-1.584, thickness 0.12 → inner face x=-1.524
# East wall centre x= 1.584, thickness 0.12 → inner face x= 1.524
WALL_S =  0.000
WALL_N =  3.658
WALL_W = -1.524
WALL_E =  1.524


class State(IntEnum):
    NAVIGATE        = 0
    RECOVER_REVERSE = 1
    RECOVER_ROTATE  = 2


class W102GazeboNav(Node):

    # ── Candidate grid ────────────────────────────────────────────────────────
    # 6 linear speeds × 11 angular speeds = 66 candidates per cycle
    N_LIN = 5    # samples from 0 to MAX_LIN  (inclusive endpoints → N_LIN+1 values)
    N_ANG = 11   # samples from -MAX_ANG to +MAX_ANG

    # ── Speed limits ──────────────────────────────────────────────────────────
    MAX_LIN = 0.12   # m/s   forward speed cap
    MAX_ANG = 1.2    # rad/s angular speed cap

    # ── Cost weights (tune here to change controller personality) ─────────────
    W_HEADING  = 2.0  # heading alignment — strongly penalise facing away from goal
    W_PROGRESS = 3.0  # progress — strongly reward getting closer to goal
    W_SMOOTH   = 0.5  # smoothness — lightly penalise jerky changes in command
    W_WALL     = 5.0  # wall avoidance — heavily penalise entering danger zone

    # ── Forward simulation horizon ────────────────────────────────────────────
    SIM_STEPS = 10    # number of integration steps per candidate evaluation
    SIM_DT    = 0.05  # seconds per step (matches timer period → 0.5 s lookahead)

    # ── Arrival tolerance ─────────────────────────────────────────────────────
    ARRIVE_DIST = 0.12   # m — declare waypoint reached when closer than this

    # ── Stuck detection ───────────────────────────────────────────────────────
    STUCK_DIST    = 0.04   # m   — minimum progress to reset the stuck clock
    STUCK_TIMEOUT = 3.0    # s   — seconds without progress before recovery

    # ── Recovery parameters ───────────────────────────────────────────────────
    RECOVERY_REVERSE_SPD  = 0.08   # m/s   reverse speed (published as negative linear.x)
    RECOVERY_REVERSE_TIME = 1.2    # s     duration of reverse phase
    RECOVERY_TURN_SPD     = 0.8    # rad/s rotation speed during disengage phase
    RECOVERY_TURN_TIME    = 1.8    # s     duration of rotation phase

    # ── Wall danger zone ──────────────────────────────────────────────────────
    WALL_DANGER = 0.30   # m — activate soft wall penalty inside this clearance

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

        # Pose (updated from /odom each callback)
        self.x   = 0.0
        self.y   = 0.750
        self.yaw = math.pi / 2   # robot spawns facing +Y (north)
        self.odom_received = False

        # Mission progress
        self.wp_idx       = 0
        self.mission_done = False
        self.state        = State.NAVIGATE

        # Previous published command (used for smoothness cost)
        self.prev_v = 0.0
        self.prev_w = 0.0

        # Stuck detection state
        self.best_dist          = float('inf')   # closest we've been to current waypoint
        self.last_progress_time = self.get_clock().now()

        # Recovery timing
        self.recovery_start: rclpy.time.Time | None = None

        self.timer = self.create_timer(self.SIM_DT, self._control_loop)
        self.get_logger().info(
            'W102 adaptive nav started (command sampler) — awaiting /odom …')

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

    # ── Main control loop ────────────────────────────────────────────────────
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
                cmd = Twist()
                cmd.linear.x  = -self.RECOVERY_REVERSE_SPD
                cmd.angular.z =  0.0
                self.cmd_pub.publish(cmd)
                return
            # Done reversing — start rotation
            self.state         = State.RECOVER_ROTATE
            self.recovery_start = self.get_clock().now()
            self.get_logger().info('Recovery phase 2: rotating to disengage …')
            self._publish_status('Recovery: rotating …')

        # ── Recovery phase 2: rotate ──────────────────────────────────────
        if self.state == State.RECOVER_ROTATE:
            if self._elapsed(self.recovery_start) < self.RECOVERY_TURN_TIME:
                cmd = Twist()
                cmd.linear.x  =  0.0
                cmd.angular.z =  self.RECOVERY_TURN_SPD   # CCW away from east wall
                self.cmd_pub.publish(cmd)
                return
            # Done rotating — resume navigation; reset stuck tracker
            self.state              = State.NAVIGATE
            self.best_dist          = float('inf')
            self.last_progress_time = self.get_clock().now()
            self.prev_v = 0.0
            self.prev_w = 0.0
            self.get_logger().info('Recovery complete — resuming navigation.')
            self._publish_status(
                f'Resuming → {WAYPOINT_LABELS[self.wp_idx]}')

        # ── Normal navigation ─────────────────────────────────────────────
        dist = math.hypot(tx - self.x, ty - self.y)

        # Arrival check
        if dist < self.ARRIVE_DIST:
            lbl = WAYPOINT_LABELS[self.wp_idx]
            self.get_logger().info(
                f'  ✓ {self.wp_idx + 1}/{len(WAYPOINTS)} reached: {lbl} '
                f'({tx:.3f}, {ty:.3f})')
            self._publish_status(f'W102 reached {lbl}')
            self._stop()
            self.wp_idx            += 1
            self.best_dist          = float('inf')
            self.last_progress_time = self.get_clock().now()
            self.prev_v = 0.0
            self.prev_w = 0.0
            return

        # Stuck detection: reset clock whenever meaningful progress is made
        if dist < self.best_dist - self.STUCK_DIST:
            self.best_dist          = dist
            self.last_progress_time = self.get_clock().now()

        if self._elapsed(self.last_progress_time) > self.STUCK_TIMEOUT:
            self.get_logger().warn(
                f'Stuck detected at ({self.x:.2f}, {self.y:.2f})! '
                f'No progress for {self.STUCK_TIMEOUT:.1f} s. Recovering …')
            self._publish_status('Stuck — entering recovery')
            self.state          = State.RECOVER_REVERSE
            self.recovery_start = self.get_clock().now()
            self._stop()
            return

        # Sample velocity commands and publish the best one
        best_v, best_w = self._best_command(tx, ty, dist)
        cmd = Twist()
        cmd.linear.x  = float(best_v)
        cmd.angular.z = float(best_w)
        self.cmd_pub.publish(cmd)
        self.prev_v = best_v
        self.prev_w = best_w

    # ── Velocity command sampler ─────────────────────────────────────────────
    def _best_command(self, tx: float, ty: float, dist_now: float):
        """
        Evaluate 66 candidate (v, ω) pairs and return the one with the
        lowest weighted cost after a 0.5-second forward simulation.

        Cost terms
        ----------
        heading  : |angle_error| / π at the predicted pose              [0, 1]
        progress : predicted_dist / current_dist                        [0, ∞)
        smooth   : change in v and ω normalised by their max values     [0, ∞)
        wall     : quadratic ramp inside WALL_DANGER of any wall face   [0, 1]
        """
        # Linear speed grid: 0 (allow pure rotation) to MAX_LIN
        lin_samples = [
            i * self.MAX_LIN / self.N_LIN
            for i in range(self.N_LIN + 1)
        ]
        # Angular speed grid: symmetric around 0
        ang_samples = [
            -self.MAX_ANG + i * 2.0 * self.MAX_ANG / (self.N_ANG - 1)
            for i in range(self.N_ANG)
        ]

        best_cost        = float('inf')
        best_v, best_w   = self.MAX_LIN * 0.5, 0.0   # safe fallback

        for v in lin_samples:
            for w in ang_samples:
                px, py, pyaw = self._simulate(v, w)

                # Cost 1 — heading: angle from predicted pose toward goal
                angle_to_goal = math.atan2(ty - py, tx - px)
                heading_err   = abs(self._wrap(angle_to_goal - pyaw))
                c_heading     = heading_err / math.pi   # normalised [0, 1]

                # Cost 2 — progress: how much closer are we to the goal?
                pred_dist  = math.hypot(tx - px, ty - py)
                c_progress = pred_dist / max(dist_now, 0.01)

                # Cost 3 — smoothness: reward commands similar to last cycle
                c_smooth = (abs(v - self.prev_v) / self.MAX_LIN
                           + abs(w - self.prev_w) / self.MAX_ANG)

                # Cost 4 — wall clearance: penalise poses near walls
                c_wall = self._wall_cost(px, py)

                total = (self.W_HEADING  * c_heading
                       + self.W_PROGRESS * c_progress
                       + self.W_SMOOTH   * c_smooth
                       + self.W_WALL     * c_wall)

                if total < best_cost:
                    best_cost      = total
                    best_v, best_w = v, w

        return best_v, best_w

    # ── Unicycle kinematic forward simulation ────────────────────────────────
    def _simulate(self, v: float, w: float):
        """
        Integrate the unicycle model  ẋ=v·cosθ, ẏ=v·sinθ, θ̇=ω
        for SIM_STEPS × SIM_DT seconds starting from the current pose.
        Uses exact arc equations when ω ≠ 0 to avoid accumulation error.
        Returns: (x, y, yaw) at the end of the horizon.
        """
        x, y, yaw = self.x, self.y, self.yaw
        dt = self.SIM_DT
        for _ in range(self.SIM_STEPS):
            if abs(w) < 1e-6:
                # Straight-line motion
                x   += v * math.cos(yaw) * dt
                y   += v * math.sin(yaw) * dt
            else:
                # Arc motion: exact integration (no Euler drift)
                r    = v / w
                x   += r * (math.sin(yaw + w * dt) - math.sin(yaw))
                y   -= r * (math.cos(yaw + w * dt) - math.cos(yaw))
                yaw += w * dt
            yaw = self._wrap(yaw)
        return x, y, yaw

    # ── Wall clearance cost ───────────────────────────────────────────────────
    def _wall_cost(self, x: float, y: float) -> float:
        """
        Soft quadratic penalty that rises from 0 at WALL_DANGER distance
        to 1.0 at the wall face.  Returns 0 when safely clear of all walls.
        """
        clearance = min(
            x   - WALL_W,   # distance from west wall
            WALL_E - x,     # distance from east wall
            y   - WALL_S,   # distance from south wall
            WALL_N - y,     # distance from north wall
        )
        if clearance < self.WALL_DANGER:
            fraction = (self.WALL_DANGER - clearance) / self.WALL_DANGER
            return fraction ** 2   # quadratic: gentle near WALL_DANGER, steep at wall
        return 0.0

    # ── Helpers ───────────────────────────────────────────────────────────────
    def _stop(self):
        self.cmd_pub.publish(Twist())

    def _publish_status(self, text: str):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def _elapsed(self, t0) -> float:
        """Seconds elapsed since rclpy.Time t0."""
        return (self.get_clock().now() - t0).nanoseconds * 1e-9

    @staticmethod
    def _wrap(angle: float) -> float:
        """Wrap angle to (−π, π]."""
        while angle >  math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


# ── Entry point ──────────────────────────────────────────────────────────────

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
