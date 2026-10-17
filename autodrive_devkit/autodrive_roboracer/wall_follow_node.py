#!/usr/bin/env python3

################################################################################

# LiDAR wall follower for the AutoDRIVE RoboRacer simulator.
#
# Steering is a PD controller that keeps the car centred between the left and
# right walls, using a lookahead projection of each wall distance. Near a
# corner, when the free space straight ahead shrinks, steering blends toward
# the heading of the deepest gap in a disparity-extended scan, so the car turns
# in early instead of reacting to the outside wall late. Speed is set from the
# free distance ahead and the steering magnitude, and tracked with a P loop on
# odometry speed.

################################################################################

# ROS 2 module imports
import rclpy # ROS 2 client library (rcl) for Python (built on rcl C API)
from rclpy.node import Node # Base class for ROS 2 nodes
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy # Quality of Service
from std_msgs.msg import Float32, Int32 # Float32 and Int32 message classes
from sensor_msgs.msg import LaserScan # LaserScan message class
from nav_msgs.msg import Odometry # Odometry message class

# Python module imports
import math # Mathematical functions
import numpy as np # Scientific computing

################################################################################

MAX_STEER_RAD = 0.52 # Steering command of 1.0 is full lock to the left (about 0.52 rad)

# Tunable parameters (override with --ros-args -p name:=value)
DEFAULTS = {
    # Speed (m/s)
    'v_max': 7.0, # Target speed on open straights
    'v_min': 2.2, # Target speed in the tightest corners
    'full_speed_dist': 5.0, # Free distance ahead (m) at which v_max is allowed
    'stop_dist': 1.0, # Free distance ahead (m) at which v_min is used
    'steer_slowdown': 0.3, # Fraction of speed removed at full steering lock
    'v_top': 22.88, # Approximate top speed at throttle 1.0, used as feedforward
    'kp_speed': 0.4, # Throttle per m/s of speed error
    # Wall following
    'kp_wall': 1.0, # Steering (rad) per metre of centring error
    'kd_wall': 0.08, # Steering (rad) per m/s of centring error rate
    'lookahead': 1.0, # Lookahead distance (m) for projecting wall distance
    'beam_theta_deg': 45.0, # Angle between the side beam and the forward-side beam
    'wall_clip': 2.5, # Side distances are clipped to this (m) so openings do not dominate
    # Corner handling (follow the gap)
    'corner_dist': 4.0, # Begin blending toward the gap when free distance ahead is below this (m)
    'car_half_width': 0.25, # Half the car width plus margin (m), used to extend disparities
    'disparity_thresh': 0.5, # Jump in range (m) treated as an obstacle edge
    'gap_window_deg': 90.0, # Search for the gap within +/- this angle
    'front_window_deg': 8.0, # Free distance ahead is the minimum over +/- this angle
}

################################################################################

class WallFollowController:
    '''Pure LiDAR-to-command logic, kept free of ROS so it can be tested offline.'''

    def __init__(self, params):
        self.p = dict(params)
        self.prev_error = None
        self.prev_time = None

    def reset(self):
        self.prev_error = None
        self.prev_time = None

    @staticmethod
    def clean(ranges, range_max):
        r = np.asarray(ranges, dtype=float)
        r = np.where(np.isfinite(r), r, range_max) # inf/nan means no return, so treat as open space
        r = np.where(r < 0.05, range_max, r)
        return np.clip(r, 0.0, range_max)

    def beam(self, r, angles, angle_deg):
        # Median of a few beams around the requested angle, to reject single-beam noise
        i = int(np.argmin(np.abs(angles - math.radians(angle_deg))))
        return float(np.median(r[max(i - 2, 0):i + 3]))

    def wall_distance(self, r, angles, side):
        # side = +1 for the left wall, -1 for the right wall
        th = math.radians(self.p['beam_theta_deg'])
        b = self.beam(r, angles, side * 90.0) # Perpendicular to the car
        a = self.beam(r, angles, side * (90.0 - self.p['beam_theta_deg'])) # Ahead of perpendicular
        alpha = math.atan2(a * math.cos(th) - b, a * math.sin(th)) # Car heading relative to the wall
        d = b * math.cos(alpha)
        d_future = d + self.p['lookahead'] * math.sin(alpha)
        return min(max(d_future, 0.0), self.p['wall_clip'])

    def gap_heading(self, r, angles):
        # Disparity extender: at each sharp jump in range, pad the closer edge by the car's
        # half width so the chosen heading leaves room for the body, then aim at the deepest point.
        ext = r.copy()
        inc = abs(angles[1] - angles[0])
        jumps = np.nonzero(np.abs(np.diff(r)) > self.p['disparity_thresh'])[0]
        for i in jumps:
            near = min(r[i], r[i + 1])
            n = int(math.ceil(math.atan2(self.p['car_half_width'], max(near, 0.1)) / inc))
            if r[i] < r[i + 1]: # Closer edge on the low-angle side, extend toward higher angles
                lo, hi = i + 1, min(i + 1 + n, len(r))
            else:
                lo, hi = max(i - n + 1, 0), i + 1
            ext[lo:hi] = np.minimum(ext[lo:hi], near)
        window = np.abs(angles) <= math.radians(self.p['gap_window_deg'])
        idx = np.nonzero(window)[0]
        best = float(np.max(ext[idx]))
        candidates = idx[ext[idx] >= best - 0.25] # Among the deepest beams, prefer the most central
        return float(angles[candidates[np.argmin(np.abs(angles[candidates]))]])

    def compute(self, ranges, angle_min, angle_increment, range_max, t, speed=None):
        p = self.p
        r = self.clean(ranges, range_max)
        angles = angle_min + angle_increment * np.arange(len(r))

        # Wall-centring PD
        d_left = self.wall_distance(r, angles, +1)
        d_right = self.wall_distance(r, angles, -1)
        error = 0.5 * (d_left - d_right) # Positive: car is right of centre, steer left
        d_error = 0.0
        if self.prev_error is not None and t > self.prev_time:
            d_error = (error - self.prev_error) / (t - self.prev_time)
        self.prev_error, self.prev_time = error, t
        steer_wall = p['kp_wall'] * error + p['kd_wall'] * d_error

        # Blend toward the gap heading as the wall ahead gets close
        front = np.abs(angles) <= math.radians(p['front_window_deg'])
        d_front = float(np.min(r[front]))
        w = (p['corner_dist'] - d_front) / max(p['corner_dist'] - p['stop_dist'], 1e-3)
        w = min(max(w, 0.0), 1.0)
        steer_gap = self.gap_heading(r, angles) if w > 0.0 else 0.0
        steer = (1.0 - w) * steer_wall + w * steer_gap
        steer_cmd = min(max(steer / MAX_STEER_RAD, -1.0), 1.0)

        # Speed from free distance ahead and steering magnitude
        frac = (d_front - p['stop_dist']) / max(p['full_speed_dist'] - p['stop_dist'], 1e-3)
        frac = min(max(frac, 0.0), 1.0)
        v_target = p['v_min'] + (p['v_max'] - p['v_min']) * frac
        v_target *= 1.0 - p['steer_slowdown'] * abs(steer_cmd)
        v_target = max(v_target, p['v_min'])

        throttle = v_target / p['v_top']
        if speed is not None:
            throttle += p['kp_speed'] * (v_target - speed)
        throttle = min(max(throttle, -1.0), 1.0)

        debug = {'d_left': d_left, 'd_right': d_right, 'd_front': d_front, 'blend': w,
                 'v_target': v_target, 'steer_cmd': steer_cmd}
        return throttle, steer_cmd, debug

################################################################################

class WallFollowNode(Node):

    def __init__(self):
        super().__init__('wall_follow_node')

        for name, value in DEFAULTS.items():
            self.declare_parameter(name, value)
        self.controller = WallFollowController(
            {name: self.get_parameter(name).value for name in DEFAULTS})
        self.add_on_set_parameters_callback(self.on_set_parameters)

        # Match the bridge's QoS profile
        qos = QoSProfile(
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1)

        self.pub_throttle = self.create_publisher(Float32, '/autodrive/roboracer_1/throttle_command', qos)
        self.pub_steering = self.create_publisher(Float32, '/autodrive/roboracer_1/steering_command', qos)
        self.create_subscription(LaserScan, '/autodrive/roboracer_1/lidar', self.on_scan, qos)
        self.create_subscription(Odometry, '/autodrive/roboracer_1/odom', self.on_odom, qos)
        self.create_subscription(Int32, '/autodrive/roboracer_1/collision_count', self.on_collision, qos)
        self.create_subscription(Float32, '/autodrive/roboracer_1/last_lap_time', self.on_last_lap, qos)
        self.create_subscription(Float32, '/autodrive/roboracer_1/best_lap_time', self.on_best_lap, qos)

        self.speed = None
        self.collisions = None
        self.last_lap = None
        self.best_lap = None
        self.scan_count = 0
        self.get_logger().info('Wall follower started')

    def on_set_parameters(self, params):
        from rcl_interfaces.msg import SetParametersResult
        for param in params:
            if param.name in DEFAULTS:
                self.controller.p[param.name] = float(param.value)
        return SetParametersResult(successful=True)

    def on_odom(self, msg):
        v = msg.twist.twist.linear
        self.speed = math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)

    def on_collision(self, msg):
        if self.collisions is not None and msg.data > self.collisions:
            self.get_logger().warn(f'Wall contact (collision count {msg.data})')
        self.collisions = msg.data

    def on_last_lap(self, msg):
        if msg.data > 0.0 and msg.data != self.last_lap:
            self.get_logger().info(f'Lap complete: {msg.data:.3f} s')
        self.last_lap = msg.data

    def on_best_lap(self, msg):
        if msg.data > 0.0 and msg.data != self.best_lap:
            self.get_logger().info(f'Best lap: {msg.data:.3f} s')
        self.best_lap = msg.data

    def on_scan(self, msg):
        t = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
        if t == 0.0:
            t = self.get_clock().now().nanoseconds * 1e-9
        throttle, steering, debug = self.controller.compute(
            msg.ranges, msg.angle_min, msg.angle_increment,
            msg.range_max if msg.range_max > 0.0 else 10.0, t, self.speed)
        self.publish(throttle, steering)

        self.scan_count += 1
        if self.scan_count % 40 == 0: # About once a second at 40 Hz
            self.get_logger().info(
                f"v={self.speed if self.speed is not None else float('nan'):.2f} "
                f"v_tgt={debug['v_target']:.2f} thr={throttle:.2f} str={steering:+.2f} "
                f"L={debug['d_left']:.2f} R={debug['d_right']:.2f} "
                f"F={debug['d_front']:.2f} blend={debug['blend']:.2f}")

    def publish(self, throttle, steering):
        self.pub_throttle.publish(Float32(data=float(throttle)))
        self.pub_steering.publish(Float32(data=float(steering)))

################################################################################

def main(args=None):
    rclpy.init(args=args)
    node = WallFollowNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.publish(0.0, 0.0) # Stop the car on exit
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

################################################################################

if __name__ == '__main__':
    main()
