# MIT License
#
# Copyright (c) 2020 Hongrui Zheng, Corey Walsh, Chinmay Samak, Tanmay Samak
# Copyright (c) 2026 RobeSafe (Universidad de Alcala)
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the 'Software'), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED 'AS IS', WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

"""
Particle-filter localization (MCL) on a known map, with RangeLibc ray casting.

Runs inside a car's namespace (/r0, ...) as the localization slot's estimator: it owns
map -> <ns>/odom (docs/DESIGN_PHILOSOPHY.md §3.2). The slot's localization/odom comes from
rbsf_identity's odom_relay, which composes that TF with odom at the odometry rate.

Subscribes  scan                   sensor_msgs/LaserScan
            odom                   nav_msgs/Odometry (frame <ns>/odom: RF2O on the car)
            /map                   nav_msgs/OccupancyGrid (latched, from the mapping slot)
            initialpose            geometry_msgs/PoseWithCovarianceStamped (frame map, RViz)
            /tf, /tf_static        <ns>/base_link -> laser (looked up once)
Publishes   /tf                    map -> <ns>/odom
            pf/pose                geometry_msgs/PoseWithCovarianceStamped (frame map)
            pf/viz/particles       geometry_msgs/PoseArray (a weighted sample of the particles)
            pf/viz/fake_scan       sensor_msgs/LaserScan (the map ray-cast from the estimate)

Algorithm (f1tenth/particle_filter, see PORTING.md), once per scan:
 1. Resample the particles (base_link poses in map) in proportion to their weights.
 2. Motion model: move each by the odometry change since the last update, plus Gaussian noise.
 3. Sensor model: ray-cast every `angle_step`-th beam from each particle's laser pose with
    RangeLibc, and weight it by the precomputed beam-model table P(measured | expected).
 4. The estimate is the weighted mean; map -> odom = estimate * (odom -> base_link)^-1.
Particles start around `initial_pose` if set, else spread over the map's free cells, and are
re-seeded by initialpose.
"""

import time

from geometry_msgs.msg import Pose, PoseArray, PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
import numpy as np
from particle_filter import mcl
import range_libc
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.duration import Duration
from rclpy.exceptions import ParameterUninitializedException
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener

MAP_FRAME = 'map'
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

# RangeLibc sensor-model variants kept from upstream (rangelib_variant): ray-cast all beams of all
# particles, then evaluate the table (2, works with every range method); or both in one call (3).
VAR_REPEAT_ANGLES_EVAL_SENSOR = 2
VAR_REPEAT_ANGLES_EVAL_SENSOR_ONE_SHOT = 3

PARAMETERS = {
    'base_frame': '',            # default <ns>/base_link
    'odom_frame': '',            # default <ns>/odom
    'max_particles': 2000,
    'max_viz_particles': 60,
    'angle_step': 18,            # use every n-th beam
    'squash_factor': 2.2,        # weights ** (1 / squash_factor): beams are not independent
    'max_range': 10.0,           # m
    'range_method': 'cddt',      # bl, rm, cddt, pcddt, glt
    'theta_discretization': 112,
    'rangelib_variant': VAR_REPEAT_ANGLES_EVAL_SENSOR,
    'z_hit': 0.75,
    'z_short': 0.01,
    'z_max': 0.07,
    'z_rand': 0.12,
    'sigma_hit': 8.0,            # map pixels
    'motion_dispersion_x': 0.05,
    'motion_dispersion_y': 0.025,
    'motion_dispersion_theta': 0.25,
    'initial_pose_std': [0.5, 0.5, 0.4],
    'transform_tolerance': 0.1,  # s the map -> odom stamp is post-dated by, as AMCL does
}


def pose_msg(pose):
    msg = Pose()
    msg.position.x, msg.position.y = float(pose[0]), float(pose[1])
    q = mcl.quaternion_from_yaw(pose[2])
    msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = q
    return msg


def pose_from_msg(msg):
    q = msg.orientation
    return np.array([msg.position.x, msg.position.y,
                     mcl.yaw_from_quaternion(q.x, q.y, q.z, q.w)])


class ParticleFilter(Node):

    def __init__(self):
        super().__init__('particle_filter')
        for name, default in PARAMETERS.items():
            self.declare_parameter(name, default)
        # Typed but unset by default (an empty list default would be typed as a byte array).
        self.declare_parameter('initial_pose', Parameter.Type.DOUBLE_ARRAY, ParameterDescriptor(
            description='[x, y, yaw] in map to start from; unset: global initialization.'))
        self.p = {name: self.get_parameter(name).value for name in PARAMETERS}
        try:
            self.p['initial_pose'] = list(self.get_parameter('initial_pose').value)
        except ParameterUninitializedException:
            self.p['initial_pose'] = []
        if self.p['rangelib_variant'] not in (VAR_REPEAT_ANGLES_EVAL_SENSOR,
                                              VAR_REPEAT_ANGLES_EVAL_SENSOR_ONE_SHOT):
            raise ValueError('rangelib_variant must be 2 or 3')
        if self.p['initial_pose'] and len(self.p['initial_pose']) != 3:
            raise ValueError('initial_pose must be [x, y, yaw] or empty')
        namespace = self.get_namespace().strip('/')
        prefix = f'{namespace}/' if namespace else ''
        self.base_frame = self.p['base_frame'] or f'{prefix}base_link'
        self.odom_frame = self.p['odom_frame'] or f'{prefix}odom'

        n = self.p['max_particles']
        self.rng = np.random.default_rng()
        self.particles = np.zeros((n, 3))
        self.weights = np.full(n, 1.0 / n)
        self.queries = np.zeros((n, 3), dtype=np.float32)
        self.dispersion = np.array([self.p['motion_dispersion_x'], self.p['motion_dispersion_y'],
                                    self.p['motion_dispersion_theta']])
        self.range_method = None
        self.map_info = None
        self.free_cells = None
        self.laser_offset = None     # base_link -> laser (x, y, yaw)
        self.laser_frame = None
        self.angles = None           # downsampled beam angles (laser frame)
        self.beam_count = 0
        self.odom = None             # latest odometry pose (odom -> base_link) and stamp
        self.odom_stamp = None
        self.last_odom = None        # odometry pose at the last filter update
        self.estimate = None
        self.ranges = None
        self.iterations, self.timing = 0, []

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'pf/pose', 10)
        self.particle_pub = self.create_publisher(PoseArray, 'pf/viz/particles', 1)
        self.scan_pub = self.create_publisher(LaserScan, 'pf/viz/fake_scan', 1)
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, LATCHED)
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, 'scan', self.scan_callback, qos_profile_sensor_data)
        self.create_subscription(
            PoseWithCovarianceStamped, 'initialpose', self.initialpose_callback, 1)
        self.get_logger().info(
            f"{n} particles, range method {self.p['range_method']}; waiting for /map")

    # Inputs

    def map_callback(self, msg):
        started = time.monotonic()
        omap = range_libc.PyOMap(msg)
        max_range_px = int(self.p['max_range'] / msg.info.resolution)
        method = self.p['range_method']
        if method == 'bl':
            caster = range_libc.PyBresenhamsLine(omap, max_range_px)
        elif method in ('cddt', 'pcddt'):
            caster = range_libc.PyCDDTCast(omap, max_range_px, self.p['theta_discretization'])
            if method == 'pcddt':
                caster.prune()
        elif method == 'rm':
            caster = range_libc.PyRayMarching(omap, max_range_px)
        elif method == 'glt':
            caster = range_libc.PyGiantLUTCast(omap, max_range_px, self.p['theta_discretization'])
        else:
            raise ValueError(f"Unknown range_method '{method}': bl, rm, cddt, pcddt or glt")
        caster.set_sensor_model(mcl.sensor_model_table(
            max_range_px, self.p['z_hit'], self.p['z_short'], self.p['z_max'], self.p['z_rand'],
            self.p['sigma_hit']))
        grid = np.asarray(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)
        first = self.range_method is None
        self.range_method, self.map_info = caster, msg.info
        self.free_cells = np.nonzero(grid == 0)
        self.get_logger().info(
            f'Map {msg.info.width}x{msg.info.height} @ {msg.info.resolution:.3f} m ready in '
            f'{time.monotonic() - started:.1f} s')
        if first:
            if self.p['initial_pose']:
                self.seed(self.p['initial_pose'])
            else:
                self.seed_global()

    def odom_callback(self, msg):
        self.odom = pose_from_msg(msg.pose.pose)
        self.odom_stamp = msg.header.stamp

    def initialpose_callback(self, msg):
        if msg.header.frame_id not in (MAP_FRAME, f'/{MAP_FRAME}'):
            self.get_logger().warn(f'Ignoring initialpose in frame {msg.header.frame_id!r}')
            return
        self.seed(pose_from_msg(msg.pose.pose))

    def scan_callback(self, msg):
        if self.range_method is None or self.odom is None or not self.lookup_laser(msg):
            return
        if self.angles is None or len(msg.ranges) != self.beam_count:
            self.beam_count = len(msg.ranges)
            angles = msg.angle_min + msg.angle_increment * np.arange(len(msg.ranges))
            self.angles = angles[::self.p['angle_step']].astype(np.float32)
        observation = mcl.clean_ranges(msg.ranges[::self.p['angle_step']], self.p['max_range'])
        self.update(observation)
        self.publish(msg.header.stamp)

    def lookup_laser(self, msg):
        if self.laser_offset is not None and msg.header.frame_id == self.laser_frame:
            return True
        try:
            tf = self.tf_buffer.lookup_transform(self.base_frame, msg.header.frame_id, Time())
        except TransformException as error:
            self.get_logger().warn(
                f'No {self.base_frame} -> {msg.header.frame_id} yet: {error}',
                throttle_duration_sec=5.0)
            return False
        t, q = tf.transform.translation, tf.transform.rotation
        self.laser_offset = np.array([t.x, t.y, mcl.yaw_from_quaternion(q.x, q.y, q.z, q.w)])
        self.laser_frame = msg.header.frame_id
        self.get_logger().info(f'Laser {self.laser_frame} at {np.round(self.laser_offset, 3)}')
        return True

    # Filter

    def seed(self, pose):
        self.particles = mcl.gaussian_poses(
            pose, self.p['initial_pose_std'], len(self.particles), self.rng)
        self.weights[:] = 1.0 / len(self.weights)
        self.get_logger().info(f'Particles seeded around {np.round(pose, 2).tolist()}')

    def seed_global(self):
        rows, cols = self.free_cells
        origin = self.map_info.origin.position
        self.particles = mcl.global_poses(rows, cols, self.map_info.resolution,
                                          (origin.x, origin.y), len(self.particles), self.rng)
        self.weights[:] = 1.0 / len(self.weights)
        self.get_logger().info('Particles spread over the free map (global initialization)')

    def update(self, observation):
        started = time.monotonic()
        odom = self.odom.copy()
        delta = np.zeros(3) if self.last_odom is None else mcl.odom_delta(self.last_odom, odom)
        self.last_odom = odom

        n = len(self.particles)
        self.particles = self.particles[self.rng.choice(n, n, p=self.weights)]
        mcl.motion_model(self.particles, delta, self.dispersion, self.rng)
        mcl.laser_poses(self.particles, self.laser_offset, out=self.queries)
        if self.p['rangelib_variant'] == VAR_REPEAT_ANGLES_EVAL_SENSOR_ONE_SHOT:
            self.range_method.calc_range_repeat_angles_eval_sensor_model(
                self.queries, self.angles, observation, self.weights)
        else:
            if self.ranges is None or len(self.ranges) != n * len(self.angles):
                self.ranges = np.zeros(n * len(self.angles), dtype=np.float32)
            self.range_method.calc_range_repeat_angles(self.queries, self.angles, self.ranges)
            self.range_method.eval_sensor_model(
                observation, self.ranges, self.weights, len(self.angles), n)
        np.power(self.weights, 1.0 / self.p['squash_factor'], self.weights)
        total = self.weights.sum()
        if not np.isfinite(total) or total <= 0.0:
            self.get_logger().warn('All particle weights vanished; resetting them to uniform',
                                   throttle_duration_sec=2.0)
            self.weights[:] = 1.0 / n
        else:
            self.weights /= total
        self.estimate = mcl.expected_pose(self.particles, self.weights)

        self.iterations += 1
        self.timing.append(time.monotonic() - started)
        if len(self.timing) == 200:
            self.get_logger().info(
                f'Update {1e3 * np.mean(self.timing):.1f} ms mean, '
                f'{1e3 * np.max(self.timing):.1f} ms max ({self.iterations} updates)')
            self.timing = []

    # Outputs

    def publish(self, scan_stamp):
        map_odom = mcl.map_to_odom(self.estimate, self.last_odom)
        # Post-dated, so lookups at the newest sensor stamps succeed until the next update.
        stamp = Time.from_msg(self.odom_stamp) + Duration(seconds=self.p['transform_tolerance'])
        tf = TransformStamped()
        tf.header.stamp = stamp.to_msg()
        tf.header.frame_id = MAP_FRAME
        tf.child_frame_id = self.odom_frame
        tf.transform.translation.x, tf.transform.translation.y = map(float, map_odom[:2])
        q = mcl.quaternion_from_yaw(map_odom[2])
        r = tf.transform.rotation
        r.x, r.y, r.z, r.w = q
        self.tf_broadcaster.sendTransform(tf)

        pose = PoseWithCovarianceStamped()
        pose.header.stamp, pose.header.frame_id = self.odom_stamp, MAP_FRAME
        pose.pose.pose = pose_msg(self.estimate)
        cov = mcl.covariance(self.particles, self.weights)
        for i, j in ((0, 0), (0, 1), (1, 0), (1, 1)):
            pose.pose.covariance[6 * i + j] = float(cov[i, j])
        for i in (0, 1):
            pose.pose.covariance[6 * i + 5] = pose.pose.covariance[30 + i] = float(cov[i, 2])
        pose.pose.covariance[35] = float(cov[2, 2])
        self.pose_pub.publish(pose)

        if self.particle_pub.get_subscription_count() > 0:
            sample = self.rng.choice(len(self.particles), min(
                self.p['max_viz_particles'], len(self.particles)), p=self.weights)
            array = PoseArray()
            array.header = pose.header
            array.poses = [pose_msg(p) for p in self.particles[sample]]
            self.particle_pub.publish(array)
        if self.scan_pub.get_subscription_count() > 0:
            self.publish_fake_scan(scan_stamp)

    def publish_fake_scan(self, stamp):
        laser = mcl.laser_poses(self.estimate[None, :], self.laser_offset)
        queries = np.repeat(laser, len(self.angles), axis=0)
        queries[:, 2] += self.angles
        ranges = np.zeros(len(queries), dtype=np.float32)
        self.range_method.calc_range_many(queries, ranges)
        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = self.laser_frame
        scan.angle_min, scan.angle_max = float(self.angles[0]), float(self.angles[-1])
        scan.angle_increment = float(self.angles[1] - self.angles[0])
        scan.range_max = float(self.p['max_range'])
        scan.ranges = ranges.tolist()
        self.scan_pub.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = ParticleFilter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
