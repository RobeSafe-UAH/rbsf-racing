"""
Save every map of the mapping slot in one session directory, for the offline map tools.

Runs in the car's namespace (/r0, ...). On a trigger it writes, for each configured map:

    <output_dir>/<YYYYMMDD_HHMMSS>/<name>/map.pgm, map.yaml   (nav2 map_server format)
    <output_dir>/<YYYYMMDD_HHMMSS>/<name>/trajectory.csv      (the car as that SLAM saw it)
    <output_dir>/<YYYYMMDD_HHMMSS>/ground_truth.csv           (simulators only, world frame)
    <output_dir>/<YYYYMMDD_HHMMSS>/session.yaml               (platform, track, primary, maps)

and asks the SLAM that built the map to save its own state next to it, so the session can be
reloaded or reprocessed later: slam_toolbox -> <name>/posegraph.{posegraph,data},
cartographer -> <name>/map.pbstream. Those native saves run asynchronously and only log their
result. A failure there never blocks the PGM/YAML save.

session.yaml gives each map's frame and its pose in `map` (pose_in_map: x, y, yaw from TF), so
the offline tools can bring the maps into one frame. Each trajectory is in its own map's frame
and comes from that SLAM alone (Cartographer: tracked_pose; SLAM Toolbox: pose; others: TF
<frame> -> base_frame), so it is consistent with its map even when the two SLAMs disagree.
The offline tools take the driving direction and the start/finish line from it, and check that
it stays inside the map's free space. Poses are kept every `trajectory_spacing` m of travel.

On the simulators the ground truth stays in the simulator's world frame, which only matches
`map` when the primary SLAM anchors at the odometry origin (Cartographer anchors at the start
pose): ground_truth_pose_in_map is the world frame's pose in `map`, a least-squares fit of the
car's ground-truth and `map` positions (pairs every 0.5 m), with its RMS.

Subscribes  <map topics>              nav_msgs/OccupancyGrid (transient local; `maps` parameter)
            joy                       sensor_msgs/Joy (rising edge of `save_button`)
            /tf, /tf_static           map -> <map frames>, <map frames> -> base_frame
            tracked_pose              geometry_msgs/PoseStamped (Cartographer's own pose)
            pose                      geometry_msgs/PoseWithCovarianceStamped (SLAM Toolbox's)
            ground_truth/odom         nav_msgs/Odometry (simulators only)
Services    mapping/save_maps         std_srvs/Trigger (message: the session directory)
Clients     slam_toolbox/serialize_map  slam_toolbox/SerializePoseGraph (map named slam_toolbox)
            write_state                 cartographer_ros_msgs/WriteState (map named cartographer)

Parameters: output_dir, maps (list of "name:topic"), save_button (-1 disables it), platform,
track, primary, base_frame (<robot>/base_link), trajectory_spacing (0.05 m).
"""

from collections import deque
import datetime
import math
import pathlib

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rbsf_dual_slam.alignment import fit_rigid_2d, residual_rms
from rbsf_dual_slam.occupancy import save_map, yaw_from_quaternion
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Joy
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
import yaml

LATCHED = QoSProfile(
    depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
# Ground truth <-> map pairs: one every PAIR_SPACING m, matched within PAIR_DT s; fit from
# MIN_PAIRS on.
PAIR_SPACING, PAIR_DT, MIN_PAIRS = 0.5, 0.05, 5


class Trajectory:
    """Poses (t, x, y, yaw) of one frame, kept every `spacing` m of travel."""

    def __init__(self, spacing):
        self.spacing = spacing
        self.poses = []

    def add(self, t, x, y, yaw):
        if not self.poses or math.dist(self.poses[-1][1:3], (x, y)) >= self.spacing:
            self.poses.append((t, x, y, yaw))

    def save(self, path, frame):
        with open(path, 'w') as f:
            f.write(f'# t_s, x_m, y_m, yaw_rad (frame {frame})\n')
            for pose in self.poses:
                f.write(', '.join(f'{v:.4f}' for v in pose) + '\n')


def stamp(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def parse_maps(entries):
    """{'name': 'topic'} from ['name:topic', ...]."""
    maps = {}
    for entry in entries:
        name, sep, topic = entry.partition(':')
        if not sep or not name or not topic:
            raise ValueError(f"maps entries are 'name:topic', got '{entry}'")
        maps[name] = topic
    return maps


class GroundTruthAlignment:
    """Pairs of the car's ground-truth and `map` positions, and the world frame's pose in map."""

    def __init__(self):
        self.recent = deque(maxlen=500)  # (t, x, y) ground truth, to match TF stamps
        self.world, self.map = [], []

    def add_ground_truth(self, t, x, y):
        self.recent.append((t, x, y))

    def add_map(self, t, x, y):
        """Pair a `map` position with the ground truth nearest in time (within PAIR_DT)."""
        if not self.recent:
            return
        nearest = min(self.recent, key=lambda pose: abs(pose[0] - t))
        if abs(nearest[0] - t) > PAIR_DT:
            return
        if self.map and math.dist(self.map[-1], (x, y)) < PAIR_SPACING:
            return
        self.world.append(nearest[1:])
        self.map.append((x, y))

    def fit(self):
        """[x, y, yaw] of the world frame in map and the fit's RMS (m), or None."""
        if len(self.world) < MIN_PAIRS:
            return None
        pose = fit_rigid_2d(self.world, self.map)
        return list(pose), residual_rms(self.world, self.map, pose)


class MapSaver(Node):

    def __init__(self):
        super().__init__('map_saver')
        self.output_dir = pathlib.Path(self.declare_parameter('output_dir', '/ros2_ws/maps').value)
        self.maps = parse_maps(self.declare_parameter('maps', ['slam_toolbox:/map']).value)
        self.save_button = self.declare_parameter('save_button', 3).value
        self.session = {
            'platform': self.declare_parameter('platform', '').value,
            'track': self.declare_parameter('track', '').value,
            'primary': self.declare_parameter('primary', '').value,
        }
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        spacing = self.declare_parameter('trajectory_spacing', 0.05).value
        self.grids = {}
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.trajectories = {name: Trajectory(spacing) for name in self.maps}
        self.ground_truth = Trajectory(spacing)
        self.alignment = GroundTruthAlignment()
        if 'cartographer' in self.maps:
            self.create_subscription(PoseStamped, 'tracked_pose', self.tracked_pose_callback, 10)
        if 'slam_toolbox' in self.maps:
            self.create_subscription(
                PoseWithCovarianceStamped, 'pose', self.slam_toolbox_pose_callback, 10)
        self.create_subscription(Odometry, 'ground_truth/odom', self.ground_truth_callback, 10)
        self.create_timer(0.1, self.record_tf_poses)
        self.last_button = 0
        for name, topic in self.maps.items():
            self.create_subscription(
                OccupancyGrid, topic, lambda msg, name=name: self.grids.__setitem__(name, msg),
                LATCHED)
        self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.create_service(Trigger, 'mapping/save_maps', self.save_callback)
        self.native = self.native_clients()
        self.get_logger().info(
            f"Saving {', '.join(f'{n} ({t})' for n, t in self.maps.items())} to "
            f'{self.output_dir}: joystick button {self.save_button} or '
            f'`ros2 service call {self.resolve_service_name("mapping/save_maps")} '
            'std_srvs/srv/Trigger`')

    def tracked_pose_callback(self, msg):
        q = msg.pose.orientation
        self.trajectories['cartographer'].add(
            stamp(msg.header), msg.pose.position.x, msg.pose.position.y,
            yaw_from_quaternion(q.x, q.y, q.z, q.w))

    def slam_toolbox_pose_callback(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.trajectories['slam_toolbox'].add(
            stamp(msg.header), p.x, p.y, yaw_from_quaternion(q.x, q.y, q.z, q.w))

    def ground_truth_callback(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.ground_truth.add(stamp(msg.header), p.x, p.y, yaw_from_quaternion(q.x, q.y, q.z, q.w))
        self.alignment.add_ground_truth(stamp(msg.header), p.x, p.y)

    def record_tf_poses(self):
        """Pose of base_frame in `map` (ground-truth pairs) and in maps without a pose topic."""
        if self.alignment.recent:
            try:
                tf = self.tf_buffer.lookup_transform('map', self.base_frame, Time())
                t = tf.transform.translation
                self.alignment.add_map(stamp(tf.header), t.x, t.y)
            except TransformException:
                pass
        for name, grid in self.grids.items():
            if name in ('cartographer', 'slam_toolbox'):
                continue
            try:
                tf = self.tf_buffer.lookup_transform(grid.header.frame_id, self.base_frame, Time())
            except TransformException:
                continue
            t, r = tf.transform.translation, tf.transform.rotation
            self.trajectories[name].add(stamp(tf.header), t.x, t.y,
                                        yaw_from_quaternion(r.x, r.y, r.z, r.w))

    def native_clients(self):
        """Service clients for the SLAMs' own state files, if their packages are installed."""
        clients = {}
        if 'slam_toolbox' in self.maps:
            from slam_toolbox.srv import SerializePoseGraph
            clients['slam_toolbox'] = self.create_client(
                SerializePoseGraph, 'slam_toolbox/serialize_map')
        if 'cartographer' in self.maps:
            from cartographer_ros_msgs.srv import WriteState
            clients['cartographer'] = self.create_client(WriteState, 'write_state')
        return clients

    def joy_callback(self, msg):
        if not 0 <= self.save_button < len(msg.buttons):
            return
        pressed = msg.buttons[self.save_button]
        if pressed and not self.last_button:
            self.save()
        self.last_button = pressed

    def save_callback(self, _request, response):
        response.success, response.message = self.save()
        return response

    def save(self):
        """Write every received map; return (success, session directory or error)."""
        missing = [name for name in self.maps if name not in self.grids]
        if len(missing) == len(self.maps):
            message = f"No map received yet on {', '.join(self.maps.values())}"
            self.get_logger().error(message)
            return False, message
        directory = self.output_dir / datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        session = {**self.session, 'stamp': directory.name, 'maps': {}}
        for name, grid in self.grids.items():
            info = grid.info
            q = info.origin.orientation
            origin = (info.origin.position.x, info.origin.position.y,
                      yaw_from_quaternion(q.x, q.y, q.z, q.w))
            yaml_path = save_map(directory / name, grid.data, info.width, info.height,
                                 info.resolution, origin)
            session['maps'][name] = {
                'yaml': str(yaml_path.relative_to(directory)), 'topic': self.maps[name],
                'frame': grid.header.frame_id, 'resolution': float(info.resolution),
                'width': info.width, 'height': info.height, 'origin': list(origin),
                'pose_in_map': self.pose_in_map(grid.header.frame_id)}
            self.trajectories[name].save(directory / name / 'trajectory.csv', grid.header.frame_id)
            session['maps'][name]['trajectory'] = f'{name}/trajectory.csv'
            self.save_native(name, directory / name)
        if self.ground_truth.poses:
            self.ground_truth.save(directory / 'ground_truth.csv', 'world')
            session['ground_truth'] = 'ground_truth.csv'
            fit = self.alignment.fit()
            if fit is None:
                self.get_logger().warn('Too few ground-truth/map pairs: ground truth not aligned')
            else:
                session['ground_truth_pose_in_map'], rms = fit
                session['ground_truth_fit_rms_m'] = round(rms, 3)
        if missing:
            session['missing'] = missing
            self.get_logger().warn(f"No map yet from {', '.join(missing)}: not saved")
        with open(directory / 'session.yaml', 'w') as f:
            yaml.safe_dump(session, f, sort_keys=False)
        self.get_logger().info(f'Saved {", ".join(session["maps"])} to {directory}')
        return True, str(directory)

    def pose_in_map(self, frame):
        """[x, y, yaw] of `frame` in map (latest TF), or None if unknown."""
        if frame == 'map':
            return [0.0, 0.0, 0.0]
        try:
            tf = self.tf_buffer.lookup_transform('map', frame, Time()).transform
        except TransformException as error:
            self.get_logger().warn(f'No map -> {frame}: pose_in_map unknown ({error})')
            return None
        r = tf.rotation
        return [tf.translation.x, tf.translation.y, yaw_from_quaternion(r.x, r.y, r.z, r.w)]

    def save_native(self, name, directory):
        client = self.native.get(name)
        if client is None:
            return
        if not client.service_is_ready():
            self.get_logger().warn(f'{client.srv_name} not available: no native {name} state')
            return
        request = client.srv_type.Request()
        if name == 'slam_toolbox':
            request.filename = str(directory / 'posegraph')
        else:
            request.filename = str(directory / 'map.pbstream')
            request.include_unfinished_submaps = True
        client.call_async(request).add_done_callback(
            lambda future: self.native_done(name, future))

    def native_done(self, name, future):
        result = future.result()
        if name == 'slam_toolbox':
            ok, detail = result.result == 0, f'result {result.result}'
        else:
            ok, detail = result.status.code == 0, result.status.message
        log = self.get_logger().info if ok else self.get_logger().error
        log(f"{name} native state {'saved' if ok else 'failed'} ({detail})")


def main(args=None):
    rclpy.init(args=args)
    node = MapSaver()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
