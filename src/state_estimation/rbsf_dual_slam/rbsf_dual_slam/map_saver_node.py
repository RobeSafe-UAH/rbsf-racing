"""
Save every map of the mapping slot in one session directory, for the offline map tools.

Runs in the car's namespace (/r0, ...). On a trigger it writes, for each configured map:

    <output_dir>/<YYYYMMDD_HHMMSS>/<name>/map.pgm, map.yaml   (nav2 map_server format)
    <output_dir>/<YYYYMMDD_HHMMSS>/session.yaml               (platform, track, primary, maps)

session.yaml gives each map's frame and its pose in `map` (pose_in_map: x, y, yaw from TF), so
the offline tools can bring the maps into one frame.

and asks the SLAM that built the map to save its own state next to it, so the session can be
reloaded or reprocessed later: slam_toolbox -> <name>/posegraph.{posegraph,data},
cartographer -> <name>/map.pbstream. Those native saves run asynchronously and only log their
result. A failure there never blocks the PGM/YAML save.

Subscribes  <map topics>              nav_msgs/OccupancyGrid (transient local; `maps` parameter)
            joy                       sensor_msgs/Joy (rising edge of `save_button`)
            /tf, /tf_static           map -> <map frames>
Services    mapping/save_maps         std_srvs/Trigger (message: the session directory)
Clients     slam_toolbox/serialize_map  slam_toolbox/SerializePoseGraph (map named slam_toolbox)
            write_state                 cartographer_ros_msgs/WriteState (map named cartographer)

Parameters: output_dir, maps (list of "name:topic"), save_button (-1 disables it), platform,
track, primary.
"""

import datetime
import pathlib

from nav_msgs.msg import OccupancyGrid
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


def parse_maps(entries):
    """{'name': 'topic'} from ['name:topic', ...]."""
    maps = {}
    for entry in entries:
        name, sep, topic = entry.partition(':')
        if not sep or not name or not topic:
            raise ValueError(f"maps entries are 'name:topic', got '{entry}'")
        maps[name] = topic
    return maps


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
        self.grids = {}
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
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
            self.save_native(name, directory / name)
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
