"""
Live time-series plots of ROS 2 message fields in a web browser.

Each entry of the `series` parameter is one numeric field of one topic (`label=topic/field/path`,
relative to the node's namespace, see series.py). The node serves a page on http://host:port/
that plots the last `history` seconds of every series and updates as messages arrive.

Subscribes  the topic of every series   any message type, resolved at runtime
Publishes   nothing
Serves      /         the plot page (web/index.html)
            /events   Server-Sent Events: JSON batches of new points, `push_rate` times a second

Algorithm:
    1. Every second, look for series whose topic is not advertised yet and subscribe to it once
       it is (one subscription per topic, best effort so it matches any publisher). Topics that
       appear late, such as a car's topics over the network, are picked up when they show up.
    2. On every message, take each series' field and store (receive time, value). Times come
       from a monotonic clock on reception, so machines with skewed clocks plot together.
    3. Every browser connection gets the whole buffer first, then only the newer points, plus the
       list of series with their status whenever it changes.
"""

from collections import deque
from dataclasses import dataclass, field
from functools import partial
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
import sys
import threading
import time

from ament_index_python.packages import get_package_share_directory
from rbsf_telemetry.series import absolute, extract, parse_series, split_topic
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.utilities import get_message


class ServeError(RuntimeError):
    """The HTTP server cannot start (usually: the port is taken)."""


@dataclass
class Series:
    label: str
    path: str
    topic: str = None
    fields: list = None
    status: str = 'waiting for topic'
    failed: bool = False
    points: deque = field(default_factory=deque)  # (seq, time, value)


class WebPlotNode(Node):

    def __init__(self):
        super().__init__('web_plot')
        self.declare_parameter('series', Parameter.Type.STRING_ARRAY)
        self.declare_parameter('title', 'Live plot')
        self.declare_parameter('unit', '')
        self.declare_parameter('host', '127.0.0.1')
        self.declare_parameter('port', 8090)
        self.declare_parameter('history', 120.0)
        self.declare_parameter('push_rate', 20.0)
        param = lambda name: self.get_parameter(name).value  # noqa: E731

        self.series = []
        for entry in param('series'):
            label, path = parse_series(entry)
            self.series.append(Series(label, absolute(path, self.get_namespace())))
        if not self.series:
            raise ValueError('The series parameter lists no series')
        self.title, self.unit = param('title'), param('unit')
        self.history, self.push_period = param('history'), 1.0 / param('push_rate')

        self.lock = threading.Lock()
        self.seq = 0
        self.version = 0  # bumped whenever the list of series or a status changes
        self.start = time.monotonic()
        self.by_topic = {}
        self.stopping = threading.Event()

        page = os.path.join(get_package_share_directory('rbsf_telemetry'), 'web', 'index.html')
        with open(page, 'rb') as f:
            self.page = f.read()
        try:
            self.server = ThreadingHTTPServer((param('host'), param('port')), self.handler())
        except OSError as e:
            raise ServeError(f"Cannot serve on {param('host')}:{param('port')}: {e.strerror}. "
                             'Is another web_plot running? Set another port.') from None
        self.server.daemon_threads = True
        threading.Thread(target=self.server.serve_forever, daemon=True).start()

        self.resolve()
        self.create_timer(1.0, self.resolve)
        self.get_logger().info(
            f"Plotting {len(self.series)} series on http://{param('host')}:{param('port')}/")

    def resolve(self):
        pending = [s for s in self.series if s.topic is None and not s.failed]
        if not pending:
            return
        types = dict(self.get_topic_names_and_types())
        for s in pending:
            try:
                hit = split_topic(s.path, types)
                if hit is None:
                    continue
                topic, fields = hit
                msg_class = get_message(types[topic][0])
                try:
                    extract(msg_class(), fields)
                except IndexError:
                    pass  # an empty array in the default message; checked on real ones
            except (AttributeError, TypeError, ValueError, ModuleNotFoundError) as e:
                s.failed, s.status = True, str(e)
                self.get_logger().error(f"Series '{s.label}' ({s.path}): {e}")
                self.bump()
                continue
            with self.lock:
                s.topic, s.fields, s.status = topic, fields, 'no data yet'
                self.version += 1
            if topic not in self.by_topic:
                self.by_topic[topic] = []
                qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
                self.create_subscription(msg_class, topic, partial(self.on_msg, topic), qos)
                self.get_logger().info(f'Subscribed to {topic} ({types[topic][0]})')
            self.by_topic[topic].append(s)

    def bump(self):
        with self.lock:
            self.version += 1

    def on_msg(self, topic, msg):
        now = time.monotonic() - self.start
        with self.lock:
            for s in self.by_topic[topic]:
                try:
                    value = extract(msg, s.fields)
                except (AttributeError, IndexError, TypeError):
                    continue
                self.seq += 1
                s.points.append((self.seq, now, value))
                while s.points[0][1] < now - self.history:
                    s.points.popleft()
                if s.status != 'live':
                    s.status = 'live'
                    self.version += 1

    def snapshot(self, after, version):
        """Return the points newer than `after`, plus the series list if `version` is old."""
        with self.lock:
            batch = {'now': time.monotonic() - self.start, 'seq': self.seq, 'points': {}}
            for i, s in enumerate(self.series):
                new = []
                for seq, t, value in reversed(s.points):
                    if seq <= after:
                        break
                    new.append((round(t, 4), value))
                if new:
                    batch['points'][i] = new[::-1]
            if version != self.version:
                batch['version'] = self.version
                batch['meta'] = {
                    'title': self.title, 'unit': self.unit, 'history': self.history,
                    'series': [{'label': s.label, 'path': s.path, 'topic': s.topic,
                                'status': s.status} for s in self.series]}
            return batch

    def handler(self):
        node = self

        class Handler(BaseHTTPRequestHandler):

            def do_GET(self):
                if self.path in ('/', '/index.html'):
                    self.send_response(200)
                    self.send_header('Content-Type', 'text/html; charset=utf-8')
                    self.send_header('Content-Length', str(len(node.page)))
                    self.end_headers()
                    self.wfile.write(node.page)
                elif self.path == '/events':
                    self.stream()
                else:
                    self.send_error(404)

            def stream(self):
                self.send_response(200)
                self.send_header('Content-Type', 'text/event-stream')
                self.send_header('Cache-Control', 'no-cache')
                self.end_headers()
                after, version = 0, -1
                try:
                    while not node.stopping.is_set():
                        batch = node.snapshot(after, version)
                        after, version = batch['seq'], batch.get('version', version)
                        self.wfile.write(f'data: {json.dumps(batch)}\n\n'.encode())
                        self.wfile.flush()
                        node.stopping.wait(node.push_period)
                except (BrokenPipeError, ConnectionResetError):
                    pass  # the browser tab closed

            def log_message(self, *args):
                pass

        return Handler

    def close(self):
        self.stopping.set()
        self.server.shutdown()
        self.server.server_close()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = WebPlotNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except ServeError as e:
        rclpy.logging.get_logger('web_plot').fatal(str(e))
        sys.exit(1)
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
