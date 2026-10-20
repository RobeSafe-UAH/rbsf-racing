"""
Series specs for web_plot (no ROS dependencies).

A series is one numeric field of one topic, written like rqt_plot: the topic name followed by the
field path, e.g. `drive/drive/speed` or `scan/ranges[0]`, optionally with a label in front:
`sent=drive/drive/speed`. A path without a leading slash is relative to the node's namespace.
The topic is the longest advertised topic name that prefixes the path, so the split is only
known once the topic exists.
"""

from numbers import Real
import re

FIELD = re.compile(r'([A-Za-z_][A-Za-z0-9_]*)(?:\[(\d+)\])?')


def parse_series(entry):
    """Split `label=path` into (label, path). Without a label, the path is the label."""
    label, sep, path = entry.partition('=')
    if not sep:
        label = path = entry
    label, path = label.strip(), path.strip()
    if not path:
        raise ValueError(f"Series '{entry}' has no topic/field path")
    return label or path, path


def absolute(path, namespace):
    """Resolve a relative path in `namespace` ('/' or '/r0')."""
    if path.startswith('/'):
        return path
    return f"{namespace.rstrip('/')}/{path}"


def split_topic(path, topics):
    """
    (topic, fields) for `path`, using the longest topic in `topics` that prefixes it.

    Returns None while no advertised topic matches. `fields` is a list of field names, each
    optionally indexed (`ranges[0]`).
    """
    best = None
    for topic in topics:
        if path.startswith(topic + '/') and (best is None or len(topic) > len(best)):
            best = topic
    if best is None:
        return None
    fields = [f for f in path[len(best) + 1:].split('/') if f]
    for field in fields:
        if not FIELD.fullmatch(field):
            raise ValueError(f"Invalid field '{field}' in '{path}'")
    return best, fields


def extract(msg, fields):
    """Return the numeric value at `fields` in `msg` as a float (bools count as 0 or 1)."""
    value = msg
    for field in fields:
        name, index = FIELD.fullmatch(field).groups()
        value = getattr(value, name)
        if index is not None:
            value = value[int(index)]
    if isinstance(value, (bool, Real)) or hasattr(value, '__float__'):
        return float(value)
    raise TypeError(f"'{'/'.join(fields)}' is a {type(value).__name__}, not a number")
