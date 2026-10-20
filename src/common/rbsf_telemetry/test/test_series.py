from types import SimpleNamespace

import pytest
from rbsf_telemetry.series import absolute, extract, parse_series, split_topic


def test_parse_series_with_and_without_label():
    assert parse_series('sent = drive/drive/speed') == ('sent', 'drive/drive/speed')
    assert parse_series('/r0/odom/twist/twist/linear/x') == (
        '/r0/odom/twist/twist/linear/x', '/r0/odom/twist/twist/linear/x')
    with pytest.raises(ValueError):
        parse_series('label=')


def test_absolute_resolves_in_namespace():
    assert absolute('drive/drive/speed', '/r0') == '/r0/drive/drive/speed'
    assert absolute('drive/drive/speed', '/') == '/drive/drive/speed'
    assert absolute('/map/info/width', '/r0') == '/map/info/width'


def test_split_topic_takes_longest_prefix():
    topics = ['/r0/drive', '/r0/drive_gate/auto_cmd', '/r0/drive_gate']
    assert split_topic('/r0/drive_gate/auto_cmd/drive/speed', topics) == (
        '/r0/drive_gate/auto_cmd', ['drive', 'speed'])
    assert split_topic('/r0/drive/drive/speed', topics) == ('/r0/drive', ['drive', 'speed'])
    assert split_topic('/r0/odom/twist/twist/linear/x', topics) is None


def test_split_topic_rejects_bad_fields():
    with pytest.raises(ValueError):
        split_topic('/r0/scan/ranges[x]', ['/r0/scan'])


def test_extract_nested_indexed_and_bool():
    msg = SimpleNamespace(drive=SimpleNamespace(speed=1.25), ranges=[3.0, 4.5], armed=True)
    assert extract(msg, ['drive', 'speed']) == 1.25
    assert extract(msg, ['ranges[1]']) == 4.5
    assert extract(msg, ['armed']) == 1.0


def test_extract_rejects_non_numbers():
    msg = SimpleNamespace(header=SimpleNamespace(frame_id='r0/laser'))
    with pytest.raises(TypeError):
        extract(msg, ['header'])
    with pytest.raises(AttributeError):
        extract(msg, ['header', 'missing'])
