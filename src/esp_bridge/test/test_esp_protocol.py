"""Hardware-free tests of the protocol between the Jetson and the ESP32-S3.

Both modules carry their own self-test, which encodes every command and
decodes every answer against the byte layout of the firmware:

    python3 esp_serial_bridge.py --selftest
    python3 timesync_jetson.py --selftest

These tests run both self-tests and check the payload decoders with packets
built byte by byte, so they also run with pytest on a laptop without ROS:
the one ROS message the bridge imports at module level is replaced by a
stand-in when ROS is not installed.

    cd src/esp_bridge && python3 -m pytest test -q
"""
import struct
import sys
import types

try:
    import nav_msgs.msg  # noqa: F401
except ImportError:                                   # no ROS on this computer
    nav_msgs = types.ModuleType('nav_msgs')
    nav_msgs.msg = types.ModuleType('nav_msgs.msg')
    nav_msgs.msg.Odometry = object
    sys.modules['nav_msgs'] = nav_msgs
    sys.modules['nav_msgs.msg'] = nav_msgs.msg

from esp_bridge import esp_serial_bridge as bridge    # noqa: E402
from esp_bridge import timesync_jetson                # noqa: E402


def test_bridge_selftest():
    assert bridge._selftest() == 0


def test_timesync_selftest():
    assert timesync_jetson._selftest() == 0


def test_telemetry_signed_and_scaled():
    # position and speed in 1/10 degree, duty signed, current in mA
    t = bridge.parse_telemetry(struct.pack('>iihh', 36000, -1800, -512, 250))
    assert t.position_deg == 3600.0
    assert t.speed_deg_s == -180.0
    assert t.duty == -512
    assert abs(t.current_a - 0.25) < 1e-9
    assert abs(t.speed_rad_s + 3.14159265) < 1e-6


def test_battery_in_millivolts():
    b = bridge.parse_battery(struct.pack('>ih', 15930, 3983), warning=True)
    assert abs(b.pack_v - 15.93) < 1e-9
    assert abs(b.cell_v - 3.983) < 1e-9
    assert b.warning is True


def test_move_done_status():
    done = bridge.parse_move_done(bytes([7, bridge.MOVE_OK]) + struct.pack('>i', -450))
    assert done.move_id == 7 and done.ok
    assert done.position_deg == -45.0
    late = bridge.parse_move_done(bytes([8, bridge.MOVE_TIMEOUT]) + struct.pack('>i', 0))
    assert not late.ok
    assert bridge.MOVE_STATUS_TEXT[late.status] == 'timeout'


def test_progress():
    p = bridge.parse_progress(bytes([3, 1, 40]) + struct.pack('>ii', 900, 1800))
    assert (p.move_id, p.active, p.percent) == (3, True, 40)
    assert (p.position_deg, p.target_deg) == (90.0, 180.0)
