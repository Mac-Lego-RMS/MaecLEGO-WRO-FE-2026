"""Restart the EKF and scan_processor fresh before a driving node gets going.

The map depends on the pose at which ekf_node starts, and the scan_processor
measures direction and position in the bay only at start-up (start_from_bay). After
putting the robot back into the bay both need a restart -- before every
run and before every unpark attempt. That happens here automatically as soon as
round1_controller or unpark_variants_node start with ros2 run.

The two nodes run in tmux windows 8 and 9 on the Jetson, and tmux cannot be
reached from the container. So the restart goes through
a watchdog on the Jetson (window 11, src/estimation_watchdog.sh):

  1. This node puts a request with an id into the shared
     workspace (/workspace = ~/ros2_ws).
  2. The watchdog restarts 8 and 9 via estimation_restart.sh -- the logs
     stay in their windows -- and replies with the same id.
  3. Here we wait until gyro ok, bay detected and localisation ok.
     If that does not work out, the driving node does not start at all.
     With after_button (competition, require_button:=true) only until gyro ok
     and the scan_processor reports LiDAR scans: the rules forbid measuring
     before the start button, so the map only comes after the press.

Call BEFORE creating your own node: its latched subscriptions
(/corner_geometry, ...) would otherwise still get the values of the old scan_processor.

To switch off for a single start: --ros-args -p estimation_restart:=false
"""
import os
import sys
import time
import uuid

import rclpy
import rclpy.logging
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, Float64, String

from robot_msgs.msg import CornerGeometry

WORKSPACE = os.environ.get('ROBOT_WORKSPACE', '/workspace')
REQUEST_FILE = os.path.join(WORKSPACE, '.estimation_restart_request')
REPLY_FILE = os.path.join(WORKSPACE, '.estimation_restart_reply')

REPLY_TIMEOUT = 25.0     # watchdog: stop the nodes (up to ~8 s) and start them again
READY_TIMEOUT = 40.0     # after that: gyro, bay, localisation


def _disabled(argv):
    for a in argv:
        k = a.replace(' ', '').lower()
        if k in ('estimation_restart:=false', 'estimation_restart:=0'):
            return True
    return False


def _open_race(argv):
    """Open challenge (-p race_mode:=open, set by start_robot.sh --open)."""
    return any(a.replace(' ', '').lower() == 'race_mode:=open' for a in argv)


LOC_GRACE_OPEN = 5.0     # open: after gyro + start pose, wait this long for 'ok'


def _remove(file_path):
    try:
        os.remove(file_path)
    except FileNotFoundError:
        pass


def restart_estimation(node_name, argv=None, scan_args='', after_button=False):
    """Trigger the restart and wait until both are ready.

    Ends the process (SystemExit 1) if the watchdog does not reply or
    gyro/bay are not in place in time -- then the driving node should not go.

    scan_args: additional ROS arguments for the scan_processor (e.g.
    '-p start_from_bay:=false -p start_straight:=CCW' for the park test).
    The watchdog only lets harmless characters through.

    after_button: the scan_processor is started with wait_for_button, so there
    is no map before the press -- wait only for gyro and LiDAR.
    """
    argv = sys.argv if argv is None else argv
    log = rclpy.logging.get_logger(node_name)
    if _disabled(argv):
        log.warn('estimation_restart:=false -- EKF and scan_processor keep running '
                 'as they are (map from the last start!).')
        return

    req_id = uuid.uuid4().hex[:8]
    _remove(REPLY_FILE)
    with open(REQUEST_FILE + '.tmp', 'w') as f:
        f.write(req_id + '\n' + scan_args.replace('\n', ' ') + '\n')
    os.replace(REQUEST_FILE + '.tmp', REQUEST_FILE)
    log.info('EKF and scan_processor are being restarted (windows 8/9) ...')

    t0 = time.monotonic()
    reply = None
    while time.monotonic() - t0 < REPLY_TIMEOUT:
        try:
            with open(REPLY_FILE) as f:
                parts = f.read().split()
        except FileNotFoundError:
            parts = []
        if len(parts) >= 2 and parts[1] == req_id:
            reply = parts[0]
            break
        time.sleep(0.1)
    _remove(REPLY_FILE)
    if reply is None:
        _remove(REQUEST_FILE)
        log.fatal('The restart watchdog does not reply (tmux window 11 '
                  '"restart", src/estimation_watchdog.sh). Not started. '
                  'Without restart: -p estimation_restart:=false')
        raise SystemExit(1)
    if reply != 'ok':
        log.fatal('Restart of EKF/scan_processor failed -- see '
                  'window 11. Not started.')
        raise SystemExit(1)

    # Subscribe only NOW: the old nodes have ended, and their latched
    # messages have gone with them.
    n = rclpy.create_node(node_name + '_restart_wait')
    q = QoSProfile(depth=1)
    q.durability = DurabilityPolicy.TRANSIENT_LOCAL
    st = {'gyro': None, 'map': False, 'loc': None, 'start': False, 'armed': False}
    n.create_subscription(Bool, '/ekf/gyro_ok', lambda m: st.update(gyro=m.data), q)
    n.create_subscription(CornerGeometry, '/corner_geometry',
                          lambda m: st.update(map=True), q)
    n.create_subscription(String, '/localization_state',
                          lambda m: st.update(loc=m.data), q)
    # Open challenge: the scan_processor only publishes the corner geometry
    # once the direction is latched, near the first corner. Before that the
    # start pose (/front_wall_x) is all there is.
    open_race = _open_race(argv)
    n.create_subscription(Float64, '/front_wall_x',
                          lambda m: st.update(start=True), q)
    n.create_subscription(Bool, '/scan_processor/armed',
                          lambda m: st.update(armed=m.data), q)
    try:
        t0 = time.monotonic()
        t_open = None
        while time.monotonic() - t0 < READY_TIMEOUT:
            rclpy.spin_once(n, timeout_sec=0.2)
            if after_button:
                if st['gyro'] and st['armed']:
                    log.info('Ready after %.0f s: gyro ok, LiDAR there. Map and start '
                             'pose only after the start button (%s).'
                             % (time.monotonic() - t0,
                                'open challenge' if open_race else 'obstacle'))
                    return
                continue
            if open_race and st['gyro'] and st['start']:
                if st['loc'] == 'ok':
                    log.info('Ready after %.0f s (open challenge): gyro ok, start '
                             'pose measured, localisation ok.' % (time.monotonic() - t0))
                    return
                t_open = t_open or time.monotonic()
                if time.monotonic() - t_open > LOC_GRACE_OPEN:
                    log.warn('Ready after %.0f s (open challenge): gyro ok, start '
                             'pose measured, localisation still %s -- starting anyway.'
                             % (time.monotonic() - t0, st['loc'] or '-'))
                    return
                continue
            if st['gyro'] and st['map'] and st['loc'] == 'ok':
                log.info('Ready after %.0f s: gyro ok, bay detected, localisation ok.'
                         % (time.monotonic() - t0))
                return
        if after_button:
            log.fatal('NOT ready after %.0f s: gyro %s, scan_processor %s. Not started.'
                      % (READY_TIMEOUT,
                         {None: 'no message', True: 'ok', False: 'FAILED'}[st['gyro']],
                         'waits for the button' if st['armed']
                         else 'no LiDAR scans (window 0 / window 9?)'))
            raise SystemExit(1)
        if open_race:
            log.fatal('NOT ready after %.0f s (open challenge): gyro %s, start pose %s, '
                      'localisation %s. Not started.'
                      % (READY_TIMEOUT,
                         {None: 'no message', True: 'ok', False: 'FAILED'}[st['gyro']],
                         'measured' if st['start'] else 'missing (/front_wall_x)',
                         st['loc'] or '-'))
            raise SystemExit(1)
        log.fatal('NOT ready after %.0f s: gyro %s, map %s, localisation %s. '
                  'Not started.'
                  % (READY_TIMEOUT,
                     {None: 'no message', True: 'ok', False: 'FAILED'}[st['gyro']],
                     'present' if st['map'] else 'missing (is it standing in the bay?)',
                     st['loc'] or '-'))
        raise SystemExit(1)
    finally:
        n.destroy_node()
