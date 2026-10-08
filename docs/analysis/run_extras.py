#!/usr/bin/env python3
"""Two per-run extracts that the run statistics need besides make_all.py.

    python3 run_extras.py BAG... --rosout rosout_dump.txt --cpu cpu_phases.txt

--rosout  the log of every run as text, one block per bag ("##### <bag>"),
          one line per message: seconds since the start, level, node, text.
          plot_overview.py classifies the outcome of each run from it.
--cpu     one line per bag: /jtop/cpu_total and the per-core /jtop/cpu_load,
          split at the first /cmd_vel with |v| > 0.05 m/s, i.e. standing
          (perception and planning only) against driving. Read by
          plot_across_runs.py.

Both files are appended to, so bags can be processed one at a time.
"""
import argparse
import sys
from pathlib import Path

import numpy as np

import bagio

SKIP_NODES = ('rosbag2_recorder', 'foxglove_bridge')
CPU_HEADER = ('bag start_utc pre_n pre_total drive_total pre_core_mean '
              'drive_core_mean drive_core_p95 drive_core_max')


def rosout(bag):
    out = [f'##### {bag.name}']
    t0 = bag.start_ns
    for _, _, t, m in bag.messages(['/rosout']):
        if m.name in SKIP_NODES or 'channel' in m.msg or 'Subscribed' in m.msg:
            continue
        out.append(f'{(t - t0) / 1e9:7.1f} {m.level:2d} {m.name[:18]:18s} {m.msg[:260]}')
    return '\n'.join(out) + '\n'


def cpu_phases(bag):
    total, core, t_drive = [], [], None
    for topic, _, t, m in bag.messages(['/jtop/cpu_total', '/jtop/cpu_load', '/cmd_vel']):
        if topic == '/cmd_vel':
            if t_drive is None and abs(m.linear.x) > 0.05:
                t_drive = t
        elif topic == '/jtop/cpu_total':
            total.append((t, m.data))
        else:
            core.append((t, np.array(m.data)))
    if not total:
        return None
    td = t_drive or float('inf')
    pre = [v for t, v in total if t < td]
    drv = [v for t, v in total if t >= td]
    cpre = np.array([v for t, v in core if t < td]) if core else np.zeros((0, 6))
    cdrv = np.array([v for t, v in core if t >= td]) if core else np.zeros((0, 6))

    def mean(a):
        return float('nan') if len(a) == 0 else float(np.mean(a))
    p95 = float(np.percentile(cdrv, 95)) if len(cdrv) else float('nan')
    mx = float(cdrv.max()) if len(cdrv) else float('nan')
    return (f'{bag.name} {bag.start_ns // 10**9} {len(pre)} {mean(pre):.1f} {mean(drv):.1f} '
            f'{mean(cpre):.1f} {mean(cdrv):.1f} {p95:.1f} {mx:.1f}\n')


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bags', nargs='+')
    ap.add_argument('--rosout', required=True)
    ap.add_argument('--cpu', required=True)
    ap.add_argument('--msg-dir', default=None)
    a = ap.parse_args(argv)
    store = bagio.make_typestore(a.msg_dir)
    cpu_path = Path(a.cpu)
    if not cpu_path.exists():
        cpu_path.write_text(CPU_HEADER + '\n')
    for path in bagio.find_bags(a.bags):
        with bagio.Bag(path, typestore=store) as bag:
            name = bag.name
            text = rosout(bag)
            line = cpu_phases(bag)
        with open(a.rosout, 'a') as fh:
            fh.write(text)
        if line:
            with open(cpu_path, 'a') as fh:
                fh.write(line)
        print(f'{name}: {text.count(chr(10)) - 1} log lines, cpu {"yes" if line else "no"}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
