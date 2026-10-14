#!/usr/bin/env python3
"""One row of key numbers per bag, parsed from the make_all.py logs.

Every per-bag tool of make_all.py prints the numbers it computed. This script
collects them from the console logs into one table, which plot_across_runs.py
plots over all runs:

    python3 bag_metrics.py logs/make_all_*.log -o ../data/bag_metrics.csv

A log section starts with "=== <step> <bag>" and ends with
"--- <step> <bag>: <status>"; the status of every step is kept as well.
"""
import argparse
import re
import sys

import pandas as pd

SECTION = re.compile(r'^=== (\w+) (\S+)\n(.*?)^--- \1 \2: (\S+)', re.S | re.M)


def parse(text):
    rows = {}
    for m in SECTION.finditer(text):
        step, bag, body, status = m.groups()
        r = rows.setdefault(bag, {'bag': bag})
        r[f'{step}_status'] = status

        def g(pattern, key):
            mm = re.search(pattern, body)
            if mm:
                r[key] = float(mm.group(1))
        if step == 'latency':
            g(r'latency\s+median ([\d.]+)', 'lat_med_ms')
            g(r'latency.*p95 ([\d.]+)', 'lat_p95_ms')
            g(r'latency.*max ([\d.]+)', 'lat_max_ms')
            g(r'rtt\s+median ([\d.]+)', 'rtt_med_ms')
            g(r'rtt.*max ([\d.]+)', 'rtt_max_ms')
            g(r'drift mean ([-\d.]+)', 'drift_ppm')
            g(r'offset slope ([-\d.]+)', 'offset_slope_ppm')
        elif step == 'cpu':
            g(r'cpu\s+mean\s+([\d.]+)', 'cpu_mean')
            g(r'cpu\s+mean\s+[\d.]+ %\s+max\s+([\d.]+)', 'cpu_max')
            g(r'gpu\s+mean\s+([\d.]+)', 'gpu_mean')
            g(r'power\s+mean\s+([\d.]+)', 'power_w')
            g(r'temp_tj\s+mean\s+[\d.]+ degC\s+max\s+([\d.]+)', 'tj_max')
            g(r'ram\s+mean\s+([\d.]+)', 'ram_mean')
        elif step == 'dead_time':
            g(r'dead time ([\d.]+) s', 'dead_s')
            g(r'peak r = ([\d.]+)', 'dead_r')
            g(r'yaw-rate gain ([\d.]+)', 'yaw_gain')
        elif step == 'localization':
            for st in ('ok', 'recovering', 'lost'):
                g(rf'{st} ([\d.]+) %', f'loc_{st}_pct')
            g(r'matched walls per scan: mean ([\d.]+)', 'walls_per_scan')
            g(r'scans without match ([\d.]+) %', 'no_match_pct')
            g(r'innovation d\s*: median ([-+\d.]+)', 'innov_d_med_cm')
            g(r'innovation d.*std ([\d.]+)', 'innov_d_std_cm')
            g(r'innovation alpha: median ([-+\d.]+)', 'innov_a_med_deg')
            g(r'innovation alpha.*std ([\d.]+)', 'innov_a_std_deg')
        elif step == 'tracking':
            g(r'straight e_ct RMS.*?e_theta RMS ([\d.]+)', 'eth_straight_rms')
            g(r'straight e_ct.*e_theta RMS [\d.]+ deg, max ([\d.]+)', 'eth_straight_max')
            g(r'arc\s+e_ct RMS ([\d.]+)', 'ect_arc_rms')
            g(r'arc\s+e_ct RMS [\d.]+ cm, max ([\d.]+)', 'ect_arc_max')
            g(r'arc\s+e_ct.*e_theta RMS ([\d.]+)', 'eth_arc_rms')
        elif step == 'trajectory':
            g(r'path length ([\d.]+)', 'path_m')
            g(r'speed mean \(moving\) ([\d.]+)', 'v_mean')
            g(r'max ([\d.]+) m/s', 'v_max')
            for pattern, key in ((r'lap times: (.*)', 'lap_times'),
                                 (r'field placed by (.*)', 'field_by')):
                mm = re.search(pattern, body)
                if mm:
                    r[key] = mm.group(1)
        elif step == 'colour':
            r['colour_has'] = 'range bin' in body
    return pd.DataFrame(rows.values())


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('logs', nargs='+')
    ap.add_argument('-o', '--out', required=True)
    a = ap.parse_args(argv)
    text = '\n'.join(open(f).read() for f in sorted(a.logs))
    df = parse(text)
    df.to_csv(a.out, index=False)
    status = [c for c in df if c.endswith('_status')]
    print(df[status].apply(pd.Series.value_counts).fillna(0).astype(int).to_string())
    print(f'{len(df)} bags -> {a.out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
