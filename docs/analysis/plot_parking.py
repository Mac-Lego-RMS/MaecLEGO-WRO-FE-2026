#!/usr/bin/env python3
"""M17 -- parking results over all runs (from runs.csv of summarize_runs.py).

    python3 plot_parking.py [docs/data/runs.csv] [--range-size 10] [--out-dir ...]

Uses the parsed controller report ("EINGEPARKT. base_link X cm von der
Aussenbande (erwartet E), Kurs H grad zur Bande = A cm Achsdifferenz"):
  park_lateral_dev_cm = X - E  (distance to the outer wall minus expected)
  park_heading_deg    = H      (heading relative to the wall)
  park_axle_diff_cm   = A = |0.105 m x sin(H)|; WRO rule: at most 2 cm, i.e.
                        |H| <= asin(0.02 / 0.105) = 11.0 deg
Every test series (run_series.py) has its own marker and colour, and runs
are drawn in recording order. Printed: success rate per series, and for
parken_test per range of --range-size runs.

Figure parking: (a) heading error vs lateral deviation, (b) axle difference
per run with the 2 cm line. Printed: success rate per run range.
"""
import argparse
import math
import re
import sys
from pathlib import Path

import numpy as np
import pandas as pd

import bagio
import run_series
import style
from robot_constants import PARK_AXLE_RULE_CM, PARK_WHEELBASE

MARKERS = ['o', 's', 'D', '^', 'v']


def family(name):
    return re.sub(r'[_-]?\d+$', '', str(name)) or str(name)


def run(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('runs_csv', nargs='?', default=str(bagio.DEFAULT_DATA_DIR / 'runs.csv'))
    ap.add_argument('--range-size', type=int, default=10)
    ap.add_argument('--out-dir', default=str(bagio.DEFAULT_FIG_DIR))
    a = ap.parse_args(argv)
    df = pd.read_csv(a.runs_csv)
    pk = df[df['parked'].fillna(False).astype(bool) & df['park_heading_deg'].notna()].copy()
    print(f'{len(df)} runs in {a.runs_csv}, {len(pk)} with a parsed parking result')
    if not len(pk):
        print('nothing to plot')
        return {}
    pk = pk[pk['bag'].map(run_series.is_run)].copy()
    pk['family'] = pk['bag'].map(run_series.family)
    pk['start'] = pd.to_datetime(pk['start_utc'], utc=True, format='ISO8601')
    pk = run_series.order(pk, 'start')
    fams = [f for f in run_series.SERIES if f in set(pk['family'])]
    h_rule = math.degrees(math.asin(PARK_AXLE_RULE_CM / 100 / PARK_WHEELBASE))

    fig, (a1, a2) = style.figure(1, 2, width=8.0, height=3.6)
    for fam in fams:
        g = pk[pk['family'] == fam]
        mk, col = run_series.marker(fam)
        kw = dict(color=col, marker=mk, s=30, edgecolors=style.SURFACE, linewidths=0.8, zorder=3,
                  label=run_series.label(fam))
        a1.scatter(g['park_lateral_dev_cm'], g['park_heading_deg'], **kw)
        a2.scatter(g['seq'], g['park_axle_diff_cm'], **kw)
    ylim = max(h_rule * 1.4, float(pk['park_heading_deg'].abs().max()) * 1.15)
    a1.set_ylim(-ylim, ylim)
    a1.axhspan(-h_rule, h_rule, color=style.GRID, alpha=0.5, lw=0, zorder=0)
    a1.axhline(0, color=style.AXIS, lw=0.8)
    a1.axvline(0, color=style.AXIS, lw=0.8)
    a1.text(a1.get_xlim()[0], h_rule, f' 2 cm rule: |heading| <= {h_rule:.1f} deg', fontsize=7,
            color=style.INK_2, va='bottom')
    a1.set_xlabel('lateral deviation from expected [cm]')
    a1.set_ylabel('heading error to the wall [deg]')
    a1.set_title('Final pose after parking')
    a2.axhline(PARK_AXLE_RULE_CM, color=style.INK_2, lw=0.9)
    a2.set_ylim(0, max(PARK_AXLE_RULE_CM * 1.6, float(pk['park_axle_diff_cm'].max()) * 1.15))
    run_series.day_lines(a2, pk, 'start', a2.get_ylim()[1] * 0.97, min_runs=20)
    a2.text(a2.get_xlim()[0], PARK_AXLE_RULE_CM, ' rule: 2 cm', fontsize=7, color=style.INK_2, va='bottom')
    a2.set_xticks([])
    a2.set_xlabel(f'{len(pk)} parked runs in recording order')
    a2.set_ylabel('axle difference [cm]')
    a2.set_title('Axle difference per run')
    a1.legend(loc='lower left', fontsize=6.5, borderaxespad=0.3)
    style.save(fig, a.out_dir, 'parking',
               style.source_caption(pk['bag'].tolist(), f'plot_parking.py ({Path(a.runs_csv).name})'))

    res = {}
    print(f'  WRO rule: axle difference <= {PARK_AXLE_RULE_CM} cm (|heading| <= {h_rule:.1f} deg)')
    groups = [(f, pk[pk['family'] == f]) for f in fams]
    rs = a.range_size
    pt = pk[pk['family'] == 'parken_test']
    for lo in sorted(set((np.floor((pt['run_no'] - 1) / rs) * rs + 1).astype(int))):
        groups.append((f'parken_test {lo}-{lo + rs - 1}', pt[(pt['run_no'] >= lo) & (pt['run_no'] < lo + rs)]))
    for key, g in groups:
        ok = int(g['park_within_2cm'].fillna(False).astype(bool).sum())
        stats = {'n': len(g), 'within': ok,
                    'median_axle_cm': float(g['park_axle_diff_cm'].median()),
                    'median_abs_heading_deg': float(g['park_heading_deg'].abs().median()),
                    'median_abs_lateral_cm': float(g['park_lateral_dev_cm'].abs().median())}
        if key in run_series.SERIES:              # the parken_test ranges are printed only
            res[key] = stats
        print(f'  {key}: {ok}/{len(g)} within 2 cm, median axle diff '
              f'{stats["median_axle_cm"]:.2f} cm, median |heading| {stats["median_abs_heading_deg"]:.1f} deg, '
              f'median |lateral| {stats["median_abs_lateral_cm"]:.1f} cm')
    return res


def main(argv=None):
    run(argv)
    return 0


if __name__ == '__main__':
    sys.exit(main())
