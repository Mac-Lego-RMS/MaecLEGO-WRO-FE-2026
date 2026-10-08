#!/usr/bin/env python3
"""A10 pooled over all bags: colour classification vs range, from the tables
plot_colour_distance.py prints (make_all logs). Point counts are summed per
range bin, so long runs weigh more.

    python3 plot_colour_pooled.py make_all_*.log [--out-dir DIR]
        [--bags LIST --stem NAME --label TEXT]

--bags restricts the pooling to the bags named in a text file (one per line),
e.g. to keep runs with different cameras apart.
"""
import argparse
import re
from pathlib import Path

import numpy as np
import pandas as pd

import style

ROW = re.compile(r'^\s+([\d.]+) m:\s+([\d.]+) % /\s+([\d.]+) % /\s+([\d.]+) %\s+\(n=(\d+)\)')


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('logs', nargs='+')
    ap.add_argument('--out-dir', default='.')
    ap.add_argument('--bags', help='file with the bag names to pool (default: all)')
    ap.add_argument('--stem', default='colour_distance_pooled')
    ap.add_argument('--label', default='', help='added to the figure caption')
    a = ap.parse_args(argv)
    keep = set(Path(a.bags).read_text().split()) if a.bags else None
    rows = []
    for f in a.logs:
        txt = Path(f).read_text()
        for m in re.finditer(r'^=== colour (\S+)\n(.*?)^--- colour', txt, re.S | re.M):
            bag, body = m.groups()
            if keep is not None and bag not in keep:
                continue
            colour = None
            for ln in body.splitlines():
                h = re.match(r'\s+true (\w+):', ln)
                if h:
                    colour = h.group(1)
                    continue
                r = ROW.match(ln)
                if r and colour:
                    rng, c, o, u, n = r.groups()
                    n = int(n)
                    rows.append(dict(bag=bag, colour=colour, range_m=float(rng), n=n,
                                     correct=float(c) / 100 * n, other=float(o) / 100 * n,
                                     none=float(u) / 100 * n))
    df = pd.DataFrame(rows)
    nbags = df['bag'].nunique()
    fig, axes = style.figure(1, 2, width=8.0, height=3.4, sharey=True)
    for ax, col in zip(axes, ['red', 'green']):
        g = df[df['colour'] == col].groupby('range_m')[['n', 'correct', 'other', 'none']].sum()
        g = g[g['n'] >= 2000]
        for k, c, lab in [('correct', style.CAT[0], 'correct'), ('other', style.CAT[1], 'other colour'),
                          ('none', style.MUTED, 'not classified')]:
            ax.plot(g.index, g[k] / g['n'] * 100, marker='o', ms=4, color=c, label=lab)
        ax.set_title(f'True {col} pillars ({int(g["n"].sum()):,} points)')
        ax.set_xlabel('range from the LiDAR [m]')
        ax.set_ylim(0, 100)
        ax.set_xlim(0, 2.3)
        print(f'{col}: ' + ', '.join(f'{r:.2f} m: {v.correct / v.n * 100:.0f} % correct / {v.other / v.n * 100:.0f} % wrong (n={int(v.n)})'
                                     for r, v in g.iterrows()))
    axes[0].set_ylabel('share of pillar points [%]')
    style.legend_below(axes[0], ncol=3)
    print(f'{nbags} bags pooled')
    style.save(fig, a.out_dir, a.stem,
               caption=f'Source: {nbags} bags with /camera_lidar/colored_scan{a.label} '
                       f'(bins with >= 2000 points)  |  plot_colour_pooled.py')


if __name__ == '__main__':
    main()
