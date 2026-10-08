"""The recorded test series and how the run statistics draw them.

A bag belongs to a series by its name without the trailing run number
(cw_pos1_12 -> cw_pos1). Only the series listed here count as runs of the
vehicle; bench measurements, calibrations and perception-only recordings are
left out of the run statistics.

Every series gets one fixed marker and colour, so a series looks the same in
every figure.
"""
import re

import numpy as np

import style

# key: (label, has a parking task, marker, colour)
SERIES = {
    'cw_pos1': ('cw_pos1 (3 laps)', False, 's', style.CAT[1]),
    'parken_test': ('parken_test (laps + park)', True, 'o', style.CAT[0]),
    'open_test': ('open_test (open challenge)', False, 'D', style.CAT[2]),
    'obstacle_test': ('obstacle_test', True, 'v', style.CAT[3]),
    'video_bag': ('video_bag', True, 'P', style.CAT[4]),
    'only_parken': ('only_parken (1 lap + park)', True, '^', style.CAT[5]),
    'sim': ('sim (3 laps + park, virtual pillars)', True, '<', style.CAT[6]),
    'cam': ('cam (3 laps + park, CSI camera)', True, '>', style.CAT[7]),
}


def family(bag):
    return re.sub(r'_\d+$', '', str(bag))


def number(bag):
    m = re.search(r'_(\d+)$', str(bag))
    return int(m.group(1)) if m else 0


def is_run(bag):
    return family(bag) in SERIES


def label(fam):
    return SERIES[fam][0]


def marker(fam):
    return SERIES[fam][2], SERIES[fam][3]


def order(df, time_col):
    """Sort by recording time and add 'seq', the position in that order."""
    df = df.sort_values(time_col).copy()
    df['seq'] = np.arange(len(df))
    return df


def day_lines(ax, df, time_col, y_text, min_runs=8):
    """Vertical line at the first run of every day; the date above days with at
    least min_runs runs (y_text None: no dates)."""
    days = df.groupby(df[time_col].dt.strftime('%d.%m.'), sort=False)['seq'].agg(['min', 'max'])
    for day, (lo, hi) in days.iterrows():
        ax.axvline(lo - 0.5, color=style.GRID, lw=0.8, zorder=0)
        if y_text is not None and hi - lo >= min_runs:
            ax.text((lo + hi) / 2, y_text, day, ha='center', fontsize=6.5, color=style.MUTED)
