#!/usr/bin/env python3
"""A1 / A7 / A11 -- run overview from runs.csv plus a /rosout text dump.

    python3 plot_overview.py runs.csv rosout_dump.txt [--out-dir DIR]

Every run gets exactly one outcome (first rule that matches):
  parked_ok        EINGEPARKT, axle difference <= 2 cm
  parked_bad       EINGEPARKT, axle difference  > 2 cm
  finished         race-only run (no parking task): ZIEL / FINISH logged. A
                   run has a parking task if its family is a parking series
                   or its log shows the robot unparking or parking; there ZIEL
                   alone is not a success
  esp_move         park manoeuvre stopped by an ESP position move (abort with
                   'Status 1/2', or the log ends right after 'Fahrt N: timeout')
  park_check       park manoeuvre refused / aborted by a plausibility check
  estop_arc        NOTSTOP / EMERGENCY STOP (turn-in point, arc, localisation)
  ends_parking     recording ends inside the park manoeuvre, no message
  ends_driving     recording ends while driving (stuck / taken off by hand)
  no_start         no controller output at all (perception-only recording)
"""
import argparse
import re
from pathlib import Path

import numpy as np
import pandas as pd

import run_series
import style
from style import plt

OUTCOMES = {  # key: (label, colour, is_success)
    'parked_ok': ('Parked, axle diff <= 2 cm', style.STATUS['good'], True),
    'finished': ('3 laps finished (no parking task)', '#7fc97f', True),
    'parked_bad': ('Parked, axle diff > 2 cm', style.STATUS['warning'], False),
    'esp_move': ('ESP position move timed out', style.CAT[7], False),
    'ends_parking': ('Log ends in park manoeuvre', style.STATUS['serious'], False),
    'park_check': ('Park plausibility check aborted', style.CAT[4], False),
    'estop_arc': ('Emergency stop (turn-in point, arc)', style.CAT[6], False),
    'ends_driving': ('Log ends while driving (stuck)', style.CAT[1], False),
    'no_start': ('No controller run', style.MUTED, None),
}


def read_dump(path):
    """rosout_dump.txt from run_extras.py, plain or gzip-compressed."""
    if str(path).endswith('.gz'):
        import gzip
        text = gzip.open(path, 'rt').read()
    else:
        text = Path(path).read_text()
    blocks = {}
    for blk in text.split('##### ')[1:]:
        name, *lines = blk.rstrip('\n').split('\n')
        rows = []
        for ln in lines:
            m = re.match(r'\s*([\d.]+)\s+(\d+)\s+(\S+)\s+(.*)', ln)
            if m:
                rows.append((float(m.group(1)), int(m.group(2)), m.group(3), m.group(4)))
        blocks[name.strip()] = rows
    return blocks


# The controller logged in German until the end of September 2026 and in
# English since; every rule accepts both texts.
PARKING_FAMILIES = {k for k, v in run_series.SERIES.items() if v[1]}
RX_FINISH = r'\bZIEL \(|\bFINISH \('
RX_ESP_ABORT = r'abgebrochen: Zug \d+ quittiert mit Status|aborted: .*move \d+ acked with status'
RX_PARK_ABORT = (r'(Einparken|Ausparken) abgebrochen|Einparken NICHT gestartet'
                 r'|(Parking|Unparking) aborted|Parking NOT started')
RX_ESTOP = r'NOTSTOP|EMERGENCY STOP'
RX_IN_PARK = (r'Zuege aus der umgekehrten Ausparkfolge|Einparken: '
              r'|moves from the reversed unpark sequence|Parking: ')
RX_UNPARK = r'Ausparken|Unparking'
RX_ESP_TIMEOUT = r'(Fahrt|Move) \d+: timeout'


def has_parking_task(row, text):
    return row['family'] in PARKING_FAMILIES or re.search(RX_IN_PARK + '|' + RX_UNPARK, text) is not None


def classify(row, log):
    ctl = [r for r in log if r[2].startswith('round1_contr')]
    text = '\n'.join(r[3] for r in ctl)
    if row['parked'] is True or str(row['parked']) == 'True':
        return 'parked_ok' if str(row['park_within_2cm']) == 'True' else 'parked_bad'
    if not has_parking_task(row, text) and re.search(RX_FINISH, text):
        return 'finished'
    if not ctl:
        return 'no_start'
    if re.search(RX_ESP_ABORT, text):
        return 'esp_move'
    if re.search(RX_PARK_ABORT, text):
        return 'park_check'
    if re.search(RX_ESTOP, text):
        return 'estop_arc'
    if re.search(RX_IN_PARK, text):
        t_last_ctl = ctl[-1][0]
        esp_tail = [r for r in log if r[2].startswith('esp_serial') and r[0] >= t_last_ctl - 0.5]
        if any(re.search(RX_ESP_TIMEOUT, r[3]) for r in esp_tail):
            return 'esp_move'
        return 'ends_parking'
    return 'ends_driving'


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('runs_csv')
    ap.add_argument('rosout_dump')
    ap.add_argument('--out-dir', default='.')
    a = ap.parse_args(argv)
    df = pd.read_csv(a.runs_csv)
    logs = read_dump(a.rosout_dump)
    df['family'] = df['bag'].map(run_series.family)
    df = df[df['bag'].map(run_series.is_run)].copy()
    df['run_no'] = df['bag'].map(run_series.number)
    df['outcome'] = [classify(r, logs.get(r['bag'], [])) for _, r in df.iterrows()]
    df['start'] = pd.to_datetime(df['start_utc'], utc=True, format='ISO8601')
    df = run_series.order(df, 'start')
    out = Path(a.out_dir)
    df[['bag', 'family', 'run_no', 'start_utc', 'outcome']].to_csv(out / 'run_outcomes.csv', index=False)

    # ---------------- text summary ----------------
    fams = [f for f in run_series.SERIES if f in set(df['family'])]
    for fam in fams:
        g = df[df['family'] == fam]
        print(f'\n{fam}: {len(g)} runs, {g.start.min():%d.%m.}-{g.start.max():%d.%m.}')
        print(g['outcome'].value_counts().to_string())
    task = df[df['family'].map(lambda f: run_series.SERIES[f][1]) & (df['outcome'] != 'no_start')]
    for fam in [f for f in fams if run_series.SERIES[f][1]]:
        g = task[task['family'] == fam]
        if len(g):
            print(f'{fam}: {len(g)} runs with a parking task, parked '
                  f'{g.outcome.isin(["parked_ok", "parked_bad"]).sum()}, <=2cm {(g.outcome == "parked_ok").sum()}')
    pk = df[(df['family'] == 'parken_test') & (df['outcome'] != 'no_start')]
    for lo, hi in [(1, 9), (14, 21), (22, 33), (34, 49)]:
        g = pk[(pk['run_no'] >= lo) & (pk['run_no'] <= hi)]
        print(f'parken_test {lo}-{hi}: {len(g)} runs, parked {g.outcome.isin(["parked_ok", "parked_bad"]).sum()}, '
              f'<=2cm {(g.outcome == "parked_ok").sum()}')

    # ---------------- A1a: outcome strip + rolling success ----------------
    fig, axes = style.figure(2, 1, height=6.2, sharex=False,
                             gridspec_kw={'height_ratios': [1.3, 1]})
    ax = axes[0]
    for yi, fam in enumerate(fams):
        g = df[df['family'] == fam]
        ax.scatter(g['seq'], np.full(len(g), yi), s=26, marker='s',
                   color=[OUTCOMES[k][1] for k in g['outcome']], edgecolor='white',
                   linewidth=0.4, zorder=3)
    ax.set_yticks(range(len(fams)), [run_series.label(f) for f in fams], fontsize=7)
    ax.set_ylim(len(fams) - 0.4, -0.9)
    ax.set_xlim(-1, len(df))
    run_series.day_lines(ax, df, 'start', -0.65)
    ax.set_xticks([])
    ax.set_xlabel(f'{len(df)} runs in recording order (vertical lines = new day)')
    ax.grid(axis='y', visible=False)
    ax.set_title('A1  Outcome of every run')
    used = [k for k in OUTCOMES if k in set(df['outcome'])]
    for k in used:
        ax.scatter([], [], s=40, marker='s', color=OUTCOMES[k][1], label=OUTCOMES[k][0])
    ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.12), ncol=3, fontsize=7)

    ax = axes[1]
    g = task.sort_values('start').copy()
    g['k'] = np.arange(len(g))
    succ = g['outcome'].eq('parked_ok').astype(float)
    parked = g['outcome'].isin(['parked_ok', 'parked_bad']).astype(float)
    w = 15
    ax.plot(g['k'], parked.rolling(w, min_periods=6).mean() * 100, color=style.CAT[0],
            lw=style.LINE_W, label=f'parked at all (rolling {w} runs)')
    ax.plot(g['k'], succ.rolling(w, min_periods=6).mean() * 100, color=style.CAT[2],
            lw=style.LINE_W, label=f'parked within 2 cm (rolling {w} runs)')
    run_series.day_lines(ax, g.assign(seq=g['k']), 'start', 101)
    ax.set_ylim(-5, 108)
    ax.set_xlim(-1, len(g))
    ax.set_xticks([])
    ax.set_ylabel('success rate [%]')
    ax.set_xlabel(f'{len(g)} runs with a parking task, in recording order')
    ax.set_title('A1  Reliability over the iterations')
    style.legend_below(ax, ncol=2)
    style.save(fig, out, 'overview_outcomes',
               caption=style.source_caption(df['bag'], 'plot_overview.py'))

    # ---------------- A1b: Pareto of failure reasons ----------------
    fail = df[df['outcome'].map(lambda k: OUTCOMES[k][2] is False)]
    cnt = fail['outcome'].value_counts()
    fig, ax = style.figure(1, 1, height=3.2)
    y = np.arange(len(cnt))[::-1]
    ax.barh(y, cnt.values, color=[OUTCOMES[k][1] for k in cnt.index], edgecolor='white', height=0.7)
    cum = cnt.cumsum() / cnt.sum() * 100
    for yi, (k, v) in zip(y, cnt.items()):
        ax.text(v + 0.15, yi, f'{v}   (cum. {cum[k]:.0f} %)', va='center', fontsize=7.5, color=style.INK_2)
    ax.set_yticks(y, [OUTCOMES[k][0] for k in cnt.index])
    ax.set_xlabel('number of runs')
    ax.set_xlim(0, cnt.max() * 1.45)
    ax.grid(axis='y', visible=False)
    ax.set_title(f'A1  Why runs did not reach the goal (Pareto, {int(cnt.sum())} of {len(df)} runs)')
    style.save(fig, out, 'overview_pareto', caption=style.source_caption(df['bag'], 'plot_overview.py'))

    # ---------------- A7 / A11: CPU and battery over the runs ----------------
    fig, axes = style.figure(2, 1, height=5.0, sharex=True)
    for fam in fams:
        g = df[df['family'] == fam]
        mk, col = run_series.marker(fam)
        axes[0].plot(g['seq'], g['cpu_mean_pct'], ls='none', marker=mk, ms=3.5, color=col,
                     label=run_series.label(fam))
        axes[1].plot(g['seq'], g['battery_min_v'], ls='none', marker=mk, ms=3.5, color=col)
    for ax in axes:
        run_series.day_lines(ax, df, 'start', ax.get_ylim()[1])
    axes[0].set_ylabel('CPU total, mean [%]')
    axes[0].set_ylim(0, 105)
    axes[0].set_title('A7  CPU load per run (/jtop/cpu_total)')
    axes[0].legend(loc='lower left', ncol=2, fontsize=6.5)
    axes[1].set_ylabel('battery min [V]')
    axes[1].set_xticks([])
    axes[1].set_xlabel('runs in recording order')
    axes[1].set_title('A11  Battery voltage per run (/esp_serial_bridge/battery)')
    style.save(fig, out, 'overview_cpu_battery', caption=style.source_caption(df['bag'], 'plot_overview.py'))
    return df


if __name__ == '__main__':
    main()
