#!/usr/bin/env python3
"""A4 / A5 / A6 / A7 across all runs, from the key numbers the per-bag tools print.

    python3 plot_across_runs.py bag_metrics.csv cpu_phases.txt --runs runs.csv [--out-dir DIR]

bag_metrics.csv  parsed from the make_all logs (one row per bag)
cpu_phases.txt   /jtop/cpu_total and /jtop/cpu_load split at the first
                 /cmd_vel with |v| > 0.05 m/s (standing vs. driving)
runs.csv         summarize_runs.py; gives every run its start time, so all
                 series are drawn in recording order
"""
import argparse
import re
from pathlib import Path

import numpy as np
import pandas as pd

import run_series
import style
from style import plt

DEAD_ASSUMED = 0.26
MIN_PATH_M = 1.0      # runs shorter than this never drove (perception-only recordings)
R_MIN = 0.85          # dead-time estimates with a weaker correlation peak are left out


fam = run_series.family
num = run_series.number


def scatter_series(ax, df, y, s=24):
    for f in run_series.SERIES:
        g = df[df['family'] == f]
        if len(g):
            mk, c = run_series.marker(f)
            ax.scatter(g['seq'], g[y], marker=mk, color=c, s=s, label=run_series.label(f), zorder=3)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('metrics_csv')
    ap.add_argument('cpu_phases')
    ap.add_argument('--runs', required=True, help='runs.csv from summarize_runs.py')
    ap.add_argument('--out-dir', default='.')
    a = ap.parse_args(argv)
    out = Path(a.out_dir)
    d = pd.read_csv(a.metrics_csv)
    d = d[d['bag'].map(run_series.is_run)].copy()
    d['family'] = d['bag'].map(fam)
    d['n'] = d['bag'].map(num)
    runs = pd.read_csv(a.runs)[['bag', 'start_utc']]
    d = d.merge(runs, on='bag', how='left')
    d['start'] = pd.to_datetime(d['start_utc'], utc=True, format='ISO8601')
    d = run_series.order(d, 'start')
    cap = style.source_caption(d['bag'], 'plot_across_runs.py')

    # ---------------- A5 dead time ----------------
    ok = d[d['dead_r'] >= R_MIN]
    fig, (a1, a2) = style.figure(1, 2, width=8.0, height=3.4, gridspec_kw={'width_ratios': [2, 1]})
    scatter_series(a1, ok.assign(dead_ms=ok['dead_s'] * 1000), 'dead_ms', s=18)
    run_series.day_lines(a1, d, 'start', 285)
    med = ok['dead_s'].median() * 1000
    a1.axhline(DEAD_ASSUMED * 1000, color=style.STATUS['critical'], lw=1,
               label=f'controller assumes {DEAD_ASSUMED * 1000:.0f} ms')
    a1.axhline(med, color=style.INK_2, lw=0.9, ls='--', label=f'median measured {med:.0f} ms')
    a1.set_ylim(0, 300)
    a1.set_xticks([])
    a1.set_xlabel('runs in recording order')
    a1.set_ylabel('dead time /cmd_vel -> yaw rate [ms]')
    a1.set_title(f'A5  Steering dead time per run (peak r >= {R_MIN})')
    a1.legend(loc='lower left', ncol=3, fontsize=6)
    style.hist(a2, ok['dead_s'] * 1000, bins=np.arange(100, 300, 10), color=style.CAT[0])
    a2.axvline(DEAD_ASSUMED * 1000, color=style.STATUS['critical'], lw=1)
    a2.set_xlabel('dead time [ms]')
    a2.set_ylabel('runs')
    a2.set_title('Distribution')
    style.save(fig, out, 'across_dead_time', caption=cap)
    print(f'A5 dead time: {len(ok)} runs with r >= {R_MIN} (of {d["dead_s"].notna().sum()}), '
          f'median {med:.0f} ms, IQR {ok["dead_s"].quantile(.25) * 1000:.0f}-{ok["dead_s"].quantile(.75) * 1000:.0f} ms, '
          f'min {ok["dead_s"].min() * 1000:.0f}, max {ok["dead_s"].max() * 1000:.0f}; by family: '
          + ', '.join(f'{f} {g["dead_s"].median() * 1000:.0f} ms' for f, g in ok.groupby('family'))
          + f'; yaw-rate gain median ' + ', '.join(f'{f} {g["yaw_gain"].median():.2f}' for f, g in ok.groupby('family')))

    # ---------------- A6 latency / clock sync ----------------
    fig, axes = style.figure(1, 3, width=9.0, height=3.0)
    lat = d.dropna(subset=['lat_med_ms'])
    x = np.arange(len(lat))
    axes[0].vlines(x, lat['lat_med_ms'], lat['lat_p95_ms'], color=style.CAT[0], lw=1.5)
    axes[0].scatter(x, lat['lat_med_ms'], s=12, color=style.CAT[0], zorder=3, label='median')
    axes[0].scatter(x, lat['lat_p95_ms'], s=12, color=style.CAT[0], marker='_', zorder=3, label='p95')
    axes[0].set_xticks([])
    axes[0].set_xlabel(f'{len(lat)} runs in recording order')
    axes[0].set_ylabel('one-way latency [ms]')
    axes[0].set_title('A6  Serial latency ESP -> Jetson')
    style.hist(axes[1], lat['rtt_med_ms'], bins=np.arange(0.55, 1.15, 0.025), color=style.CAT[2])
    axes[1].set_xlabel('RTT median [ms]')
    axes[1].set_ylabel('runs')
    axes[1].set_title('Round-trip time')
    style.hist(axes[2], lat['drift_ppm'].clip(-160, 160), bins=np.arange(-160, 170, 10), color=style.CAT[3])
    axes[2].set_xlabel('clock drift ESP vs Jetson [ppm]')
    axes[2].set_ylabel('runs')
    axes[2].set_title('Drift (clipped at +/-160)')
    style.save(fig, out, 'across_latency', caption=cap)
    print(f'A6 latency: median of run medians {lat["lat_med_ms"].median():.2f} ms, p95 median '
          f'{lat["lat_p95_ms"].median():.2f} ms, worst p95 {lat["lat_p95_ms"].max():.1f} ms ({lat.loc[lat["lat_p95_ms"].idxmax(), "bag"]}), '
          f'max single {lat["lat_max_ms"].max():.0f} ms ({lat.loc[lat["lat_max_ms"].idxmax(), "bag"]}); '
          f'runs with max > 100 ms: {", ".join(lat.loc[lat["lat_max_ms"] > 100, "bag"])}; '
          f'RTT median {lat["rtt_med_ms"].median():.3f} ms (range {lat["rtt_med_ms"].min():.3f}-{lat["rtt_med_ms"].max():.3f}); '
          f'drift median {lat["drift_ppm"].median():.1f} ppm, |drift| > 30 ppm in {(lat["drift_ppm"].abs() > 30).sum()} runs: '
          + ', '.join(f'{b} {v:+.0f}' for b, v in lat.loc[lat['drift_ppm'].abs() > 30, ['bag', 'drift_ppm']].values))

    # ---------------- A4 localisation ----------------
    loc = d[d['loc_ok_pct'].notna() & d['walls_per_scan'].notna() & (d['path_m'] >= MIN_PATH_M)]
    fig, (a1, a2) = style.figure(2, 1, height=5.0, sharex=True)
    ok_ = loc['loc_ok_pct'].fillna(0)
    lost = loc['loc_lost_pct'].fillna(0)
    rec = (100 - ok_ - lost).clip(lower=0)
    a1.bar(loc['seq'], ok_, color=style.LOC_STATE_COLORS['ok'], label='ok', width=0.9)
    a1.bar(loc['seq'], rec, bottom=ok_, color=style.LOC_STATE_COLORS['recovering'], label='recovering', width=0.9)
    a1.bar(loc['seq'], lost, bottom=ok_ + rec, color=style.LOC_STATE_COLORS['lost'], label='lost', width=0.9)
    run_series.day_lines(a1, d, 'start', 102)
    a1.set_ylim(0, 100)
    a1.set_ylabel('time share [%]')
    a1.set_title(f'A4  Localisation state per driving run ({len(loc)} runs, /localization_state)')
    style.legend_above(a1, ncol=3)
    a2.plot(loc['seq'], loc['walls_per_scan'], ls='none', marker='o', ms=3, color=style.CAT[0], label='matched walls per scan')
    a2.plot(loc['seq'], loc['innov_d_std_cm'], ls='none', marker='s', ms=3, color=style.CAT[1], label='wall distance innovation std [cm]')
    run_series.day_lines(a2, d, 'start', 6.1)
    a2.set_xticks([])
    a2.set_xlabel('runs in recording order')
    a2.set_ylim(0, 6)
    n_out = int((loc['innov_d_std_cm'] > 6).sum())
    for _, r in loc[loc['innov_d_std_cm'] > 6].nlargest(1, 'innov_d_std_cm').iterrows():
        a2.annotate(f'{n_out} runs above 6 cm, max {r["bag"]}: {r["innov_d_std_cm"]:.1f} cm', (r['seq'], 5.9), xytext=(8, -6),
                    textcoords='offset points', fontsize=7, color=style.INK_2)
    a2.set_title('Wall matching quality')
    style.legend_above(a2, ncol=2)
    style.save(fig, out, 'across_localization', caption=cap)
    allp = loc
    print(f'A4 localisation ({len(loc)} driving runs): ok median {ok_.median():.1f} %, mean {ok_.mean():.1f} %; '
          f'runs with lost > 0: {(lost > 0).sum()} ({", ".join(f"{b} {v:.1f}%" for b, v in loc.loc[lost > 0, ["bag", "loc_lost_pct"]].values)}); '
          f'walls/scan median {loc["walls_per_scan"].median():.2f}; innovation d std median {loc["innov_d_std_cm"].median():.2f} cm, '
          f'alpha std median {loc["innov_a_std_deg"].median():.2f} deg; outlier d std {allp["innov_d_std_cm"].max():.1f} cm in '
          f'{allp.loc[allp["innov_d_std_cm"].idxmax(), "bag"]}')

    # ---------------- A8 tracking ----------------
    tr = d.dropna(subset=['ect_arc_rms'])
    tr = tr[tr['dead_r'] >= R_MIN]
    fig, (a1, a2) = style.figure(1, 2, width=8.0, height=3.4)
    scatter_series(a1, tr, 'ect_arc_rms', s=16)
    scatter_series(a2, tr, 'eth_straight_rms', s=16)
    for ax in (a1, a2):
        run_series.day_lines(ax, d, 'start', 0)
        ax.set_xticks([])
    a1.set_ylabel('corner cross-track RMS [cm]')
    a1.set_xlabel('runs in recording order')
    a1.set_ylim(0, None)
    a1.set_title('A8  Corner cross-track error')
    a2.set_ylabel('straight heading error RMS [deg]')
    a2.set_xlabel('runs in recording order')
    a2.set_ylim(0, None)
    a2.set_title('Straight heading error')
    a1.legend(loc='upper left', ncol=2, fontsize=6)
    style.save(fig, out, 'across_tracking', caption=cap)
    print('A8 tracking: ' + '; '.join(
        f'{f}: corner e_ct RMS median {g["ect_arc_rms"].median():.2f} cm (max RMS {g["ect_arc_rms"].max():.1f}), '
        f'straight e_theta RMS median {g["eth_straight_rms"].median():.1f} deg' for f, g in tr.groupby('family')))

    # ---------------- A7 CPU ----------------
    cp = pd.read_csv(a.cpu_phases, sep=' ')
    cp = cp[cp['bag'].map(run_series.is_run)].copy()
    cp['family'] = cp['bag'].map(fam)
    cp['start'] = pd.to_datetime(cp['start_utc'], unit='s', utc=True)
    cp = cp.sort_values('start')
    fig, (a1, a2) = style.figure(1, 2, width=9.0, height=3.4, gridspec_kw={'width_ratios': [2.2, 1]})
    cp['seq'] = np.arange(len(cp))
    scatter_series(a1, cp, 'drive_core_mean', s=14)
    run_series.day_lines(a1, cp, 'start', 3)
    a1.set_xticks([])
    a1.set_xlabel('runs in recording order (vertical lines = new day)')
    a1.set_ylabel('mean load of the 6 cores [%]')
    a1.set_ylim(0, 105)
    a1.set_title('A7  CPU load while driving (/jtop/cpu_load)')
    a1.legend(loc='lower left', bbox_to_anchor=(0.0, 0.08), ncol=3, fontsize=6)
    pair = cp.set_index('bag').loc[['parken_test_48', 'parken_test_49']]
    xs = np.arange(2)
    a2.bar(xs - 0.2, pair['drive_core_mean'], width=0.38, color=style.CAT[0], label='mean of cores')
    a2.bar(xs + 0.2, pair['drive_core_max'], width=0.38, color=style.CAT[1], label='hottest core (max)')
    for xi, (m, mx) in enumerate(pair[['drive_core_mean', 'drive_core_max']].values):
        a2.text(xi - 0.2, m + 1.5, f'{m:.0f}', ha='center', fontsize=7.5, color=style.INK_2)
        a2.text(xi + 0.2, mx + 1.5, f'{mx:.0f}', ha='center', fontsize=7.5, color=style.INK_2)
    a2.set_xticks(xs, ['run 48\nbefore a14524e', 'run 49\nafter'])
    a2.set_ylim(0, 110)
    a2.set_ylabel('CPU [%] while driving')
    a2.set_title('Before / after')
    style.legend_below(a2, ncol=1)
    style.save(fig, out, 'across_cpu', caption=cap)
    same_day = cp[cp['start'].dt.strftime('%d.%m.') == '28.09.']
    print(f'A7 CPU: run 48 cores mean {pair.loc["parken_test_48", "drive_core_mean"]:.1f} % '
          f'(p95 {pair.loc["parken_test_48", "drive_core_p95"]:.0f}, max {pair.loc["parken_test_48", "drive_core_max"]:.0f}) -> run 49 '
          f'{pair.loc["parken_test_49", "drive_core_mean"]:.1f} % (p95 {pair.loc["parken_test_49", "drive_core_p95"]:.0f}, '
          f'max {pair.loc["parken_test_49", "drive_core_max"]:.0f}); median driving load of all runs before 49: '
          f'{cp.loc[cp["bag"] != "parken_test_49", "drive_core_mean"].median():.1f} %; '
          f'28.09.: cpu_total {same_day["pre_total"].iloc[0]:.0f} % -> {same_day["pre_total"].iloc[-1]:.0f} % over the day')


if __name__ == '__main__':
    main()
