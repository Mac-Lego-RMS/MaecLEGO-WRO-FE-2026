"""End-to-end tests of the analysis toolkit on synthetic bags.

    cd docs/analysis && python3 -m pytest -q tests
    (or without pytest:  python3 tests/test_toolkit.py)

Three synthetic Humble-layout bags are generated once per session
(tests/make_fake_bag.py): parken_test_7 (German log text), cw_pos1_3 (English
log text, e_ct also on straights) and parken_test_12 (dead time 0.40 s), plus a
copy of parken_test_7 without metadata.yaml. Every tool is run on them and the
numbers it returns are checked against the values the generator put in.
"""
import json
import math
import shutil
import sqlite3
import sys
import tempfile
from pathlib import Path

import numpy as np
import pandas as pd

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
sys.path.insert(0, str(HERE))

import bag_export            # noqa: E402
import bagio                 # noqa: E402
import logpatterns           # noqa: E402
import make_all              # noqa: E402
import make_fake_bag         # noqa: E402
import metrics               # noqa: E402
import plot_colour_distance  # noqa: E402
import plot_cpu              # noqa: E402
import plot_dead_time        # noqa: E402
import plot_latency          # noqa: E402
import plot_localization     # noqa: E402
import plot_manual           # noqa: E402
import plot_parking          # noqa: E402
import plot_steer_lut        # noqa: E402
import plot_tracking         # noqa: E402
import plot_trajectory       # noqa: E402
import summarize_runs        # noqa: E402

try:
    import pytest
except ImportError:                                      # plain runner below
    pytest = None


def make_bags(root):
    root = Path(root)
    bags = root / 'bags'
    bags.mkdir(parents=True)
    make_fake_bag.write_bag(bags, 'parken_test_7', lang='de', seed=1)
    make_fake_bag.write_bag(bags, 'cw_pos1_3', lang='en', straight_ect=True, seed=2)
    make_fake_bag.write_bag(bags, 'parken_test_12', lang='de', lag=0.40, laps=2, seed=3)
    bare = root / 'bare'
    bare.mkdir()
    shutil.copy(bags / 'parken_test_7' / 'parken_test_7.db3', bare / 'parken_test_7_0.db3')
    return {'root': root, 'bags': bags, 'de': bags / 'parken_test_7', 'en': bags / 'cw_pos1_3',
            'lag40': bags / 'parken_test_12', 'bare': bare / 'parken_test_7_0.db3', 'out': root / 'out'}


if pytest:
    @pytest.fixture(scope='session')
    def env(tmp_path_factory):
        return make_bags(tmp_path_factory.mktemp('toolkit'))


def figs_exist(out_dir, stem):
    for ext in ('svg', 'png'):
        p = Path(out_dir) / f'{stem}.{ext}'
        assert p.exists() and p.stat().st_size > 1000, p


# --------------------------------------------------------------------------
def test_humble_layout_and_custom_types(env):
    meta = (env['de'] / 'metadata.yaml').read_text()
    assert 'version: 5' in meta
    con = sqlite3.connect(env['de'] / 'parken_test_7.db3')
    tables = {r[0] for r in con.execute("SELECT name FROM sqlite_master WHERE type='table'")}
    con.close()
    assert 'message_definitions' not in tables          # types must come from the .msg files
    run = bagio.load_run(env['de'])
    assert run.name == 'parken_test_7'
    assert 44 < run.duration_s < 46
    wm = run.get('/wall_matches')
    assert len(wm) > 400 and {'alpha_meas', 'd_meas', 'alpha_map', 'd_map'} <= set(wm.columns)
    obs = run.get('/obstacles')
    assert set(obs['color_name'].dropna()) == {'red', 'green'}
    assert run.get('/ekf/odom')['t_header'].notna().all()
    assert '/scan' not in run.tables                   # heavy topics only on request


def test_unregistered_types_are_skipped(env):
    empty = Path(tempfile.mkdtemp())
    run = bagio.load_run(env['de'], msg_dir=empty)
    assert run.has('/ekf/odom')
    assert not run.has('/wall_matches')                # robot_msgs unknown -> skipped, no crash


def test_bare_db3_without_metadata(env):
    run = bagio.load_run(env['bare'])
    ref = bagio.load_run(env['de'])
    assert run.name == 'parken_test_7'
    assert len(run.get('/ekf/odom')) == len(ref.get('/ekf/odom'))
    assert len(run.get('/wall_matches')) == len(ref.get('/wall_matches'))


def test_log_patterns():
    m = dict((k, g) for k, _, g in logpatterns.match_line(
        'EINGEPARKT. base_link 4.3 cm von der Aussenbande (erwartet 4.0 cm), Kurs +0.8 grad zur Bande '
        '= 0.1 cm Achsdifferenz (Regel: hoechstens 2 cm).'))
    assert m['parked']['dist_cm'] == '4.3' and m['parked']['axle_cm'] == '0.1'
    m = dict((k, g) for k, _, g in logpatterns.match_line(
        'PARKED. base_link 6,1 cm from the outer wall (expected ?), heading -12.5 deg to the wall = '
        '2.3 cm axle difference.'))
    assert logpatterns.parse_number(m['parked']['dist_cm']) == 6.1
    assert math.isnan(logpatterns.parse_number(m['parked']['expected']))
    assert logpatterns.match_line('Notstopp: Wand zu nah', level=40)[0][0] == 'estop'
    assert logpatterns.match_line("NOTSTOP: Lokalisierung seit 2.0 s 'lost'", level=40)[0][0] == 'estop'
    assert logpatterns.match_line('Emergency stop: localization lost', level=40)[0][0] == 'estop'
    assert logpatterns.match_line('emergency stop service ready', level=20) == []
    assert logpatterns.estop_kind("Lokalisierung seit 2.1 s 'lost'") == 'localisation_lost'
    assert logpatterns.estop_kind('etwas 3 cm vor der Nase, Rangieren nicht moeglich.') == 'obstacle_ahead'
    m = logpatterns.match_line('NOTFALL-RANGIEREN 2/2: Bogen nicht fahrbar -- setzt 15 cm zurueck '
                               '(Lenkung -30 %, Kurs +5 grad zur Geraden), dann neu planen.', level=30)
    assert [k for k, _, _ in m] == ['manoeuvre'] and m[0][2]['back_cm'] == '15'
    assert logpatterns.match_line('Rangieren: schon 2 Versuche an dieser Ecke -- gibt auf (x).',
                                  level=40)[0][0] == 'manoeuvre_giveup'
    assert logpatterns.match_line('Einparken abgebrochen: Wand', level=40)[0][0] == 'abort'
    assert logpatterns.match_line('Parking aborted: wall too close', level=40)[0][0] == 'abort'
    k = [x[0] for x in logpatterns.match_line('ZIEL (12 Ecken, 0.45 m vor Frontwand, v=0.10). STOP.')]
    assert k == ['finish']


def test_bag_export_and_csv_input(env):
    out = env['out'] / 'export_de'
    assert bag_export.main([str(env['de']), '-o', str(out)]) == 0
    info = json.loads((out / 'export_info.json').read_text())
    assert (out / 'ekf__odom.csv').exists() and (out / 'round1_controller__lap_state.csv').exists()
    assert '/scan' in info['skipped_topics'] and '/camera_lidar/colored_scan' in info['skipped_topics']
    odom = pd.read_csv(out / 'ekf__odom.csv')
    assert list(odom.columns[:3]) == ['t_bag', 't_header', 'msg_index']
    assert {'x', 'y', 'yaw', 'v', 'omega', 'cov_x'} <= set(odom.columns)
    heavy = env['out'] / 'export_heavy'
    bag_export.main([str(env['de']), '-o', str(heavy), '--heavy', '--topics', '/scan,/camera_lidar/*'])
    cloud = pd.read_csv(heavy / 'camera_lidar__colored_scan.csv')
    assert {'x', 'y', 'rgb', 'label'} <= set(cloud.columns) and set(cloud['label']) >= {'red', 'green'}
    assert (heavy / 'scan.csv').exists()
    # a plot tool on the CSV export gives the same numbers as on the bag
    a = plot_tracking.run([str(out), '--out-dir', str(env['out'] / 'fig_csv')])['parken_test_7']['stats']
    b = plot_tracking.run([str(env['de']), '--out-dir', str(env['out'] / 'fig_bag')])['parken_test_7']['stats']
    assert abs(a['ect_arc_rms_cm'] - b['ect_arc_rms_cm']) < 1e-6


def test_summarize_runs(env):
    csv = env['out'] / 'runs.csv'
    assert summarize_runs.main([str(env['bags']), '-o', str(csv)]) == 0
    df = pd.read_csv(csv).set_index('bag')
    assert list(df.index) == ['cw_pos1_3', 'parken_test_7', 'parken_test_12']      # natural sort
    de, en = df.loc['parken_test_7'], df.loc['cw_pos1_3']
    for row, lang in ((de, 'de'), (en, 'en')):
        exp = make_fake_bag.EXPECTED[lang]
        assert bool(row['parked']) is exp['parked']
        assert row['park_dist_outer_cm'] == exp['dist_cm']
        assert row['park_expected_cm'] == exp['expected_cm']
        assert row['park_heading_deg'] == exp['heading_deg']
        assert row['park_axle_diff_cm'] == exp['axle_cm']
        assert bool(row['park_within_2cm']) is exp['within']
        assert bool(row['emergency_stop']) is exp['estop']
        assert row['estop_kind'] == exp['estop_kind']
        assert row['n_manoeuvres'] == 1
        assert ('nose' in row['manoeuvre_reasons']) or ('Nase' in row['manoeuvre_reasons'])
        assert row['corners'] == 12 and row['laps'] == 3 and row['race_direction'] == 'CW'
        assert abs(row['loc_recovering_frac'] - 2.0 / (row['duration_s'] - 0.5)) < 0.005
        assert abs(row['loc_lost_s'] - 1.0) < 0.01
        assert 7.8 < row['battery_min_v'] < 7.9
        assert 1.3 < row['ect_arc_rms_cm'] < 1.6        # 2 cm sine -> 1.41 cm RMS
    assert math.isnan(de['ect_straight_rms_cm'])        # current controller: no e_ct on straights
    assert 0.95 < en['ect_straight_rms_cm'] < 1.2       # 1.5 cm sine -> 1.06 cm RMS
    assert df.loc['parken_test_12', 'corners'] == 8


def test_dead_time(env):
    out = env['out'] / 'fig'
    r = plot_dead_time.run([str(env['de']), str(env['lag40']), '--out-dir', str(out)])
    assert abs(r['parken_test_7']['lag_s'] - 0.25) < 0.01
    assert abs(r['parken_test_7']['gain'] - 0.84) < 0.02
    assert abs(r['parken_test_7']['assumed_s'] - 0.26) < 1e-9        # parsed from the log line
    assert abs(r['parken_test_12']['lag_s'] - 0.40) < 0.01
    r = plot_dead_time.run([str(env['en']), '--source', 'odom', '--out-dir', str(out)])
    assert abs(r['cw_pos1_3']['lag_s'] - 0.25) < 0.02
    figs_exist(out, 'dead_time_parken_test_7')


def test_latency_and_cpu(env):
    out = env['out'] / 'fig'
    r = plot_latency.run([str(env['de']), str(env['en']), '--pool', '--out-dir', str(out)])
    lat = r['parken_test_7']['latency']
    assert 2.0 < lat['median'] < 2.4 and lat['p95'] > lat['median'] and lat['n'] > 4000
    assert abs(r['parken_test_7']['drift_mean_ppm'] - 100) < 2
    assert abs(r['parken_test_7']['offset_slope_ppm'] - 100) < 5
    figs_exist(out, 'latency_parken_test_7')
    figs_exist(out, 'clocksync_parken_test_7')
    figs_exist(out, 'latency_pooled')
    r = plot_cpu.run([str(env['de']), '--out-dir', str(out)])['parken_test_7']
    assert 40 < r['cpu']['mean'] < 50 and 'temp_tj' in r and abs(r['temp_tj']['max'] - 54) < 0.5
    figs_exist(out, 'cpu_parken_test_7')


def test_tracking(env):
    out = env['out'] / 'fig'
    r = plot_tracking.run([str(env['en']), '--out-dir', str(out)])['cw_pos1_3']
    assert 0.95 < r['stats']['ect_straight_rms_cm'] < 1.2
    laps = r['per_lap_ect']
    assert set(laps['lap']) >= {1, 2, 3}
    figs_exist(out, 'tracking_cw_pos1_3')
    figs_exist(out, 'tracking_hist_cw_pos1_3')


def test_localization(env):
    out = env['out'] / 'fig'
    r = plot_localization.run([str(env['de']), '--out-dir', str(out)])['parken_test_7']
    assert abs(r['fractions']['lost'] * 44.2 - 1.0) < 0.1
    assert 2.0 < r['matches_mean'] < 2.6
    assert r['innov_d']['std'] < 0.8 and abs(r['innov_d']['median']) < 0.2        # cm
    assert r['innov_alpha']['std'] < 1.0                                        # deg
    figs_exist(out, 'localization_parken_test_7')
    figs_exist(out, 'innovations_parken_test_7')


def test_trajectory(env):
    out = env['out'] / 'fig'
    r = plot_trajectory.run([str(env['de']), '--out-dir', str(out)])['parken_test_7']
    assert r['field_placement'] == '/corner_geometry'
    lap_len = 4 * 1.0 + 2 * math.pi * 0.5
    assert abs(r['path_length_m'] - 3 * lap_len) < 0.3
    assert len(r['lap_times_s']) == 3 and all(10 < t < 13 for t in r['lap_times_s'])
    assert len(r['obstacles']) == 5
    figs_exist(out, 'trajectory_parken_test_7')
    figs_exist(out, 'trajectory_laps_parken_test_7')
    # fallback placement from the start pose gives the same field for this bag
    r2 = plot_trajectory.run([str(env['de']), '--start-pose', 'cw_pos1', '--out-dir', str(out / 'sp')])
    assert r2['parken_test_7']['field_placement'].startswith('start pose')


def test_colour_distance(env):
    out = env['out'] / 'fig'
    r = plot_colour_distance.run([str(env['de']), '--out-dir', str(out)])['parken_test_7']
    for truth in ('red', 'green'):
        tab = r[truth].dropna(subset=['correct'])
        assert tab['correct'].iloc[0] > 0.85            # close: mostly correct
        assert tab['correct'].iloc[-1] < 0.6            # far: much worse
        assert (tab['other colour'] < 0.1).all()
    figs_exist(out, 'colour_distance_parken_test_7')


def test_steer_lut_parking_manual(env):
    out = env['out'] / 'fig'
    r = plot_steer_lut.run(['--out-dir', str(out)])
    calib = json.loads((HERE.parents[2] / 'src/esp_bridge/esp_bridge/steer_calib.json').read_text())
    assert len(r['speeds']) == len(calib['speeds']) >= 2
    assert all(v['right']['full_lock_deg'] < -20 for v in r['speeds'].values())
    figs_exist(out, 'steer_lut')
    csv = env['out'] / 'runs_pk.csv'
    summarize_runs.main([str(env['de']), str(env['en']), '-o', str(csv)])
    r = plot_parking.run([str(csv), '--out-dir', str(out)])
    assert sum(v['n'] for v in r.values()) == 2 and sum(v['within'] for v in r.values()) == 1
    figs_exist(out, 'parking')
    r = plot_manual.run(['--example', '--out-dir', str(out)])
    assert abs(r['m3_encoder'][0] - 0.0150) < 0.0002
    assert abs(r['m3_gyro'][0] - 0.9675) < 0.002
    assert len(r['m1']) == 4 and (r['m1']['pos_err_cm'] < 3).all()
    pk = r['m17_parking']                               # no matching runs.csv: ruler values only
    assert len(pk) == 3 and int(pk['ruler_within_2cm'].sum()) == 2
    assert abs(pk['ruler_axle_diff_cm'].iloc[1] - 2.7) < 1e-9 and pk['park_axle_diff_cm'].isna().all()
    for stem in ('manual_m1_pose', 'manual_m3_encoder', 'manual_m3_gyro', 'manual_m17_parking'):
        figs_exist(out, stem)
    deg = plot_manual.run(['--gyro-integral', str(env['de'])])['integrated_deg']
    assert abs(deg - 3 * 360 / 0.9674) < 5            # 3 CW laps, raw sign/scale


def test_make_all(env):
    data, fig = env['out'] / 'mk_data', env['out'] / 'mk_fig'
    rc = make_all.main([str(env['de']), '--data-dir', str(data), '--fig-dir', str(fig)])
    assert rc == 0
    assert (data / 'runs.csv').exists() and (data / 'parken_test_7' / 'export_info.json').exists()
    for stem in ('latency', 'cpu', 'tracking', 'dead_time', 'localization', 'trajectory',
                 'colour_distance'):
        figs_exist(fig / 'parken_test_7', f'{stem}_parken_test_7')
    figs_exist(fig, 'steer_lut')
    figs_exist(fig, 'parking')
    assert 'make_all finished: 0 failed' in (fig / 'make_all_log.txt').read_text()


# --------------------------------------------------------------------------
if __name__ == '__main__':
    import inspect
    import traceback
    root = Path(tempfile.mkdtemp(prefix='toolkit_'))
    e = make_bags(root)
    tests = [(n, f) for n, f in sorted(globals().items()) if n.startswith('test_') and callable(f)]
    failed = 0
    for n, f in tests:
        try:
            f(*([e] if 'env' in inspect.signature(f).parameters else []))
            print(f'PASS {n}')
        except Exception:                                # noqa: BLE001
            failed += 1
            traceback.print_exc()
            print(f'FAIL {n}')
    print(f'{len(tests) - failed}/{len(tests)} passed (outputs in {root})')
    sys.exit(1 if failed else 0)
