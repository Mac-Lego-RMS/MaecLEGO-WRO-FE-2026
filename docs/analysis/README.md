# Offline rosbag analysis toolkit

Turns the recorded ROS 2 bags of the robot (`parken_test_*`, `cw_pos1_*`, ...)
into CSV tables (`docs/data/`) and figures (`docs/figures/`, SVG for the PDF,
PNG for GitHub) for the engineering journal. Every plot script also prints the
key numbers it computed, so they can be quoted in the text.

No ROS installation is needed: bags are read with the pure-Python
[`rosbags`](https://pypi.org/project/rosbags/) package, and the custom
`robot_msgs` types are registered at run time from `src/robot_msgs/msg/*.msg`
(override with `--msg-dir`).

## Install

```bash
cd docs/analysis
pip install -r requirements.txt        # numpy, pandas, matplotlib, rosbags
```

Python 3.10+ (required by current `rosbags`) on a laptop (Linux/macOS/Windows) or directly on the Jetson host.
On the Jetson, run it outside the ROS container (or inside, it does not care
about a sourced ROS environment). Copy the bags to the laptop with e.g.
`rsync -av jetson:/workspace/bags/ ~/bags/`.

## Quick start

```bash
cd docs/analysis
python3 make_all.py ~/bags/                    # every bag below ~/bags
python3 make_all.py ~/bags/parken_test_4*      # a selection
```

This writes `docs/data/<bag>/*.csv`, `docs/data/runs.csv`,
`docs/figures/<bag>/*.svg|png`, `docs/figures/{parking,steer_lut}.svg|png`,
and the whole console output to `docs/figures/make_all_log.txt`.
Roughly 10-30 s per bag; `--skip colour,trajectory` / `--no-export` make it faster.

A "bag" argument can be a bag directory, its `metadata.yaml`, a `.db3` file
(also without `metadata.yaml`, e.g. after a killed recorder), a directory that
contains many bags, or a CSV export directory written by `bag_export.py`.

## Tools (one line each)

| Tool | Usage |
|---|---|
| `bag_export.py` | `python3 bag_export.py BAG... [-o OUT] [--topics '/ekf/*,/cmd_vel'] [--heavy]` - one CSV per topic (`/ekf/odom` -> `ekf__odom.csv`), `/scan` and point clouds only with `--heavy` |
| `summarize_runs.py` | `python3 summarize_runs.py BAGS_OR_DIRS... -o ../data/runs.csv` - one row per bag + text summary (column list in the script docstring) |
| `plot_latency.py` | `python3 plot_latency.py BAG... [--pool]` - M14 serial latency / RTT histograms, clock offset and drift |
| `plot_cpu.py` | `python3 plot_cpu.py BAG...` - M15 CPU/GPU/RAM, power, thermal zones |
| `plot_tracking.py` | `python3 plot_tracking.py BAG...` - M9 cross-track and heading error, per phase and per lap |
| `plot_dead_time.py` | `python3 plot_dead_time.py BAG... [--source gyro\|odom]` - M11 steering dead time by cross-correlation |
| `plot_localization.py` | `python3 plot_localization.py BAG...` - M2 state timeline, matched walls per scan, innovations |
| `plot_trajectory.py` | `python3 plot_trajectory.py BAG... [--start-pose cw_pos1]` - top-down field with EKF trajectory and obstacles |
| `plot_colour_distance.py` | `python3 plot_colour_distance.py BAG...` - M4 colour classification vs range (needs `/camera_lidar/colored_scan`) |
| `plot_steer_lut.py` | `python3 plot_steer_lut.py [--calib ...steer_calib.json]` - M12 steering characteristic, no bag needed |
| `plot_parking.py` | `python3 plot_parking.py [../data/runs.csv] [--range-size 10]` - M17 parking results over the runs |
| `plot_manual.py` | `python3 plot_manual.py [--example] [--true-frame field] [--side-to-centre-cm W]` - M1 / M3 / M17 ruler from the CSVs in `docs/data/manual/`; `--gyro-integral BAG --t0 T --t1 T` helps filling M3 |
| `make_all.py` | `python3 make_all.py BAGS...` - everything above |
| `run_extras.py` | `python3 run_extras.py BAG... --rosout rosout_dump.txt --cpu cpu_phases.txt` - controller log as text and CPU load standing / driving, appended per run |
| `bag_metrics.py` | `python3 bag_metrics.py make_all_*.log -o ../data/bag_metrics.csv` - key numbers of every run from the make_all logs |
| `plot_overview.py` | `python3 plot_overview.py ../data/runs.csv ../data/rosout_dump.txt.gz` - outcome of every run, failure Pareto, CPU and battery per run |
| `plot_across_runs.py` | `python3 plot_across_runs.py ../data/bag_metrics.csv ../data/cpu_phases.txt --runs ../data/runs.csv` - dead time, latency, localisation, tracking and CPU over all runs |
| `plot_colour_pooled.py` | `python3 plot_colour_pooled.py make_all_*.log [--bags LIST]` - colour classification against range, pooled |
| `run_series.py` | the test series that count as runs, with their marker and colour |
| `bag_to_mcap.py` | `python3 bag_to_mcap.py BAG...` - recording to MCAP with message definitions, for Foxglove |

All plot tools take `--out-dir` (default `docs/figures/`) and `--msg-dir`.
Shared code: `bagio.py` (reading, flattening, topic names), `metrics.py`
(metric definitions), `logpatterns.py` (log-text patterns), `style.py`
(figure style), `robot_constants.py` (constants copied from `src/`).

## Measurement series -> tools

| Series | What | Tool | Output |
|---|---|---|---|
| M1 | EKF pose vs ground truth at checkpoints | `plot_manual.py` + `data/manual/m1_pose_checkpoints.csv` | `manual_m1_pose` |
| M2 | Localisation quality | `plot_localization.py`, runs.csv `loc_*` | `localization_<bag>`, `innovations_<bag>` |
| M3 | Encoder distance and gyro scale calibration | `plot_manual.py` + `m3_encoder_distance.csv`, `m3_gyro_turns.csv` | `manual_m3_encoder`, `manual_m3_gyro` |
| M4 | Colour classification vs distance | `plot_colour_distance.py` | `colour_distance_<bag>` |
| M9 | Stanley / arc tracking error | `plot_tracking.py`, runs.csv `ect_*`, `eth_*` | `tracking_<bag>`, `tracking_hist_<bag>` |
| M11 | Steering dead time | `plot_dead_time.py` | `dead_time_<bag>` |
| M12 | Steering characteristic | `plot_steer_lut.py` | `steer_lut` |
| M14 | Serial latency and time sync | `plot_latency.py`, runs.csv `latency_*` | `latency_<bag>`, `clocksync_<bag>` |
| M15 | CPU / thermal | `plot_cpu.py`, runs.csv `cpu_*`, `temp_max_c` | `cpu_<bag>` |
| M17 | Parking | `summarize_runs.py` -> `plot_parking.py` | `parking` |
| M17 | Parking measured with a ruler vs the robot's estimate | `plot_manual.py` + `m17_parking_ruler.csv` + runs.csv | `manual_m17_parking` |
| - | Trajectory (replaces an overhead camera) | `plot_trajectory.py` | `trajectory_<bag>`, `trajectory_laps_<bag>` |
| - | Run summary | `summarize_runs.py` | `data/runs.csv` |

## Topic and field assumptions (from the source, please confirm)

| Topic | Type | Used as |
|---|---|---|
| `/ekf/odom` | nav_msgs/Odometry | x, y, yaw (quaternion), v = twist.linear.x, omega = twist.angular.z, covariance diagonal; start-anchored map frame, base_link = rear axle |
| `/bno055/imu` | sensor_msgs/Imu | RAW gyro z; REP-103 yaw rate = gyro_z x GYRO_SCALE (-0.9674, `ekf_node.py`) |
| `/cmd_vel` | geometry_msgs/Twist | linear.x = target speed [m/s] (PI speed control in the bridge), angular.z = target **yaw rate** [rad/s], REP 103; the bridge converts it with delta = atan(L omega / v_ist) and the steer LUT. Exception: bridge parameter `steer_raw_bypass:=true` makes angular.z a raw servo fraction |
| `/round1_controller/lap_state` | std_msgs/Int32MultiArray | [corner_idx, corner_count, lap = corner_count // 4]; latched, published at drive start and after every corner |
| `/round1_controller/dbg/e_ct` | Float64 [m] | corners: distance to planned circle - R (>0 outside); straights: publish is commented out in the current controller, so "straight cross-track" columns are usually NaN |
| `/round1_controller/dbg/e_theta_deg`, `delta_deg` | Float64 [deg] | heading error / steering angle, both phases |
| `/round1_controller/dbg/k_h_eff` / `arc_R`, `arc_dist` | Float64 | only on straights / only in corners - used to tell the phase of each sample |
| `/wall_matches` | robot_msgs/WallMatchArray | per scan; alpha/d measured (robot frame) and map (map frame), HNF |
| `/localization_state` | String | ok / recovering / lost, latched, on change |
| `/race_direction` | String | CW / CCW |
| `/corner_geometry`, `/inner_geometry` | robot_msgs/CornerGeometry | outer / inner box corners in the map frame; used to place the field in the trajectory plot |
| `/obstacles` | robot_msgs/ObstacleArray | full current set (map frame), colour 0 unknown / 1 red / 2 green; last message = final map |
| `/camera_lidar/colored_scan` | sensor_msgs/PointCloud2 | fields x, y, z, rgb (float32 holding 0xRRGGBB); raw LiDAR frame (mounted 180 deg: x_b = -x + 0.1101, y_b = -y); in `cloud_color_mode: label` rgb is the class: 0xFF0000 red, 0x00FF00 green, 0xFF00FF magenta, 0x2D2D2D black, 0x555555 unknown |
| `/esp_serial_bridge/latency_ms`, `rtt_ms`, `drift_ppm` | Float32 | ms / ms / ppm; `offset_ms` Float64 (only its change matters) |
| `/esp_serial_bridge/joint_states` | sensor_msgs/JointState | drive shaft position [rad] and velocity [rad/s] (x r_eff = 0.0150 m -> m/s) |
| `/esp_serial_bridge/battery` | sensor_msgs/BatteryState | voltage [V] |
| `/jtop/cpu_total`, `gpu_load`, `ram_percent` [%], `power_total` [W] | Float32 | ~1 Hz, only when jetson-stats runs |
| `/jtop/temp/{cpu,gpu,soc,tj}` | sensor_msgs/Temperature | degC |
| `/viz/run_time`, `/viz/run_state` | Float32 / String | run clock of the Foxglove overlay |
| `/rosout` | rcl_interfaces/Log | controller log lines (parking result, emergency stops, ...) |

Time: `t_bag` = recorder receive time in seconds since the first message of
the bag; `t_header` = header stamp on the same time base (NaN without header).

Field geometry (3 x 3 m outer wall, 1 x 1 m inner wall, 24 seats, start poses)
is copied from `src/ekf/ekf/field_map.py` into `robot_constants.py` (that
module imports `ekf.ekf` and is not importable without the ROS package).

## Log text patterns (German now, English soon)

Parking result, emergency stops, emergency manoeuvres, aborts, finish/laps,
the configured dead time and gyro failures are only available as `/rosout`
text. All patterns live in ONE table, `PATTERNS` in `logpatterns.py`, with one
row for the current German string (copied from the source) and one tolerant
row for the expected English translation. After the translation, compare the
new strings with the `en` rows and adjust them there - nothing else needs to
change. `python3 logpatterns.py --check-source` reports which events no
longer have their German anchor string in `src/` (i.e. were translated), and
`python3 -m pytest tests -k log_patterns` checks the table.

Definitions used in `runs.csv` (see `summarize_runs.py`):
- `park_lateral_dev_cm` = logged distance to the outer wall minus the logged
  expected value; `park_axle_diff_cm` = logged |0.105 m x sin(heading)|;
  `park_within_2cm` = axle difference <= 2 cm (WRO rule).
- `ect_*` / `eth_*`: RMS and max |.| of the controller's own debug errors per
  phase (straight = Stanley tick with `k_h_eff`, arc = TURN tick with `arc_R`).
- `loc_*_frac`: time share of each `/localization_state` value from its first
  message to the end of the bag.

## How to take the manual measurements

**M1 - EKF pose vs ground truth** (`docs/data/manual/m1_pose_checkpoints.csv`)
1. Stick 4-8 small markers ("checkpoints", C1, C2, ...) on the mat at known
   positions, e.g. on the lane centre line in the middle of each straight.
   Measure each marker with a folding rule from two outer walls: field frame
   x = (distance to the west wall) - 1.5 m, y = (distance to the south wall) -
   1.5 m. Mark the reference point of the robot (base_link = middle of the
   rear axle) on the chassis.
2. Start the normal stack and record a bag. Drive (or push) the robot so its
   reference point stops exactly over a marker; wait 1-2 s standing still.
3. Read the EKF pose: live with `ros2 topic echo --once /ekf/odom`
   (yaw = 2 atan2(z, w) of the orientation) or later from the bag
   (`bag_export.py`, `ekf__odom.csv` at the stop time). Measure the true
   heading against a wall line with a set square / protractor.
4. Enter one row per stop. Map-frame truth goes in directly; field-frame
   truth: `python3 plot_manual.py --true-frame field --start-pose cw_pos1`.
   Repeat for 2-3 laps to see drift.

**M3 - encoder and gyro calibration**
- Encoder (`m3_encoder_distance.csv`): record a bag, drive straight >= 2 m
  slowly along a wall, mark the rear axle position on the floor at start and
  end, measure the distance with the folding rule. Put the change of
  `/esp_serial_bridge/joint_states` position [rad] between the two stops into
  `shaft_rad` (preferred) or an encoder count into `ticks` (then check
  `--rad-per-tick`). Result: r_eff = distance / shaft angle vs 0.0150 m in
  `src/ekf/ekf/ekf.py`. 5+ trials, both directions.
- Gyro (`m3_gyro_turns.csv`): align the robot with a wall line, record a bag,
  turn it exactly N full turns (N = 5) on the spot, re-align, stop. Get
  `integrated_deg` with `python3 plot_manual.py --gyro-integral BAG --t0 T0 --t1 T1`
  (raw /bno055/imu, standing still before T0 and after T1). Result:
  scale = 360 N / integrated_deg vs |GYRO_SCALE| = 0.9674 in `ekf_node.py`.

**M17 - parking measured with a ruler** (`m17_parking_ruler.csv`)
- After every parked run, before anybody touches the car: measure the distance
  from the outer wall to the side of the car at the front axle and at the rear
  axle, always to the same edge of the chassis (`front_axle_cm`,
  `rear_axle_cm`). Optionally the gap from the car to the front and rear
  magenta wall (`bay_front_cm`, `bay_rear_cm`). Put the bag name in `bag`.
- Result: axle difference |front - rear| against the 2 cm rule and the heading,
  each compared with the robot's own estimate from runs.csv (the
  "EINGEPARKT/PARKED" log line). Measure the half width of the car at the rear
  axle once and pass it as `--side-to-centre-cm` to compare the distance of
  base_link to the outer wall as well.

The `*_example.csv` files contain FAKE numbers for trying the tool
(`plot_manual.py --example`); the real templates contain only the header.

## Tests

```bash
cd docs/analysis
python3 -m pytest -q tests          # or: python3 tests/test_toolkit.py
python3 tests/make_fake_bag.py /tmp/bags --name parken_test_7   # a synthetic bag to play with
```

`tests/make_fake_bag.py` writes a synthetic bag in the Humble layout (metadata
version 5, no embedded message definitions) with all topics above, 3 laps on
the field, a known 0.25 s steering dead time, German or English log lines.
The tests run every tool on it and compare the results with the generated
values (dead time 0.250 s, parking values, localisation shares, ...).

## Known gaps

- Straight-line cross-track error: the controller does not publish `e_ct` on
  straights (line commented out in `_stanley_steer`); only the heading error
  is available there. Re-enable the publish to get it in future bags.
- `/rosout` is assumed to be recorded (`ros2 bag record -a` does). Without it
  the parking / emergency-stop columns stay empty (summary prints a warning).
- The M4 ground truth comes from the robot's own final `/obstacles` map and
  EKF pose; for a clean measurement place known pillars. Needs the
  `/camera_lidar/colored_scan` topic in the bag and `cloud_color_mode: label`.
- Dead time is measured on recorder receive times; message transport adds a
  few ms. It is the command-to-yaw-rate lag of the whole chain (bridge, servo,
  tyres), which is what `steer_dead_time` models.
- In bags that start in the parking bay the map frame is only defined after
  the start detection; the trajectory plot needs `/corner_geometry` (or
  `--start-pose`) to place the field correctly.
