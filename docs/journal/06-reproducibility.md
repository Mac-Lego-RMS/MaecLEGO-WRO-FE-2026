# Reproducibility

**Evidence at a glance.** Where this chapter answers each point of the rubric
for criterion 5:

| The rubric asks for | Section | Key evidence |
| --- | --- | --- |
| Fully reproducible | [Building and flashing](#building-and-flashing) | environment recipe, device rules, calibration files, firmware; the three upstream packages pinned or included with their changes |
| Clear project structure | [Repository structure](#repository-structure) | the WRO folder layout; one folder per component |
| CAD, code and wiring included | [Repository structure](#repository-structure) | PCB in KiCad with BOM, chassis CAD, all code; wiring in [chapter 2](03-power-sensors.md) |
| Documented testing workflow | [Testing workflow](#testing-workflow) | unit tests without hardware, reproducible bench sweeps, every field run recorded and evaluated, CI |
| Meaningful commits, versioning | [Versions](#versions) | more than 300 commits since November 2025; tagged releases with notes |

## Repository structure

| Path | Content |
| --- | --- |
| [`src/`](../../src) | all code: the ROS 2 packages, the ESP32 firmware, the start script |
| [`src/esp_firmware/`](../../src/esp_firmware) | firmware of the ESP32-S3 on the main board (PlatformIO) |
| [`schemes/`](../../schemes) | main PCB: KiCad project, schematic PDF, BOM and placement files |
| [`models/`](../../models) | CAD as STEP: full assembly and chassis |
| [`src/Hardware/`](../../src/Hardware) | chassis CAD: Fusion 360 source and STL |
| [`config/`](../../config) | calibration files the nodes load at run time |
| [`setup/`](../../setup) | everything the Jetson needs beyond the repository |
| [`docs/`](..) | this journal, the analysis scripts, the measurement data behind every figure |
| `v-photos/`, `t-photos/`, `video/` | vehicle photos, team photos, driving videos |

The ROS 2 packages in `src/`:

| Package | Role |
| --- | --- |
| `esp_bridge` | serial link to the ESP32; speed controller, steering table, clock sync; `pwm_sweep` for bench tests |
| `ekf` | localisation (EKF and field map), scan processing, race controller, parking and unparking |
| `camera_lidar_fusion` | colour for every LiDAR point; camera calibration tools |
| `robot_msgs` | message definitions |
| `sllidar_ros2`, `bno055`, `ros_deep_learning` | sensor drivers from upstream projects, see below |
| `robot_vision`, `camera_capture` | the first vehicle generation (YOLO) and dataset capture; no longer started |

## Building and flashing

**Software environment.** [`setup/README.md`](../../setup/README.md) takes a
fresh Jetson Orin Nano to a running vehicle in seven steps: JetPack 6.2, clone
with Git LFS and submodules, device rules for `/dev/rplidar` and `/dev/picam`, container
image, workspace build, firmware, autostart. The container image is described by
[`setup/Dockerfile`](../../setup/Dockerfile). The original image had been set up
by hand; the Dockerfile was reconstructed from the running container and lists
the installed versions, but it has not been test-built.

Three packages come from upstream projects. Each is either pinned or included
together with its changes, so the repository builds exactly what the vehicle
runs:

| Package | Source | State |
| --- | --- | --- |
| `ros_deep_learning` | dusty-nv/ros_deep_learning | submodule, unchanged, commit `5229849` |
| `sllidar_ros2` | Slamtec/sllidar_ros2 at `3430009` | included; port `/dev/rplidar`, motor at 900 rpm for 15 Hz |
| `bno055` | flynneva/bno055 at `45e1ff1` | included; parameter `publish_only_imu` |

**Firmware.** A PlatformIO project with the platform release pinned
(pioarduino 54.03.21, Arduino core 3.x) and its libraries listed in
[`platformio.ini`](../../src/esp_firmware/platformio.ini):

```bash
cd src/esp_firmware
pio run -t upload        # over the USB-C port of the main board
```

The serial protocol between the Jetson and the ESP32 is specified in
[`src/esp_firmware/docs/JETSON_BRIDGE.md`](../../src/esp_firmware/docs/JETSON_BRIDGE.md).

**Hardware.** The main PCB is a KiCad project in
[`schemes/MainPCB`](../../schemes/MainPCB); BOM and placement files for
assembly are in its `production/` folder, the Gerber files are generated from the
board when ordering. The CAD is in [`models`](../../models) (STEP) and
[`src/Hardware`](../../src/Hardware) (Fusion 360 source, STL).

## Running the vehicle

At power-on `robot.service` runs [`src/start_robot.sh`](../../src/start_robot.sh).
It starts the container and one `tmux` window per node — LiDAR, IMU, ESP32
bridge, camera, fusion, EKF, scan processor, race controller, Foxglove. The run
itself starts with the start button, as the rules require: one switch powers
the vehicle, one button starts the program.

The challenge is selected at the top of the start script:

| Variable | Values | Effect |
| --- | --- | --- |
| `RACE_MODE` | `open`, `obstacle` | map and pillar detection for the challenge |
| `UNPARK` | `true`, `false` | start from the parking bay |
| `PACE` | `slow`, `medium`, `fast` | speed profile of the race controller |

`tmux attach -t robot_session` shows every node; Foxglove shows the map, the
scan and the camera live. `./start_robot.sh --calib` additionally starts the
camera calibration tool.

## Testing workflow

Every change is checked on three levels.

**Without hardware.** Unit tests run on any computer, without ROS:

```bash
cd src/camera_lidar_fusion && python3 -m pytest test -q   # 56 tests: fisheye model, colours, blind sectors, white point
cd src/ekf/ekf && python3 test_unpark.py                  # unpark geometry; likewise the other test_*.py,
                                                          # some of which take a recorded bag as argument
pip install -r docs/analysis/requirements.txt
cd docs/analysis && python3 -m pytest -q tests            # 14 tests: analysis toolkit on synthetic bags
```

**On the bench.** Measurements are repeatable by construction: the speed sweep
`ros2 run esp_bridge pwm_sweep` drives identical steps before and after a change,
and hand measurements go to [`docs/data/manual/`](../data/manual) — see
[chapter 2](03-power-sensors.md).

**On the field.** Every run is recorded as a rosbag. The scripts in
[`docs/analysis/`](../analysis) turn the bags into metrics and figures; the
results over all 293 recorded runs are in [chapter 3](04-software.md). Per run,
`make_all.py` writes the key numbers of every tool to a log and `run_extras.py`
extracts the controller log and the CPU load; `bag_metrics.py`,
`plot_overview.py`, `plot_across_runs.py` and `plot_parking.py` then combine all
runs. The combined inputs — [`runs.csv`](../data/runs.csv),
[`bag_metrics.csv`](../data/bag_metrics.csv),
[`rosout_dump.txt.gz`](../data/rosout_dump.txt.gz),
[`cpu_phases.txt`](../data/cpu_phases.txt) and the per-run logs
([`make_all_logs.tar.gz`](../data/make_all_logs.tar.gz)) — are in the
repository, so every figure can be rebuilt without the 15 GB of recordings.
[`run_series.py`](../analysis/run_series.py) lists which recordings count as
test runs.

On every push to `main`, GitHub Actions checks the README length and
untranslated text and builds this journal as a PDF.

### Evaluation in Foxglove

Every run is looked at twice: live on the field, and afterwards from its
recording. Both use [Foxglove](https://foxglove.dev) with the same view.

**Live.** `foxglove_bridge` runs on the vehicle (window 4 of the start script) and
a laptop connects to it over the network. The node `foxglove_overlay`
([`ekf/foxglove_overlay_node.py`](../../src/ekf/ekf/foxglove_overlay_node.py))
draws what the software believes: the field as the scan processor measured it,
the walls matched to the map, the driven path, the pose with its uncertainty, the
detected pillars and the run clock. It also forwards the Jetson's load and
temperatures. Nothing on the vehicle depends on it; it exists only to be looked
at.

One layout shows a run at a glance: the camera with the colour of every LiDAR
point, the overlay, the diagnostics, the ESP link, the battery, the
race state (direction, lap, corner) and the log. Further tabs group the plots
for control, drive, localisation, geometry and the system.

![The Foxglove layout replaying run `parken_test_4`. Left: the fisheye image with the calibrated LiDAR zone and the colour count of the current scan (33 red, 2 190 black, 113 unknown points), below it the unwrapped panorama. Middle: the field overlay. Right: diagnostics, here reporting a saturated CPU core and the Jetson temperature, the ESP link and the battery. Bottom: race state and log.](../figures/foxglove_layout_camera_run.png)

**From the recording.** The `/viz` topics of the overlay are recorded with every
run, so the same view can be replayed and stepped through. A `.db3` recording
does not contain its message definitions, and Foxglove's built-in definition of
`visualization_msgs/Marker` does not match ROS 2 Humble, so the overlay markers
fail to decode. [`bag_to_mcap.py`](../analysis/bag_to_mcap.py) converts a
recording to MCAP with the Humble definitions and our own `robot_msgs` embedded,
without changing a byte of the messages and without a ROS installation:

```bash
pip install rosbags
python3 docs/analysis/bag_to_mcap.py ~/runs/cw_pos1_22   # -> ~/runs/cw_pos1_22_mcap/
```

![Replay of run `cw_pos1_22` (obstacle challenge, clockwise): the outer and inner walls as measured, with their normals; the driven path in yellow; the detected pillars in red and green; the vehicle with the LiDAR points of the current scan, coloured by the camera (grey: black wall, red and green: pillar).](../figures/foxglove_3d_obstacle.png)

<!-- TODO figures: open challenge replay; Jetson load of the last obstacle run
(parken_test_49). TODO: export the Foxglove layouts to setup/foxglove/ and link
them here (the layout is called wro_overlay_layout). -->

The replay answers the questions a number cannot: where the vehicle lost a wall,
which pillar was seen from where, what the controller saw when it braked. The
numbers themselves then come from the scripts in
[`docs/analysis/`](../analysis), over all runs at once
([chapter 3](04-software.md)).

**A fault that was in every recording.** Plotted over a run (bottom of the
figure below), the reported pack voltage is a flat line at 17.52 V, 4.38 V per cell. That is impossible twice: a
full LiPo cell has 4.2 V, and under load the voltage has to fall. The table of
all runs ([`data/runs.csv`](../data/runs.csv)) shows the same value as the
minimum voltage of all 73 recorded runs between 11 and 29 September. The reading
had been stuck for weeks, and because 17.5 V looks like a full battery, nobody
noticed while driving; the low-voltage warning could never have fired. The cause
was a failed resistor in the voltage divider
([chapter 2](03-power-sensors.md#fault-found-and-fixed-the-divider-read-18--high)).
A plausible value is not proof of a working sensor; a signal that never moves is
the warning sign.

![The system tab replaying run `parken_test_4`. Top: the ESP link's own diagnostics. Middle: latency of the link, a few milliseconds with single peaks, and the clock drift between ESP and Jetson, re-estimated in steps. Bottom: the reported battery voltage, a flat line at 17.52 V (4.38 V per cell) through the whole run.](../figures/foxglove_system_tab.png)

## Versions

| Version | Commit | State |
| --- | --- | --- |
| [`v1.0`](https://github.com/Mac-Lego-RMS/MaecLEGO-WRO-FE-2026/releases/tag/v1.0) | `40f0dad` | national final, June 2026 |
| `v2.0` | — | international final; tagged with the submission |

Each version is a tagged release with notes on GitHub. For every tag starting
with `v`, the CI attaches the journal PDF to the release. Between the versions
the history has more than 300 commits since November 2025, each one change with
a message that says what changed and why.
