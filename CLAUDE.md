# CLAUDE.md

ROS 2 navigation stack for an underwater/surface navigation setup: a moving agent with a GPS and an NM3
acoustic modem that ranges remote NM3 modems (beacons). Repo: https://github.com/ashikrasul/uw_nav

## Working preferences (from the user)
- **Share a plan and get approval before editing code.** Explore read-only first.
- Keep code **minimal but readable**; nodes stay lite.
- **All parameters live in the YAML config**, not in node code. Nodes use
  `automatically_declare_parameters_from_overrides=True` and read values with `get_parameter`;
  no defaults in code, so a missing key fails at startup.
- **One launch file** for the whole stack (`nav.launch.py`); no per-node launch files, no launch-argument
  overrides of YAML values (ports etc. come only from YAML).

## Environment
- Ubuntu 22.04, Python 3.10, ROS 2 Humble (`/opt/ros/humble`).
- `mapviz`, `mapviz_plugins`, `tile_map`, `swri_transform_util`, `rviz2`, `rosbag2_py` are installed;
  `folium` and `gh` CLI are not.
- VS Code is a snap: GUI tools (rviz2, mapviz) started from its terminal can crash on snap env vars;
  restore the `<VAR>_VSCODE_SNAP_ORIG` values (see
  `utils/nm3-python-simulator/ros2_ws/src/nm3_beacon_sim/nm3_beacon_sim/launch_utils.py::clean_gui_env`).

## Layout
| path | what |
|---|---|
| `src/nav_pkg/` | ament_python package (the stack) |
| `src/nav_pkg/nav_pkg/gps_fix.py` | Adafruit Ultimate GPS (MTK3339) driver → `sensor_msgs/NavSatFix` on `gps/fix` |
| `src/nav_pkg/nav_pkg/acoustic_node.py` | NM3 ranging node (node name `acoustic_ranger`) |
| `src/nav_pkg/config/gps_fix.yaml` | GPS params (node `gps_publisher`) |
| `src/nav_pkg/config/acoustic_config.yaml` | acoustic params (node `acoustic_ranger`) |
| `src/nav_pkg/nav_pkg/imu_node.py` | BNO055 9-DOF IMU over I2C (node `imu_publisher`) |
| `src/nav_pkg/config/imu_config.yaml` | IMU params: bus, address, mode, rate, which outputs to publish |
| `src/nav_pkg/launch/nav.launch.py` | starts all three nodes, `respawn=True` |
| `src/nav_pkg/nav_pkg/waypoint_node.py`, `gps_test.py` | older files, not part of the launch |
| `utils/nm3-python-simulator/` | **git submodule** (fork `ashikrasul/nm3-python-simulator`, upstream bensherlock). NM3 driver, simulator, examples. Has its own `CLAUDE.md`. Treat `nm3sim/` and `nm3driver/` as a library; do not edit. |

## acoustic_node behaviour
- Imports `nm3driver.nm3driver.Nm3` by adding `nm3_driver_path` (YAML, absolute path to the submodule)
  to `sys.path`; the driver is not pip-installed.
- Serial: `serial.Serial(port, 9600, 8N1, timeout=0.1)`, same as `examples/ranging_example.py`.
- On connect it calls `Nm3.get_address()` (`$?` → `#AnnnV...`, 0.5 s timeout). `-1` → warn
  "open but no NM3 is responding", close, retry next cycle. Warns if local address == `remote_address`.
- Worker thread loop (send_ping blocks), start-to-start pacing at `publish_rate_hz` (default 0.5 Hz).
- `range = tof * speed_of_sound * tof_scale`, published as `sensor_msgs/Range` on `acoustic/range`
  **every cycle** (REP 117): measured range; `+inf` when the remote does not reply; `NaN` when the local
  modem is not connected/responding or on a serial error. `radiation_type = ULTRASOUND`.
- Reconnects every cycle after an unplug; catches `ExternalShutdownException` for a clean SIGTERM stop.

## imu_node behaviour
- Runs on the Pi/Jetson: BNO055 on the GPIO I2C pins, opened with `adafruit_extended_bus.ExtendedI2C(i2c_bus)` +
  `adafruit_bno055.BNO055_I2C` (pip: `adafruit-circuitpython-bno055 adafruit-extended-bus`; user needs the `i2c` group).
- `operation_mode` from YAML (NDOF, IMUPLUS, ...). Each `publish_*` flag enables one topic: `imu/data` (Imu, REP 145,
  accel incl. gravity, covariance[0] = -1 for fields the mode lacks), `imu/mag` (T), `imu/euler` (rad, x=roll y=pitch
  z=heading, BNO heading is clockwise from north), `imu/linear_accel`, `imu/gravity`, `imu/temperature`, `imu/calibration`.
  A flag the mode can't provide fails at startup.
- Worker thread at `publish_rate_hz`; on I2C error disconnects and retries every `reconnect_period_s`; publishes nothing
  while disconnected.

## Build / run
```bash
cd ~/works/ros_nav_ws
colcon build --packages-select nav_pkg          # add --symlink-install to skip rebuilds on .py/.yaml edits
source install/setup.bash
ros2 launch nav_pkg nav.launch.py
ros2 topic echo /acoustic/range
```
- Without `--symlink-install`, edits to `.py`/`.yaml` need a rebuild. New files in `config/`/`launch/` always do.
- Renamed/deleted files stay in `install/` until removed by hand.
- Lint: `python3 -m flake8 --max-line-length 99 <files>`.
- Test nodes without disturbing a running stack: use a separate `ROS_DOMAIN_ID` and a fake port
  (`-p port:=/dev/does_not_exist`).

## Hardware notes
- `/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A94OLO1E-if00-port0` (was `ttyUSB1`): presumably the NM3.
- `/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_...` (was `ttyUSB0`): GPS.
- Config still uses `/dev/ttyUSB0` / `/dev/ttyUSB1`; switching to the by-id paths is recommended.
- 2026-10-26: the NM3 on ttyUSB1 did **not** answer `$?` (no `#A` reply). Not yet diagnosed: check power,
  whether opening the FTDI port resets the modem (DTR/RTS), baud rate. Next step was a raw `$?` probe
  with the stack stopped.
- **NM3 ToF convention unverified** (one-way vs round-trip). Range over a known baseline before trusting
  data; set `tof_scale: 0.5` if it is round-trip.

## History / next steps
- 2026-10-26: a beacon-triangulation step was implemented and then **reverted at the user's request**
  (not committed). Its approved design, for when it is picked up again:
  - multiple beacons in `acoustic_config.yaml` under a shared `/**` section (`beacon_addresses`,
    `beacon_depths_m`, `transducer_depth_m`, `speed_of_sound`); acoustic node pings all beacons per cycle,
    one `Range` topic per beacon (`acoustic/beacon_<addr>/range`).
  - `beacon_locator` node: Mode 1 "find the beacons". GPS → local ENU (equirectangular, datum = first fix),
    pair each range with GPS at ping mid-time, linear LS per beacon with known depth
    (method1 lifting from `utils/.../examples/basic_triangulation.py`). Publishes per beacon
    `NavSatFix` (lat/lon) and `[slant, horizontal]` distances, plus a MarkerArray.
  - Visualization: node only publishes markers; RViz started by the launch file. Post-experiment map
    comparison: record with `ros2 bag record -a` into `~/nav_logs/`, replay in mapviz
    (`tile_map` + `navsat` plugins, origin from `swri_transform_util initialize_origin.py`).
- `~/nav_logs/nav_20261026_205455` is a leftover bag from that period.
