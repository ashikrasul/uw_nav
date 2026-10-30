# uw_nav

ROS 2 Humble navigation stack for a surface agent with a GPS, a BNO055 IMU and an NM3 acoustic modem that
ranges remote NM3 modems (beacons). It estimates the agent's position, heading and velocity
(robot_localization EKF) and locates the beacons by least squares. It runs in Docker, so the host only
needs Docker. All parameters are in `src/nav_bringup/config/*.yaml`.

## Packages

| package | contents |
|---|---|
| `src/nav_drivers` | `gps_fix` (`gps/fix`, `gps/vel`), `imu_node` (`imu/*`), `acoustic_node` (`acoustic/beacon_<addr>/range`) |
| `src/nav_localization` | `beacon_locator` (`beacons/beacon_<addr>/pose`, `.../fix`) |
| `src/nav_bringup` | `launch/nav.launch.py` (whole stack), `config/` (all YAML), `urdf/agent.urdf` (sensor offsets), `rviz/` |

The EKF and GPS conversion come from `robot_localization` (configured in `ekf.yaml`, `navsat.yaml`).
To add a node: put it in a package (or a new one under `src/`), add one row to `NODES` in
`nav.launch.py` and its YAML in `config/`.

## Setup (once)

```bash
git clone --recurse-submodules https://github.com/ashikrasul/uw_nav.git && cd uw_nav
echo "I2C_GID=$(getent group i2c | cut -d: -f3)" > docker/.env   # lets the container open /dev/i2c-*
echo "GPIO_GID=$(getent group gpio | cut -d: -f3)" >> docker/.env  # /dev/gpiochip* (Jetson); no gpio group on a PC: reuse I2C_GID
echo "export ROS_DOMAIN_ID=11" >> ~/.bashrc && source ~/.bashrc  # container uses the same domain
mkdir -p ~/nav_logs                                               # rosbag recordings
```

Then set in `src/nav_bringup/config/`:
- `port:` in `gps_fix.yaml` (GPS, CP2102N) and `acoustic_config.yaml` (NM3, FTDI). Use the stable paths
  from `ls -l /dev/serial/by-id/`.
- `i2c_bus:` in `imu_config.yaml` (7 on the Jetson Orin Nano; `sudo i2cdetect -y -r <bus>` shows `28`).
- `remote_addresses:` in `beacons.yaml` (the remote modems, polled in turn).
- `transducer_depth_m:` in `beacon_locator.yaml`, `magnetic_declination_radians:` in `navsat.yaml`,
  sensor offsets in `urdf/agent.urdf`.

## Run

```bash
cd docker
docker compose up -d --build     # build and start nav + record; restarts on its own after a crash or reboot
docker compose logs -f nav       # check the nodes connect to the GPS, NM3 and IMU
./restart_service.sh nav         # after editing YAML/Python (Dockerfile changes need up -d --build)
docker compose down              # stop
```

Topics, from a shell inside the container (`./enter_console.sh`):

| topic | type | what |
|---|---|---|
| `gps/fix` | NavSatFix | raw GPS, 10 Hz |
| `gps/vel` | TwistWithCovarianceStamped | GPS Doppler velocity, ENU (x east, y north) m/s, 10 Hz; not fused |
| `imu/data` | Imu | orientation (ENU yaw, 0 = magnetic east), gyro, accel |
| `acoustic/beacon_<addr>/range` | Range | NaN = local modem not working, inf = beacon did not reply |
| `odometry/filtered` | Odometry | EKF: position in `odom` (local ENU, m), heading, velocities in `base_link` |
| `gps/filtered` | NavSatFix | EKF position as lat/lon |
| `beacons/beacon_<addr>/pose` | PoseWithCovarianceStamped | beacon in `odom`, z = −depth |
| `beacons/beacon_<addr>/fix` | NavSatFix | beacon lat/lon, altitude = −depth |
| `diagnostics` | DiagnosticArray | per-sensor status: OK / WARN / ERROR with the reason |

A sensor that is unplugged or not responding does not stop the stack: its node logs an error, shows it
on `/diagnostics` and keeps retrying.

## Live view (Foxglove)

`foxglove_bridge` runs in the stack on port 8765.
1. On a laptop on the same network, open Foxglove (app or https://app.foxglove.dev) →
   *Open connection* → *Foxglove WebSocket* → `ws://<agent-ip>:8765`.
2. Add panels:
   - **Map**: `gps/fix`, `gps/filtered` (trajectory), `beacons/beacon_<addr>/fix`. Needs internet on the laptop for tiles.
   - **3D**: display frame `odom`; `odometry/filtered`, `/tf`, `beacons/beacon_<addr>/pose`.
   - **Plot**: `odometry/filtered.twist.twist.linear.x` / `.y`, `imu/data` ...
   - **Diagnostics**: sensor status.
3. Save the layout (*Layout → Export*) to reuse it.

On a PC with ROS 2: `rviz2 -d src/nav_bringup/rviz/nav.rviz` (needs `ROS_LOCALHOST_ONLY=0` on both machines).

## Logging and replay

The `record` service records every topic to `~/nav_logs/nav_<date>_<time>/` (rosbag2, MCAP, zstd-compressed) whenever the
stack runs; each start makes a new bag.

```bash
docker compose stop record        # close the current bag (start: docker compose start record)
ros2 bag info ~/nav_logs/<bag>    # inside a ROS environment
ros2 bag play ~/nav_logs/<bag>
```

Or open the `.mcap` file directly in Foxglove (*Open local file*) with the same layout.

## Before trusting the results

- NM3 time of flight: one-way or round trip is not verified. Range over a known baseline;
  set `tof_scale: 0.5` in `acoustic_config.yaml` if it is round trip.
- Heading: calibrate the BNO055 (Diagnostics → imu → calibration sys/gyro/accel/mag, aim for sys = 3), then point the bow east and check
  the yaw in `odometry/filtered` ≈ 0; correct with `heading_offset_rad` in `imu_config.yaml`.
- Beacons: survey around each beacon (circle or crossing legs), not in a straight line.
