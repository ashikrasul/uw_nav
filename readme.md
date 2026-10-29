# ros_nav_ws

ROS 2 Humble navigation stack (`src/nav_pkg`): GPS (`gps/fix`), NM3 acoustic ranging (`acoustic/range`) and
BNO055 IMU (`imu/data`, `imu/mag`, `imu/calibration`). All three nodes start from one launch file,
`nav.launch.py`; all parameters are in `src/nav_pkg/config/*.yaml`.

The stack runs in Docker, so the Jetson only needs Docker. ROS on the host is not required.

# Run on a Jetson

### 1. Prerequisites (once)
```bash
docker --version && docker compose version   # JetPack ships Docker; if `compose` is missing:
sudo apt install docker-compose-plugin       #   (or docker-compose-v2, depending on the apt source)
sudo usermod -aG docker,dialout,i2c $USER    # then log out and back in
id -u; id -g                                 # should both be 1000 (see step 3 if not)
```

### 2. Get the code
```bash
git clone --recurse-submodules https://github.com/ashikrasul/uw_nav.git ~/ros_nav_ws
cd ~/ros_nav_ws
# already cloned without submodules:  git submodule update --init
```

### 3. Machine-specific settings: `docker/docker-compose.override.yml`
`docker compose` loads this file automatically next to `docker-compose.yml`. Keep it untracked.

The container runs as user `ros` (UID 1000). Without the right group it can't open `/dev/i2c-*`:
```bash
getent group i2c          # e.g. i2c:x:108:...  -> GID 108
```
```yaml
# docker/docker-compose.override.yml
services:
  nav:
    group_add: ["108"]    # i2c GID on this Jetson
    # Only if `id -u` / `id -g` are not 1000:
    # build:
    #   args: {USER_UID: 1001, USER_GID: 1001}
```

### 4. Set ports and the I2C bus in the YAML
- Serial ports: list them with `ls -l /dev/serial/by-id/`, then set `port:` in
  `src/nav_pkg/config/gps_fix.yaml` (GPS, CP2102N) and `src/nav_pkg/config/acoustic_config.yaml` (NM3, FTDI).
  The `/dev/serial/by-id/...` paths don't change when you re-plug.
- I2C bus: the header pins 3/5 bus number depends on the Jetson model. Find the BNO055 (address `0x28`):
  ```bash
  i2cdetect -l                 # list buses
  sudo i2cdetect -y -r 7       # try each bus number; look for 28
  ```
  Set `i2c_bus:` in `src/nav_pkg/config/imu_config.yaml`.

### 5. ROS network settings
`docker compose` takes `ROS_DOMAIN_ID` and `ROS_LOCALHOST_ONLY` from the shell that runs it (default 0 / 0).
Set them in `~/.bashrc` before step 6:
```bash
export ROS_DOMAIN_ID=11
export ROS_LOCALHOST_ONLY=0    # 0: other machines on the network can see the topics; 1: this Jetson only
```

### 6. Start
```bash
cd ~/ros_nav_ws/docker
docker compose up -d --build   # first build takes a while; after that it starts on every boot
docker compose logs -f nav     # check that the nodes connect to the GPS, NM3 and IMU
```

### 7. Check the topics
```bash
./enter_console.sh             # shell inside the container, ROS already sourced
ros2 topic list
ros2 topic echo /gps/fix
ros2 topic echo /acoustic/range    # NaN = modem not connected, inf = remote did not reply
ros2 topic echo /imu/data
```

# Docker commands

```bash
cd docker
docker compose up -d --build     # build the image and the workspace, run nav.launch.py
docker compose logs -f nav       # node output
./enter_console.sh               # shell in the nav container
./restart_service.sh nav         # restart after editing config/launch
docker compose down              # stop (otherwise it restarts after a crash or reboot)
```
- `Dockerfile` (repo root) has all dependencies. `src/` and `utils/` are mounted, not copied, so YAML/Python
  edits only need `./restart_service.sh nav`; a Dockerfile change needs `up -d --build`.
- The container uses the host network and runs as UID 1000, so ROS on the host (if installed, same
  `ROS_DOMAIN_ID`) sees its topics.
- Don't run the stack on the host and in the container at the same time: both open the same serial ports.

# Run without Docker (host with ROS 2 Humble)

```bash
pip3 install pyserial adafruit-circuitpython-gps adafruit-circuitpython-bno055 adafruit-extended-bus
colcon build --symlink-install --packages-select nav_pkg
source install/setup.bash
ros2 launch nav_pkg nav.launch.py
```
`nm3_driver_path` in `acoustic_config.yaml` holds the container path; set it to the host path of
`utils/nm3-python-simulator` for this.
