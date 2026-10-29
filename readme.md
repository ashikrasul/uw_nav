# uw_nav

ROS 2 Humble navigation stack (`src/nav_pkg`): GPS (`gps/fix`), NM3 acoustic ranging (`acoustic/range`) and
BNO055 IMU (`imu/*`). It runs in Docker, so the host only needs Docker. All parameters are in
`src/nav_pkg/config/*.yaml`.

## Setup (once)

```bash
git clone --recurse-submodules https://github.com/ashikrasul/uw_nav.git && cd uw_nav
echo "I2C_GID=$(getent group i2c | cut -d: -f3)" > docker/.env   # lets the container open /dev/i2c-*
echo "export ROS_DOMAIN_ID=11" >> ~/.bashrc && source ~/.bashrc  # container uses the same domain
```

Then set the devices in the YAML:
- `port:` in `config/gps_fix.yaml` (GPS, CP2102N) and `config/acoustic_config.yaml` (NM3, FTDI).
  Use the stable paths from `ls -l /dev/serial/by-id/`.
- `i2c_bus:` in `config/imu_config.yaml` (7 on the Jetson Orin Nano). To find it: `sudo i2cdetect -y -r <bus>`
  shows `28` on the bus with the BNO055.

## Run

```bash
cd docker
docker compose up -d --build     # build and start; restarts on its own after a crash or reboot
docker compose logs -f nav       # check the nodes connect to the GPS, NM3 and IMU
./restart_service.sh nav         # after editing YAML/Python (Dockerfile changes need up -d --build)
docker compose down              # stop
```

Check the topics from a shell inside the container:
```bash
./enter_console.sh
ros2 topic echo /gps/fix
ros2 topic echo /acoustic/range  # NaN = modem not connected, inf = remote did not reply
ros2 topic echo /imu/data
```
