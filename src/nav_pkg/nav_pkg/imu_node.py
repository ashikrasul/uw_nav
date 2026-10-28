"""ROS 2 node for the Adafruit BNO055 9-DOF IMU on an I2C bus.

Publishes, each one switched on/off in config/imu_config.yaml:
  imu/data          sensor_msgs/Imu            orientation, gyro [rad/s],
                                               accel incl. gravity [m/s^2]
  imu/mag           sensor_msgs/MagneticField  [T]
  imu/euler         geometry_msgs/Vector3Stamped  x=roll, y=pitch, z=heading [rad]
                    (BNO055 convention: heading is clockwise from magnetic north, not ROS ENU yaw)
  imu/linear_accel  geometry_msgs/Vector3Stamped  accel without gravity [m/s^2]
  imu/gravity       geometry_msgs/Vector3Stamped  gravity vector [m/s^2]
  imu/temperature   sensor_msgs/Temperature    [degC]
  imu/calibration   std_msgs/UInt8MultiArray   [sys, gyro, accel, mag], each 0-3

All parameters come from the YAML (auto-declared, no defaults in code). Reading runs in a
worker thread at publish_rate_hz. The node starts without the sensor and reconnects after
an I2C error; nothing is published while disconnected.
"""

import math
import threading
import time

import adafruit_bno055 as bno
import rclpy
from adafruit_extended_bus import ExtendedI2C
from geometry_msgs.msg import Vector3Stamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField, Temperature
from std_msgs.msg import UInt8MultiArray

# Which sensor data each BNO055 operation mode produces (datasheet table 3-3)
ACCEL_MODES = {bno.ACCONLY_MODE, bno.ACCMAG_MODE, bno.ACCGYRO_MODE, bno.AMG_MODE,
               bno.IMUPLUS_MODE, bno.COMPASS_MODE, bno.M4G_MODE,
               bno.NDOF_FMC_OFF_MODE, bno.NDOF_MODE}
MAG_MODES = {bno.MAGONLY_MODE, bno.ACCMAG_MODE, bno.MAGGYRO_MODE, bno.AMG_MODE,
             bno.COMPASS_MODE, bno.M4G_MODE, bno.NDOF_FMC_OFF_MODE, bno.NDOF_MODE}
GYRO_MODES = {bno.GYRONLY_MODE, bno.ACCGYRO_MODE, bno.MAGGYRO_MODE, bno.AMG_MODE,
              bno.IMUPLUS_MODE, bno.NDOF_FMC_OFF_MODE, bno.NDOF_MODE}
FUSION_MODES = {bno.IMUPLUS_MODE, bno.COMPASS_MODE, bno.M4G_MODE,
                bno.NDOF_FMC_OFF_MODE, bno.NDOF_MODE}


def _diag(stddev):
    var = stddev * stddev
    return [var, 0.0, 0.0, 0.0, var, 0.0, 0.0, 0.0, var]


def _vec(v, xyz, scale=1.0):
    v.x, v.y, v.z = (float(a) * scale for a in xyz)


class ImuPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher',
                         automatically_declare_parameters_from_overrides=True)

        def get(name):
            return self.get_parameter(name).value

        self.bus = int(get('i2c_bus'))
        self.address = int(get('i2c_address'))
        self.mode_name = str(get('operation_mode')).upper()
        self.period = 1.0 / float(get('publish_rate_hz'))
        self.frame_id = get('frame_id')
        self.reconnect_period = float(get('reconnect_period_s'))
        self.cov_orient = _diag(float(get('orientation_stddev_rad')))
        self.cov_gyro = _diag(float(get('gyro_stddev_rad_s')))
        self.cov_accel = _diag(float(get('accel_stddev_m_s2')))
        self.cov_mag = _diag(float(get('mag_stddev_t')))

        self.mode = getattr(bno, f'{self.mode_name}_MODE', None)
        if self.mode is None or self.mode == bno.CONFIG_MODE:
            raise ValueError(f'operation_mode {self.mode_name!r} is not a BNO055 data mode')
        self.has_accel = self.mode in ACCEL_MODES
        self.has_mag = self.mode in MAG_MODES
        self.has_gyro = self.mode in GYRO_MODES
        self.has_fusion = self.mode in FUSION_MODES

        # (param, topic, type, available in this mode)
        outputs = [
            ('publish_imu', 'imu/data', Imu, True),
            ('publish_mag', 'imu/mag', MagneticField, self.has_mag),
            ('publish_euler', 'imu/euler', Vector3Stamped, self.has_fusion),
            ('publish_linear_accel', 'imu/linear_accel', Vector3Stamped, self.has_fusion),
            ('publish_gravity', 'imu/gravity', Vector3Stamped, self.has_fusion),
            ('publish_temperature', 'imu/temperature', Temperature, True),
            ('publish_calibration', 'imu/calibration', UInt8MultiArray, True),
        ]
        self.pubs = {}
        for param, topic, msg_type, available in outputs:
            if not get(param):
                continue
            if not available:
                raise ValueError(f'{param} is true but {self.mode_name} mode does not provide it')
            self.pubs[topic] = self.create_publisher(msg_type, topic, 10)

        self.i2c = None
        self.sensor = None
        self.last_attempt = -math.inf
        self.running = True
        self.worker = threading.Thread(target=self.read_loop, daemon=True)
        self.worker.start()
        self.get_logger().info(
            f'IMU publisher started: /dev/i2c-{self.bus} @ {self.address:#04x}, '
            f'mode={self.mode_name}, rate={1.0 / self.period} Hz, '
            f'topics={list(self.pubs)}')

    def connect(self):
        self.last_attempt = time.monotonic()
        try:
            self.i2c = ExtendedI2C(self.bus)
            self.sensor = bno.BNO055_I2C(self.i2c, self.address)   # resets the chip
            self.sensor.mode = self.mode
        except RuntimeError as e:   # bad chip id
            self.get_logger().warning(
                f'Device at {self.address:#04x} on /dev/i2c-{self.bus} is not a BNO055: {e}',
                throttle_duration_sec=10.0)
            self.disconnect()
            return False
        except (OSError, ValueError) as e:
            self.get_logger().warning(
                f'No BNO055 at {self.address:#04x} on /dev/i2c-{self.bus}: {e}',
                throttle_duration_sec=10.0)
            self.disconnect()
            return False
        self.get_logger().info(f'Connected to BNO055 on /dev/i2c-{self.bus}')
        return True

    def disconnect(self):
        if self.i2c is not None:
            try:
                self.i2c.deinit()
            except Exception:
                pass
        self.i2c = None
        self.sensor = None

    def read_loop(self):
        next_read = time.monotonic()
        while self.running and rclpy.ok():
            if self.sensor is not None or (
                    time.monotonic() - self.last_attempt >= self.reconnect_period
                    and self.connect()):
                try:
                    self.publish_all(self.get_clock().now().to_msg())
                except OSError as e:
                    self.get_logger().error(f'I2C read failed, reconnecting: {e}')
                    self.disconnect()

            # Start-to-start pacing; after an overrun, read again right away
            next_read = max(next_read + self.period, time.monotonic())
            self._sleep(next_read - time.monotonic())

    def publish_all(self, stamp):
        s = self.sensor
        pub = self.pubs

        if 'imu/data' in pub:
            msg = Imu()
            msg.header.stamp = stamp
            msg.header.frame_id = self.frame_id
            msg.orientation_covariance[0] = -1.0     # REP 145: -1 = not provided
            msg.angular_velocity_covariance[0] = -1.0
            msg.linear_acceleration_covariance[0] = -1.0
            if self.has_fusion:
                w, x, y, z = s.quaternion
                o = msg.orientation
                o.x, o.y, o.z, o.w = float(x), float(y), float(z), float(w)
                msg.orientation_covariance = self.cov_orient
            if self.has_gyro:
                _vec(msg.angular_velocity, s.gyro)
                msg.angular_velocity_covariance = self.cov_gyro
            if self.has_accel:
                _vec(msg.linear_acceleration, s.acceleration)
                msg.linear_acceleration_covariance = self.cov_accel
            pub['imu/data'].publish(msg)

        if 'imu/mag' in pub:
            msg = MagneticField()
            msg.header.stamp = stamp
            msg.header.frame_id = self.frame_id
            _vec(msg.magnetic_field, s.magnetic, 1e-6)  # uT -> T
            msg.magnetic_field_covariance = self.cov_mag
            pub['imu/mag'].publish(msg)

        if 'imu/euler' in pub:
            heading, roll, pitch = s.euler
            self._publish_vector('imu/euler', stamp, (roll, pitch, heading), math.pi / 180.0)
        if 'imu/linear_accel' in pub:
            self._publish_vector('imu/linear_accel', stamp, s.linear_acceleration)
        if 'imu/gravity' in pub:
            self._publish_vector('imu/gravity', stamp, s.gravity)

        if 'imu/temperature' in pub:
            msg = Temperature()
            msg.header.stamp = stamp
            msg.header.frame_id = self.frame_id
            msg.temperature = float(s.temperature)
            pub['imu/temperature'].publish(msg)

        if 'imu/calibration' in pub:
            pub['imu/calibration'].publish(UInt8MultiArray(data=list(s.calibration_status)))

    def _publish_vector(self, topic, stamp, xyz, scale=1.0):
        msg = Vector3Stamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        _vec(msg.vector, xyz, scale)
        self.pubs[topic].publish(msg)

    def _sleep(self, duration):
        """Sleep in short steps so shutdown stays responsive."""
        end = time.monotonic() + duration
        while self.running and time.monotonic() < end:
            time.sleep(min(0.05, max(0.0, end - time.monotonic())))

    def destroy_node(self):
        self.running = False
        self.worker.join(timeout=1.0)
        self.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ImuPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
