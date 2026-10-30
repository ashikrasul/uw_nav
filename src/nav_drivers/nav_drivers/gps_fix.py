"""ROS 2 driver node for the Adafruit Ultimate GPS (MTK3339) over USB serial.

Publishes sensor_msgs/NavSatFix on 'gps/fix', one message per GGA sentence,
so the publish rate equals the receiver's configured update rate.

Also publishes geometry_msgs/TwistWithCovarianceStamped on 'gps/vel', one per RMC
sentence with a fix: the receiver's Doppler speed over ground and course as an ENU
velocity (linear.x east, linear.y north, m/s; course is relative to true north).

Robustness:
  - starts even if the GPS is not plugged in, and keeps retrying
  - detects a USB unplug, closes the port, reconnects and reconfigures
  - publishes STATUS_NO_FIX (NaN position) when the receiver has no fix
  - watchdog warns when no GGA data has arrived within stale_timeout_s
  - hardware errors never stop the node; the state is shown on /diagnostics
"""

import math
import threading
import time

import adafruit_gps
import rclpy
import serial
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_updater import Updater
from geometry_msgs.msg import TwistWithCovarianceStamped
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus

DEFAULT_BAUD = 9600            # MTK3339 power-up baud rate
NMEA_BYTES_PER_FIX = 145       # approx. one GGA + one RMC sentence incl. CRLF
SUPPORTED_BAUDS = (9600, 14400, 19200, 38400, 57600, 115200)
KNOT_M_S = 0.514444            # 1 knot in m/s


def _f(x):
    """Convert a driver value to float, mapping None to NaN."""
    return float(x) if x is not None else math.nan


class GpsPublisher(Node):
    def __init__(self):
        super().__init__('gps_publisher')

        # ---------------- parameters ----------------
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('update_rate_hz', 10.0)
        self.declare_parameter('frame_id', 'gps_link')
        self.declare_parameter('uere_m', 3.0)
        self.declare_parameter('stale_timeout_s', 2.0)
        self.declare_parameter('reconnect_period_s', 2.0)
        self.declare_parameter('vel_frame_id', 'odom')
        self.declare_parameter('speed_stddev_m_s', 0.2)

        self.port = self.get_parameter('port').value
        self.baud = int(self.get_parameter('baud_rate').value)
        self.rate_hz = float(self.get_parameter('update_rate_hz').value)
        self.frame_id = self.get_parameter('frame_id').value
        self.uere = float(self.get_parameter('uere_m').value)
        self.stale_timeout = float(self.get_parameter('stale_timeout_s').value)
        self.reconnect_period = float(self.get_parameter('reconnect_period_s').value)
        self.vel_frame_id = self.get_parameter('vel_frame_id').value
        self.speed_var = float(self.get_parameter('speed_stddev_m_s').value) ** 2
        self._validate_parameters()

        # ---------------- ROS interfaces ----------------
        self.pub = self.create_publisher(NavSatFix, 'gps/fix', 10)
        self.vel_pub = self.create_publisher(TwistWithCovarianceStamped, 'gps/vel', 10)
        self.create_timer(1.0, self.watchdog)

        # ---------------- reader thread state ----------------
        self.uart = None
        self.gps = None
        self.last_gga_time = None   # time.monotonic() of the last GGA sentence
        self.error = 'not connected yet'   # port problem, None while connected
        self.fix_status = None             # NavSatStatus of the last GGA

        self.diagnostics = Updater(self)
        self.diagnostics.setHardwareID(f'GPS on {self.port}')
        self.diagnostics.add('gps', self.diagnose)

        self.running = True

        self.reader = threading.Thread(target=self.reader_loop, daemon=True)
        self.reader.start()
        self.get_logger().info(
            f'GPS publisher started: port={self.port}, baud={self.baud}, '
            f'rate={self.rate_hz} Hz')

    def _validate_parameters(self):
        if self.baud not in SUPPORTED_BAUDS:
            raise ValueError(f'baud_rate must be one of {SUPPORTED_BAUDS}, got {self.baud}')
        if not 1.0 <= self.rate_hz <= 10.0:
            raise ValueError(f'update_rate_hz must be in [1, 10], got {self.rate_hz}')

        # Serial line utilization U = f * L / (baud / 10)  (8N1: 10 bits per byte)
        utilization = self.rate_hz * NMEA_BYTES_PER_FIX / (self.baud / 10.0)
        if utilization > 1.0:
            raise ValueError(
                f'{self.rate_hz} Hz needs {utilization:.0%} of a {self.baud} baud line; '
                f'increase baud_rate')
        if utilization > 0.5:
            self.get_logger().warning(
                f'Serial line utilization is {utilization:.0%}; consider a higher baud_rate')

    # ------------------------------------------------------------------ #
    # Connection management (runs in the reader thread)
    # ------------------------------------------------------------------ #
    def connect(self):
        """Open the port and configure the receiver. Returns True on success."""
        try:
            self.uart = serial.Serial(self.port, baudrate=DEFAULT_BAUD, timeout=0.5)
            self.gps = adafruit_gps.GPS(self.uart, debug=False)
            self.configure_receiver()
        except (serial.SerialException, OSError) as e:
            self.error = f'cannot open {self.port}: {e}'
            self.get_logger().error(self.error, throttle_duration_sec=10.0)
            self.disconnect()
            return False

        self.error = None
        self.last_gga_time = time.monotonic()   # grace period for the watchdog
        self.get_logger().info(f'Connected to GPS on {self.port}')
        return True

    def configure_receiver(self):
        # The module is either at its 9600 default (fresh power-up) or already at
        # self.baud (node restarted without a power cycle). Sending the baud
        # command at 9600 works in the first case and is ignored as noise in the
        # second; after switching our side we are in sync either way.
        if self.baud != DEFAULT_BAUD:
            self.gps.send_command(f'PMTK251,{self.baud}'.encode())
            self.uart.flush()                 # wait until bytes are transmitted
            time.sleep(0.1)                   # let the module switch
            self.uart.baudrate = self.baud

        self.uart.reset_input_buffer()        # drop bytes garbled by the switch
        time.sleep(0.1)

        self.gps.send_command(b'PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
        time.sleep(0.1)
        period_ms = int(round(1000.0 / self.rate_hz))
        self.gps.send_command(f'PMTK220,{period_ms}'.encode())

    def disconnect(self):
        if self.uart is not None:
            try:
                self.uart.close()
            except Exception:
                pass
        self.uart = None
        self.gps = None

    # ------------------------------------------------------------------ #
    # Reader thread
    # ------------------------------------------------------------------ #
    def reader_loop(self):
        while self.running and rclpy.ok():
            if self.uart is None and not self.connect():
                time.sleep(self.reconnect_period)
                continue

            try:
                prev = self.gps.nmea_sentence
                self.gps.update()
                sentence = self.gps.nmea_sentence
                if sentence is not prev:
                    self.get_logger().debug(f'RX: {sentence}')
                    if 'GGA' in sentence:
                        self.last_gga_time = time.monotonic()
                        self.publish_fix()
                    elif 'RMC' in sentence:
                        self.publish_vel()
            except (serial.SerialException, OSError) as e:
                self.error = f'serial connection to {self.port} lost: {e}'
                self.get_logger().error(self.error)
                self.disconnect()
                continue
            except Exception as e:
                self.get_logger().error(f'Reader error: {e!r}', throttle_duration_sec=2.0)

            time.sleep(0.005)

    def publish_fix(self):
        g = self.gps
        msg = NavSatFix()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        q = g.fix_quality or 0
        if q == 0:
            status = NavSatStatus.STATUS_NO_FIX
        elif q == 2:
            status = NavSatStatus.STATUS_SBAS_FIX
        elif q in (4, 5):
            status = NavSatStatus.STATUS_GBAS_FIX
        else:
            status = NavSatStatus.STATUS_FIX
        msg.status.status = status
        self.fix_status = status
        msg.status.service = NavSatStatus.SERVICE_GPS

        if status == NavSatStatus.STATUS_NO_FIX:
            msg.latitude = msg.longitude = msg.altitude = math.nan
            self.get_logger().warning('No GPS fix', throttle_duration_sec=5.0)
        else:
            msg.latitude = _f(g.latitude)
            msg.longitude = _f(g.longitude)
            msg.altitude = _f(g.altitude_m) + _f(g.height_geoid)   # h = h_MSL + N

        hdop = g.horizontal_dilution
        if status != NavSatStatus.STATUS_NO_FIX and hdop is not None:
            sigma_h = hdop * self.uere
            sigma_v = 2.0 * sigma_h
            msg.position_covariance = [sigma_h**2, 0.0, 0.0,
                                       0.0, sigma_h**2, 0.0,
                                       0.0, 0.0, sigma_v**2]
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        else:
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

        self.pub.publish(msg)

    def publish_vel(self):
        g = self.gps
        if not g.fix_quality or g.speed_knots is None:
            return                            # RMC status V (no fix) or no speed field
        if g.track_angle_deg is None:
            v, course = 0.0, 0.0              # receiver omits the course when stationary
        else:
            v, course = g.speed_knots * KNOT_M_S, math.radians(g.track_angle_deg)

        msg = TwistWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.vel_frame_id
        msg.twist.twist.linear.x = v * math.sin(course)   # east (course is clockwise from north)
        msg.twist.twist.linear.y = v * math.cos(course)   # north
        for i in (0, 7, 14):                  # vx, vy, vz on the 6x6 diagonal
            msg.twist.covariance[i] = self.speed_var
        self.vel_pub.publish(msg)

    # ------------------------------------------------------------------ #
    # Watchdog (runs in the executor, main thread)
    # ------------------------------------------------------------------ #
    def watchdog(self):
        if self.uart is None:
            return                            # disconnect is reported by the reader
        last = self.last_gga_time
        if last is None or time.monotonic() - last > self.stale_timeout:
            self.get_logger().warning(
                f'No GGA data for more than {self.stale_timeout:.1f} s '
                f'(check baud rate and receiver configuration)',
                throttle_duration_sec=5.0)

    def diagnose(self, stat):
        last = self.last_gga_time
        if self.error:
            stat.summary(DiagnosticStatus.ERROR, self.error)
        elif last is None or time.monotonic() - last > self.stale_timeout:
            stat.summary(DiagnosticStatus.ERROR,
                         f'no GGA data for more than {self.stale_timeout:.1f} s')
        elif self.fix_status == NavSatStatus.STATUS_NO_FIX:
            stat.summary(DiagnosticStatus.WARN, 'no fix')
        else:
            stat.summary(DiagnosticStatus.OK, 'fix')
        stat.add('port', self.port)
        return stat

    def destroy_node(self):
        self.running = False
        self.reader.join(timeout=1.0)
        self.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GpsPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
