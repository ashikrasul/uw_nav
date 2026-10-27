"""ROS 2 node that ranges a remote NM3 acoustic modem.

Pings the remote modem, converts time of flight to range
(range = tof * speed_of_sound * tof_scale) and publishes sensor_msgs/Range
on 'acoustic/range' every cycle at publish_rate_hz (REP 117: range = +inf
when the remote does not reply, NaN when the local modem is not connected).

All parameters come from config/acoustic_config.yaml (auto-declared, no
defaults in code). Pinging runs in a worker thread because send_ping blocks
until the reply arrives. The node starts without the modem and reconnects
after an unplug.
"""

import math
import sys
import threading
import time

import rclpy
import serial
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Range


class AcousticRanger(Node):
    def __init__(self):
        super().__init__('acoustic_ranger',
                         automatically_declare_parameters_from_overrides=True)

        def get(name):
            return self.get_parameter(name).value

        self.port = get('port')
        self.address = int(get('remote_address'))
        self.period = 1.0 / float(get('publish_rate_hz'))
        self.ping_timeout = float(get('ping_timeout_s'))
        self.range_scale = float(get('speed_of_sound')) * float(get('tof_scale'))
        self.frame_id = get('frame_id')
        self.min_range = float(get('min_range_m'))
        self.max_range = float(get('max_range_m'))
        if not 0 <= self.address <= 255:
            raise ValueError(f'remote_address must be 0-255, got {self.address}')

        # nm3driver is not an installed package; import it from the repo folder
        sys.path.insert(0, get('nm3_driver_path'))
        from nm3driver.nm3driver import Nm3  # noqa: E402
        self._nm3_cls = Nm3

        self.pub = self.create_publisher(Range, 'acoustic/range', 10)
        self.uart = None
        self.nm3 = None
        self.running = True
        self.worker = threading.Thread(target=self.ping_loop, daemon=True)
        self.worker.start()
        self.get_logger().info(
            f'Acoustic ranger started: port={self.port}, remote={self.address}, '
            f'rate={1.0 / self.period} Hz')

    def connect(self):
        try:
            # Same settings as ranging_example.py: 9600 8N1, 100 ms read timeout
            self.uart = serial.Serial(self.port, 9600, 8, serial.PARITY_NONE,
                                      serial.STOPBITS_ONE, 0.1)
        except (serial.SerialException, OSError) as e:
            self.get_logger().warning(f'Cannot open {self.port}: {e}',
                                      throttle_duration_sec=10.0)
            return False
        self.nm3 = self._nm3_cls(input_stream=self.uart, output_stream=self.uart)

        # Confirm a modem is answering on this port ('$?' -> '#AnnnV...')
        local_address = self.nm3.get_address()
        if local_address < 0:
            self.get_logger().warning(
                f'{self.port} is open but no NM3 is responding '
                f'(check power, cable and baud rate)', throttle_duration_sec=10.0)
            self.disconnect()
            return False
        if local_address == self.address:
            self.get_logger().warning(
                f'Local modem address {local_address} equals remote_address; '
                f'it would ping itself (check config or cabling)')
        self.get_logger().info(f'Connected to NM3 (address {local_address}) on {self.port}')
        return True

    def disconnect(self):
        if self.uart is not None:
            try:
                self.uart.close()
            except Exception:
                pass
        self.uart = None
        self.nm3 = None

    def ping_loop(self):
        """One publish per cycle (REP 117): range, +inf on timeout, NaN if no modem."""
        next_ping = time.monotonic()
        while self.running and rclpy.ok():
            stamp = self.get_clock().now().to_msg()   # time the ping was sent
            self.publish_range(stamp, self.ping_once())

            # Start-to-start pacing; after an overrun, ping again right away
            next_ping = max(next_ping + self.period, time.monotonic())
            self._sleep(next_ping - time.monotonic())

    def ping_once(self):
        """Return range in m, math.inf if the remote did not reply, NaN if no local modem."""
        if self.uart is None and not self.connect():
            return math.nan
        try:
            tof = self.nm3.send_ping(self.address, timeout=self.ping_timeout)
        except (serial.SerialException, OSError) as e:
            self.get_logger().error(f'Serial connection lost: {e}')
            self.disconnect()
            return math.nan
        except Exception as e:
            self.get_logger().error(f'Ping error: {e!r}', throttle_duration_sec=5.0)
            return math.nan
        if tof is None or tof < 0:
            self.get_logger().warning(f'Ping to {self.address} failed or timed out',
                                      throttle_duration_sec=10.0)
            return math.inf
        return float(tof * self.range_scale)

    def publish_range(self, stamp, range_m):
        msg = Range()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.radiation_type = Range.ULTRASOUND
        msg.min_range = self.min_range
        msg.max_range = self.max_range
        msg.range = range_m
        self.pub.publish(msg)
        self.get_logger().debug(f'range={range_m:.2f} m')

    def _sleep(self, duration):
        """Sleep in short steps so shutdown stays responsive."""
        end = time.monotonic() + duration
        while self.running and time.monotonic() < end:
            time.sleep(0.05)

    def destroy_node(self):
        self.running = False
        self.worker.join(timeout=1.0)
        self.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AcousticRanger()
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
