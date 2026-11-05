"""ROS 2 node that ranges remote NM3 acoustic modems (beacons), one after another.

For each address in remote_addresses, in turn: ping up to 1 + ping_retries times, convert the
time of flight to range (range = tof * speed_of_sound * tof_scale) and publish one
sensor_msgs/Range on 'acoustic/beacon_<addr>/range'. One beacon per slot at ping_rate_hz.
REP 117: range = +inf when the beacon did not reply, NaN when the local modem is not working.

Hardware errors never stop the node: they are logged, shown on /diagnostics and retried.
All parameters come from config/beacons.yaml + config/acoustic_config.yaml (no code defaults).
"""

import math
import sys
import threading
import time

import rclpy
import serial
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_updater import Updater
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Range

from nav_drivers.serial_ports import find_port


class AcousticRanger(Node):
    def __init__(self):
        super().__init__('acoustic_ranger',
                         automatically_declare_parameters_from_overrides=True)

        def get(name):
            return self.get_parameter(name).value

        self.port_pattern = get('port')            # glob, e.g. /dev/ttyUSB*
        self.port = None                           # the port found, once connected
        self.addresses = [int(a) for a in get('remote_addresses')]
        self.period = 1.0 / float(get('ping_rate_hz'))
        self.ping_timeout = float(get('ping_timeout_s'))
        self.ping_retries = int(get('ping_retries'))
        self.range_scale = float(get('speed_of_sound')) * float(get('tof_scale'))
        self.frame_id = get('frame_id')
        self.min_range = float(get('min_range_m'))
        self.max_range = float(get('max_range_m'))
        if not self.addresses or any(not 0 <= a <= 255 for a in self.addresses):
            raise ValueError(f'remote_addresses must be a list of 0-255, got {self.addresses}')
        if self.ping_retries < 0:
            raise ValueError(f'ping_retries must be >= 0, got {self.ping_retries}')

        # nm3driver is not an installed package; import it from the repo folder
        sys.path.insert(0, get('nm3_driver_path'))
        from nm3driver.nm3driver import Nm3  # noqa: E402
        self._nm3_cls = Nm3

        self.pubs = {a: self.create_publisher(Range, f'acoustic/beacon_{a}/range', 10)
                     for a in self.addresses}
        self.uart = None
        self.nm3 = None
        self.error = 'not connected yet'           # local modem problem, None when working
        self.last = {a: 'no ping yet' for a in self.addresses}
        self.no_reply = set()                      # beacons that did not answer last time

        self.diagnostics = Updater(self)
        self.diagnostics.setHardwareID(f'NM3 on {self.port_pattern}')
        self.diagnostics.add('acoustic modem', self.diagnose)

        self.running = True
        self.worker = threading.Thread(target=self.ping_loop, daemon=True)
        self.worker.start()
        self.get_logger().info(
            f'Acoustic ranger started: port={self.port_pattern}, beacons={self.addresses}, '
            f'{1.0 / self.period} pings/s, retries={self.ping_retries}')

    # ------------------------------------------------------------------ connection
    def probe(self, port):
        """Return (uart, nm3, local address) if an NM3 answers '$?' on port, else None."""
        # Same settings as ranging_example.py: 9600 8N1, 100 ms read timeout
        uart = serial.Serial(port, 9600, 8, serial.PARITY_NONE, serial.STOPBITS_ONE, 0.1,
                             exclusive=True)
        try:
            nm3 = self._nm3_cls(input_stream=uart, output_stream=uart)
            address = nm3.get_address()              # '$?' -> '#AnnnV...', -1 if no reply
        except BaseException:
            uart.close()
            raise
        if address < 0:
            uart.close()
            return None
        return uart, nm3, address

    def connect(self):
        port, found = find_port(self.port_pattern, self.probe)
        if port is None:
            return self.fail(f'no NM3 is responding: {found} (check power and cable)')
        self.port = port
        self.uart, self.nm3, local_address = found
        if local_address in self.addresses:
            self.get_logger().warning(
                f'Local modem address {local_address} is in remote_addresses; '
                f'it would ping itself (check config or cabling)')
        self.error = None
        self.get_logger().info(f'Connected to NM3 (address {local_address}) on {self.port}')
        return True

    def fail(self, message):
        """Record a local modem error, close the port. Returns False."""
        self.error = message
        self.get_logger().error(message, throttle_duration_sec=10.0)
        self.disconnect()
        return False

    def disconnect(self):
        if self.uart is not None:
            try:
                self.uart.close()
            except Exception:
                pass
        self.uart = None
        self.nm3 = None

    # ------------------------------------------------------------------ worker thread
    def ping_loop(self):
        next_slot = time.monotonic()
        while self.running and rclpy.ok():
            for address in self.addresses:
                if not self.running:
                    break
                stamp = self.get_clock().now().to_msg()
                try:
                    stamp, range_m = self.range_beacon(address)
                except Exception as e:     # keep the thread alive whatever happens
                    self.fail(f'unexpected error while pinging {address}: {e!r}')
                    range_m = math.nan
                self.publish_range(address, stamp, range_m)

                # Start-to-start pacing; after an overrun, ping again right away
                next_slot = max(next_slot + self.period, time.monotonic())
                self._sleep(next_slot - time.monotonic())

    def range_beacon(self, address):
        """Return (stamp, range): range in m, inf if no reply after retries, NaN if no modem."""
        for _ in range(1 + self.ping_retries):
            stamp = self.get_clock().now().to_msg()   # time the ping was sent
            if self.uart is None and not self.connect():
                return stamp, math.nan
            try:
                tof = self.nm3.send_ping(address, timeout=self.ping_timeout)
            except (serial.SerialException, OSError) as e:
                self.fail(f'serial connection to {self.port} lost: {e}')
                return stamp, math.nan
            if tof is not None and tof >= 0:
                range_m = float(tof * self.range_scale)
                self.last[address] = f'{range_m:.2f} m'
                self.no_reply.discard(address)
                return stamp, range_m

        self.last[address] = 'no reply'
        self.no_reply.add(address)
        self.get_logger().warning(
            f'Beacon {address}: no reply after {1 + self.ping_retries} ping(s)',
            throttle_duration_sec=10.0)
        return stamp, math.inf

    def publish_range(self, address, stamp, range_m):
        msg = Range()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.radiation_type = Range.ULTRASOUND
        msg.min_range = self.min_range
        msg.max_range = self.max_range
        msg.range = range_m
        self.pubs[address].publish(msg)

    # ------------------------------------------------------------------ diagnostics
    def diagnose(self, stat):
        if self.error:
            stat.summary(DiagnosticStatus.ERROR, self.error)
        elif self.no_reply:
            stat.summary(DiagnosticStatus.WARN, f'no reply from beacon(s) {sorted(self.no_reply)}')
        else:
            stat.summary(DiagnosticStatus.OK, 'connected')
        stat.add('port', self.port or 'none')
        for address, last in self.last.items():
            stat.add(f'beacon {address}', last)
        return stat

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
