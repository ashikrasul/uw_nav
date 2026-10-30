"""ROS 2 node that locates remote NM3 modems (beacons) from acoustic ranges by least squares.

For each beacon address (beacons.yaml), every finite range rho on 'acoustic/beacon_<addr>/range'
is paired with the transducer position (px, py) at the ping mid-time, looked up in TF
(world_frame -> transducer_frame; the EKF trajectory plus the URDF lever arm). With the beacon
at (x, y) and the transducer at a known depth, each ping gives one linear equation
(method 1 lifting, utils/nm3-python-simulator/examples/basic_triangulation.py):

    rho^2 - px^2 - py^2 = -2 px x - 2 py y + gamma,     gamma = x^2 + y^2 + dz^2

All pings so far form A theta = b, theta = [x, y, gamma]. The overdetermined system is solved
through the normal equations (A^T A) theta = A^T b; depth = transducer depth + dz.

Publishes per beacon, after every new solution:
  beacons/beacon_<addr>/pose   geometry_msgs/PoseWithCovarianceStamped in world_frame (z = -depth)
  beacons/beacon_<addr>/fix    sensor_msgs/NavSatFix (lat/lon from navsat_transform's toLL
                               service; altitude = -depth below the surface, not ellipsoidal)
State per beacon (waiting for pings, poor geometry, ...) is shown on /diagnostics.
All parameters come from config/beacons.yaml + config/beacon_locator.yaml (no code defaults).
"""

import math

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_updater import Updater
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from robot_localization.srv import ToLL
from sensor_msgs.msg import NavSatFix, NavSatStatus, Range
from tf2_ros import Buffer, TransformException, TransformListener


def spread(p_xy):
    """Smallest horizontal spread [m] of the ping positions (0 for points on a line)."""
    return float(np.sqrt(max(np.linalg.eigvalsh(np.cov(p_xy.T)).min(), 0.0)))


def solve(p_xy, rho, transducer_depth):
    """Least squares beacon position from transducer positions p_xy (N, 2) and ranges rho (N,).

    Returns (x, y, depth, cov): cov is the approximate 3x3 covariance of (x, y, depth).
    depth and its variances are NaN when gamma < x^2 + y^2 (ranges inconsistent with the track).
    """
    A = np.column_stack([-2.0 * p_xy[:, 0], -2.0 * p_xy[:, 1], np.ones(len(rho))])
    b = rho ** 2 - p_xy[:, 0] ** 2 - p_xy[:, 1] ** 2

    N = A.T @ A                                 # normal equations: N theta = A^T b
    theta = np.linalg.solve(N, A.T @ b)
    x, y, gamma = theta

    residual = b - A @ theta
    sigma2 = residual @ residual / max(len(rho) - 3, 1)
    cov_theta = sigma2 * np.linalg.inv(N)       # in the lifted (x, y, gamma) space

    dz2 = gamma - x * x - y * y
    if dz2 <= 0.0:
        cov = np.full((3, 3), math.nan)
        cov[:2, :2] = cov_theta[:2, :2]
        return x, y, math.nan, cov
    dz = math.sqrt(dz2)
    J = np.array([[1.0, 0.0, 0.0],              # d(x, y, depth) / d(x, y, gamma)
                  [0.0, 1.0, 0.0],
                  [-x / dz, -y / dz, 0.5 / dz]])
    return x, y, transducer_depth + dz, J @ cov_theta @ J.T


class Beacon:
    def __init__(self):
        self.p_xy = []                  # transducer positions at ping mid-time
        self.rho = []                   # measured ranges
        self.estimate = None            # (x, y, depth)
        self.rejected = 0               # ranges dropped by the outlier gate
        self.status = (DiagnosticStatus.WARN, 'no ranges yet')


class BeaconLocator(Node):
    def __init__(self):
        super().__init__('beacon_locator',
                         automatically_declare_parameters_from_overrides=True)

        def get(name):
            return self.get_parameter(name).value

        self.addresses = [int(a) for a in get('remote_addresses')]
        self.speed_of_sound = float(get('speed_of_sound'))
        self.transducer_depth = float(get('transducer_depth_m'))
        self.world_frame = get('world_frame')
        self.transducer_frame = get('transducer_frame')
        self.min_pings = max(int(get('min_pings')), 4)   # 3 unknowns + 1 for the residual
        self.min_spread = float(get('min_spread_m'))
        self.gate = float(get('outlier_gate_m'))
        self.max_range = float(get('max_range_m'))

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.to_ll = self.create_client(ToLL, 'toLL')

        self.beacons = {a: Beacon() for a in self.addresses}
        self.pose_pubs, self.fix_pubs = {}, {}
        self.diagnostics = Updater(self)
        self.diagnostics.setHardwareID('beacon_locator')
        for a in self.addresses:
            self.create_subscription(Range, f'acoustic/beacon_{a}/range',
                                     lambda msg, a=a: self.on_range(a, msg), 10)
            self.pose_pubs[a] = self.create_publisher(
                PoseWithCovarianceStamped, f'beacons/beacon_{a}/pose', 10)
            self.fix_pubs[a] = self.create_publisher(NavSatFix, f'beacons/beacon_{a}/fix', 10)
            self.diagnostics.add(f'beacon {a}', lambda stat, a=a: self.diagnose(a, stat))

        self.get_logger().info(
            f'Beacon locator started: beacons={self.addresses}, '
            f'transducer depth={self.transducer_depth} m, frame={self.world_frame}')

    def on_range(self, address, msg):
        try:
            self.add_range(address, msg)
        except Exception as e:     # bad data must not stop the node
            self.beacons[address].status = (DiagnosticStatus.ERROR, f'unexpected error: {e!r}')
            self.get_logger().error(f'Beacon {address}: {e!r}', throttle_duration_sec=10.0)

    def add_range(self, address, msg):
        beacon = self.beacons[address]
        rho = msg.range
        if not math.isfinite(rho) or not 0.0 < rho <= self.max_range:
            return                          # no reply / no modem: nothing to add

        # Transducer position at the ping mid-time (send time + one-way travel time)
        t = Time.from_msg(msg.header.stamp) + Duration(seconds=rho / self.speed_of_sound)
        try:
            tf = self.tf_buffer.lookup_transform(self.world_frame, self.transducer_frame, t)
        except TransformException as e:
            beacon.status = (DiagnosticStatus.WARN, f'no agent position at ping time: {e}')
            return
        px, py = tf.transform.translation.x, tf.transform.translation.y

        if beacon.estimate is not None and self.gate > 0.0:
            x, y, depth = beacon.estimate
            predicted = math.hypot(px - x, py - y, depth - self.transducer_depth)
            if abs(rho - predicted) > self.gate:
                beacon.rejected += 1
                return

        beacon.p_xy.append((px, py))
        beacon.rho.append(rho)
        n = len(beacon.rho)
        if n < self.min_pings:
            beacon.status = (DiagnosticStatus.WARN, f'{n}/{self.min_pings} pings')
            return
        p_xy = np.array(beacon.p_xy)
        s = spread(p_xy)
        if s < self.min_spread:
            beacon.status = (DiagnosticStatus.WARN,
                             f'{n} pings, but spread {s:.1f} m < {self.min_spread} m: '
                             f'move across the beacon, not in a straight line')
            return

        x, y, depth, cov = solve(p_xy, np.array(beacon.rho), self.transducer_depth)
        if math.isnan(depth):
            beacon.status = (DiagnosticStatus.WARN,
                             'ranges inconsistent with the track (no real depth): '
                             'check tof_scale, speed of sound and transducer depth')
            return
        beacon.estimate = (x, y, depth)
        sigma_xy = math.sqrt(max(cov[0, 0], 0.0) + max(cov[1, 1], 0.0))
        beacon.status = (DiagnosticStatus.OK,
                         f'{n} pings, depth {depth:.1f} m, horizontal sigma {sigma_xy:.1f} m')
        self.publish(address, x, y, depth, cov)

    def publish(self, address, x, y, depth, cov):
        cov = cov * np.outer([1, 1, -1], [1, 1, -1])    # (x, y, depth) -> ENU (x, y, z = -depth)
        stamp = self.get_clock().now().to_msg()
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.world_frame
        p = msg.pose.pose.position
        p.x, p.y, p.z = float(x), float(y), -float(depth)
        msg.pose.pose.orientation.w = 1.0
        c = [0.0] * 36
        for i in range(3):
            for j in range(3):
                c[6 * i + j] = float(cov[i, j])
        msg.pose.covariance = c
        self.pose_pubs[address].publish(msg)

        if not self.to_ll.service_is_ready():
            self.beacons[address].status = (
                DiagnosticStatus.WARN, 'no lat/lon: navsat_transform toLL service not ready')
            return
        request = ToLL.Request()
        request.map_point.x, request.map_point.y = float(x), float(y)
        future = self.to_ll.call_async(request)
        future.add_done_callback(
            lambda f: self.publish_fix(address, stamp, f, depth, cov))

    def publish_fix(self, address, stamp, future, depth, cov):
        result = future.result()
        if result is None:
            return
        msg = NavSatFix()
        msg.header.stamp = stamp
        msg.header.frame_id = self.world_frame
        msg.status.status = NavSatStatus.STATUS_FIX
        msg.latitude = result.ll_point.latitude
        msg.longitude = result.ll_point.longitude
        msg.altitude = -float(depth)        # below the surface, not ellipsoidal height
        msg.position_covariance = [float(v) for v in cov.flatten()]   # ENU
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        self.fix_pubs[address].publish(msg)

    def diagnose(self, address, stat):
        beacon = self.beacons[address]
        stat.summary(*beacon.status)
        stat.add('pings used', str(len(beacon.rho)))
        stat.add('pings rejected (outlier gate)', str(beacon.rejected))
        if beacon.estimate is not None:
            x, y, depth = beacon.estimate
            stat.add(f'position in {self.world_frame} [m]', f'x={x:.1f} y={y:.1f}')
            stat.add('depth [m]', f'{depth:.1f}')
        return stat


def main(args=None):
    rclpy.init(args=args)
    node = BeaconLocator()
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
