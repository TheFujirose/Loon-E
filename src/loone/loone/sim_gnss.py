"""Synthetic GNSS for simulation runs: odometry -> sensor_msgs/NavSatFix.

    Isaac Sim -> ZED wrapper (sim mode) -> /<cam>/<node>/odom
        --> [THIS NODE] local metres -> lat/lon
        --> /navsatfix
        --> navsat_transform_node latches its datum and serves /fromLL
        --> gps_waypoint_mission converts the course into Nav2 goals

WHY THIS NODE EXISTS
    On the boat, `phone.py` supplies /navsatfix over ADB. bringup.launch.py
    excludes it in sim (no phone, no adb), which leaves nothing publishing a
    fix -- and navsat_transform_node will not latch a datum or answer /fromLL
    without one. task1.launch.py then hangs forever on "Waiting for /fromLL
    service", which looks like a broken service rather than a missing sensor.

WHY NOT SIMULATE GNSS INSIDE ISAAC SIM
    Isaac Sim 5.1's Kit runs Python 3.11 and Humble is 3.10, so `import rclpy`
    fails inside Kit -- see ros2_bridge.py's "WHY NO rclpy" note. Publishing a
    NavSatFix from there would mean another hand-built OmniGraph node, and Isaac
    has no GNSS publisher to reuse. Deriving the fix on the ROS 2 side from
    odometry the simulator already publishes is far less machinery, and it keeps
    the sim/hardware split exactly where every other sim stand-in sits.

ON ACCURACY
    The conversion is a local tangent-plane ("flat earth") approximation around
    the datum, good to well under a metre over a course a few km across, which
    is far better than the GNSS noise it stands in for.

    What actually matters for waypoint following is SELF-CONSISTENCY, not
    absolute position: navsat_transform latches a datum from the first fix and
    /fromLL converts lat/lon back into that same local frame, so the round trip
    cancels. Absolute coordinates only matter when comparing against a real
    surveyed course -- set `datum_lat`/`datum_lon` to the real launch point then.
"""

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus


def meters_per_degree(lat_rad: float) -> tuple:
    """Metres per degree of latitude and longitude at `lat_rad` (WGS84 series).

    The standard truncated series for the lengths of a degree of meridian and of
    parallel. Using these rather than a single 111_320 constant keeps the
    longitude scaling honest as the latitude changes.
    """
    m_per_deg_lat = (111132.92
                     - 559.82 * math.cos(2 * lat_rad)
                     + 1.175 * math.cos(4 * lat_rad)
                     - 0.0023 * math.cos(6 * lat_rad))
    m_per_deg_lon = (111412.84 * math.cos(lat_rad)
                     - 93.5 * math.cos(3 * lat_rad)
                     + 0.118 * math.cos(5 * lat_rad))
    return m_per_deg_lat, m_per_deg_lon


class SimGnss(Node):
    """Publish a NavSatFix derived from simulated odometry."""

    def __init__(self) -> None:
        super().__init__('sim_gnss')

        # Datum: the lat/lon that the odometry origin corresponds to. Defaults to
        # Humber College's Lakeshore campus waterfront -- the same placeholder
        # region as config/task1_waypoints.json, so the stock course lands in
        # front of the boat rather than in another hemisphere.
        self.declare_parameter('datum_lat', 43.5906)
        self.declare_parameter('datum_lon', -79.5352)
        self.declare_parameter('datum_alt', 76.0)

        # Rotation from the odometry frame's +x axis to EAST, in degrees, CCW.
        #
        # The odom frame is REP-103 body-relative (x forward at startup), NOT
        # ENU, so it only coincides with east/north if the boat happens to start
        # facing east. Leave this at 0 for a self-consistent sim; set it when the
        # sim heading has to agree with a real-world course.
        self.declare_parameter('odom_yaw_to_east_deg', 0.0)

        self.declare_parameter('odom_topic', '/zedx/zed_node/odom')
        # gps_link, matching phone.py. navsat_transform_node looks this frame up
        # in TF to offset the antenna from base_link, so it must be a real frame
        # in the URDF -- an empty or invented frame_id makes it drop the fix.
        self.declare_parameter('frame_id', 'gps_link')
        # Metres of 1-sigma horizontal error to advertise. Not added as noise:
        # this only populates the covariance so navsat_transform weights the fix
        # sensibly instead of treating it as exact.
        self.declare_parameter('horizontal_stddev', 1.5)

        self.datum_lat = self.get_parameter('datum_lat').value
        self.datum_lon = self.get_parameter('datum_lon').value
        self.datum_alt = self.get_parameter('datum_alt').value
        self.frame_id = self.get_parameter('frame_id').value
        yaw_deg = self.get_parameter('odom_yaw_to_east_deg').value
        stddev = self.get_parameter('horizontal_stddev').value

        self.yaw = math.radians(yaw_deg)
        self.var = stddev * stddev
        self.m_per_deg_lat, self.m_per_deg_lon = meters_per_degree(
            math.radians(self.datum_lat))

        odom_topic = self.get_parameter('odom_topic').value
        self.pub = self.create_publisher(NavSatFix, 'navsatfix', 10)
        self.sub = self.create_subscription(
            Odometry, odom_topic, self.odom_callback, 10)

        self.logged_first = False
        self.get_logger().info(
            f'sim GNSS: {odom_topic} -> /navsatfix, datum '
            f'({self.datum_lat:.6f}, {self.datum_lon:.6f}), '
            f'odom +x is {yaw_deg:.1f} deg CCW from east')

    def odom_callback(self, msg: Odometry) -> None:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z

        # Rotate odom (x, y) into ENU. Identity when odom_yaw_to_east_deg is 0.
        cos_y, sin_y = math.cos(self.yaw), math.sin(self.yaw)
        east = x * cos_y - y * sin_y
        north = x * sin_y + y * cos_y

        fix = NavSatFix()
        # Reuse the odometry stamp rather than now(): navsat_transform only
        # latches its datum from a gps/imu/odom triple it considers synchronised,
        # and under use_sim_time a wall-clock stamp would never line up with the
        # simulator's clock.
        fix.header.stamp = msg.header.stamp
        fix.header.frame_id = self.frame_id

        fix.status.status = NavSatStatus.STATUS_FIX
        fix.status.service = NavSatStatus.SERVICE_GPS

        fix.latitude = self.datum_lat + north / self.m_per_deg_lat
        fix.longitude = self.datum_lon + east / self.m_per_deg_lon
        fix.altitude = self.datum_alt + z

        # Diagonal ENU covariance. Vertical is deliberately far looser than
        # horizontal, as it is on real GNSS -- and navsat_transform is
        # configured with zero_altitude anyway.
        fix.position_covariance = [
            self.var, 0.0, 0.0,
            0.0, self.var, 0.0,
            0.0, 0.0, self.var * 9.0,
        ]
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

        self.pub.publish(fix)

        if not self.logged_first:
            self.logged_first = True
            self.get_logger().info(
                f'first fix: ({fix.latitude:.7f}, {fix.longitude:.7f}) '
                f'from odom ({x:.2f}, {y:.2f})')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimGnss()
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
