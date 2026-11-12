"""hardware_node: ROS interface to the boat's hardware.

Replaces motor.py. All bus / GPIO access goes through HardwareController
(loone/hardware/hardware_controller.py); this node only turns topics into
controller calls and controller state back into topics:

    HardwareController
        .propellors     PropellorController   (stop / forward / reverse / turn / PID drive)
        .ball_launcher  BallLauncherController (load / launch)
        .battery        BatteryController     (BatteryState per battery, None if no INA3221)
        .auto / .estop  RC line levels
        .set_led()      status LED

Subscribes:
    <odom_topic>           (nav_msgs/Odometry, ZED odometry; heading only)
    motor_command          (Float32MultiArray: [command, target_heading, dir])
    ball_launcher_command  (Float32MultiArray: [command, power]; command 0 stop,
                            1 load, 2 launch; power <= 0 uses the configured default)
Publishes:
    motor          (Float32MultiArray: [prop_l, prop_r, rudder] servo fractions)
    led            (ColorRGBA)
    battery_status/prop_l  (sensor_msgs/BatteryState, Dewalt, left propulsion)
    battery_status/prop_r  (sensor_msgs/BatteryState, Dewalt, right propulsion)
    battery_status/main    (sensor_msgs/BatteryState, Blue Robotics, primary battery)
    rc_state       (Int8MultiArray: [auto, estop] raw line levels)

The battery topics report present=False when the INA3221 can't be read, so that
subscribers can tell a failed sensor from a flat battery.
"""

import math
import threading

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float32MultiArray, Int8MultiArray, ColorRGBA

from loone.hardware.battery import BatteryController
from loone.hardware.hardware_controller import HardwareController

# Seconds between repeats of an I2C error. A dead bus fails on every timer tick.
BUS_ERROR_LOG_PERIOD = 5.0


def yaw_to_heading(x: float, y: float, z: float, w: float) -> float:
    """Convert an ENU orientation quaternion to a heading in degrees [0, 360).

    Yaw is counter-clockwise from +x; heading is clockwise from +y. On the ZED's odom
    frame this is relative to where the camera started, not to true north, until
    something (GPS, compass) anchors that frame.
    """
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return (90.0 - math.degrees(yaw)) % 360.0


class HardwareNode(Node):
    """Bridge ROS topics to the HardwareController."""

    def __init__(self) -> None:
        super().__init__('hardware_node')

        # ---- Parameters (defaults mirror config.yaml /hardware_node block) ----
        self.declare_parameter('timer_period', 0.5) # Timer period to publish battery / RC / LED state
        self.declare_parameter('freq', 50)          # PCA9685 PWM frequency (Hz)
        self.declare_parameter('pca_clock', 25000000)  # PCA9685 measured oscillator (Hz)
        self.declare_parameter('factor', 0.5)       # Propeller cruise throttle while driving
        self.declare_parameter('speed', 0.2)        # Propeller throttle for plain forward / reverse
        self.declare_parameter('diff', 0.4)         # Propeller throttle offset when turning in place
        self.declare_parameter('prop_min', 1120)    # propeller ESC min pulse (us)
        self.declare_parameter('prop_max', 1880)    # propeller ESC max pulse (us)
        self.declare_parameter('carriage_min', 500)   # ball carriage servo min pulse (us)
        self.declare_parameter('carriage_max', 2500)  # ball carriage servo max pulse (us)
        self.declare_parameter('carriage_range', 180) # ball carriage servo travel (degrees)
        self.declare_parameter('launcher_min', 1100)  # ball launcher ESC min pulse (us)
        self.declare_parameter('launcher_max', 1900)  # ball launcher ESC max pulse (us)
        self.declare_parameter('kp', 2.0)
        self.declare_parameter('ki', 0.5)
        self.declare_parameter('kd', 0.02)
        self.declare_parameter('max', 45.0)
        self.declare_parameter('dw_min', 15.0)   # Dewalt (prop) battery min voltage
        self.declare_parameter('dw_max', 21.0)   # Dewalt (prop) battery max voltage
        self.declare_parameter('br_min', 13.2)   # Blue Robotics (main) battery min voltage
        self.declare_parameter('br_max', 16.8)   # Blue Robotics (main) battery max voltage
        self.declare_parameter('i2c', True)
        self.declare_parameter('odom_topic', '/zedx/zed_node/odom')  # ZED odometry, for heading
        self.declare_parameter('load_time', 5.0)     # Seconds the wheel runs backwards to load
        self.declare_parameter('load_power', 0.1)    # Default launcher throttle while loading
        self.declare_parameter('launch_time', 5.0)   # Seconds the wheel runs after firing
        self.declare_parameter('launch_power', 0.1)  # Default launcher throttle while firing

        timer_period = self.get_parameter('timer_period').value
        self.i2c = self.get_parameter('i2c').value
        self.load_time = self.get_parameter('load_time').value
        self.load_power = self.get_parameter('load_power').value
        self.launch_time = self.get_parameter('launch_time').value
        self.launch_power = self.get_parameter('launch_power').value

        # ---- Hardware bring-up ----
        self.hardware = None
        if self.i2c:
            self.hardware = HardwareController(
                freq=self.get_parameter('freq').value,
                pca_clock=self.get_parameter('pca_clock').value,
                prop_min=self.get_parameter('prop_min').value,
                prop_max=self.get_parameter('prop_max').value,
                carriage_min=self.get_parameter('carriage_min').value,
                carriage_max=self.get_parameter('carriage_max').value,
                carriage_range=self.get_parameter('carriage_range').value,
                launcher_min=self.get_parameter('launcher_min').value,
                launcher_max=self.get_parameter('launcher_max').value,
                propellor_kwargs={
                    'factor': self.get_parameter('factor').value,
                    'speed': self.get_parameter('speed').value,
                    'diff': self.get_parameter('diff').value,
                    'kp': self.get_parameter('kp').value,
                    'ki': self.get_parameter('ki').value,
                    'kd': self.get_parameter('kd').value,
                    'max_output': self.get_parameter('max').value,
                },
                battery_kwargs={
                    'dw_min': self.get_parameter('dw_min').value,
                    'dw_max': self.get_parameter('dw_max').value,
                    'br_min': self.get_parameter('br_min').value,
                    'br_max': self.get_parameter('br_max').value,
                },
                logger=self.get_logger(),
            )

        # LED stuff
        self.led = ColorRGBA()
        self.led_state = True

        # Variables from topics
        self.current_heading = math.nan
        self.command = None

        # True while reads of a working INA3221 are failing, so that a dropout is logged
        # once rather than on every timer tick.
        self.battery_read_failed = False

        # Ball launcher sequences block for seconds, so they run on their own thread
        # to keep the RC-state / LED timers (and e-stop handling) responsive.
        self.ball_thread = None

        # Best effort matches the ZED's reliable publisher too, and only the newest
        # heading is worth having.
        self.odom_sub = self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value, self.odom_callback,
            QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT))
        self.task_sub = self.create_subscription(Float32MultiArray, 'motor_command', self.task_callback, 10)
        self.ball_sub = self.create_subscription(
            Float32MultiArray, 'ball_launcher_command', self.ball_launcher_callback, 10)
        self.motor_pub = self.create_publisher(Float32MultiArray, 'motor', 10)
        self.led_pub = self.create_publisher(ColorRGBA, 'led', 10)
        self.prop_l_battery_pub = self.create_publisher(BatteryState, 'battery_status/prop_l', 10)
        self.prop_r_battery_pub = self.create_publisher(BatteryState, 'battery_status/prop_r', 10)
        self.main_battery_pub = self.create_publisher(BatteryState, 'battery_status/main', 10)

        # Raw autonomy / e-stop line levels. This node already owns board.D12 and
        # D13 for the status LEDs, and two processes opening the same gpiochip
        # line fails with "Device or resource busy" under libgpiod, so terminal_node
        # reads the switches from here instead of opening them a second time. It
        # fails closed if this goes stale, so publish at a steady rate.
        self.rc_state_pub = self.create_publisher(Int8MultiArray, 'rc_state', 10)

        ## Publish
        self.battery_timer = self.create_timer(timer_period, self.publish_battery)
        self.rc_state_timer = self.create_timer(timer_period, self.publish_rc_state)
        self.led_state_timer = self.create_timer(timer_period, self.publish_led)
        self.led_timer = self.create_timer(timer_period, self.set_led)
        # motor_command only arrives when something is commanding the boat, but the
        # basestation treats a silent 'motor' topic as lost telemetry.
        self.motor_timer = self.create_timer(timer_period, self.publish_motor)
        self.get_logger().info('hardware_node ready.')

    # ------------------------------------------------------------------ errors

    def log_bus_error(self, action: str, error: Exception) -> None:
        """Log a failed I2C call, rate limited.

        An uncaught error in any callback escapes rclpy.spin and ends this node, taking
        motor control with it. The INA3221 shares the bus with the PCA9685, so one flaky
        sensor can make any bus call fail. Keep running and report it instead.
        """
        self.get_logger().error(
            f"I2C error while {action}: {error}", throttle_duration_sec=BUS_ERROR_LOG_PERIOD)

    # ------------------------------------------------------------------ publishers

    def publish_battery(self) -> None:
        """Publish a sensor_msgs/BatteryState (voltage, percentage, health) per battery.

        Each battery reports present=False if the INA3221 failed to initialize (see
        HardwareController._init_busio) or can't be read. A sensor that drops out while
        running recovers by itself as soon as a read succeeds again.
        """
        stamp = self.get_clock().now().to_msg()
        battery = None if self.hardware is None else self.hardware.battery
        states = None
        if battery is not None:
            try:
                states = battery.states(stamp)
            except Exception as e:  # Any sensor failure must not take the motors down.
                if not self.battery_read_failed:
                    self.get_logger().error(f"Lost the INA3221 battery sensor: {e}")
                self.battery_read_failed = True
            else:
                if self.battery_read_failed:
                    self.get_logger().info("INA3221 battery sensor recovered.")
                self.battery_read_failed = False

        if states is None:
            states = (BatteryController.unavailable(location, stamp)
                      for location in ('prop_l', 'prop_r', 'main'))
        prop_l, prop_r, main = states
        self.prop_l_battery_pub.publish(prop_l)
        self.prop_r_battery_pub.publish(prop_r)
        self.main_battery_pub.publish(main)

    def publish_rc_state(self) -> None:
        """Publish the raw autonomy / e-stop line levels as [auto, estop].

        The levels are published raw; task.py maps them through its
        auto_active_high / estop_clear_high parameters so the switch sense can
        be corrected from YAML without touching this node.
        """
        if self.hardware is None:
            return
        msg = Int8MultiArray()
        msg.data = [int(self.hardware.auto), int(self.hardware.estop)]
        self.rc_state_pub.publish(msg)

    def set_led(self) -> None:
        """Blink the status LED: yellow = manual, green = autonomous, red = e-stop."""
        if self.hardware is None:
            return
        match self.led_state:
            case True:
                red = green = True # Yellow
                if not self.hardware.estop:
                    green = False # red
                elif self.hardware.auto:
                    red = False # green
                self.led_state = False

            case False:
                red = green = False # off
                self.led_state = True

        try:
            self.hardware.set_led(red, green)
        except OSError as e:
            self.log_bus_error('setting the status LED', e)
            return
        self.led.r = 1.0 if red else 0.0
        self.led.g = 1.0 if green else 0.0

    def publish_led(self) -> None:
        """Publish the current LED state"""
        msg = ColorRGBA()
        msg.r = self.led.r
        msg.g = self.led.g
        msg.b = self.led.b
        msg.a = 1.0
        self.led_pub.publish(msg)

    def publish_motor(self) -> None:
        """Publish the current propeller state as servo fractions."""
        if self.hardware is None:
            return
        msg = Float32MultiArray()
        # Fractions (0 reverse, 0.5 neutral, 1 forward) rather than throttles, to keep
        # the [left, right, rudder] wire format the 'motor' subscribers already expect.
        try:
            left = self.hardware.prop_l.fraction
            right = self.hardware.prop_r.fraction
        except OSError as e:
            self.log_bus_error('reading the propeller state', e)
            return
        # A fraction is None while its PWM channel is off, e.g. if the PCA9685 reset.
        if left is None or right is None:
            self.get_logger().error(
                "Propeller PWM channel is off, not publishing motor state.",
                throttle_duration_sec=BUS_ERROR_LOG_PERIOD)
            return
        msg.data = [
            left,
            right,
            # TODO: publish the real fraction when the rudder servo is added back.
            # Until then this neutral placeholder keeps the wire format stable.
            0.5,
        ]
        self.motor_pub.publish(msg)
        self.get_logger().debug(f"Motor: {msg.data}")

    # ------------------------------------------------------------------ callbacks

    def odom_callback(self, msg) -> None:
        """Update the current heading from ZED odometry.

        Args:
            msg: nav_msgs/Odometry. See yaw_to_heading for what the heading is relative to.
        """
        q = msg.pose.pose.orientation
        self.current_heading = yaw_to_heading(q.x, q.y, q.z, q.w)

    def task_callback(self, msg) -> None:
        """Handle incoming task commands and drive the propellers.

        Args:
            msg: Float32MultiArray where index 0 is the command, index 1 is the
                target heading and index 2 is the turn direction.
        """
        data = msg.data
        self.get_logger().info(f"Task: {msg.data}")
        self.command = int(data[0])
        target_heading = data[1]
        direction = data[2]

        if self.hardware is None:
            self.get_logger().info("I2C disabled, no action")
            return
        propellors = self.hardware.propellors

        try:
            match self.command:
                case -1: #reverse
                    propellors.reverse()

                case 0: #stop
                    propellors.stop()

                case 1: #forward
                    propellors.forward()

                case 2: #turn
                    if direction != -999:
                        propellors.turn_in_place(direction)

                case 3: #drive
                    if math.isnan(self.current_heading):
                        self.get_logger().warning("No ZED heading yet, stopping instead of driving.")
                        propellors.stop()
                    else:
                        propellors.drive(self.current_heading, target_heading)

                case _: #other
                    self.get_logger().info("Improper command, no action")
                    return
        except OSError as e:
            self.log_bus_error(f'running motor command {self.command}', e)
            return

        self.publish_motor()

    def ball_launcher_callback(self, msg) -> None:
        """Handle a ball launcher command.

        Args:
            msg: Float32MultiArray where index 0 is the command (0 stops, aborting any
                running sequence; 1 loads a ball; 2 launches it) and index 1 is the
                launcher throttle in (0, 1], or <= 0 (or absent) for the default.
        """
        if self.hardware is None:
            self.get_logger().info("I2C disabled, no action")
            return
        ball_launcher = self.hardware.ball_launcher

        if len(msg.data) == 0:
            self.get_logger().info("Improper ball launcher command, no action")
            return
        command = int(msg.data[0])
        power = msg.data[1] if len(msg.data) > 1 else 0.0
        if power > 1.0:
            self.get_logger().error(f"Ball launcher power must be at most 1, got {power}.")
            return

        match command:
            case 0: #stop
                try:
                    ball_launcher.cancel()
                except OSError as e:
                    self.log_bus_error('stopping the ball launcher', e)
                return
            case 1: #load
                sequence = ball_launcher.load
                kwargs = {'time': self.load_time,
                          'power': power if power > 0.0 else self.load_power}
            case 2: #launch
                sequence = ball_launcher.launch
                kwargs = {'time': self.launch_time,
                          'power': power if power > 0.0 else self.launch_power}
            case _: #other
                self.get_logger().info("Improper ball launcher command, no action")
                return

        if self.ball_thread is not None and self.ball_thread.is_alive():
            self.get_logger().warning("Ball launcher busy, ignoring command.")
            return
        self.ball_thread = threading.Thread(target=sequence, kwargs=kwargs, daemon=True)
        self.ball_thread.start()

    def shutdown(self) -> None:
        """Return the actuators to neutral and release the hardware on node shutdown."""
        if self.hardware is not None:
            self.hardware.shutdown()


def main(args=None) -> None:
    """Initialize the ROS2 node and spin."""
    rclpy.init(args=args)
    node = HardwareNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('hardware_node interrupted by user.')
    except Exception as e:
        node.get_logger().error(f'hardware_node error: {e}')
    finally:
        node.shutdown()
        node.destroy_node()
        # The SIGINT handler may have already shut down the context.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
