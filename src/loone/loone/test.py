"""hardware_test: short one-shot hardware demo, then exits.

Prints battery levels, pulses both motors, exercises the ball loader/launcher
and flashes the LEDs. Stop hardware_node before running (same I2C bus).
"""

import time

import rclpy
from rclpy.node import Node

from loone.hardware.hardwareController import HardwareController


class HardwareTest(Node):
    """Run the PCA9685 / INA3221 hardware demo once."""

    def __init__(self) -> None:
        super().__init__('hardware_test')

    def run(self) -> None:
        # speed is the throttle forward() uses for the motor pulse below.
        hardware = HardwareController(
            freq=50, propellor_kwargs={'speed': 0.12}, logger=self.get_logger())
        try:
            if hardware.battery is None:
                self.get_logger().warning("No INA3221, skipping battery levels.")
            else:
                for state in hardware.battery.states():
                    self.get_logger().info(
                        f"{state.location} battery: {round(state.percentage * 100)}% "
                        f"({state.voltage:.1f} V)")

            input('Load ball, then press Enter to run the loader.')
            hardware.ball_launcher.load()

            self.get_logger().info("Motors...")
            hardware.propellors.forward()
            time.sleep(0.5)
            hardware.propellors.stop()

            input('Press Enter to launch the ball.')
            hardware.ball_launcher.launch()
            input('Load ball, then press Enter to run the loader.')
            hardware.ball_launcher.load()

            self.get_logger().info("Party mode!")
            for _ in range(2):
                for red, green in ((True, False), (True, True), (False, True), (False, False)):
                    hardware.set_led(red, green)
                    time.sleep(0.15)
        finally:
            hardware.shutdown()


def main(args=None) -> None:
    """Initialize the ROS2 node, run the demo once and exit."""
    rclpy.init(args=args)
    node = HardwareTest()
    try:
        node.run()
        node.get_logger().info('hardware_test done.')
    except KeyboardInterrupt:
        node.get_logger().info('hardware_test interrupted by user.')
    except Exception as e:
        node.get_logger().error(f'hardware_test error: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
