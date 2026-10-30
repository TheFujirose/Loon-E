"""terminal_node: drive the ball launcher and status LED from terminal input.

Commands (typed at the '>' prompt):
    l, load            load a ball (LED yellow while loading)
    f, fire [power]    launch a ball; asks for the launcher power (0-1) if not
                       given (LED red while firing)
    p, party           toggle LED party mode
    s, stop            abort a running load / launch
    q, quit            exit

The LED is off otherwise. Like demo.py this owns the hardware directly, so stop
hardware_node before running (same I2C bus). Run it with `ros2 run`, not from a
launch file: launched nodes don't get the terminal's stdin.
"""

import math
import threading

import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node

from loone.hardware.hardware_controller import HardwareController

HELP = __doc__.split('\n\n')[1]

# (red, green, blue) steps the LED cycles through in party mode, one per LED tick.
PARTY_COLORS = ((True, False, False), (True, True, False), (False, True, False),
                (False, True, True), (False, False, True), (True, False, True),
                (True, True, True), (False, False, False))


class TerminalNode(Node):
    """Run ball launcher / LED commands typed into the terminal."""

    def __init__(self) -> None:
        super().__init__('terminal_node')

        self.declare_parameter('freq', 50)               # PCA9685 PWM frequency (Hz)
        self.declare_parameter('pca_clock', 25760154)    # PCA9685 measured oscillator (keep in sync with config.yaml)
        self.declare_parameter('load_time', 5.0)         # Seconds the wheel runs backwards to draw a ball in
        self.declare_parameter('load_power', 0.1)        # Launcher throttle while loading
        self.declare_parameter('launch_time', 5.0)       # Seconds the wheel keeps running after the carriage fires
        self.declare_parameter('launch_power', 0.1)      # Default launch power, used when Enter is pressed at the prompt
        self.declare_parameter('led_period', 0.15)       # LED update / party mode step (s)

        self.load_time = self.get_parameter('load_time').value
        self.load_power = self.get_parameter('load_power').value
        self.launch_time = self.get_parameter('launch_time').value
        self.launch_power = self.get_parameter('launch_power').value

        self.hardware = HardwareController(
            freq=self.get_parameter('freq').value,
            pca_clock=self.get_parameter('pca_clock').value,
            logger=self.get_logger())

        self.party = False
        self.party_step = 0
        # 'load' / 'launch' while a sequence runs, else None; picks the LED color.
        self.action = None

        # Ball launcher sequences block for seconds, so they run on their own thread
        # to keep the prompt (and 'stop') responsive.
        self.ball_thread = None

        self.led_timer = self.create_timer(self.get_parameter('led_period').value, self.update_led)
        self.get_logger().info('terminal_node ready.')

    def update_led(self) -> None:
        """Yellow while loading, red while firing, party colors if enabled, else off."""
        match self.action:
            case 'load':
                self.hardware.set_led(True, True)
            case 'launch':
                self.hardware.set_led(True, False)
            case _ if self.party:
                self.hardware.set_led(*PARTY_COLORS[self.party_step])
                self.party_step = (self.party_step + 1) % len(PARTY_COLORS)
            case _:
                self.hardware.set_led(False, False)

    # ------------------------------------------------------------------ ball launcher

    def busy(self) -> bool:
        """True while a load / launch sequence is running."""
        return self.ball_thread is not None and self.ball_thread.is_alive()

    def start(self, action: str, sequence, **kwargs) -> None:
        """Run a ball launcher sequence on its own thread, flagging action for the LED."""
        if self.busy():
            self.get_logger().warning("Ball launcher busy, ignoring command ('stop' to abort).")
            return
        self.action = action
        self.ball_thread = threading.Thread(
            target=self._run_sequence, args=(sequence,), kwargs=kwargs, daemon=True)
        self.ball_thread.start()

    def _run_sequence(self, sequence, **kwargs) -> None:
        try:
            sequence(**kwargs)
        finally:
            self.action = None

    def ask_power(self, text: str) -> float | None:
        """Parse a launch power in (0, 1], prompting for it if text is empty.

        An empty answer reuses the last power. Returns None if the input is invalid.
        """
        if not text:
            text = input(f'Launch power (0-1) [{self.launch_power}]: ').strip()
            if not text:
                return self.launch_power
        try:
            power = float(text)
        except ValueError:
            power = math.nan
        if not 0.0 < power <= 1.0:
            self.get_logger().error(f"Launch power must be in (0, 1], got '{text}'.")
            return None
        return power

    # ------------------------------------------------------------------ terminal

    def run(self) -> None:
        """Read and run commands from the terminal until 'quit'."""
        print(HELP)
        ball_launcher = self.hardware.ball_launcher
        while True:
            command, _, arg = input('> ').strip().lower().partition(' ')
            match command:
                case '':
                    continue

                case 'l' | 'load':
                    self.start('load', ball_launcher.load, time=self.load_time, power=self.load_power)

                case 'f' | 'fire':
                    if self.busy():
                        self.get_logger().warning("Ball launcher busy, ignoring command ('stop' to abort).")
                        continue
                    power = self.ask_power(arg.strip())
                    if power is None:
                        continue
                    self.launch_power = power
                    self.start('launch', ball_launcher.launch, time=self.launch_time, power=power)

                case 'p' | 'party':
                    self.party = not self.party
                    self.get_logger().info(f"Party mode {'on' if self.party else 'off'}.")

                case 's' | 'stop':
                    ball_launcher.cancel()

                case 'q' | 'quit':
                    return

                case _:
                    print(HELP)

    def shutdown(self) -> None:
        """Abort any running sequence, then return the actuators to neutral and release the hardware."""
        self.hardware.ball_launcher.cancel()
        if self.busy():
            self.ball_thread.join()
        self.hardware.shutdown()


def spin(executor) -> None:
    """Spin the executor until it, or the ROS context, is shut down."""
    try:
        executor.spin()
    except ExternalShutdownException:
        pass


def main(args=None) -> None:
    """Initialize the ROS2 node and read commands from the terminal."""
    rclpy.init(args=args)
    node = TerminalNode()
    # input() blocks the main thread, so the LED timer spins on its own thread.
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=spin, args=(executor,), daemon=True).start()
    try:
        node.run()
    except (KeyboardInterrupt, EOFError):
        node.get_logger().info('terminal_node interrupted by user.')
    except Exception as e:
        node.get_logger().error(f'terminal_node error: {e}')
    finally:
        # Stop the LED timer before the hardware is released underneath it.
        executor.shutdown()
        node.shutdown()
        node.destroy_node()
        # The SIGINT handler may have already shut down the context.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
