"""terminal_node: relay commands typed into a terminal to hardware_node.

Commands (typed at the '>' prompt):
    w, forward         drive both propellers forward
    x, reverse         drive both propellers in reverse
    a, left            turn left in place
    d, right           turn right in place
    h, heading <deg>   hold a heading (relative to where the ZED started, not north)
    l, load            load a ball
    f, fire [power]    launch a ball; asks for the launcher power (0-1) if not given
    s, stop            stop the propellers and abort a running load / launch
    q, quit            stop everything and exit

This stands in for the task logic until loone/task.py exists. It only publishes
commands, so it runs alongside hardware_node, which owns the hardware. Run it with
`ros2 run`, not from a launch file: launched nodes don't get the terminal's stdin.

Commands fail closed: anything that moves the boat or the launcher is refused while
the e-stop is engaged or rc_state has gone quiet, and if either happens mid-run this
node sends stop. A command otherwise holds until the next one, so this can't protect
against being killed outright; hardware_node has no timeout of its own.
"""

import math
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int8MultiArray

HELP = __doc__.split('\n\n')[1]

# hardware_node's motor_command values.
MOTOR_REVERSE = -1
MOTOR_STOP = 0
MOTOR_FORWARD = 1
MOTOR_TURN = 2
MOTOR_DRIVE = 3

# hardware_node's turn directions: 1 turns left, anything else turns right.
TURN_LEFT = 1
TURN_RIGHT = -1

# hardware_node's ball_launcher_command values.
BALL_STOP = 0
BALL_LOAD = 1
BALL_LAUNCH = 2


class TerminalNode(Node):
    """Turn commands typed into the terminal into hardware_node commands."""

    def __init__(self) -> None:
        super().__init__('terminal_node')

        self.declare_parameter('launch_power', 0.1)   # Used when Enter is pressed at the prompt
        self.declare_parameter('rc_timeout', 2.0)     # Seconds without rc_state before refusing
        self.declare_parameter('update_period', 0.1)  # Seconds between safety checks
        self.declare_parameter('quit_flush', 0.2)     # Seconds to let the final stop go out

        self.launch_power = self.get_parameter('launch_power').value
        self.rc_timeout = self.get_parameter('rc_timeout').value
        self.quit_flush = self.get_parameter('quit_flush').value

        # Shared between the terminal thread and the executor thread.
        self.lock = threading.Lock()
        self.estop_clear = False       # Last e-stop line level; low means engaged
        self.rc_stamp = None           # time.monotonic() of the last rc_state
        self.active = False            # A command we sent may still be running
        self.target_heading = None     # Heading being held, or None

        self.motor_pub = self.create_publisher(Float32MultiArray, 'motor_command', 10)
        self.ball_pub = self.create_publisher(Float32MultiArray, 'ball_launcher_command', 10)
        self.rc_sub = self.create_subscription(Int8MultiArray, 'rc_state', self.rc_callback, 10)
        self.update_timer = self.create_timer(
            self.get_parameter('update_period').value, self.update)
        self.get_logger().info('terminal_node ready.')

    # ------------------------------------------------------------------ safety

    def rc_callback(self, msg) -> None:
        """Record the e-stop line from hardware_node's rc_state ([auto, estop])."""
        with self.lock:
            self.estop_clear = bool(msg.data[1])
            self.rc_stamp = time.monotonic()

    def fault(self) -> str | None:
        """Return why commands aren't safe to run right now, or None if they are."""
        with self.lock:
            stamp = self.rc_stamp
            estop_clear = self.estop_clear
        if stamp is None or time.monotonic() - stamp > self.rc_timeout:
            return 'no recent rc_state from hardware_node'
        if not estop_clear:
            return 'e-stop is engaged'
        return None

    def update(self) -> None:
        """Stop if it has become unsafe; otherwise keep any held heading going.

        hardware_node runs one PID cycle per drive command, so holding a heading means
        sending them continuously.
        """
        fault = self.fault()
        with self.lock:
            active = self.active
            target_heading = self.target_heading
        if fault is not None:
            if active:
                self.get_logger().error(f'Stopping: {fault}.')
                self.send_stop()
        elif target_heading is not None:
            self.send_motor(MOTOR_DRIVE, target_heading)

    # ------------------------------------------------------------------ commands

    def send_motor(self, command: int, heading: float = 0.0, direction: float = 0.0) -> None:
        """Publish a motor_command."""
        msg = Float32MultiArray()
        msg.data = [float(command), float(heading), float(direction)]
        self.motor_pub.publish(msg)

    def send_ball(self, command: int, power: float = 0.0) -> None:
        """Publish a ball_launcher_command; power 0 uses hardware_node's default."""
        msg = Float32MultiArray()
        msg.data = [float(command), float(power)]
        self.ball_pub.publish(msg)

    def send_stop(self) -> None:
        """Stop the propellers and the ball launcher."""
        with self.lock:
            self.active = False
            self.target_heading = None
        self.send_motor(MOTOR_STOP)
        self.send_ball(BALL_STOP)

    def allowed(self) -> bool:
        """Report whether a command may run, logging why if it may not."""
        fault = self.fault()
        if fault is not None:
            self.get_logger().error(f'Command refused: {fault}.')
            return False
        return True

    def run_motor(self, command: int, direction: float = 0.0) -> None:
        """Run a propeller command, ending any held heading."""
        if not self.allowed():
            return
        with self.lock:
            self.active = True
            self.target_heading = None
        self.send_motor(command, direction=direction)

    def run_heading(self, text: str) -> None:
        """Start holding the heading in text, in degrees."""
        try:
            heading = float(text)
        except ValueError:
            heading = math.nan
        if not math.isfinite(heading):
            self.get_logger().error(f"Heading must be a number of degrees, got '{text}'.")
            return
        if not self.allowed():
            return
        # PropellorController.drive() takes targets in [-180, 180].
        heading = (heading + 180.0) % 360.0 - 180.0
        with self.lock:
            self.active = True
            self.target_heading = heading
        self.send_motor(MOTOR_DRIVE, heading)

    def run_ball(self, command: int, power: float = 0.0) -> None:
        """Run a ball launcher sequence."""
        if not self.allowed():
            return
        with self.lock:
            self.active = True
        self.send_ball(command, power)

    def ask_power(self, text: str) -> float | None:
        """Parse a launch power in (0, 1], prompting for it if text is empty.

        An empty answer reuses the last power. Returns None if the input is invalid.
        """
        if not text:
            text = input(f'Launch power (0-7) [{self.launch_power}]: ').strip()
            if not text:
                return self.launch_power
        try:
            power = float(text) / 10
        except ValueError:
            power = math.nan
        if not 0.0 < power <= 0.7: # limit to 0.1-0.7 range
            self.get_logger().error(f"Launch power must be in (0, 0.7], got '{text}'.")
            return None
        return power

    # ------------------------------------------------------------------ terminal

    def run_line(self, line: str) -> bool:
        """Run one line of terminal input. Returns False if it asks to quit."""
        command, _, arg = line.strip().lower().partition(' ')
        arg = arg.strip()
        match command:
            case '':
                pass

            case 'w' | 'forward':
                self.run_motor(MOTOR_FORWARD)

            case 'x' | 'reverse':
                self.run_motor(MOTOR_REVERSE)

            case 'a' | 'left':
                self.run_motor(MOTOR_TURN, TURN_LEFT)

            case 'd' | 'right':
                self.run_motor(MOTOR_TURN, TURN_RIGHT)

            case 'h' | 'heading':
                self.run_heading(arg)

            case 'l' | 'load':
                self.run_ball(BALL_LOAD)

            case 'f' | 'fire':
                power = self.ask_power(arg)
                if power is not None:
                    self.launch_power = power
                    self.run_ball(BALL_LAUNCH, power)

            case 's' | 'stop':
                self.send_stop()

            case 'q' | 'quit':
                return False

            case _:
                print(HELP)

        return True

    def run(self) -> None:
        """Read and run commands from the terminal until 'quit'."""
        print(HELP)
        while self.run_line(input('> ')):
            pass

    def shutdown(self) -> None:
        """Stop everything, leaving time for the messages to go out."""
        self.send_stop()
        time.sleep(self.quit_flush)


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
    # input() blocks the main thread, so the safety timer spins on its own thread.
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
        # Stop the timer first so that it can't restart a held heading after the stop.
        executor.shutdown()
        node.shutdown()
        node.destroy_node()
        # The SIGINT handler may have already shut down the context.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
