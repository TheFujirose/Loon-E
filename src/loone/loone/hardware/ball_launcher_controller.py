import logging
import threading

from adafruit_motor import servo


class BallLauncherController:
    """Run the ball carriage and launcher wheel.

    This is a pure controller: it is not a ROS node and never touches I2C itself.
    HardwareController creates the Servo / ContinuousServo objects and hands them in.

    load(), cycle_carriage() and launch() block for the length of their sequence,
    so callers that must stay responsive should run them off the main thread. A
    cancel() from another thread aborts a running sequence immediately.
    """

    """
    Carriage Holds the balls for the launcher. Positional servo: parked at
    LOAD_ANGLE to load, pushed counter-clockwise to the end of its travel to feed a
    ball to the wheel.
    NOTE: Not technically designed for automatic loading
    """
    carriage: servo.Servo

    """
    Motor that runs a wheel that launchers the balls
    """
    launcher: servo.ContinuousServo # Blue Robotics Basic ESC

    LOAD_ANGLE = 0     # Carriage angle (degrees) that lines up a ball for loading
    SWEEP_STEP = 0.5    # Seconds allowed for the carriage to reach the end of its travel
    SPIN_UP = 1.0       # Seconds for the launcher wheel to reach speed before firing

    def __init__(self, carriage: servo.Servo, launcher: servo.ContinuousServo, logger=None):
        self.carriage = carriage
        self.launcher = launcher
        # Accepts an rclpy logger (node.get_logger()) or falls back to stdlib logging.
        self.logger = logger if logger is not None else logging.getLogger(__name__)
        self._cancel = threading.Event()

    def stop(self) -> None:
        """Stop the launcher wheel. The carriage servo holds its current angle."""
        self.launcher.throttle = 0.0

    def cancel(self) -> None:
        """Abort a running sequence and stop the launcher wheel."""
        self._cancel.set()
        self.stop()

    def _wait(self, seconds) -> bool:
        """Sleep for a sequence step; return False if the sequence was cancelled."""
        return not self._cancel.wait(seconds)

    def _sweep(self) -> bool:
        """Push the carriage counter-clockwise to the end of its travel and leave it there.

        The next load() brings it back to LOAD_ANGLE. Returns False if cancelled
        part way (the carriage stays where it got to).
        """
        # Increasing angle (longer pulse) is counter-clockwise on this servo.
        self.carriage.angle = self.carriage.actuation_range
        return self._wait(self.SWEEP_STEP)

    def load(self, time: float = 1.0, power: float = 0.3) -> None:
        """Park the carriage at LOAD_ANGLE and run the wheel backwards to draw a ball in."""
        self.logger.info('Loading ball.')
        self._cancel.clear()
        try:
            self._sweep()
            self.launcher.throttle = -power
            self._wait(time)
        finally:
            self.stop()

    def cycle_carriage(self) -> None:
        """Push the carriage counter-clockwise to the end of its travel (no launcher)."""
        self.logger.info('Cycling carriage.')
        self._cancel.clear()
        self._sweep()

    def launch(self, time: float = 4.0, power: float = 0.3) -> None:
        """Spin up the launcher wheel, then cycle the carriage to fire a loaded ball.

        time is how long the wheel keeps running after the carriage sweep.
        """
        self.logger.info('Launching ball.')
        self._cancel.clear()
        try:
            self.launcher.throttle = power
            if not self._wait(self.SPIN_UP) or not self._sweep():
                return
            self.carriage.angle = self.LOAD_ANGLE
            self._wait(time)
        finally:
            self.stop()
