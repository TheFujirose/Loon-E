import logging
import threading

from adafruit_motor import servo


class BallLauncherController:
    """Run the ball carriage and launcher wheel.

    This is a pure controller: it is not a ROS node and never touches I2C itself.
    HardwareController creates the ContinuousServo objects and hands them in.

    load() and launch() block for the length of their sequence (2 s / 4 s), so
    callers that must stay responsive should run them off the main thread. A
    cancel() from another thread aborts a running sequence immediately.
    """

    """
    Carriage Holds the balls for the launcher.
    NOTE: Not technically designed for automatic loading
    """
    carriage: servo.ContinuousServo

    """
    Motor that runs a wheel that launchers the balls
    """
    launcher: servo.ContinuousServo # Blue Robotics Basic ESC

    def __init__(self, carriage: servo.ContinuousServo, launcher: servo.ContinuousServo, logger=None):
        self.carriage = carriage
        self.launcher = launcher
        # Accepts an rclpy logger (node.get_logger()) or falls back to stdlib logging.
        self.logger = logger if logger is not None else logging.getLogger(__name__)
        self._cancel = threading.Event()

    def stop(self) -> None:
        self.carriage.throttle = 0.0
        self.launcher.throttle = 0.0

    def cancel(self) -> None:
        """Abort a running load() / launch() and stop both motors."""
        self._cancel.set()
        self.stop()

    def _wait(self, seconds) -> bool:
        """Sleep for a sequence step; return False if the sequence was cancelled."""
        return not self._cancel.wait(seconds)

    def load(self) -> None:
        """Run the launcher and carriage in sequence to load a ball."""
        self.logger.info('Loading ball.')
        self._cancel.clear()
        try:
            self.launcher.throttle = -0.5
            if not self._wait(1):
                return
            self.carriage.throttle = 0.5
            self._wait(1)
        finally:
            self.stop()

    def launch(self) -> None:
        """Run the launcher and carriage to fire a loaded ball."""
        self.logger.info('Launching ball.')
        self._cancel.clear()
        try:
            self.launcher.throttle = 0.5
            self.carriage.throttle = -0.5
            self._wait(4)
        finally:
            self.stop()
