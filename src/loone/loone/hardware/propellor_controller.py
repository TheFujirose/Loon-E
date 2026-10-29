import logging
import time

from adafruit_motor import servo


class PropellorController:
    """Drive the left/right propeller ESCs for differential-thrust steering.

    Ported from motor.py, propellers only. This is a pure controller: it is not a
    ROS node and never touches I2C / the PCA9685 itself. The caller (motor.py, via
    HardwareController) creates the ContinuousServo objects on the PCA9685 channels
    (with the prop_min/prop_max pulse widths) and hands them in, and is responsible
    for publishing state and releasing the bus.

    Commands are ContinuousServo throttles in [-1, 1]:
        * -1.0 full reverse, 0.0 neutral, 1.0 full forward
    """

    prop_l: servo.ContinuousServo # Blue Robotics Basic ESC
    prop_r: servo.ContinuousServo

    def __init__(
        self,
        prop_l: servo.ContinuousServo,
        prop_r: servo.ContinuousServo,
        factor: float = 0.5,    # Cruise throttle while driving
        speed: float = 0.2,     # Throttle for plain forward / reverse
        diff: float = 0.4,      # Throttle offset from neutral when turning in place
        kp: float = 2.0,
        ki: float = 0.5,
        kd: float = 0.02,
        max_output: float = 45.0,
        logger=None,
    ) -> None:
        self.prop_l = prop_l
        self.prop_r = prop_r
        self.factor = factor
        self.speed = speed
        self.diff = diff
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max = max_output
        # Accepts an rclpy logger (node.get_logger()) or falls back to stdlib logging.
        self.logger = logger if logger is not None else logging.getLogger(__name__)

        self.reset()

    @property
    def throttles(self) -> tuple[float, float]:
        """Current [left, right] propeller throttles, for the caller to publish."""
        return (self.prop_l.throttle, self.prop_r.throttle)

    def reset(self) -> None:
        """Clear the PID state (call before (re)entering drive mode)."""
        self.i = 0
        self.last_error = 0
        self.last_time = time.time()

    @staticmethod
    def convert(angle) -> float:
        """Convert a heading angle from [0, 360] to [-180, 180].

        Args:
            angle: Heading in degrees

        Returns:
            Converted angle
        """
        if angle > 180:
            angle -= 360

        return angle

    def remap(self, error, outMin=0.1, outMax=1.0) -> float:
        """Map a heading error to a proportional throttle scale.

        Larger errors produce a smaller scale (stronger correction).

        Args:
            error: Heading error in degrees.
            outMin: Scale corresponding to maximum correction.
            outMax: Scale corresponding to minimum correction.

        Returns:
            Throttle scale in [outMin, outMax].
        """
        output = outMax + (min(abs(error), self.max) / self.max * (outMin - outMax))
        return output

    def reverse(self) -> None:
        """Set throttle to move backwards"""
        self.prop_l.throttle = -self.speed
        self.prop_r.throttle = -self.speed

    def stop(self) -> None:
        """Set propeller throttle to neutral/no motion"""
        self.prop_l.throttle = 0.0
        self.prop_r.throttle = 0.0

    def forward(self) -> None:
        """Set throttle to move forward"""
        self.prop_l.throttle = self.speed
        self.prop_r.throttle = self.speed

    def turn_in_place(self, direction) -> None:
        """Set throttle to turn in place.

        Args:
            direction: 1 turns left, anything else turns right.
        """
        if direction == 1: #turn left
            self.prop_l.throttle = -self.diff # backward
            self.prop_r.throttle = self.diff  # forward
        else: #turn right
            self.prop_l.throttle = self.diff  # forward
            self.prop_r.throttle = -self.diff # backward

    def drive(self, current_heading, target_heading) -> None:
        """Run one PID control cycle and update the propeller throttles.

        Args:
            current_heading: Measured heading in degrees [0, 360].
            target_heading: Desired heading in degrees [-180, 180].
        """
        current_time = time.time()
        current_error = self.convert(self.convert(current_heading) - target_heading)
        dt = current_time - self.last_time
        # Two commands arriving inside the same clock tick gave dt = 0 and a
        # ZeroDivisionError in the middle of the control loop.
        if dt <= 0.0:
            return
        de = (current_error - self.last_error) / dt
        self.logger.debug(f"Heading error: {current_error}")
        if abs(current_error) <= 45:
            self.i = self.i + self.ki * current_error
            if self.i < -self.max:
                self.i = -self.max
            elif self.i > self.max:
                self.i = self.max

            output = self.kp * current_error + self.i * dt + self.kd * de
            if output < -self.max:
                output = -self.max
            elif output > self.max:
                output = self.max

            # Scale the slower side's throttle down between factor (no error)
            # and factor * outMin (max error); it never drops into reverse.
            reduced = self.remap(output) * self.factor

            if current_error > 0:  # turn right: reduce right propeller
                self.prop_l.throttle = self.factor
                self.prop_r.throttle = reduced
                self.logger.debug(f"Sending Right: {reduced}")
            else:  # turn left: reduce left propeller
                self.prop_l.throttle = reduced
                self.prop_r.throttle = self.factor
                self.logger.debug(f"Sending Left: {reduced}")

            self.last_error = current_error
            self.last_time = current_time

        else:
            self.turn_in_place(1 if current_error > 0 else -1)
            self.logger.debug("Turning in place")
