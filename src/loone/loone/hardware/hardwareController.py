import logging

import board
import busio
import digitalio
from adafruit_ina3221 import INA3221
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685

from loone.hardware.ball_launcher_controller import BallLauncherController
from loone.hardware.battery import BatteryController
from loone.hardware.propellor_controller import PropellorController

# Valid frequency range for PCA9685 (Hz)
PCA_FREQ_MIN = 24
PCA_FREQ_MAX = 1526

# Absolute pulse width limits enforced by hardware (microseconds)
PULSE_MIN_LIMIT = 500
PULSE_MAX_LIMIT = 2500


class HardwareController:
    """Own the I2C bus / PCA9685 / INA3221 / GPIO and the controllers built on them.

    This is the only place that talks to the hardware. It is not a ROS node: the
    hardware node creates one of these and does all the topic handling.

    PCA9685 (address 0x40):
        * ch0 prop_l, ch1 prop_r      -> propellors
        * ch2 carriage, ch3 launcher  -> ball_launcher
        * ch4 red, ch5 green, ch6 blue -> status LED
    INA3221 (address 0x41, optional)  -> battery
    GPIO D12 autonomy, D13 e-stop RC lines
    """

    pca: PCA9685
    ina: INA3221 | None
    propellors: PropellorController
    battery: BatteryController | None
    ball_launcher: BallLauncherController

    def __init__(
        self,
        freq: int = 50,             # PCA9685 PWM frequency (Hz)
        pca_clock: int = 25_000_000,  # PCA9685 oscillator (Hz); calibrate per board, see _init_busio
        prop_min: int = 1120,       # propeller ESC min pulse (us)
        prop_max: int = 1880,       # propeller ESC max pulse (us)
        carriage_min: int = 500,    # ball carriage servo min pulse (us), 0 degrees
        carriage_max: int = 2500,   # ball carriage servo max pulse (us), full travel CCW
        carriage_range: int = 180,  # ball carriage servo travel (degrees): 180 or 270 model
        launcher_min: int = 1100,   # ball launcher ESC min pulse (us), Basic ESC full reverse
        launcher_max: int = 1900,   # ball launcher ESC max pulse (us), Basic ESC full forward
        propellor_kwargs: dict | None = None,   # passed through to PropellorController
        battery_kwargs: dict | None = None,     # passed through to BatteryController
        logger=None,
    ) -> None:
        # Accepts an rclpy logger (node.get_logger()) or falls back to stdlib logging.
        self.logger = logger if logger is not None else logging.getLogger(__name__)

        self._init_busio(freq, pca_clock)
        self._init_servos(prop_min, prop_max, carriage_min, carriage_max, carriage_range,
                          launcher_min, launcher_max)
        self._init_gpio()

        self.propellors = PropellorController(
            self.prop_l, self.prop_r, logger=self.logger, **(propellor_kwargs or {}))
        self.ball_launcher = BallLauncherController(self.carriage, self.launcher, logger=self.logger)
        # The INA3221 is optional (see _init_busio), so battery telemetry is too.
        self.battery = (BatteryController(self.ina, **(battery_kwargs or {}))
                        if self.ina is not None else None)

        self.stop()

    # ------------------------------------------------------------------ setup

    def _init_busio(self, freq, pca_clock=25_000_000) -> None:
        """Initialize the PCA9685 PWM driver and INA3221 monitor over I2C.

        The PCA9685's internal oscillator is nominally 25 MHz but varies by a few
        percent per board, and every pulse width scales with that error. The ESC
        neutral deadband is only 1500 +/- 25 us, so pca_clock should be the
        measured value: 25e6 * (measured PWM Hz) / (configured freq).
        """
        if not (PCA_FREQ_MIN <= freq <= PCA_FREQ_MAX):
            self.logger.error(
                f"PCA9685 frequency {freq} Hz out of range "
                f"[{PCA_FREQ_MIN}, {PCA_FREQ_MAX}] Hz.")
            raise ValueError(f"Invalid PCA9685 frequency: {freq}")

        try:
            i2c = busio.I2C(board.SCL, board.SDA)
        except Exception as e:
            self.logger.error(f"Failed to initialize I2C bus (check SDA/SCL): {e}")
            raise

        try:
            self.pca = PCA9685(i2c, address = 64, reference_clock_speed = pca_clock)
        except Exception as e:
            self.logger.error(f"PCA9685 not found on I2C bus (check wiring/address 0x40): {e}")
            raise

        self.pca.frequency = freq
        self.logger.info(f"PCA9685 initialized at {freq} Hz (clock {pca_clock} Hz).")

        # INA3221 (battery voltage monitor) is a separate chip from the PCA9685 that
        # drives the propellers. A wiring fault on this sensor must not take the
        # motors down with it, so its failure is logged but not fatal here --
        # self.battery is None while self.ina is None.
        try:
            self.ina = INA3221(i2c, address = 65, enable = [0, 1, 2])
        except Exception as e:
            self.logger.error(
                f"INA3221 not found on I2C bus (check wiring/address 0x41): {e}. "
                "Battery telemetry disabled; motors are unaffected.")
            self.ina = None

    def _validate_pulse_range(self, min_pulse, max_pulse, channel_name) -> None:
        """Validate PWM pulse widths are ordered and within hardware limits"""
        if min_pulse >= max_pulse:
            raise ValueError(
                f"{channel_name}: min_pulse ({min_pulse}) must be < max_pulse ({max_pulse}).")
        if min_pulse < PULSE_MIN_LIMIT or max_pulse > PULSE_MAX_LIMIT:
            raise ValueError(
                f"{channel_name}: pulse range [{min_pulse}, {max_pulse}] us exceeds hardware "
                f"limits [{PULSE_MIN_LIMIT}, {PULSE_MAX_LIMIT}] us.")

    def _init_servos(self, prop_min, prop_max, carriage_min, carriage_max, carriage_range,
                     launcher_min, launcher_max) -> None:
        """Set up the servo / ESC and LED channels on the PCA9685"""
        self._validate_pulse_range(prop_min, prop_max, "prop_l (ch 0)")
        self._validate_pulse_range(prop_min, prop_max, "prop_r (ch 1)")
        self._validate_pulse_range(carriage_min, carriage_max, "carriage (ch 2)")
        self._validate_pulse_range(launcher_min, launcher_max, "launcher (ch 3)")

        try:
            # Blue Robotics Basic ESCs, so ContinuousServo (throttle -1..1);
            # the carriage is a positional Servo (angle).
            self.prop_l = servo.ContinuousServo(self.pca.channels[0], min_pulse=prop_min, max_pulse=prop_max)
            self.prop_r = servo.ContinuousServo(self.pca.channels[1], min_pulse=prop_min, max_pulse=prop_max)
            self.carriage = servo.Servo(self.pca.channels[2], actuation_range=carriage_range,
                                        min_pulse=carriage_min, max_pulse=carriage_max)
            self.launcher = servo.ContinuousServo(self.pca.channels[3], min_pulse=launcher_min, max_pulse=launcher_max)
            self.red = self.pca.channels[4]
            self.green = self.pca.channels[5]
            self.blue = self.pca.channels[6]
        except Exception as e:
            self.logger.error(f"Failed to initialize servo channels: {e}")
            raise

        self.logger.info(
            "Servo PWM channels initialized (ch0 prop_l, ch1 prop_r, ch2 carriage, "
            "ch3 launcher, ch4/5/6 LED).")

    def _init_gpio(self) -> None:
        """Set up the autonomy / e-stop RC input lines."""
        self.auto_pin = digitalio.DigitalInOut(board.D12)
        self.auto_pin.direction = digitalio.Direction.INPUT
        self.estop_pin = digitalio.DigitalInOut(board.D13)
        self.estop_pin.direction = digitalio.Direction.INPUT

    # ------------------------------------------------------------------ IO

    @property
    def auto(self) -> bool:
        """Raw level of the autonomy RC line (D12)."""
        return bool(self.auto_pin.value)

    @property
    def estop(self) -> bool:
        """Raw level of the e-stop RC line (D13); low means e-stop is engaged."""
        return bool(self.estop_pin.value)

    def set_led(self, red: bool, green: bool, blue: bool = False) -> None:
        """Turn the red / green / blue channels of the status LED fully on or off."""
        self.red.duty_cycle = 0xFFFF if red else 0x0000
        self.green.duty_cycle = 0xFFFF if green else 0x0000
        self.blue.duty_cycle = 0xFFFF if blue else 0x0000

    def stop(self) -> None:
        """Put every actuator in its neutral / off state."""
        self.propellors.stop()
        self.ball_launcher.cancel()
        self.set_led(False, False)

    def shutdown(self) -> None:
        """Return channels to neutral and release the PCA9685 and GPIO lines."""
        try:
            self.stop()
        except Exception as e:
            self.logger.error(f"Failed to neutralize actuators on shutdown: {e}")
        finally:
            self.pca.deinit()
            self.auto_pin.deinit()
            self.estop_pin.deinit()
