from contextlib import ExitStack
from unittest.mock import MagicMock, call, patch

import pytest


MODULE = 'loone.hardware.hardwareController'


# ── Shared fixture ─────────────────────────────────────────────────────────────
#
# HardwareController is not a ROS node, so no rclpy context is involved here.
# Only the hardware libraries (busio/board/digitalio/PCA9685/INA3221/servo) are
# faked; the propellor / ball launcher / battery controllers it builds are real.
#
def _patch_hardware(stack, pca_side_effect=None, ina_side_effect=None):
    """Patch the hardware libraries and return the mocks by name."""
    mocks = {name: stack.enter_context(patch(f'{MODULE}.{name}'))
             for name in ('busio', 'board', 'digitalio', 'servo')}
    mocks['PCA9685'] = stack.enter_context(patch(f'{MODULE}.PCA9685', side_effect=pca_side_effect))
    mocks['INA3221'] = stack.enter_context(patch(f'{MODULE}.INA3221', side_effect=ina_side_effect))

    # PCA9685(i2c, address=...) must return an object with a .channels list.
    pca_instance = MagicMock()
    pca_instance.channels = [MagicMock() for _ in range(16)]
    mocks['PCA9685'].return_value = pca_instance

    # ContinuousServo(channel, ...) is called once per channel (prop_l, prop_r,
    # carriage, launcher). A bare MagicMock call returns the same `return_value`
    # no matter the arguments, so without this side_effect all four servo
    # attributes would alias one fake object.
    mocks['servo'].ContinuousServo.side_effect = lambda *args, **kwargs: MagicMock()

    # Each DigitalInOut(pin) is a separate line (auto / e-stop).
    mocks['digitalio'].DigitalInOut.side_effect = lambda *args, **kwargs: MagicMock()
    return mocks


@pytest.fixture
def hardware_mocks():
    with ExitStack() as stack:
        yield _patch_hardware(stack)


@pytest.fixture
def hardware(hardware_mocks):
    from loone.hardware.hardwareController import HardwareController

    return HardwareController(logger=MagicMock())


# ── Validation tests ───────────────────────────────────────────────────────────

class TestInitValidation:
    """Tests for frequency and pulse-range validation during initialisation."""

    def test_freq_too_low_raises(self, hardware_mocks):
        from loone.hardware.hardwareController import HardwareController

        # freq=10 is below the minimum of 24 Hz -> should raise ValueError
        with pytest.raises(ValueError):
            HardwareController(freq=10, logger=MagicMock())

    def test_freq_too_high_raises(self, hardware_mocks):
        from loone.hardware.hardwareController import HardwareController

        with pytest.raises(ValueError):
            HardwareController(freq=9999, logger=MagicMock())

    def test_validate_pulse_range_min_equals_max_raises(self, hardware):
        with pytest.raises(ValueError):
            hardware._validate_pulse_range(1500, 1500, 'test_channel')

    def test_validate_pulse_range_min_greater_than_max_raises(self, hardware):
        with pytest.raises(ValueError):
            hardware._validate_pulse_range(1800, 1200, 'test_channel')

    def test_validate_pulse_range_below_hardware_min_raises(self, hardware):
        with pytest.raises(ValueError):
            hardware._validate_pulse_range(400, 1500, 'test_channel')

    def test_validate_pulse_range_above_hardware_max_raises(self, hardware):
        with pytest.raises(ValueError):
            hardware._validate_pulse_range(1000, 2600, 'test_channel')

    def test_validate_pulse_range_valid_does_not_raise(self, hardware):
        hardware._validate_pulse_range(1120, 1880, 'prop')

    def test_invalid_prop_pulse_range_raises(self, hardware_mocks):
        from loone.hardware.hardwareController import HardwareController

        with pytest.raises(ValueError):
            HardwareController(prop_min=1500, prop_max=1500, logger=MagicMock())

    def test_invalid_launcher_pulse_range_raises(self, hardware_mocks):
        from loone.hardware.hardwareController import HardwareController

        with pytest.raises(ValueError):
            HardwareController(launcher_min=1000, launcher_max=2600, logger=MagicMock())

    def test_ina_failure_is_non_fatal(self):
        # A wiring fault on the INA3221 (battery monitor) must not prevent the
        # PCA9685 (propeller driver) from being set up.
        with ExitStack() as stack:
            mocks = _patch_hardware(
                stack, ina_side_effect=ValueError('No I2C device at address: 0x41'))
            from loone.hardware.hardwareController import HardwareController

            hardware = HardwareController(logger=MagicMock())

            assert hardware.pca is mocks['PCA9685'].return_value
            assert hardware.ina is None
            assert hardware.battery is None
            assert hardware.propellors is not None

    def test_pca_failure_still_raises(self):
        # Unlike the INA3221, the PCA9685 is required to drive the motors at all,
        # so its failure must still be fatal.
        with ExitStack() as stack:
            _patch_hardware(stack, pca_side_effect=ValueError('No I2C device at address: 0x40'))
            from loone.hardware.hardwareController import HardwareController

            with pytest.raises(ValueError):
                HardwareController(logger=MagicMock())


# ── Construction tests ─────────────────────────────────────────────────────────

class TestConstruction:
    """The controller wires each PCA9685 channel to the right actuator."""

    def test_sets_pca_frequency(self, hardware_mocks):
        from loone.hardware.hardwareController import HardwareController

        hardware = HardwareController(freq=60, logger=MagicMock())

        assert hardware.pca.frequency == 60

    def test_servo_channels_and_pulse_widths(self, hardware_mocks, hardware):
        channels = hardware.pca.channels
        hardware_mocks['servo'].ContinuousServo.assert_has_calls([
            call(channels[0], min_pulse=1120, max_pulse=1880),  # prop_l
            call(channels[1], min_pulse=1120, max_pulse=1880),  # prop_r
            call(channels[2], min_pulse=1220, max_pulse=1820),  # carriage
            call(channels[3], min_pulse=1120, max_pulse=1880),  # launcher
        ])
        assert hardware.red is channels[4]
        assert hardware.green is channels[5]

    def test_controllers_are_built_on_the_servos(self, hardware):
        assert hardware.propellors.prop_l is hardware.prop_l
        assert hardware.propellors.prop_r is hardware.prop_r
        assert hardware.ball_launcher.carriage is hardware.carriage
        assert hardware.ball_launcher.launcher is hardware.launcher
        assert hardware.battery.ina is hardware.ina

    def test_passes_controller_kwargs_through(self, hardware_mocks):
        from loone.hardware.hardwareController import HardwareController

        hardware = HardwareController(
            propellor_kwargs={'factor': 0.3, 'diff': 0.1},
            battery_kwargs={'dw_min': 14.0, 'br_max': 17.0},
            logger=MagicMock())

        assert hardware.propellors.factor == 0.3
        assert hardware.propellors.diff == 0.1
        assert hardware.battery.dw_min == 14.0
        assert hardware.battery.br_max == 17.0

    def test_starts_with_everything_neutral(self, hardware):
        assert hardware.prop_l.throttle == 0.0
        assert hardware.prop_r.throttle == 0.0
        assert hardware.carriage.throttle == 0.0
        assert hardware.launcher.throttle == 0.0
        assert hardware.red.duty_cycle == 0x0000
        assert hardware.green.duty_cycle == 0x0000


# ── IO tests ───────────────────────────────────────────────────────────────────

class TestIO:

    def test_auto_and_estop_read_the_gpio_lines(self, hardware):
        hardware.auto_pin.value = True
        hardware.estop_pin.value = False

        assert hardware.auto is True
        assert hardware.estop is False

    @pytest.mark.parametrize('red, green', [(True, False), (False, True), (True, True), (False, False)])
    def test_set_led_drives_duty_cycles(self, hardware, red, green):
        hardware.set_led(red, green)

        assert hardware.red.duty_cycle == (0xFFFF if red else 0x0000)
        assert hardware.green.duty_cycle == (0xFFFF if green else 0x0000)


# ── Shutdown tests ──────────────────────────────────────────────────────────────

class TestShutdown:

    def test_neutralizes_then_releases_hardware(self, hardware):
        hardware.prop_l.throttle = 0.5
        hardware.launcher.throttle = 0.5
        hardware.set_led(True, True)

        hardware.shutdown()

        assert hardware.prop_l.throttle == 0.0
        assert hardware.launcher.throttle == 0.0
        assert hardware.red.duty_cycle == 0x0000
        hardware.pca.deinit.assert_called_once()
        hardware.auto_pin.deinit.assert_called_once()
        hardware.estop_pin.deinit.assert_called_once()

    def test_releases_hardware_even_if_stop_raises(self, hardware):
        # shutdown()'s try/finally must still release the I2C bus on failure.
        hardware.stop = MagicMock(side_effect=RuntimeError('boom'))

        hardware.shutdown()

        hardware.logger.error.assert_called_once()
        hardware.pca.deinit.assert_called_once()
