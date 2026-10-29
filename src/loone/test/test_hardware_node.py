import math
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

import pytest
from sensor_msgs.msg import BatteryState


# ── Shared fixture ─────────────────────────────────────────────────────────────
#
# hardware_node.py never touches the bus itself -- everything goes through
# HardwareController -- so that one class is the only thing faked here. The
# real session-scoped ROS context from conftest.py is enough for Node.__init__,
# create_subscription/create_publisher/create_timer/get_clock to work for real.
#
@pytest.fixture
def controller_mock():
    with patch('loone.hardware_node.HardwareController') as controller_mock:
        hardware = controller_mock.return_value
        # Real floats so they fit in Float32MultiArray.
        hardware.prop_l.fraction = 0.5
        hardware.prop_r.fraction = 0.5
        yield controller_mock


@pytest.fixture
def node(controller_mock):
    from loone.hardware_node import HardwareNode

    node = HardwareNode()

    # Patch the publishers so we can inspect what gets published in tests.
    node.motor_pub = MagicMock()
    node.led_pub = MagicMock()
    node.rc_state_pub = MagicMock()
    node.prop_l_battery_pub = MagicMock()
    node.prop_r_battery_pub = MagicMock()
    node.main_battery_pub = MagicMock()

    # Patch get_logger() so log calls inside methods don't crash.
    node.get_logger = MagicMock(return_value=MagicMock())

    yield node

    node.destroy_node()


def _task(command, target_heading=0.0, direction=0.0):
    return SimpleNamespace(data=[float(command), float(target_heading), float(direction)])


# ── Construction tests ─────────────────────────────────────────────────────────

class TestInit:

    def test_builds_hardware_controller_from_parameters(self, controller_mock, node):
        controller_mock.assert_called_once()
        kwargs = controller_mock.call_args.kwargs

        assert kwargs['freq'] == 50
        assert kwargs['prop_min'] == 1120 and kwargs['prop_max'] == 1880
        assert kwargs['carriage_min'] == 1220 and kwargs['carriage_max'] == 1820
        assert kwargs['launcher_min'] == 1120 and kwargs['launcher_max'] == 1880
        assert kwargs['propellor_kwargs'] == {
            'factor': 0.5, 'speed': 0.2, 'diff': 0.4,
            'kp': 2.0, 'ki': 0.5, 'kd': 0.02, 'max_output': 45.0,
        }
        assert kwargs['battery_kwargs'] == {
            'dw_min': 15.0, 'dw_max': 21.0, 'br_min': 13.2, 'br_max': 16.8,
        }
        assert node.hardware is controller_mock.return_value

    def test_heading_unknown_until_phone_data(self, node):
        assert math.isnan(node.current_heading)


# ── motor_command tests ────────────────────────────────────────────────────────

class TestTaskCallback:
    """task_callback() maps [command, target_heading, dir] onto the propellors."""

    def test_reverse(self, node):
        node.task_callback(_task(-1))
        node.hardware.propellors.reverse.assert_called_once()
        node.motor_pub.publish.assert_called_once()

    def test_stop(self, node):
        node.task_callback(_task(0))
        node.hardware.propellors.stop.assert_called_once()
        node.motor_pub.publish.assert_called_once()

    def test_forward(self, node):
        node.task_callback(_task(1))
        node.hardware.propellors.forward.assert_called_once()
        node.motor_pub.publish.assert_called_once()

    def test_turn_passes_direction(self, node):
        node.task_callback(_task(2, direction=1))
        node.hardware.propellors.turn_in_place.assert_called_once_with(1.0)

    def test_turn_ignored_without_direction(self, node):
        node.task_callback(_task(2, direction=-999))
        node.hardware.propellors.turn_in_place.assert_not_called()

    def test_drive_uses_phone_heading(self, node):
        node.phone_callback(SimpleNamespace(data=[0.0, 0.0, 1.5, 270.0]))

        node.task_callback(_task(3, target_heading=90))

        node.hardware.propellors.drive.assert_called_once_with(270.0, 90.0)
        node.motor_pub.publish.assert_called_once()

    def test_drive_stops_without_phone_heading(self, node):
        # No phone data yet: driving on a NaN heading would be meaningless.
        node.task_callback(_task(3, target_heading=90))

        node.hardware.propellors.drive.assert_not_called()
        node.hardware.propellors.stop.assert_called_once()

    def test_unknown_command_does_nothing(self, node):
        node.task_callback(_task(7))
        node.motor_pub.publish.assert_not_called()

    def test_no_hardware_does_nothing(self, node):
        node.hardware = None
        node.task_callback(_task(1))
        node.motor_pub.publish.assert_not_called()


class TestPublishMotor:

    def test_publishes_prop_fractions_and_rudder_placeholder(self, node):
        node.hardware.prop_l.fraction = 0.75
        node.hardware.prop_r.fraction = 0.25

        node.publish_motor()

        msg = node.motor_pub.publish.call_args[0][0]
        # Float32MultiArray.data is a float32 array.array, not a list.
        assert list(msg.data) == pytest.approx([0.75, 0.25, 0.5])


# ── ball_launcher_command tests ────────────────────────────────────────────────

class TestBallLauncherCallback:

    def test_stop_cancels(self, node):
        node.ball_launcher_callback(SimpleNamespace(data=0))
        node.hardware.ball_launcher.cancel.assert_called_once()

    @pytest.mark.parametrize('command, sequence', [(1, 'load'), (2, 'launch')])
    def test_runs_sequence_on_a_thread(self, node, command, sequence):
        node.ball_launcher_callback(SimpleNamespace(data=command))
        node.ball_thread.join(timeout=1.0)

        getattr(node.hardware.ball_launcher, sequence).assert_called_once()

    def test_ignores_command_while_busy(self, node):
        node.ball_thread = MagicMock(is_alive=MagicMock(return_value=True))

        node.ball_launcher_callback(SimpleNamespace(data=2))

        node.hardware.ball_launcher.launch.assert_not_called()

    def test_stop_works_while_busy(self, node):
        node.ball_thread = MagicMock(is_alive=MagicMock(return_value=True))

        node.ball_launcher_callback(SimpleNamespace(data=0))

        node.hardware.ball_launcher.cancel.assert_called_once()


# ── Battery telemetry tests ─────────────────────────────────────────────────────

class TestPublishBattery:

    def test_publishes_each_battery_state_on_its_topic(self, node):
        prop_l, prop_r, main = BatteryState(), BatteryState(), BatteryState()
        node.hardware.battery.states.return_value = (prop_l, prop_r, main)

        node.publish_battery()

        node.prop_l_battery_pub.publish.assert_called_once_with(prop_l)
        node.prop_r_battery_pub.publish.assert_called_once_with(prop_r)
        node.main_battery_pub.publish.assert_called_once_with(main)

    def test_skips_publishing_when_ina_unavailable(self, node):
        # hardware.battery is None when the INA3221 failed to initialize.
        node.hardware.battery = None

        node.publish_battery()

        node.prop_l_battery_pub.publish.assert_not_called()
        node.main_battery_pub.publish.assert_not_called()


# ── RC state / LED tests ────────────────────────────────────────────────────────

class TestRcState:

    def test_publishes_raw_auto_and_estop_levels(self, node):
        node.hardware.auto = True
        node.hardware.estop = False

        node.publish_rc_state()

        msg = node.rc_state_pub.publish.call_args[0][0]
        assert list(msg.data) == [1, 0]


class TestSetLed:
    """set_led() blinks: on (colour from RC state), then off, alternately."""

    @pytest.mark.parametrize('auto, estop, red, green', [
        (False, True, True, True),    # manual -> yellow
        (True, True, False, True),    # autonomous -> green
        (True, False, True, False),   # e-stop engaged -> red (wins over auto)
    ])
    def test_on_phase_colour(self, node, auto, estop, red, green):
        node.hardware.auto = auto
        node.hardware.estop = estop

        node.set_led()

        node.hardware.set_led.assert_called_once_with(red, green)
        assert (node.led.r, node.led.g) == (float(red), float(green))

    def test_off_phase_follows_on_phase(self, node):
        node.hardware.auto = False
        node.hardware.estop = True

        node.set_led()
        node.set_led()

        assert node.hardware.set_led.call_args.args == (False, False)
        assert (node.led.r, node.led.g) == (0.0, 0.0)


# ── Shutdown tests ──────────────────────────────────────────────────────────────

class TestShutdown:

    def test_shuts_down_hardware(self, node):
        node.shutdown()
        node.hardware.shutdown.assert_called_once()

    def test_no_hardware_is_fine(self, node):
        node.hardware = None
        node.shutdown()
