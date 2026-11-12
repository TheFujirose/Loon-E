import math
from types import SimpleNamespace
from unittest.mock import MagicMock, PropertyMock, patch

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


def _ball(command, power=0.0):
    return SimpleNamespace(data=[float(command), float(power)])


def _odom(yaw_degrees):
    half = math.radians(yaw_degrees) / 2.0
    orientation = SimpleNamespace(x=0.0, y=0.0, z=math.sin(half), w=math.cos(half))
    return SimpleNamespace(pose=SimpleNamespace(pose=SimpleNamespace(orientation=orientation)))


# ── Construction tests ─────────────────────────────────────────────────────────

class TestInit:

    def test_builds_hardware_controller_from_parameters(self, controller_mock, node):
        controller_mock.assert_called_once()
        kwargs = controller_mock.call_args.kwargs

        assert kwargs['freq'] == 50
        assert kwargs['prop_min'] == 1120 and kwargs['prop_max'] == 1880
        assert kwargs['carriage_min'] == 500 and kwargs['carriage_max'] == 2500
        assert kwargs['launcher_min'] == 1100 and kwargs['launcher_max'] == 1900
        assert kwargs['propellor_kwargs'] == {
            'factor': 0.5, 'speed': 0.2, 'diff': 0.4,
            'kp': 2.0, 'ki': 0.5, 'kd': 0.02, 'max_output': 45.0,
        }
        assert kwargs['battery_kwargs'] == {
            'dw_min': 15.0, 'dw_max': 21.0, 'br_min': 13.2, 'br_max': 16.8,
        }
        assert node.hardware is controller_mock.return_value

    def test_heading_unknown_until_odometry(self, node):
        assert math.isnan(node.current_heading)


# ── Heading tests ───────────────────────────────────────────────────────────────

class TestOdomCallback:
    """odom_callback() turns ENU yaw into a clockwise-from-+y heading."""

    @pytest.mark.parametrize('yaw, heading', [
        (90.0, 0.0),     # facing +y
        (0.0, 90.0),     # facing +x
        (-90.0, 180.0),  # facing -y
        (180.0, 270.0),  # facing -x
    ])
    def test_converts_yaw_to_heading(self, node, yaw, heading):
        node.odom_callback(_odom(yaw))

        # Near 0 the result may land just under 360.
        assert (node.current_heading + 1e-6) % 360.0 == pytest.approx(heading, abs=1e-3)


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

    def test_drive_uses_odometry_heading(self, node):
        node.odom_callback(_odom(180.0))

        node.task_callback(_task(3, target_heading=90))

        node.hardware.propellors.drive.assert_called_once()
        current, target = node.hardware.propellors.drive.call_args.args
        assert current == pytest.approx(270.0)
        assert target == 90.0
        node.motor_pub.publish.assert_called_once()

    def test_drive_stops_without_heading(self, node):
        # No odometry yet: driving on a NaN heading would be meaningless.
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

    def test_timer_publishes_without_any_command(self, node):
        # The basestation reads a silent 'motor' topic as lost telemetry.
        assert node.publish_motor in [timer.callback for timer in node.timers]

    def test_stays_up_when_channel_is_off(self, node):
        node.hardware.prop_l.fraction = None

        node.publish_motor()

        node.motor_pub.publish.assert_not_called()


# ── ball_launcher_command tests ────────────────────────────────────────────────

class TestBallLauncherCallback:

    def test_stop_cancels(self, node):
        node.ball_launcher_callback(_ball(0))
        node.hardware.ball_launcher.cancel.assert_called_once()

    @pytest.mark.parametrize('command, sequence, time, power', [
        (1, 'load', 5.0, 0.1),
        (2, 'launch', 5.0, 0.1),
    ])
    def test_runs_sequence_on_a_thread_with_defaults(self, node, command, sequence, time, power):
        node.ball_launcher_callback(_ball(command))
        node.ball_thread.join(timeout=1.0)

        getattr(node.hardware.ball_launcher, sequence).assert_called_once_with(
            time=time, power=power)

    def test_command_can_choose_the_power(self, node):
        node.ball_launcher_callback(_ball(2, power=0.5))
        node.ball_thread.join(timeout=1.0)

        node.hardware.ball_launcher.launch.assert_called_once_with(time=5.0, power=0.5)

    def test_rejects_power_above_full_throttle(self, node):
        node.ball_launcher_callback(_ball(2, power=1.5))

        assert node.ball_thread is None

    def test_ignores_empty_command(self, node):
        node.ball_launcher_callback(SimpleNamespace(data=[]))

        assert node.ball_thread is None

    def test_ignores_command_while_busy(self, node):
        node.ball_thread = MagicMock(is_alive=MagicMock(return_value=True))

        node.ball_launcher_callback(_ball(2))

        node.hardware.ball_launcher.launch.assert_not_called()

    def test_stop_works_while_busy(self, node):
        node.ball_thread = MagicMock(is_alive=MagicMock(return_value=True))

        node.ball_launcher_callback(_ball(0))

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

    @staticmethod
    def _published(node):
        return [pub.publish.call_args[0][0] for pub in (
            node.prop_l_battery_pub, node.prop_r_battery_pub, node.main_battery_pub)]

    def test_reports_absent_when_ina_failed_at_startup(self, node):
        # hardware.battery is None when the INA3221 failed to initialize.
        node.hardware.battery = None

        node.publish_battery()

        assert [msg.present for msg in self._published(node)] == [False, False, False]
        assert [msg.location for msg in self._published(node)] == ['prop_l', 'prop_r', 'main']

    def test_reports_absent_without_hardware(self, node):
        node.hardware = None

        node.publish_battery()

        assert [msg.present for msg in self._published(node)] == [False, False, False]

    def test_dropout_reports_absent_instead_of_raising(self, node):
        node.hardware.battery.states.side_effect = OSError('Remote I/O error')

        node.publish_battery()

        assert [msg.present for msg in self._published(node)] == [False, False, False]

    def test_dropout_is_logged_once(self, node):
        node.hardware.battery.states.side_effect = OSError('Remote I/O error')

        node.publish_battery()
        node.publish_battery()

        node.get_logger().error.assert_called_once()

    def test_recovers_after_dropout(self, node):
        node.hardware.battery.states.side_effect = OSError('Remote I/O error')
        node.publish_battery()

        readings = BatteryState(), BatteryState(), BatteryState()
        node.hardware.battery.states.side_effect = None
        node.hardware.battery.states.return_value = readings
        node.publish_battery()

        node.prop_l_battery_pub.publish.assert_called_with(readings[0])
        assert node.battery_read_failed is False


# ── I2C failure tests ───────────────────────────────────────────────────────────

class TestBusErrors:
    """A bus error must never escape a callback: that would stop rclpy.spin."""

    def test_led_keeps_last_state(self, node):
        node.hardware.auto = False
        node.hardware.estop = True
        node.hardware.set_led.side_effect = OSError('Remote I/O error')

        node.set_led()

        assert (node.led.r, node.led.g) == (0.0, 0.0)
        node.get_logger().error.assert_called_once()

    def test_motor_state_read(self, node):
        type(node.hardware.prop_l).fraction = PropertyMock(side_effect=OSError('Remote I/O error'))

        node.publish_motor()

        node.motor_pub.publish.assert_not_called()
        node.get_logger().error.assert_called_once()

    def test_motor_command(self, node):
        node.hardware.propellors.forward.side_effect = OSError('Remote I/O error')

        node.task_callback(_task(1))

        node.motor_pub.publish.assert_not_called()
        node.get_logger().error.assert_called_once()

    def test_ball_launcher_stop(self, node):
        node.hardware.ball_launcher.cancel.side_effect = OSError('Remote I/O error')

        node.ball_launcher_callback(_ball(0))

        node.get_logger().error.assert_called_once()


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
