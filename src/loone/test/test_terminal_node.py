import time
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

import pytest

from loone.terminal_node import TerminalNode


# terminal_node only touches ROS, so its publishers are the only thing faked here.
@pytest.fixture
def node():
    node = TerminalNode()
    node.motor_pub = MagicMock()
    node.ball_pub = MagicMock()
    node.get_logger = MagicMock(return_value=MagicMock())
    node.quit_flush = 0.0

    yield node

    node.destroy_node()


def _rc(node, auto=0, estop=1):
    node.rc_callback(SimpleNamespace(data=[auto, estop]))


def _sent(pub):
    return [list(call.args[0].data) for call in pub.publish.call_args_list]


# ── Command relay tests ─────────────────────────────────────────────────────────

class TestCommands:

    @pytest.fixture(autouse=True)
    def _safe(self, node):
        _rc(node)

    @pytest.mark.parametrize('line, expected', [
        ('w', [1.0, 0.0, 0.0]),
        ('forward', [1.0, 0.0, 0.0]),
        ('x', [-1.0, 0.0, 0.0]),
        ('a', [2.0, 0.0, 1.0]),
        ('d', [2.0, 0.0, -1.0]),
    ])
    def test_motor_commands(self, node, line, expected):
        node.run_line(line)

        assert _sent(node.motor_pub) == [expected]

    def test_load_uses_the_hardware_default_power(self, node):
        node.run_line('l')

        assert _sent(node.ball_pub) == [[1.0, 0.0]]

    def test_fire_with_power(self, node):
        node.run_line('f 0.5')

        assert _sent(node.ball_pub) == [[2.0, 0.5]]
        assert node.launch_power == 0.5

    def test_fire_prompts_for_power(self, node):
        with patch('builtins.input', return_value='0.25'):
            node.run_line('fire')

        assert _sent(node.ball_pub) == [[2.0, 0.25]]

    def test_fire_prompt_reuses_last_power(self, node):
        with patch('builtins.input', return_value=''):
            node.run_line('f')

        assert _sent(node.ball_pub) == [[2.0, pytest.approx(0.1)]]

    @pytest.mark.parametrize('power', ['0', '1.5', 'lots'])
    def test_fire_rejects_bad_power(self, node, power):
        node.run_line(f'f {power}')

        node.ball_pub.publish.assert_not_called()

    def test_stop_stops_propellers_and_launcher(self, node):
        node.run_line('s')

        assert _sent(node.motor_pub) == [[0.0, 0.0, 0.0]]
        assert _sent(node.ball_pub) == [[0.0, 0.0]]

    def test_quit_returns_false(self, node):
        assert node.run_line('q') is False

    def test_other_commands_keep_going(self, node, capsys):
        assert node.run_line('') is True
        assert node.run_line('nonsense') is True
        node.motor_pub.publish.assert_not_called()


# ── Heading tests ───────────────────────────────────────────────────────────────

class TestHeading:

    @pytest.fixture(autouse=True)
    def _safe(self, node):
        _rc(node)

    @pytest.mark.parametrize('text, target', [
        ('90', 90.0),
        ('-90', -90.0),
        ('270', -90.0),   # drive() takes [-180, 180]
    ])
    def test_sends_drive_command(self, node, text, target):
        node.run_line(f'h {text}')

        assert _sent(node.motor_pub) == [[3.0, target, 0.0]]

    @pytest.mark.parametrize('text', ['', 'north', 'nan', 'inf'])
    def test_rejects_bad_heading(self, node, text):
        node.run_line(f'h {text}')

        node.motor_pub.publish.assert_not_called()

    def test_update_repeats_held_heading(self, node):
        # hardware_node runs one PID cycle per message.
        node.run_line('h 90')

        node.update()
        node.update()

        assert _sent(node.motor_pub) == [[3.0, 90.0, 0.0]] * 3

    def test_another_command_ends_held_heading(self, node):
        node.run_line('h 90')
        node.run_line('w')
        node.motor_pub.reset_mock()

        node.update()

        node.motor_pub.publish.assert_not_called()

    def test_stop_ends_held_heading(self, node):
        node.run_line('h 90')
        node.run_line('s')
        node.motor_pub.reset_mock()

        node.update()

        node.motor_pub.publish.assert_not_called()


# ── Fail closed tests ───────────────────────────────────────────────────────────

class TestFailClosed:

    REFUSED = ['w', 'x', 'a', 'd', 'h 90', 'l', 'f 0.5']

    @pytest.mark.parametrize('line', REFUSED)
    def test_refuses_before_any_rc_state(self, node, line):
        node.run_line(line)

        node.motor_pub.publish.assert_not_called()
        node.ball_pub.publish.assert_not_called()

    @pytest.mark.parametrize('line', REFUSED)
    def test_refuses_while_estop_engaged(self, node, line):
        _rc(node, estop=0)

        node.run_line(line)

        node.motor_pub.publish.assert_not_called()
        node.ball_pub.publish.assert_not_called()

    @pytest.mark.parametrize('line', REFUSED)
    def test_refuses_when_rc_state_is_stale(self, node, line):
        _rc(node)
        node.rc_stamp = time.monotonic() - node.rc_timeout - 1.0

        node.run_line(line)

        node.motor_pub.publish.assert_not_called()
        node.ball_pub.publish.assert_not_called()

    def test_stop_is_always_allowed(self, node):
        _rc(node, estop=0)

        node.run_line('s')

        assert _sent(node.motor_pub) == [[0.0, 0.0, 0.0]]

    def test_estop_mid_run_sends_stop(self, node):
        _rc(node)
        node.run_line('w')
        node.motor_pub.reset_mock()

        _rc(node, estop=0)
        node.update()

        assert _sent(node.motor_pub) == [[0.0, 0.0, 0.0]]
        assert _sent(node.ball_pub) == [[0.0, 0.0]]

    def test_rc_state_going_stale_mid_run_sends_stop(self, node):
        _rc(node)
        node.run_line('f 0.5')
        node.ball_pub.reset_mock()

        node.rc_stamp = time.monotonic() - node.rc_timeout - 1.0
        node.update()

        assert _sent(node.motor_pub) == [[0.0, 0.0, 0.0]]
        assert _sent(node.ball_pub) == [[0.0, 0.0]]

    def test_fault_ends_held_heading_for_good(self, node):
        _rc(node)
        node.run_line('h 90')
        _rc(node, estop=0)
        node.update()
        node.motor_pub.reset_mock()

        # Clearing the e-stop must not start the boat moving again by itself.
        _rc(node)
        node.update()

        node.motor_pub.publish.assert_not_called()

    def test_stop_is_sent_once(self, node):
        _rc(node)
        node.run_line('w')
        node.motor_pub.reset_mock()

        _rc(node, estop=0)
        node.update()
        node.update()

        assert _sent(node.motor_pub) == [[0.0, 0.0, 0.0]]

    def test_no_stop_when_nothing_is_running(self, node):
        node.update()

        node.motor_pub.publish.assert_not_called()


# ── Shutdown tests ──────────────────────────────────────────────────────────────

def test_shutdown_stops_everything(node):
    node.shutdown()

    assert _sent(node.motor_pub) == [[0.0, 0.0, 0.0]]
    assert _sent(node.ball_pub) == [[0.0, 0.0]]
