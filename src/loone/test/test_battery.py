from types import SimpleNamespace

import pytest
from sensor_msgs.msg import BatteryState
from loone.hardware.battery import BatteryController, battery_voltage


# ── to_battery_state() tests ────────────────────────────────────────────────────

class TestToBatteryState:
    """to_battery_state() converts a raw voltage into percentage + health."""

    def test_mid_range_voltage_reports_good_health(self):
        msg = BatteryController.to_battery_state(18.0, min_v=15.0, max_v=21.0, location='prop_l')

        assert 0.0 < msg.percentage < 1.0
        assert msg.voltage == 18.0
        assert msg.present is True
        assert msg.location == 'prop_l'
        assert msg.power_supply_health == BatteryState.POWER_SUPPLY_HEALTH_GOOD

    def test_below_min_voltage_clamps_percentage_and_reports_dead(self):
        msg = BatteryController.to_battery_state(10.0, min_v=15.0, max_v=21.0, location='prop_l')

        assert msg.percentage == 0.0
        assert msg.power_supply_health == BatteryState.POWER_SUPPLY_HEALTH_DEAD

    def test_above_max_voltage_clamps_percentage_and_reports_overvoltage(self):
        msg = BatteryController.to_battery_state(25.0, min_v=15.0, max_v=21.0, location='prop_l')

        assert msg.percentage == 1.0
        assert msg.power_supply_health == BatteryState.POWER_SUPPLY_HEALTH_OVERVOLTAGE


def test_battery_voltage_includes_shunt_drop():
    class Channel:
        bus_voltage = 15.0
        shunt_voltage = 125.0

    assert battery_voltage(Channel()) == pytest.approx(15.125)


# ── states() tests ──────────────────────────────────────────────────────────────

class TestStates:
    """states() reads INA3221 channels 0/1/2 into [prop_l, prop_r, main] BatteryStates."""

    def test_reads_each_channel_into_its_battery(self):
        # INA3221 is indexable by channel; bus volts + shunt mV
        ina = [
            SimpleNamespace(bus_voltage=17.0, shunt_voltage=0.0),  # dw_l
            SimpleNamespace(bus_voltage=18.0, shunt_voltage=0.0),  # dw_r
            SimpleNamespace(bus_voltage=14.5, shunt_voltage=0.0),  # br
        ]

        prop_l_msg, prop_r_msg, main_msg = BatteryController(ina).states()

        assert prop_l_msg.voltage == 17.0 and prop_l_msg.location == 'prop_l'
        assert prop_r_msg.voltage == 18.0 and prop_r_msg.location == 'prop_r'
        assert main_msg.voltage == 14.5 and main_msg.location == 'main'

    def test_uses_dewalt_and_blue_robotics_ranges(self):
        ina = [SimpleNamespace(bus_voltage=16.0, shunt_voltage=0.0)] * 3

        prop_l_msg, _, main_msg = BatteryController(
            ina, dw_min=15.0, dw_max=21.0, br_min=13.2, br_max=16.8).states()

        # 16 V is low for a Dewalt pack but nearly full for the Blue Robotics one
        assert prop_l_msg.percentage == pytest.approx(1 / 6)
        assert main_msg.percentage == pytest.approx(2.8 / 3.6)
