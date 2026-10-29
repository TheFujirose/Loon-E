from typing import Protocol

from adafruit_ina3221 import INA3221
from sensor_msgs.msg import BatteryState


class INA3221Channel(Protocol):
    """INA3221 channel measurements used to calculate battery-side voltage."""

    @property
    def bus_voltage(self) -> float:
        """Voltage at the load side of the shunt, in volts."""

    @property
    def shunt_voltage(self) -> float:
        """Voltage drop across the shunt, in millivolts."""


def battery_voltage(channel: INA3221Channel) -> float:
    """Return battery-side voltage when the battery is wired to VIN+."""
    return channel.bus_voltage + channel.shunt_voltage / 1000.0


class BatteryController:
    """Read the three batteries on the INA3221 and convert them to BatteryState.

    Ported from the old battery_node.py / motor.py. This is a pure controller: it is not a
    ROS node and never touches I2C itself. The caller (via HardwareController)
    creates the INA3221 on the bus and hands it in, and is responsible for
    publishing the messages.

    INA3221 channels:
        * 0 prop_l (Dewalt, left propulsion)
        * 1 prop_r (Dewalt, right propulsion)
        * 2 main   (Blue Robotics, primary battery)
    """

    ina: INA3221

    def __init__(
        self,
        ina: INA3221,
        dw_min: float = 15.0,   # Dewalt (prop) battery min voltage
        dw_max: float = 21.0,   # Dewalt (prop) battery max voltage
        br_min: float = 13.2,   # Blue Robotics (main) battery min voltage
        br_max: float = 16.8,   # Blue Robotics (main) battery max voltage
    ) -> None:
        self.ina = ina
        self.dw_min = dw_min
        self.dw_max = dw_max
        self.br_min = br_min
        self.br_max = br_max

    def voltages(self) -> tuple[float, float, float]:
        """Return battery-side voltages [dwL, dwR, br] in volts."""
        return (
            battery_voltage(self.ina[0]),
            battery_voltage(self.ina[1]),
            battery_voltage(self.ina[2]),
        )

    @staticmethod
    def to_battery_state(
            voltage: float, min_v: float, max_v: float, location: str, stamp=None) -> BatteryState:
        """Build a BatteryState from a raw voltage and the battery's nominal range.

        Args:
            voltage: Battery-side voltage in volts.
            min_v: Voltage that maps to 0% (below it the battery is reported dead).
            max_v: Voltage that maps to 100% (above it the battery is reported overvoltage).
            location: BatteryState.location label.
            stamp: Optional header stamp (e.g. node.get_clock().now().to_msg()).
        """
        msg = BatteryState()
        if stamp is not None:
            msg.header.stamp = stamp
        msg.voltage = float(voltage)
        msg.percentage = max(0.0, min(1.0, (voltage - min_v) / (max_v - min_v)))
        msg.present = True
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_UNKNOWN
        if voltage < min_v:
            msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_DEAD
        elif voltage > max_v:
            msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_OVERVOLTAGE
        else:
            msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        msg.location = location
        return msg

    def states(self, stamp=None) -> tuple[BatteryState, BatteryState, BatteryState]:
        """Read the INA3221 and return [prop_l, prop_r, main] BatteryState messages."""
        dw_l, dw_r, br = self.voltages()
        return (
            self.to_battery_state(dw_l, self.dw_min, self.dw_max, 'prop_l', stamp),
            self.to_battery_state(dw_r, self.dw_min, self.dw_max, 'prop_r', stamp),
            self.to_battery_state(br, self.br_min, self.br_max, 'main', stamp),
        )
