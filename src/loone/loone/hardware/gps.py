"""GPS sensor controller: a shell for future development.

TODO: The GPS sensor hasn't been chosen, so nothing here is implemented or wired in.
HardwareController doesn't import this yet. Likely shape, none of it decided:

    * The sensor sits on the bus HardwareController owns, because only one object can
      hold the busio bus. So, like BatteryController, this would be a pure controller
      that is handed its device and never touches I2C itself.
    * hardware_node would publish its fixes as sensor_msgs/NavSatFix on /navsatfix with
      header.frame_id = 'gps_link', which navsat_transform and the ZED's GNSS fusion
      are already configured to read.
    * The basestation's latitude / longitude still come from the removed `phone` topic
      and would need pointing at the new source.

See wiki/TODO.md for the rest of what replacing the phone GPS involves.
"""


class GpsController:
    """Read position fixes from the GPS sensor. Not implemented yet."""

    def __init__(self, logger=None) -> None:
        self.logger = logger

    def read(self) -> tuple[float, float]:
        """Return the current (latitude, longitude) in degrees."""
        raise NotImplementedError('GPS sensor not chosen yet; see wiki/TODO.md.')
