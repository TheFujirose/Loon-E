# loone

`loone` is [Humber ASV](https://humberasv.ca/)'s `ament_python` colcon package for
the Loon-E boat: `hardware_node` (propellers, ball launcher, status LED, battery
telemetry and RC state over the PCA9685 / INA3221), SLAM with the ZED X camera, and
the nav2 configuration. See the [repo root README](../../README.md) for build/setup
instructions common to the whole workspace.

## Constraints

The breakout board of the jetson connects to one PCA9685 meaning we use the [adafruit_motor library](https://docs.circuitpython.org/projects/motor/en/latest) for the motor interface. Unfortunately this also means that our scripts are limited to on I/O script as the library handles the busio at kernal level.

| Connection | Purpose |
| PCA9685 | Motor and Servo Control |
|  WCMCU-3221 / INA3221 | Voltage Monitor |
| Custom E-stop montior | Checks for E-stop |


A Blue Robotics Basic ESC handles the connection to the motors so we code them like continous servos
