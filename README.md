# Loon-E
[![Loone Tests](https://github.com/HumberASV/Loon-E/actions/workflows/tests.yml/badge.svg)](https://github.com/HumberASV/Loon-E/actions/workflows/tests.yml)

[Humber ASV](https://humberasv.ca/)'s Autonomous Surface Vehicle (ASV) project: Loon-E.
The boat's public colcon package, containing the code for the boat's various systems, including control, mapping, planning, and communication.

# Technology

| Technology | Version | Purpose |
|------------|---------|---------|
| Ubuntu Jammy | 22.04 | Operating system for development and deployment. |
| ROS 2 Humble Hawksbill | 2.0 | Robotics middleware for communication between different components of the system. |
| ZED SDK | 5.2 | Software Development Kit for the ZED stereo camera, used for vision processing. |

### Prerequisites

- Install ROS 2 following [installation](https://docs.ros.org/en/humble/Installation.html) page.
- Set up your environment following [instructions](./wiki/setup/ENV.md) document.
- Install colcon following [instructions](./wiki/setup/COLCON.md) document.

## Development

### Building Workspace

Ensure you have set up your environment following [instructions](./wiki/setup/ENV.md) document, and have installed `colcon` following [this instructions](./wiki/setup/COLCON.md).

For building the workspace with Windows see the below note: Building with Windows.

Regardless of Linux or macOs distrubution, navigate to the project root `Loon-E/` and run `colcon build`. It may be needed to use the option `--symlink-install` as some build types do not support `devel` spaces.

For more information on `catkin`'s `devel` space read [this documentation](https://catkin-tools.readthedocs.io/en/latest/advanced/linked_develspace.html).

```bash
colcon build --symlink-install
```

You can test it with the following command:

```bash
colcon test
```
### Project Structure

```text
Loon-E/
├── .github/workflows/           # CI (Loone Tests)
├── build/                       # Colcon intermediate files (not tracked)
├── install/                     # Colcon package installation (not tracked)
├── log/                         # Colcon logging information (not tracked)
├── src/                         # Source packages
│   ├── loone/                   # Main boat package (ament_python)
│   │   ├── config/              # Node parameters (config.yaml), nav2 / SLAM / navsat configs, waypoints
│   │   ├── launch/              # Launch files
│   │   ├── loone/               # Python nodes
│   │   │   ├── hardware/        # HardwareController + propellor, ball launcher and battery controllers
│   │   │   ├── hardware_node.py # ROS interface to the hardware
│   │   │   ├── demo.py          # One-shot hardware demo (hardware_test)
│   │   │   └── terminal_node.py # Ball launcher / LED control from terminal input
│   │   ├── model/               # ONNX model
│   │   ├── rviz/                # RViz config
│   │   └── test/                # pytest suite
│   ├── loone_interfaces/        # Custom messages (Phone, Task)
│   ├── loone_urdf/              # Robot description, package le1000_urdf_t4 (submodule)
│   └── zedx/                    # ZED camera ROS 2 wrapper (submodule)
├── tools/                       # Scripts for hardware management, calibration and diagnostics
├── wiki/                        # Additional documentation
│   ├── setup/                   # Setup documentation
│   ├── CONTRIBUTING.md          # Instructions on contributing to this repository
│   └── TODO.md                  # Outstanding work (e.g. replacing the phone GPS source)
├── CAMERA_NOT_WORKING.md        # Debugging the ZED X camera
├── DEBUG.md                     # Debugging the PWM hardware (ESCs, servos)
├── requirements.txt             # Python dependencies
├── LICENSE                      # MIT license
└── README.md                    # Primary documentation
```

## Contributing

See the [contribution guide](wiki/CONTRIBUTING.md) to see how you can contribute!

## fast build
colcon build --symlink-install --packages-skip zed_debug
