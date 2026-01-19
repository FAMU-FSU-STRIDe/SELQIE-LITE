# SELQIE-LITE

SELQIE-LITE is a ROS 2 Humble workspace that brings together motor control, sensor
bring-up, and operator tooling for the SELQIE-LITE robot running on NVIDIA
Jetson hardware. The workspace centers on CubeMars servo-mode CAN control, a
terminal UI, and supporting peripherals such as leak detection, battery
telemetry, latch actuation, and LED feedback.

## Repository layout

| Path | Type | Purpose |
| ---- | ---- | ------- |
| `battery/` | ROS 2 Python package | TinyBMS UART voltage telemetry publisher (`/tinybms/pack_voltage`). |
| `latch/` | ROS 2 Python package | Teensy-based latch controller over USB serial + reed switch reporting. |
| `leak_sensor/` | ROS 2 Python package | GPIO leak sensor monitor for Jetson GPIO. |
| `motor_interfaces/` | ROS 2 interface package | Shared `MotorState` message for CubeMars drivers. |
| `reed_switch/` | ROS 2 Python package | GPIO reed switch monitor for Jetson GPIO. |
| `selqie_bringup/` | ROS 2 package | Launch files that bundle core hardware nodes and sensing. |
| `selqie_ui/` | ROS 2 Python package | Interactive command terminal for servo-mode control. |
| `sensing/bno08x-ros2-driver/` | ROS 2 C++ package | BNO08x IMU driver (I2C). |
| `sensing/ms5837_bar_ros/` | ROS 2 Python package | Bar30/Bar02 pressure and depth sensor nodes. |
| `sensing/sensing_bringup/` | ROS 2 package | Launch files for IMU and Bar30 bring-up. |
| `servo/` | ROS 2 Python package | CubeMars servo-mode CAN driver node. |
| `ws2812b_ros/` | ROS 2 Python package | WS2812B LED SPI driver, CLI, and demos. |
| `zed-ros2-wrapper/` | Vendor package | ZED camera driver (upstream copy). |
| `zed-ros2-examples/` | Vendor package | ZED example applications. |
| `tmux/` | Scripts | tmux helper scripts for multi-pane workflows. |
| `python/` | Utilities | Non-colcon helper scripts (ignored via `COLCON_IGNORE`). |

## Prerequisites

* Ubuntu 22.04 with ROS 2 Humble installed.
* `python3-colcon-common-extensions`, `python3-rosdep`, `python3-can`,
  `python3-serial`, `tmux`.
* Jetson-specific dependencies when using GPIO/SPI: `python3-jetson-gpio`,
  `python3-spidev`.
* A configured SocketCAN interface (usually `can0`).

Install missing dependencies after cloning:

```bash
rosdep install --from-paths src -i -y
```

## Build

```bash
mkdir -p ~/selqie_ws/src
cd ~/selqie_ws/src
git clone https://github.com/<your-user>/SELQIE-LITE.git
cd ~/selqie_ws
rosdep install --from-paths src -i -y
colcon build --symlink-install
source install/setup.bash
```

## Bring-up

### Core hardware stack

The default hardware launch bundles the servo driver, latch, leak sensor,
TinyBMS voltage monitor, and WS2812B LEDs:

```bash
ros2 launch selqie_bringup selqie_hw.launch.py
```

Adjust the launch files to match your serial ports, CAN IDs, and GPIO wiring.

### Sensing stack

Bring up the IMU (and optionally the Bar30 pressure sensor) with:

```bash
ros2 launch selqie_bringup sensing.launch.py
```

The Bar30 launch inclusion is currently commented out; edit the file or use the
`sensing_bringup` launch files directly if you need the depth sensor.

## Operator tooling

The interactive terminal lives in `selqie_ui` and targets the servo-mode
control topics. To start it after the workspace is sourced:

```bash
ros2 run selqie_ui selqie_terminal
```

The `tmux/` directory includes helper scripts for spinning up multi-pane
sessions when you want to watch logs and send commands simultaneously.

## Notes

* Servo-mode control topics follow the `/motorX/servo_cmd` and
  `/motorX/special_cmd` naming pattern, with feedback on
  `/motorX/motor_state` and `/motorX/error_code`.
* The vendor ZED and BNO08x directories are included for convenience and can be
  updated from their upstream repositories as needed.

## License

Each package carries its own license. See the individual package directories
for details.
