# SELQIE-LITE

SELQIE-LITE is a ROS 2 Humble workspace that combines everything needed to run
a small quadruped prototype on NVIDIA Jetson hardware: high-level gait control,
low-level CubeMars motor drivers, a tmux-based operator UI, LED feedback, and
3D perception stacks for ZED cameras and BNO080 IMUs. Clone the repository into
`<workspace>/src`, fetch the optional vendor submodules, and build it with
`colcon` like any other ROS 2 overlay. 

> **Status.** The repository mixes actively maintained Python packages with
> vendor trees that are tracked as Git submodules (`bno08x-ros2-driver`,
> `zed-ros2-examples`, `zed-ros2-wrapper`). If those directories are empty after
> cloning, run `git submodule update --init --recursive` to populate them.

## Repository layout

| Path | Type | Purpose |
| ---- | ---- | ------- |
| `quad_legs/` | Python ROS 2 package | Quadruped control stack (CubeMars CAN driver, joystick loop, and PyQt5 UI). |
| `motor_interfaces/` | Interface package | Custom `MotorState` message exported for all other nodes. |
| `selqie_ui/` | Python ROS 2 package | Interactive console for controlling the quadruped stack and monitoring motor state. |
| `ws2812b_ros/` | Python ROS 2 package | SPI-based driver, CLI, and demo launch for WS2812B (NeoPixel) LED strips on Jetson boards. |
| `bno08x-ros2-driver/` | Submodule | Upstream 9-DOF IMU driver from SparkFun (used for state estimation). |
| `zed-ros2-wrapper/` | Submodule | Stereolabs ZED ROS 2 camera wrapper. |
| `zed-ros2-examples/` | Submodule | Sample applications that exercise the ZED wrapper. |

## Prerequisites

* Ubuntu 22.04 with ROS 2 Humble (desktop or ros-base install).
* `python3-colcon-common-extensions`, `python3-rosdep`, `python3-pyqt5`,
  `python3-can`, `tmux`, and Jetson-specific packages such as `python3-spidev`
  if you plan to drive LEDs.
* A CAN interface configured as `can0` (e.g., `sudo ip link set can0 up type can bitrate 1000000`).
* Optional hardware/software: CubeMars AK-series actuators on CAN, Logitech-style
  joystick publishing `sensor_msgs/Joy`, WS2812B LED chains wired to SPI, ZED
  stereo cameras, and a BNO080 IMU.

Use `rosdep install --from-paths src -i -y` inside your workspace to pull any
remaining dependencies once ROS 2 is set up.

## Building the workspace

```bash
mkdir -p ~/selqie_ws/src
cd ~/selqie_ws/src
git clone https://github.com/<your-user>/SELQIE-LITE.git
cd SELQIE-LITE
git submodule update --init --recursive  # populate vendor drivers
cd ~/selqie_ws
rosdep install --from-paths src -i -y
colcon build --symlink-install
source install/setup.bash
```

## Core packages

### `motor_interfaces`

Defines the `motor_interfaces/msg/MotorState` message that reports the pose,
velocity, torque, current, and temperature for a single CubeMars actuator.
Import this interface wherever you need typed feedback from the low-level driver.

### `quad_legs`

The heart of the robot. It contains three major nodes:

1. **`motor_node`** – Handles CubeMars CAN communication. Parameters cover CAN
   interface selection, CAN ID, actuator type (AK10-9, AK80-6, etc.), control
   frequency, joint naming, and startup behavior. Each node publishes
   `MotorState`, error codes, and temperature readings while subscribing to both
   MIT-style command arrays (`/motorX/mit_cmd`) and text-based special commands
   (`/motorX/special_cmd`).
2. **`main_control`** – Listens to `sensor_msgs/Joy` and mirrors the left stick
   into symmetric velocity commands for all four legs. It also exposes simple A/B
   button behavior for reinitializing or stopping the actuators and automatically
   zeroes encoders during the first interaction.
3. **`quad_leg_ui`** – A PyQt5 GUI that lets you dial in a per-leg rotation
   frequency (including phase offset between leg pairs), pause/resume motion, and
   issue `start`, `zero`, or `exit` commands. Under the hood it publishes MIT
   velocity-only commands at 100 Hz using a non-uniform speed profile (fast when
   the legs are under the body, slow elsewhere).

Launch everything together with `ros2 launch quad_legs quad_full.launch.py`. The
launch file spins four `motor_node` instances and the PyQt UI. Uncomment the
`main_control` block if you want joystick control in the same session.

### `selqie_ui`

A ROS 2 entry point that opens an interactive SELQIE motor console. It publishes
MIT commands to each `/motorX/mit_cmd` topic, handles special commands like
`start`, `zero`, and `exit`, and prints recent `MotorState` feedback. Run it
via:

```bash
ros2 run selqie_ui selqie_terminal
```

### `ws2812b_ros`

Offers a Jetson-friendly SPI driver for WS2812B/NeoPixel LEDs:

* `led_node` subscribes to `/led_colors` (`std_msgs/UInt32MultiArray`) and pushes
  24-bit colors across MOSI at ~2.4 MHz.
* `led_tester` publishes a simple color cycle for quick bring-up.
* `ws2812b-set` is a CLI helper that publishes one-shot color updates (hex,
  white, per-index, or all-off) without writing custom ROS 2 code.
* `led_demo.launch.py` starts the driver and tester with parameter overrides for
  LED count, brightness, SPI bus/device, clock rate, pixel order, and timing.

Follow the dedicated hardware instructions in
`ws2812b_ros/README_ws2812b_ros.md` to enable SPI in Jetson-IO, add the proper
5 V level shifting, and install `python3-spidev` before running the driver.

## Supporting drivers

* **`bno08x-ros2-driver`** – SparkFun’s ROS 2 port of the BNO080 9-DOF IMU. Keep
  it updated if you rely on orientation data for closed-loop control.
* **`zed-ros2-wrapper`** and **`zed-ros2-examples`** – Official Stereolabs
  packages for streaming depth and tracking data from ZED cameras.

Because these three folders are submodules, their code lives in separate
repositories. Update them independently (`git submodule update --remote`) to
pick up upstream fixes.

## Development tips

* Use `tmux` or the provided tmux UI so you can watch motor logs and run
  debugging commands side by side.
* Keep ROS_DOMAIN_IDs consistent across machines before launching distributed
  control stacks.
* `colcon test` is available, but only lint/ament tests from each package will
  run; hardware drivers expect the real actuators and CAN bus.
* When iterating on LED or motor parameters, prefer ROS 2 parameters (`ros2 run
  ... --ros-args -p key:=value`) instead of editing launch files so that your
  workspace stays reproducible.

## License

Each package keeps its own license (MIT for `quad_legs` and `ws2812b_ros`,
Apache-2.0 for `motor_interfaces`, vendor licenses for the submodules). See the
respective subdirectories for details.
