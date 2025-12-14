# SELQIE tmux UI

This ROS 2 Humble Python package provides an interactive console for the
SELQIE quadruped. It publishes MIT commands directly to each motor topic,
forwards special commands such as `start`, `exit`, `zero`, and `clear`, and
prints the latest `MotorState` feedback so you can monitor the actuators in
real time.

## Prerequisites

Install tmux (the UI backend) if it is not already available:

```bash
sudo apt update && sudo apt install tmux
# or via rosdep
rosdep install selqie_ui
```

## Usage

```bash
# Launch the console as a ROS node
ros2 run selqie_ui selqie_terminal
```

Once the console is running you can:

* `start_motors`, `stop_motors`, `zero`, or `clear` (optionally targeting a
  single motor or `all`).
* Send MIT commands with `set_cmd <motor|all> <p> <v> <kp> <kd> <torque>`.
* Send quick velocity commands with `set_vel <motor|all> <vel> [kp] [kd]
  [torque]`.
* Inspect feedback with `status` (latest MotorState) and `errors` (last error
  string per motor).
The tool exits cleanly with `exit`.
