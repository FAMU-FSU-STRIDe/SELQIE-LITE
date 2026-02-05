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
* Send MIT commands with `set_cmd <motor|all> <p> <v> <kp> <kd> <torque>`. Motors
  auto-start into MIT mode on the first command, so you can send a velocity even
  if you forgot to call `start_motors`.
* Send quick velocity commands with `set_vel <motor|all> <vel> [kp] [kd]
  [torque]`.
* Run a Beuhler clock velocity pattern with `beuhler <freq_hz> [offset_deg]
  [fast_band_deg] [kp] [kd] [max_vel_abs] [control_hz]`. Re-running the command
  while the pattern is active updates parameters in real time without stopping;
  use `beuhler stop` to halt it.
* Inspect feedback with `status` (latest MotorState) and `errors` (last error
  string per motor).
The tool exits cleanly with `exit`.

## Connections chart

| UI endpoint | ROS topic/interface | Downstream node |
| --- | --- | --- |
| `set_cmd` / `set_vel` | `/motorX/mit_cmd` (`std_msgs/Float32MultiArray`) | `quad_legs/motor_node` |
| `start_motors` / `stop_motors` / `zero` | `/motorX/special_cmd` (`std_msgs/String`) | `quad_legs/motor_node` |
| `status` / `errors` view | `/motorX/motor_state` + `/motorX/error` | Feedback from motor nodes |

## Electrical schematic (control-path)

```text
Operator keyboard
      |
      v
selqie_terminal (tmux UI)
      |
      +--> /motorX/mit_cmd ---------> motor_node --> CAN transceiver --> CubeMars actuator
      +--> /motorX/special_cmd -----> motor_node --> CAN transceiver --> CubeMars actuator
      +<-- /motorX/motor_state ------ motor_node <-- CAN telemetry   <-- CubeMars actuator
```
