# SELQIE Terminal UI

`selqie_ui` provides an interactive ROS 2 command-line console for the
SELQIE-LITE servo-mode control stack. It publishes servo commands and special
commands to each motor, exposes gait helpers, and prints live motor state,
errors, latch status, and battery telemetry.

## Prerequisites

* ROS 2 Humble with this workspace built.
* The `servo` package running (or another node publishing the expected motor
  topics).
* Optional: the `latch`, `battery`, and `ws2812b_ros` packages if you want latch,
  voltage, or LED commands available.

## Running

```bash
ros2 run selqie_ui selqie_terminal
```

You can also launch the supporting hardware stack first:

```bash
ros2 launch selqie_bringup selqie_hw.launch.py
```

## Published topics

| Topic | Type | Purpose |
| ----- | ---- | ------- |
| `/motorX/servo_cmd` | `std_msgs/Float64MultiArray` | Servo command array for motor X (1–4). |
| `/motorX/special_cmd` | `std_msgs/String` | `start`, `zero`, and `clear` commands. |
| `/latch_angle_cmd` | `std_msgs/Float64` | Command latch angle in degrees. |
| `/led_colors` | `std_msgs/UInt32MultiArray` | Packed RGB LED colors (two LEDs by default). |

## Subscribed topics

| Topic | Type | Purpose |
| ----- | ---- | ------- |
| `/motorX/motor_state` | `motor_interfaces/MotorState` | Servo-mode state feedback. |
| `/motorX/current_state` | `motor_interfaces/MotorState` | Optional relative encoder state feedback. |
| `/motorX/error_code` | `std_msgs/String` | Last error string per motor. |
| `/tinybms/pack_voltage` | `std_msgs/Float32` | Battery pack voltage (optional). |

## Command reference

Available commands (type `help` in the console for the same list):

* `zero [motor_id|all]` – Idle and zero encoders.
* `clear [motor_id|all]` – Clear queued commands and hold zeros.
* `beuhler stop | <frequency_hz> [slow_band_deg] [alpha]` – Start/stop the
  Beuhler clock gait.
* `swim stop | <frequency_hz> [center_angle] [delta_angle]` – Start/stop the
  swim gait.
* `stand` – Hold all legs at 0° position.
* `idle [motor_id|all]` – Send idle mode (mode 7) to motors.
* `brake <current_a> [motor_id|all]` – Apply braking current (mode 2).
* `snap [motor_id|all]` – Snap to nearest 360° multiple and zero.
* `latch open|close` – Command latch angle (0° or 180°).
* `status` – Print the latest motor state messages.
* `errors` – Print last error strings.
* `battery` – Print the latest battery voltage (if available).
* `exit` – Quit the console and cleanly shut down ROS.

## Tips

* If motors are not responding, ensure the `servo` nodes are running and that
  your CAN interface is configured (e.g., `sudo ip link set can0 up type can bitrate 1000000`).
* The terminal uses motor IDs 1–4 by default; update the `servo_motor.launch.py`
  file if your CAN IDs differ.
