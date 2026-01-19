# servo (ROS 2)

`servo` provides the `servo_motor_node` ROS 2 driver for CubeMars servo-mode
control over SocketCAN. It exposes a unified `servo_cmd` topic for duty, current,
RPM, position, and combined position+speed commands, plus a `special_cmd` topic
for start/zero/clear events.

## Build

Inside your workspace:

```bash
cp -r servo/ <your_ws>/src/
cd <your_ws>
colcon build --packages-select servo
source install/setup.bash
```

## Run

### Single motor

```bash
ros2 run servo servo_motor_node --ros-args \
  -p can_interface:=can0 \
  -p can_id:=1 \
  -p joint_name:=motor1
```

### Four-motor launch

```bash
ros2 launch servo servo_motor.launch.py
```

The launch file starts four motor nodes (`motor1`–`motor4`) and flips polarity on
motors 2 and 4. Adjust CAN IDs or polarity in `servo_motor.launch.py` if your
hardware differs.

## Topics

| Topic | Type | Direction | Description |
| ----- | ---- | --------- | ----------- |
| `/<joint_name>/servo_cmd` | `std_msgs/Float64MultiArray` | Subscribe | Servo command array `[mode, v0, v1, v2]`. |
| `/<joint_name>/special_cmd` | `std_msgs/String` | Subscribe | `start`, `exit`, `zero`, `clear`. |
| `/<joint_name>/motor_state` | `motor_interfaces/MotorState` | Publish | Motor feedback state. |
| `/<joint_name>/error_code` | `std_msgs/String` | Publish | Human-readable error string. |

### Servo command modes

| Mode | Meaning | v0 | v1 | v2 |
| ---- | ------- | -- | -- | -- |
| `0` | Duty | duty (-1..1) | – | – |
| `1` | Current | current (A) | – | – |
| `2` | Current brake | brake current (A) | – | – |
| `3` | RPM | ERPM | – | – |
| `4` | Position | position (deg) | – | – |
| `5` | Set origin | origin mode (0/1/2) | – | – |
| `6` | Position + speed | position (deg) | ERPM | accel (ERPM/s) |
| `7` | Idle | – | – | – |

Example:

```bash
ros2 topic pub /motor1/servo_cmd std_msgs/msg/Float64MultiArray "{data: [4, 90, 0, 0]}" --once
```

## Parameters

| Parameter | Type | Default | Description |
| --------- | ---- | ------- | ----------- |
| `can_interface` | string | `can0` | SocketCAN interface name. |
| `can_id` | int | `1` | Motor CAN ID (0–255). |
| `motor_type` | string | `AK40-10` | Used for torque scaling. |
| `control_hz` | double | `50.0` | State publish/control loop frequency. |
| `joint_name` | string | `joint` | Namespace for topics. |
| `auto_start` | bool | `false` | Automatically power on motors. |
| `reverse_polarity` | bool | `false` | Flip polarity on the motor direction. |
