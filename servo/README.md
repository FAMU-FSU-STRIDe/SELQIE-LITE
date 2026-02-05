# servo (ROS 2)

Python package providing `servo_motor_node` for CubeMars servo-mode control over CAN (socketcan).

## Build
Inside your workspace:
```bash
cp -r servo/ <your_ws>/src/
cd <your_ws>
colcon build --packages-select servo
source install/setup.bash
```

## Run
```bash
ros2 launch servo servo_motor.launch.py can_interface:=can0 can_id:=1 joint_name:=joint1
```

## Connections chart

| Servo package element | Interface | Connects to |
| --- | --- | --- |
| `servo_motor_node` | ROS topics (`/motorX/servo_cmd`) | Command source from UI/autonomy nodes |
| `servo_motor_node` | SocketCAN (`can0`) | CubeMars motor controller CAN bus |
| CubeMars motor controller | Phase outputs | BLDC actuator |
| Host + motor network | Ground | Shared reference for CAN integrity |

## Electrical schematic

```text
ROS command publisher -> /motorX/servo_cmd -> servo_motor_node -> can0
                                                        |
                                                        v
                                              CAN transceiver/bus
                                                        |
                                                        v
                                                CubeMars driver -> motor

Power rail (battery/BEC) ------------------------------> CubeMars driver
Common GND --------------------------------------------> Host + driver
```
