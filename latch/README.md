# latch

ROS 2 serial driver package for the Hitec D954SW latch controller. The node uses
pyserial to send angle commands over USB to a Teensy.

## Topics

- `latch_angle_cmd` (`std_msgs/Float64`): target angle in degrees.

## Parameters

- `port` (string, default: `/dev/ttyACM0`)
- `baud` (int, default: 115200)
- `timeout_s` (float, default: 0.2)

## Example

```bash
ros2 run latch latch_node
# Open latch (0 degrees)
ros2 topic pub /latch_angle_cmd std_msgs/msg/Float64 "{data: 0.0}" --once
# Close latch (180 degrees)
ros2 topic pub /latch_angle_cmd std_msgs/msg/Float64 "{data: 180.0}" --once
```

## Connections chart

| Component | Interface | Connects to |
| --- | --- | --- |
| ROS 2 latch node | Topic `/latch_angle_cmd` | Receives commanded angle |
| Jetson host | USB serial (`/dev/ttyACM0`) | Teensy latch controller |
| Teensy latch controller | PWM/servo output | Hitec D954SW latch actuator |
| Jetson + Teensy + servo supply | Ground | Common reference |

## Electrical schematic

```text
ROS 2 topic /latch_angle_cmd
           |
           v
     latch_node (Python)
           |
     USB serial (/dev/ttyACM0)
           |
           v
   Teensy latch controller ---- PWM ----> Hitec D954SW
           |                                   |
           +------------ common GND -----------+
```
