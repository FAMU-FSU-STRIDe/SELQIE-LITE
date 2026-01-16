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
