# latch

ROS 2 PWM driver package for the Hitec D954SW R/C servo motor. The node writes to the
Linux PWM sysfs interface and exposes topics for angle, pulse width, and enable control.

## Topics

- `angle_cmd` (`std_msgs/Float64`): desired angle in degrees.
- `pulse_us_cmd` (`std_msgs/Float64`): raw pulse width in microseconds.
- `enable_cmd` (`std_msgs/Bool`): enable/disable PWM output.

## Parameters

- `pwm_chip` (int, default: 0)
- `pwm_channel` (int, default: 0)
- `period_us` (int, default: 20000)
- `min_pulse_us` (int, default: 1000)
- `max_pulse_us` (int, default: 2000)
- `min_angle_deg` (float, default: 0.0)
- `max_angle_deg` (float, default: 180.0)
- `neutral_angle_deg` (float, default: 90.0)
- `auto_enable` (bool, default: True)
- `startup_angle_deg` (float, default: 90.0)

## Example

```bash
ros2 run latch latch_node
# Set 45 degrees
ros2 topic pub /angle_cmd std_msgs/msg/Float64 "{data: 45.0}" --once
```
