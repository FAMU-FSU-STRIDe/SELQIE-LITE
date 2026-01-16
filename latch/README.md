# latch

ROS 2 GPIO driver package for the Hitec D954SW latch servo. The node uses
Jetson.GPIO to drive a GPIO pin high or low based on open/close commands.

## Topics

- `latch_cmd` (`std_msgs/Bool`): `True` = open (pin HIGH), `False` = close (pin LOW).

## Parameters

- `gpio_pin` (int, default: 15)
- `gpio_mode` (string, default: `BOARD`)
- `active_high` (bool, default: True)

## Example

```bash
ros2 run latch latch_node
# Open latch (pin HIGH)
ros2 topic pub /latch_cmd std_msgs/msg/Bool "{data: true}" --once
# Close latch (pin LOW)
ros2 topic pub /latch_cmd std_msgs/msg/Bool "{data: false}" --once
```
