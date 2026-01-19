# latch

ROS 2 serial driver package for the Hitec D954SW latch controller. The node
streams latch angle commands to a Teensy over USB serial and relays reed switch
state updates published by the microcontroller.

## Topics

| Topic | Type | Direction | Description |
| ----- | ---- | --------- | ----------- |
| `latch_angle_cmd` | `std_msgs/Float64` | Subscribe | Target latch angle in degrees. |
| `reed_switch` | `std_msgs/Bool` | Publish | Reed switch state reported by the Teensy. |

## Parameters

| Parameter | Type | Default | Description |
| --------- | ---- | ------- | ----------- |
| `port` | string | `/dev/ttyACM0` | Serial port connected to the Teensy. |
| `baud` | int | `115200` | UART baud rate. |
| `timeout_s` | float | `0.2` | Serial read/write timeout. |

## Usage

```bash
ros2 run latch latch_node
```

### Example commands

```bash
# Open latch (0 degrees)
ros2 topic pub /latch_angle_cmd std_msgs/msg/Float64 "{data: 0.0}" --once

# Close latch (180 degrees)
ros2 topic pub /latch_angle_cmd std_msgs/msg/Float64 "{data: 180.0}" --once
```

## Launch file

```bash
ros2 launch latch latch.launch.py
```

Edit the launch file to match the USB serial path for your Teensy.
