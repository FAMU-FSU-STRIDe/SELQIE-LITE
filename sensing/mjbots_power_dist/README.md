# mjbots_power_dist

ROS 2 Humble driver that listens to an MJBots power dist r4.5b on a SocketCAN
interface (default: `can0`) and republishes telemetry to ROS topics. The node
can both passively listen for periodic status frames *and* actively poll the
board using the upstream register protocol demonstrated in the
[mjbots/power_dist](https://github.com/mjbots/power_dist) Pi3Hat example. It
extracts bus voltage/current, rail telemetry, board temperature, switch state,
energy, and status flags.

## Usage

Build the workspace with `colcon build --symlink-install`, source it, then run:

```bash
ros2 run mjbots_power_dist power_dist_node --ros-args \
  -p can_interface:=can0
```

A launch file is also provided:

```bash
ros2 launch mjbots_power_dist power_dist.launch.py
```

### Parameters

| Name | Default | Description |
| ---- | ------- | ----------- |
| `can_interface` | `can0` | SocketCAN interface that the power dist board is connected to. |
| `status_id` | `0x2000` | Set to the expected telemetry or register-reply arbitration ID. Set to `0` to auto-detect. |
| `status_id_mask` | `0x1FFFFFFF` | Optional mask applied to the arbitration ID match (useful when the firmware encodes a device ID in the CAN identifier). |
| `poll_hz` | `50.0` | How often the node polls buffered CAN frames for new telemetry. |
| `frame_length_warning` | `16` | Minimum payload length expected before logging a warning. |
| `register_query_id` | `0x8020` | Arbitration ID used when sending Pi3Hat-style register queries. Set to `0` to disable active polling. |
| `register_reply_id` | `0x2000` | Expected arbitration ID for register replies; also used as the default `status_id`. |
| `register_poll_hz` | `10.0` | Rate at which register queries are sent when `register_query_id` is non-zero. |

### Topics

* `/power_dist/battery` (`sensor_msgs/BatteryState`): bus voltage, bus current, and board temperature.
* `/power_dist/diagnostics` (`diagnostic_msgs/DiagnosticArray`): detailed rail telemetry, status flags, switch state, and energy.
* `/power_dist/raw_frame` (`std_msgs/UInt8MultiArray`): raw payload bytes from the telemetry frame.

### Telemetry layout

Two layouts are supported and auto-detected:

1. **Register reply (Pi3Hat example)**: matches the upstream demonstration and
   includes bus voltage/current, temperature, cumulative energy, and switch
   state.
2. **16-byte periodic telemetry**: the original power dist broadcast format
   containing bus and rail measurements plus status flags.

Frames shorter than the expected layout are accepted; missing fields are
published as `NaN`.
