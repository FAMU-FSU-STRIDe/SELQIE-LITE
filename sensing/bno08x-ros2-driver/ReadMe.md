# BNO08x ROS 2 Driver

ROS 2 driver for the BNO08x IMU family (BNO085/BNO086) based on the SH-2
protocol. The driver provides IMU and magnetic field data over I²C and exposes
configuration parameters via standard ROS 2 YAML files.

For hardware details, refer to the
[datasheet](./docs/BNO080_085-Datasheet.pdf).

[![CI](https://github.com/bnbhat/bno08x_ros2_driver/actions/workflows/ci.yaml/badge.svg)](https://github.com/bnbhat/bno08x_ros2_driver/actions/workflows/ci.yaml)

## Supported features

### Communication interfaces

* I²C (implemented)
* UART (not implemented)
* SPI (not implemented)

### Data rates

* IMU data up to 400 Hz
* Magnetic field data up to 100 Hz

## Parameters

| Parameter | Type | Default | Description |
| --------- | ---- | ------- | ----------- |
| `frame_id` | string | `bno085` | Frame ID for sensor data messages. |
| `publish.magnetic_field.enabled` | bool | `true` | Enable magnetic field publishing. |
| `publish.magnetic_field.rate` | int | `100` | Magnetic field publish rate (Hz). |
| `publish.imu.enabled` | bool | `true` | Enable IMU publishing. |
| `publish.imu.rate` | int | `100` | IMU publish rate (Hz). |
| `i2c.enabled` | bool | `true` | Enable I²C communication. |
| `i2c.device` | string | `/dev/i2c-7` | I²C device path. |
| `i2c.address` | string | `0x4A` | I²C address of the sensor. |

## Build

From the workspace root:

```bash
colcon build --packages-select bno08x_driver
source install/setup.bash
```

## Usage

The driver defaults to I²C. If you want to use UART or SPI, you must change the
hardware configuration per the datasheet; those transports are not implemented
in this driver.

Example config (see `config/bno085_i2c.yaml`):

```yaml
bno08x_driver:
  ros__parameters:
    frame_id: "bno085"
    i2c:
      enabled: true
      device: "/dev/i2c-18"
      address: "0x4A"
    publish:
      all: false
      magnetic_field:
        enabled: true
        rate: 100
      imu:
        enabled: true
        rate: 100
```

Launch the driver:

```bash
ros2 launch bno08x_driver bno085_i2c.launch.py
```

## Code structure

The driver wraps the SH-2 protocol in a reusable C++ class (`BNO08x`), with
`CommInterface` adapters for different buses. ROS 2 integration lives in
`BNO08xROS`, which wires the sensor driver into rclcpp publishers.

Directory structure:

```plaintext
.
├── CMakeLists.txt
├── config
│   └── bno085_i2c.yaml
├── docs
│   ├── BNO080_085-Datasheet.pdf
│   └── SH-2-Reference-Manual.pdf
├── include
│   ├── bno08x_driver
│   │   ├── bno08x.hpp
│   │   ├── bno08x_ros.hpp
│   │   ├── comm_interface.hpp
│   │   ├── i2c_interface.hpp
│   │   ├── logger.h
│   │   ├── spi_interface.hpp
│   │   └── uart_interface.hpp
│   └── sh2
│       ├── CMakeLists.txt
│       ├── NOTICE.txt
│       ├── README.md
│       ├── sh2.c
│       ├── sh2_err.h
│       ├── sh2.h
│       ├── sh2_hal.h
│       ├── sh2_SensorValue.c
│       ├── sh2_SensorValue.h
│       ├── sh2_util.c
│       ├── sh2_util.h
│       ├── shtp.c
│       └── shtp.h
├── launch
│   └── bno085_i2c.launch.py
├── LICENSE
├── package.xml
├── ReadMe.md
└── src
    ├── bno08x.cpp
    ├── bno08x_ros.cpp
    └── ros_node.cpp
```

## Acknowledgements

This driver uses the SH-2 protocol library provided by Hillcrest Labs (now CEVA).
The library is vendored in the `include/sh2` directory.

## License

Apache License 2.0. See [LICENSE](./LICENSE).
