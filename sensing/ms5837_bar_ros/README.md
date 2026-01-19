# ms5837_bar_ros

ROS 2 package for the Blue Robotics Bar30 and Bar02 pressure/depth sensors using
MS5837 drivers. The nodes publish pressure, temperature, depth, and an odometry
message derived from the depth reading.

## Requirements

* Bar30 (300 m) or Bar02 (10 m) sensor connected over I²C.
* `i2c-tools` and Python `smbus2`.

```bash
sudo apt-get install -y i2c-tools
sudo pip3 install smbus2
```

Verify the I²C address (default `0x76`):

```bash
i2cdetect -y 7
```

> **Note**
> The driver initializes the sensor on I²C bus 7 by default. Update the bus
> number in `ms5837.py` or the `BarComponentr` class if your sensor is on a
> different bus.

## Build

From the workspace root:

```bash
colcon build --packages-select ms5837_bar_ros
source install/setup.bash
```

## Run

```bash
# Bar30
ros2 run ms5837_bar_ros bar30_node

# Bar02
ros2 run ms5837_bar_ros bar02_node
```

You can also launch through the sensing bringup package:

```bash
ros2 launch sensing_bringup bar30.launch.py
ros2 launch sensing_bringup bar02.launch.py
```

## Topics

### Bar30

* `bar30/pressure` (`std_msgs/Float32`)
* `bar30/temperature` (`std_msgs/Float32`)
* `bar30/depth` (`std_msgs/Float32`)
* `bar30/odom` (`nav_msgs/Odometry`)

### Bar02

* `bar02/pressure` (`std_msgs/Float32`)
* `bar02/temperature` (`std_msgs/Float32`)
* `bar02/depth` (`std_msgs/Float32`)
* `bar02/odom` (`nav_msgs/Odometry`)

## License

This package is released under the MIT License. The MS5837 Python driver and
sensor documentation remain under their respective upstream licenses.
