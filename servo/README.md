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
