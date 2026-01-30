from __future__ import annotations

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from std_msgs.msg import Float32

from . import ms5837


class Bar02Driver:
    def __init__(self, bus: int = 7):
        self.sensor = ms5837.MS5837_02BA(bus)

        if not self.sensor.init():
            raise RuntimeError("Sensor could not be initialized")

        if not self.sensor.read():
            raise RuntimeError("Sensor initial read failed")

    def read(self) -> bool:
        return bool(self.sensor.read())

    def pressure_mbar(self) -> float:
        return float(self.sensor.pressure())

    def temperature_c(self) -> float:
        return float(self.sensor.temperature())

    def depth_m(self, fluid_density: float) -> float:
        self.sensor.setFluidDensity(fluid_density)
        return float(self.sensor.depth())


class BarNode(Node):

    def __init__(self):
        super().__init__("bar02_node")
        self.pub_pressure = self.create_publisher(Float32, "bar02/pressure", 10)
        self.pub_temp = self.create_publisher(Float32, "bar02/temperature", 10)
        self.pub_depth = self.create_publisher(Float32, "bar02/depth", 10)
        self.pub_odom = self.create_publisher(Odometry, "bar02/odom", 10)

        self.declare_parameter("i2c_bus", 7)
        self.declare_parameter("rate_hz", 50.0)
        self.declare_parameter("density_fresh", 997.0)
        self.declare_parameter("zero_on_start", True)

        bus = int(self.get_parameter("i2c_bus").value)
        rate_hz = float(self.get_parameter("rate_hz").value)
        self.density_fresh = float(self.get_parameter("density_fresh").value)
        self.zero_on_start = bool(self.get_parameter("zero_on_start").value)

        try:
            self.driver = Bar02Driver(bus=bus)
        except Exception as exc:
            self.get_logger().error(f"Failed to start MS5837: {exc}")
            raise

        self.zero_depth = 0.0
        if self.zero_on_start:
            self._zero_depth()

        timer_period = 1.0 / max(rate_hz, 1e-6)
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info(
            f"bar02_node running: bus={bus}, rate={rate_hz} Hz, "
            f"density_fresh={self.density_fresh}, zero_on_start={self.zero_on_start}"
        )

        self.msg_pressure = Float32()
        self.msg_temp = Float32()
        self.msg_depth = Float32()
        self.msg_odom = Odometry()

    def _zero_depth(self) -> None:
        if not self.driver.read():
            self.get_logger().warn("Zeroing failed: sensor read() failed")
            return
        self.zero_depth = self.driver.depth_m(self.density_fresh)
        self.get_logger().info(f"Zeroed depth: fresh={self.zero_depth:.4f} m")

    def timer_callback(self):
        if not self.driver.read():
            self.get_logger().warn("MS5837 read() failed")
            return

        pressure_mbar = self.driver.pressure_mbar()
        temp_c = self.driver.temperature_c()
        depth_m = self.driver.depth_m(self.density_fresh) - self.zero_depth

        self.msg_pressure.data = round(pressure_mbar, 1)
        self.msg_temp.data = round(temp_c, 1)
        self.msg_depth.data = round(depth_m, 3)

        self.msg_odom.header.stamp = self.get_clock().now().to_msg()
        self.msg_odom.header.frame_id = "bar02_link"
        self.msg_odom.pose.pose.position.z = -self.msg_depth.data

        self.pub_pressure.publish(self.msg_pressure)
        self.pub_temp.publish(self.msg_temp)
        self.pub_depth.publish(self.msg_depth)
        self.pub_odom.publish(self.msg_odom)


def main(args=None):
    rclpy.init(args=args)
    ms5837_node = BarNode()

    rclpy.spin(ms5837_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ms5837_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
