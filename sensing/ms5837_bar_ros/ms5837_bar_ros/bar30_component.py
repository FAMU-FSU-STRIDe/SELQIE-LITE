#!/usr/bin/env python3
from __future__ import annotations

import csv
import os
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

from . import ms5837


class Bar30Driver:
    """
    Thin wrapper around ms5837 that returns depth (m) and pressure (mbar).
    """

    def __init__(self, bus: int = 7, model: str = "30BA"):
        model_u = model.upper().strip()
        if model_u == "30BA":
            self.sensor = ms5837.MS5837_30BA(bus)
        elif model_u == "02BA":
            self.sensor = ms5837.MS5837_02BA(bus)
        else:
            raise ValueError(f"Unknown model '{model}'. Use '30BA' or '02BA'.")

        if not self.sensor.init():
            raise RuntimeError("MS5837 init() failed")

        if not self.sensor.read():
            raise RuntimeError("MS5837 initial read() failed")

    def read(self) -> bool:
        return bool(self.sensor.read())

    def pressure_mbar(self) -> float:
        # library default is mbar/hPa
        return float(self.sensor.pressure())

    def temperature_c(self) -> float:
        return float(self.sensor.temperature())

    def depth_m(self, fluid_density: float) -> float:
        self.sensor.setFluidDensity(fluid_density)
        return float(self.sensor.depth())


@dataclass
class CsvConfig:
    enabled: bool
    path: str
    flush_every_n: int
    log_rate_hz: float  # 0 -> log every sample


class CsvLogger:
    def __init__(self, cfg: CsvConfig, logger) -> None:
        self.cfg = cfg
        self._logger = logger
        self._file = None
        self._writer: Optional[csv.DictWriter] = None
        self._rows_since_flush = 0
        self._last_log_wall = 0.0

        if not self.cfg.enabled:
            return

        p = Path(self.cfg.path).expanduser()
        p.parent.mkdir(parents=True, exist_ok=True)

        # Open in append mode; write header if file is new/empty
        is_new = (not p.exists()) or (p.stat().st_size == 0)

        self._file = open(p, "a", newline="")
        self._writer = csv.DictWriter(
            self._file,
            fieldnames=[
                "wall_time_s",
                "ros_time_s",
                "pressure_mbar",
                "temp_c",
                "depth_fresh_m",
                "depth_salt_m",
                "zero_fresh_m",
                "zero_salt_m",
            ],
        )

        if is_new:
            self._writer.writeheader()
            self._file.flush()
            os.fsync(self._file.fileno())

        self._logger.info(f"CSV logging enabled -> {str(p)}")

    def should_log_now(self) -> bool:
        if not self.cfg.enabled:
            return False
        hz = float(self.cfg.log_rate_hz)
        if hz <= 0.0:
            return True
        period = 1.0 / hz
        now = time.time()
        if (now - self._last_log_wall) >= period:
            self._last_log_wall = now
            return True
        return False

    def write_row(
        self,
        *,
        wall_time_s: float,
        ros_time_s: float,
        pressure_mbar: float,
        temp_c: float,
        depth_fresh_m: float,
        depth_salt_m: float,
        zero_fresh_m: float,
        zero_salt_m: float,
    ) -> None:
        if not self.cfg.enabled or self._writer is None or self._file is None:
            return

        self._writer.writerow(
            {
                "wall_time_s": f"{wall_time_s:.6f}",
                "ros_time_s": f"{ros_time_s:.6f}",
                "pressure_mbar": f"{pressure_mbar:.4f}",
                "temp_c": f"{temp_c:.4f}",
                "depth_fresh_m": f"{depth_fresh_m:.6f}",
                "depth_salt_m": f"{depth_salt_m:.6f}",
                "zero_fresh_m": f"{zero_fresh_m:.6f}",
                "zero_salt_m": f"{zero_salt_m:.6f}",
            }
        )

        self._rows_since_flush += 1
        if self.cfg.flush_every_n > 0 and self._rows_since_flush >= self.cfg.flush_every_n:
            self.flush()

    def flush(self) -> None:
        if self._file is None:
            return
        try:
            self._file.flush()
            os.fsync(self._file.fileno())
        except Exception as e:
            self._logger.warn(f"CSV flush failed: {e}")
        finally:
            self._rows_since_flush = 0

    def close(self) -> None:
        if self._file is None:
            return
        try:
            self.flush()
        finally:
            try:
                self._file.close()
            except Exception:
                pass
            self._file = None
            self._writer = None


class BarNode(Node):
    def __init__(self):
        super().__init__("bar30_node")

        # ---- Parameters ----
        self.declare_parameter("i2c_bus", 7)
        self.declare_parameter("model", "30BA")  # "30BA" or "02BA"
        self.declare_parameter("rate_hz", 50.0)

        self.declare_parameter("density_fresh", 997.0)   # kg/m^3
        self.declare_parameter("density_salt", 1029.0)   # kg/m^3
        self.declare_parameter("zero_on_start", True)

        # Debug logging
        self.declare_parameter("debug", False)
        self.declare_parameter("debug_period", 1.0)  # seconds

        # CSV logging
        self.declare_parameter("csv_enabled", False)
        self.declare_parameter("csv_path", "/tmp/bar30_log.csv")
        self.declare_parameter("csv_flush_every_n", 25)  # fsync every N rows
        self.declare_parameter("csv_log_rate_hz", 0.0)   # 0 -> log every sample

        bus = int(self.get_parameter("i2c_bus").value)
        model = str(self.get_parameter("model").value)
        rate_hz = float(self.get_parameter("rate_hz").value)

        self.density_fresh = float(self.get_parameter("density_fresh").value)
        self.density_salt = float(self.get_parameter("density_salt").value)
        self.zero_on_start = bool(self.get_parameter("zero_on_start").value)

        self.debug = bool(self.get_parameter("debug").value)
        self.debug_period = float(self.get_parameter("debug_period").value)
        self._last_debug_time = self.get_clock().now()

        csv_cfg = CsvConfig(
            enabled=bool(self.get_parameter("csv_enabled").value),
            path=str(self.get_parameter("csv_path").value),
            flush_every_n=int(self.get_parameter("csv_flush_every_n").value),
            log_rate_hz=float(self.get_parameter("csv_log_rate_hz").value),
        )
        self.csv_logger = CsvLogger(csv_cfg, self.get_logger())

        # ---- Publishers ----
        self.pub_depth_fresh = self.create_publisher(Float32, "bar30/depth", 10)
        self.pub_depth_salt = self.create_publisher(Float32, "bar30/depth_salt", 10)
        self.pub_pressure = self.create_publisher(Float32, "bar30/pressure_mbar", 10)
        self.pub_temp = self.create_publisher(Float32, "bar30/temp_c", 10)

        # ---- Sensor ----
        try:
            self.driver = Bar30Driver(bus=bus, model=model)
        except Exception as e:
            self.get_logger().error(f"Failed to start MS5837: {e}")
            raise

        # ---- Zero offsets ----
        self.zero_fresh = 0.0
        self.zero_salt = 0.0
        if self.zero_on_start:
            self._zero_depth()

        # ---- Timer ----
        period = 1.0 / max(rate_hz, 1e-6)
        self.timer = self.create_timer(period, self._timer_cb)

        self.get_logger().info(
            f"bar30_node running: bus={bus}, model={model}, rate={rate_hz} Hz, "
            f"dens_fresh={self.density_fresh}, dens_salt={self.density_salt}, "
            f"zero_on_start={self.zero_on_start}, csv_enabled={csv_cfg.enabled}"
        )

    def _zero_depth(self) -> None:
        """Capture current depth as zero reference."""
        if not self.driver.read():
            self.get_logger().warn("Zeroing failed: sensor read() failed")
            return

        self.zero_fresh = self.driver.depth_m(self.density_fresh)
        self.zero_salt = self.driver.depth_m(self.density_salt)

        self.get_logger().info(
            f"Zeroed depth: fresh={self.zero_fresh:.4f} m, salt={self.zero_salt:.4f} m"
        )

    def _timer_cb(self) -> None:
        if not self.driver.read():
            self.get_logger().warn("MS5837 read() failed")
            return

        # Raw values
        p_mbar = self.driver.pressure_mbar()
        t_c = self.driver.temperature_c()

        # Depths (meters), relative to zero
        fresh_abs = self.driver.depth_m(self.density_fresh)
        salt_abs = self.driver.depth_m(self.density_salt)
        fresh_m = fresh_abs - self.zero_fresh
        salt_m = salt_abs - self.zero_salt

        # Publish
        msg = Float32()
        msg.data = float(fresh_m)
        self.pub_depth_fresh.publish(msg)

        msg2 = Float32()
        msg2.data = float(salt_m)
        self.pub_depth_salt.publish(msg2)

        msg3 = Float32()
        msg3.data = float(p_mbar)
        self.pub_pressure.publish(msg3)

        msg4 = Float32()
        msg4.data = float(t_c)
        self.pub_temp.publish(msg4)

        # Throttled debug logs
        if self.debug:
            now = self.get_clock().now()
            dt = (now - self._last_debug_time).nanoseconds * 1e-9
            if dt >= self.debug_period:
                self.get_logger().info(
                    "BAR30 | "
                    f"P={p_mbar:.2f} mbar | T={t_c:.2f} C | "
                    f"FreshAbs={fresh_abs:.4f} m FreshRel={fresh_m:.4f} m (zero={self.zero_fresh:.4f}) | "
                    f"SaltAbs={salt_abs:.4f} m SaltRel={salt_m:.4f} m (zero={self.zero_salt:.4f})"
                )
                self._last_debug_time = now

        # CSV logging (optional, throttled by csv_log_rate_hz)
        if self.csv_logger.should_log_now():
            wall_s = time.time()
            ros_now = self.get_clock().now()
            ros_s = ros_now.nanoseconds * 1e-9  # ROS clock seconds
            self.csv_logger.write_row(
                wall_time_s=wall_s,
                ros_time_s=ros_s,
                pressure_mbar=p_mbar,
                temp_c=t_c,
                depth_fresh_m=fresh_m,
                depth_salt_m=salt_m,
                zero_fresh_m=self.zero_fresh,
                zero_salt_m=self.zero_salt,
            )

    def destroy_node(self):
        # ensure CSV file is closed cleanly
        try:
            if hasattr(self, "csv_logger") and self.csv_logger is not None:
                self.csv_logger.close()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BarNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
