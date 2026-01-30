"""MS5837 sensor driver selection.

This module prefers the real ``ms5837`` Python driver when it is installed so
ROS 2 nodes can read live sensor values. If the external driver is missing, a
minimal shim is used to keep nodes running in simulation and tests.
"""

from __future__ import annotations

from typing import Optional

import importlib


def _load_external_driver() -> Optional[object]:
    try:
        return importlib.import_module("ms5837")
    except Exception:
        return None


_driver = _load_external_driver()

if _driver is not None:
    UNITS_mbar = _driver.UNITS_mbar
    UNITS_atm = _driver.UNITS_atm
    UNITS_Torr = _driver.UNITS_Torr
    UNITS_psi = _driver.UNITS_psi
    UNITS_Centigrade = _driver.UNITS_Centigrade
    UNITS_Farenheit = _driver.UNITS_Farenheit
    UNITS_Kelvin = _driver.UNITS_Kelvin

    MS5837_MODEL_02BA = _driver.MS5837_MODEL_02BA
    MS5837_MODEL_30BA = _driver.MS5837_MODEL_30BA

    DENSITY_FRESHWATER = _driver.DENSITY_FRESHWATER
    DENSITY_SALTWATER = _driver.DENSITY_SALTWATER

    MS5837 = _driver.MS5837
    MS5837_30BA = _driver.MS5837_30BA
    MS5837_02BA = _driver.MS5837_02BA
else:
    # Conversion constants
    UNITS_mbar = 0
    UNITS_atm = 1
    UNITS_Torr = 2
    UNITS_psi = 3
    UNITS_Centigrade = 4
    UNITS_Farenheit = 5
    UNITS_Kelvin = 6

    # Sensor models
    MS5837_MODEL_02BA = 0
    MS5837_MODEL_30BA = 1

    # Fluid densities (kg/m^3)
    DENSITY_FRESHWATER = 997.0474
    DENSITY_SALTWATER = 1029

    class MS5837:
        """Simple mock of the MS5837 pressure/temperature sensor.

        The methods mirror the public API of the upstream driver closely enough for
        the ROS nodes in this repository to run without modification.
        """

        def __init__(self, model: int = MS5837_MODEL_30BA, bus: int = 7) -> None:
            self.model = model
            self.bus = bus
            self._fluid_density = DENSITY_FRESHWATER
            # Nominal sample values
            self._pressure_mbar = 1013.25
            self._temperature_c = 20.0
            self._depth_m = 0.0

        def init(self) -> bool:  # pragma: no cover - trivial wrapper
            """Pretend to initialize the device.

            Always returns ``True`` so downstream nodes can continue running even
            when the hardware is absent.
            """

            return True

        def read(self) -> bool:  # pragma: no cover - trivial wrapper
            """Refresh sensor state.

            In this mock implementation there is no external device, so the stored
            sample values are reused and the method always reports success.
            """

            return True

        def pressure(self, unit: int = UNITS_mbar) -> float:
            """Return pressure in the requested unit."""

            if unit == UNITS_atm:
                return self._pressure_mbar / 1013.25
            if unit == UNITS_Torr:
                return self._pressure_mbar * 0.750062
            if unit == UNITS_psi:
                return self._pressure_mbar * 0.0145038
            return self._pressure_mbar

        def temperature(self, unit: int = UNITS_Centigrade) -> float:
            """Return temperature in the requested unit."""

            if unit == UNITS_Farenheit:
                return self._temperature_c * 9.0 / 5.0 + 32
            if unit == UNITS_Kelvin:
                return self._temperature_c + 273.15
            return self._temperature_c

        def depth(self) -> float:
            """Return the current depth in meters."""

            return self._depth_m

        def setFluidDensity(self, density: float) -> None:
            """Set the density used for depth calculations."""

            self._fluid_density = density

        def altitude(self) -> float:
            """Estimate altitude relative to mean sea level.

            This simplified implementation uses the barometric formula with
            constant temperature.
            """

            # Constants
            sea_level_pressure = 1013.25  # mbar
            temperature_lapse = 0.0065  # K/m
            gas_constant = 8.3144598  # J/(mol*K)
            molar_mass = 0.0289644  # kg/mol
            gravity = 9.80665  # m/s^2
            temperature_k = self._temperature_c + 273.15

            ratio = (self._pressure_mbar / sea_level_pressure) ** (gas_constant * temperature_lapse / (gravity * molar_mass))
            return (temperature_k / temperature_lapse) * (ratio - 1)

    class MS5837_30BA(MS5837):
        def __init__(self, bus: int = 7) -> None:
            super().__init__(MS5837_MODEL_30BA, bus)

    class MS5837_02BA(MS5837):
        def __init__(self, bus: int = 7) -> None:
            super().__init__(MS5837_MODEL_02BA, bus)
