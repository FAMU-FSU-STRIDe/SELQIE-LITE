# SH-2 Sensor Hub Library (No-RTOS)

This directory contains the SH-2 protocol implementation used by the
BNO08x ROS 2 driver. It provides application-level sensor hub support and is
vendored from the upstream Hillcrest/CEVA SH-2 library.

## Using the library

To integrate the SH-2 library in another project, you must:

* Add the source files to your build system.
* Implement the platform hooks specified in `sh2_hal.h`.
* Call the API surface in `sh2.h` from your application logic.

The full reference documentation is available in:

* [SH2 Library User's Guide](UserGuide.pdf)

An example project based on this driver can be found here:

* [sh2-demo-nucleo](https://github.com/hcrest/sh2-demo-nucleo)
