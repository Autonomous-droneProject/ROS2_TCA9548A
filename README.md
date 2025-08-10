# TCA9548A I2C Multiplexer Manager for ROS 2

## Overview

This ROS 2 package provides a centralized "gatekeeper" manager node to manage a TCA9548A I2C multiplexer. In a robotic system with multiple I2C sensors connected to a single bus via a multiplexer, this node prevents resource conflicts by ensuring that only one device is active on the bus at any given time.

It exposes its own services and a class called I2CDevice. The I2CDevice is a template class which is supposed to wrap around the driver implementations. Then, the TCA Manager node will initialize the I2CDevices and execute its functions when TCA services are called.
