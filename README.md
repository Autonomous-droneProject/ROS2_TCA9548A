# TCA9548A I2C Multiplexer Manager for ROS 2

## Overview

This ROS 2 package provides a centralized "gatekeeper" node to manage a TCA9548A I2C multiplexer. In a robotic system with multiple I2C sensors connected to a single bus via a multiplexer, this node prevents resource conflicts by ensuring that only one device is active on the bus at any given time.

It works by exposing a ROS 2 Service that other sensor nodes can call to request control of a specific I2C channel before they attempt to communicate with their hardware.

### Key Features

  - **Centralized Hardware Management:** Owns the I2C bus connection to the TCA9548A, eliminating race conditions between multiple sensor nodes.
  - **ROS 2 Service Interface:** Provides a simple and robust service named `/select_channel` for requesting channel changes.
  - **Configurable:** The I2C bus and the multiplexer's I2C address can be configured via ROS parameters.
  - **Efficient:** Avoids redundant I2C writes by keeping track of the currently active channel.

## Dependencies

  - ROS 2 (Humble, Iron, or newer)
  - `rclcpp`
  - `rosidl_default_generators` (for building the service)

## Installation and Building

Follow these steps to build the package from the source.

**1. Navigate to your ROS 2 Workspace**

```bash
cd ~/ros2_ws/src
```

**2. Clone or Copy the Package**

If this package were in a git repository, you would clone it. For now, ensure the `tca9548a` package directory you've created is inside the `src` folder.

**3. Build the Package**

Navigate to the root of your workspace and use `colcon` to build the package.

```bash
cd ~/ros2_ws
colcon build --packages-select tca9548a
```

**4. Source the Workspace**

Before running the node, source your workspace's setup file in every new terminal.

```bash
source install/setup.bash
```

## Usage

This node is designed to run as a background service that your other sensor nodes will communicate with.

### Running the Manager Node

To run the node with its default settings, use the following command:

```bash
ros2 run tca9548a tca_manager_node
```

You should see an output message confirming that the service is ready:

```
[INFO] [1719520801.123456789] [tca_manager_node]: TCA9548A Manager is ready and providing '/select_channel' service.
```

#### Running with Custom Parameters

You can override the default parameters from the command line. This is useful if your multiplexer is on a different I2C bus or has a non-default address.

```bash
# Example: Multiplexer at address 0x71 on i2c-7
ros2 run tca9548a tca_manager_node --ros-args -p tca_address:=0x71 -p i2c_bus:="/dev/i2c-7"
```

### Interacting with the `/select_channel` Service

The core functionality of this node is exposed through the `/select_channel` service.

  - **Service Name:** `/select_channel`
  - **Service Type:** `tca9548a/srv/SelectChannel`

#### Service Definition

  - **Request:**
      - `int8 channel`: The channel you want to activate (0-7).
  - **Response:**
      - `bool success`: `true` if the channel was selected, `false` otherwise.
      - `string message`: A human-readable status message.

#### Calling the Service from the Command Line

You can use the `ros2 service call` command to test the node and manually change the channel. This is extremely useful for debugging.

**Example: Select channel 4**

```bash
ros2 service call /select_channel tca9548a/srv/SelectChannel '{channel: 4}'
```

**Expected Response:**

```
requester: making request: tca9548a.srv.SelectChannel_Request(channel=4)

response:
tca9548a.srv.SelectChannel_Response(success=True, message='Successfully selected channel 4')
```

### Integrating with a Sensor Node (Client Logic)

This manager node is not meant to be used alone. Your individual sensor nodes (e.g., for a VL53L1X, BNO055, etc.) must be written as **clients** of this service.

The logic within your sensor node's code should be:

1.  Create a ROS 2 service client for `/select_channel`.
2.  In a timer or callback where you need to read from the sensor:
    a.  Call the `/select_channel` service with the correct channel number for your sensor.
    b.  Wait for the service call to return with `success: true`.
    c.  **Only then**, proceed with your I2C communication (e.g., `read()`, `write()`) to the sensor.

This ensures the multiplexer is correctly configured before any sensor communication is attempted.

## ROS 2 Parameters

The following parameters can be configured when launching the `tca_manager_node`.

| Parameter     | Type     | Default Value  | Description                                        |
|---------------|----------|----------------|----------------------------------------------------|
| `i2c_bus`     | `string` | `/dev/i2c-1`   | The Linux device file for the I2C bus.             |
| `tca_address` | `int`    | `112` (`0x70`) | The 7-bit I2C address of the TCA9548A multiplexer. |
