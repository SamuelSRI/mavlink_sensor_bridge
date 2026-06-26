# mavlink_sensor_bridge

ROS 2 package that reads MAVLink data from a Cube Orange flight controller over UART and publishes standard ROS 2 sensor topics.

This package is intended for rover or robotics platforms where the GPS and IMU are connected to the Cube Orange, and the companion computer needs to receive these sensors through MAVLink.

## Features

- Read MAVLink from UART
- Receive IMU data from Cube Orange
- Receive GPS data from Cube Orange
- Publish standard ROS 2 messages
- Configurable serial port and baudrate
- Simple launch file and YAML configuration

## Published topics

| Topic | Type | Description |
|---|---|---|
| `/imu/data` | `sensor_msgs/msg/Imu` | IMU orientation, angular velocity and linear acceleration |
| `/gps/fix` | `sensor_msgs/msg/NavSatFix` | GPS latitude, longitude, altitude and fix status |
| `/gps/status` | `diagnostic_msgs/msg/DiagnosticStatus` | GPS diagnostic information |

## Parameters

| Parameter | Default | Description |
|---|---:|---|
| `port` | `/dev/ttyS0` | UART port connected to the Cube Orange |
| `baudrate` | `115200` | MAVLink serial baudrate |
| `imu_frame_id` | `imu_link` | Frame ID used for IMU messages |
| `gps_frame_id` | `gps_link` | Frame ID used for GPS messages |
| `publish_rate_hz` | `100.0` | Node loop frequency |
| `heartbeat_rate_hz` | `1.0` | MAVLink companion heartbeat frequency |
| `debug` | `false` | Print received MAVLink message types |

## Installation

Clone the package into a ROS 2 workspace:

```bash
cd ~/ros2_ws/src
git clone git@github.com:SamuelSRI/mavlink_sensor_bridge.git
```

Install Python dependency:

```bash
sudo apt update
sudo apt install python3-pip
python3 -m pip install pymavlink --break-system-packages
```

Build the package:

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select mavlink_sensor_bridge
source install/setup.bash
```

## Usage

Launch the node:

```bash
ros2 launch mavlink_sensor_bridge mavlink_sensor_bridge.launch.py
```

Check published topics:

```bash
ros2 topic list
```

Expected topics:

```text
/imu/data
/gps/fix
/gps/status
```

Echo IMU data:

```bash
ros2 topic echo /imu/data
```

Echo GPS data:

```bash
ros2 topic echo /gps/fix
```

Check topic frequency:

```bash
ros2 topic hz /imu/data
ros2 topic hz /gps/fix
```

## Configuration

Edit the configuration file:

```bash
nano config/mavlink_sensor_bridge.yaml
```

Example configuration:

```yaml
mavlink_sensor_bridge_node:
  ros__parameters:
    port: "/dev/ttyS0"
    baudrate: 115200

    imu_frame_id: "imu_link"
    gps_frame_id: "gps_link"

    publish_rate_hz: 100.0
    heartbeat_rate_hz: 1.0

    debug: false
```

Common UART ports:

```text
/dev/ttyS0
/dev/ttyAMA0
/dev/serial0
/dev/ttyUSB0
/dev/ttyACM0
```

## Serial permissions

Add the user to the required serial groups:

```bash
sudo usermod -a -G dialout $USER
sudo usermod -a -G tty $USER
sudo reboot
```

After reboot:

```bash
cd ~/ros2_ws
source install/setup.bash
```

## Cube Orange / ArduPilot configuration

The Cube Orange serial port connected to the companion computer must be configured to output MAVLink.

Example for `SERIAL2`:

```text
SERIAL2_PROTOCOL = 2
SERIAL2_BAUD     = 115
```

Recommended stream rates:

```text
SR2_RAW_SENS = 10
SR2_POSITION = 5
SR2_EXTRA1   = 10
```

If another serial port is used, replace `SERIAL2` and `SR2` with the correct port number.

Example for `SERIAL1`:

```text
SERIAL1_PROTOCOL = 2
SERIAL1_BAUD     = 115

SR1_RAW_SENS = 10
SR1_POSITION = 5
SR1_EXTRA1   = 10
```

## ROS 2 integration

This package only publishes sensor data. Sensor transforms should be handled separately with `robot_state_publisher`, URDF, or static TF publishers.

Typical rover localization stack:

```text
Cube Orange
    ↓ MAVLink UART
mavlink_sensor_bridge
    ↓
/imu/data
/gps/fix
    ↓
robot_localization
    ↓
/odometry/filtered
```

Example `robot_localization` inputs:

```yaml
imu0: /imu/data
gps0: /gps/fix
```

## Notes

- The IMU frame is defined by the `imu_frame_id` parameter.
- The GPS frame is defined by the `gps_frame_id` parameter.
- The physical transforms between `base_link`, `imu_link`, and `gps_link` should not be published by this node.
- The Cube Orange must already receive valid IMU and GPS data.
- GPS fix quality depends on the GPS module and ArduPilot configuration.

## License

This project is licensed under the MIT License.