
# Xsens MTi CAN ROS 2 Driver

ROS 2 Humble port of the original ROS 1 Noetic driver. Tested on MTi-680-DK on Ubuntu 22.04 LTS with ROS 2 Humble. The build files use only Humble-compatible APIs and have also been verified to build cleanly under ROS 2 Jazzy on Ubuntu 24.04.

#### Note: SampleTime and UTC Time must be enabled at the device so the driver knows where each multi-frame packet starts. In MT Manager → Device Settings → Output Configuration → CAN mode, select "SampleTime, UTC Time" (or whichever frame ID you set as `start_frame_id`) plus the data fields you want, then click "Apply".

Recommended Output Configurations and Device Settings:

![Alt text](MTi-680-Output_Configuration_CAN_mode.png)

![Alt text](MTi-680-Device_Settings.png)

## How to Install

Clone the source into a colcon workspace (this repo's top-level `src/` already lays out as one). From the repo root:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select xsens_mti_can_ros_driver
source install/setup.bash
```

Optionally add the source line to your shell startup:

```bash
echo "source $(pwd)/install/setup.bash" >> ~/.bashrc
```

## How to Use

Bring up the CAN bus first (default 250 kbps; edit `setup_can0.sh` for 500k/1M):

```bash
sudo ./src/xsens_mti_can_ros_driver/config/setup_can0.sh
```

Then launch the driver:

```bash
ros2 launch xsens_mti_can_ros_driver xsens_mti_can_node.launch.py
```

Or with the 3D RViz display:

```bash
ros2 launch xsens_mti_can_ros_driver display.launch.py
```

## Troubleshooting

Verify CAN frames are arriving:

```bash
candump -ta can0
```

List active topics from the running node:

```bash
ros2 topic list
ros2 topic echo /imu/data
```

## ROS 2 Topics

| topic                    | Message Type                          | Message Contents                                                                                                                              | Data Output Rate<br>(Depending on Model and Output Configuration in MT Manager) |
| ------------------------ | ------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------- |
| filter/free_acceleration | geometry_msgs/msg/Vector3Stamped      | free acceleration from filter (local earth frame, gravity removed)                                                                            | 1-400Hz (MTi-600/100), 1-100Hz (MTi-1)                                          |
| filter/positionlla       | geometry_msgs/msg/Vector3Stamped      | filtered lat (x), lon (y), altitude (z) in WGS84                                                                                              | 1-400Hz (MTi-600/100), 1-100Hz (MTi-1)                                          |
| filter/quaternion        | geometry_msgs/msg/QuaternionStamped   | quaternion from filter                                                                                                                        | 1-400Hz (MTi-600/100), 1-100Hz (MTi-1)                                          |
| filter/twist             | geometry_msgs/msg/TwistStamped        | velocity and angular velocity                                                                                                                 | 1-400Hz (MTi-600/100), 1-100Hz (MTi-1)                                          |
| filter/velocity          | geometry_msgs/msg/Vector3Stamped      | filtered velocity output                                                                                                                      | 1-400Hz (MTi-600/100), 1-100Hz (MTi-1)                                          |
| gnss_pose                | geometry_msgs/msg/PoseStamped         | filtered position (lat, lon, alt) and quaternion                                                                                              | 1-400Hz (MTi-600/100), 1-100Hz (MTi-1)                                          |
| imu/acceleration         | geometry_msgs/msg/Vector3Stamped      | calibrated acceleration                                                                                                                       | 1-400Hz (MTi-600/100), 1-100Hz (MTi-1)                                          |
| imu/angular_velocity     | geometry_msgs/msg/Vector3Stamped      | calibrated angular velocity                                                                                                                   | 1-400Hz (MTi-600/100), 1-100Hz (MTi-1)                                          |
| imu/data                 | sensor_msgs/msg/Imu                   | quaternion, calibrated angular velocity and acceleration                                                                                      | 1-400Hz (MTi-600/100), 1-100Hz (MTi-1)                                          |
| imu/dq                   | geometry_msgs/msg/QuaternionStamped   | integrated angular velocity (quaternion increment)                                                                                            | 1-400Hz (MTi-600/100), 1-100Hz (MTi-1)                                          |
| imu/dv                   | geometry_msgs/msg/Vector3Stamped      | integrated acceleration (velocity increment)                                                                                                  | 1-400Hz (MTi-600/100), 1-100Hz (MTi-1)                                          |
| imu/mag                  | sensor_msgs/msg/MagneticField         | calibrated magnetic field                                                                                                                     | 1-100Hz                                                                         |
| imu/time_ref             | sensor_msgs/msg/TimeReference         | SampleTimeFine timestamp from device                                                                                                          | depending on packet                                                             |
| imu/utctime              | sensor_msgs/msg/TimeReference         | UTC Time from the device                                                                                                                      | depending on packet                                                             |
| pressure                 | sensor_msgs/msg/FluidPressure         | barometric pressure from device                                                                                                               | 1-100Hz                                                                         |
| status                   | xsens_mti_can_ros_driver/msg/XsStatusWord | 32-bit status word                                                                                                                        | depending on packet                                                             |
| temperature              | sensor_msgs/msg/Temperature           | temperature from device                                                                                                                       | 1-400Hz (MTi-600/100), 1-100Hz (MTi-1)                                          |
| /tf                      | tf2_msgs/msg/TFMessage                | transformed orientation                                                                                                                       | 1-400Hz (MTi-600/100), 1-100Hz (MTi-1)                                          |

Refer to the [MTi Family Reference Manual](https://mtidocs.xsens.com/mti-system-overview) for detailed field definitions.
