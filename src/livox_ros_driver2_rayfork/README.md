# VoFOD-Mid360 controlled Livox ROS Driver 2 fork

This directory is the workspace-controlled fork of
`Livox-SDK/livox_ros_driver2` at commit
`6b9356cadf77084619ba406e6a0eb41163b08039`. The upstream provenance and the
precise local boundary are recorded in [UPSTREAM.md](UPSTREAM.md). The original
driver outputs remain available; the ROS1 RayBundle bridge is opt-in.

> **Workspace safety:** do not run this repository's upstream `build.sh` from
> the shared catkin workspace. That script recursively deletes the workspace's
> `build/`, `devel/`, and `install/` directories and rewrites package files.
> Build and test this controlled fork with catkin tools from the workspace root:

```bash
source /opt/ros/noetic/setup.bash
catkin build livox_ros_driver2 --no-deps --cmake-args \
  -DCMAKE_BUILD_TYPE=Release -DROS_EDITION=ROS1
catkin run_tests livox_ros_driver2
catkin_test_results build/livox_ros_driver2
```

Focused tests cover zero-depth direction preservation, the Livox angle
convention, range-state classification, raw timestamp provenance and type-3
response rejection. Generated result XML is not retained, and software tests
are not physical-device evidence.

## Local raw-spherical RayBundle extension

`config/MID360_spherical_config.json` requests `pcl_data_type: 3`, and
`launch_ROS1/msg_MID360_spherical_rays.launch` enables the optional bridge. The
hook observes each untouched spherical `RawPacket` before upstream
`ProcessSphericalPoint()`. It preserves raw `depth`, `theta`, `phi`,
`reflectivity`, and `tag`, derives the packet line as `index % line_num`, and
copies the packet `time_type` plus all eight raw timestamp bytes before using
the selected frame stamp and point interval to retain per-sample timing and a
monotonic pattern index.

Synchronized gPTP/PTP (`time_type=1`) and GPS (`time_type=2`) packets use the
decoded eight-byte device timestamp only when it is valid and exactly matches
the selected frame stamp. No-sync (`time_type=0`) instead uses the host
system-clock receipt time for compatibility framing while retaining the raw
bytes for audit. That host receipt stamp is explicitly unsynchronized: it must
not be treated as lidar device time, used for per-ray motion compensation, or
accepted as exact hardware evidence. The independent exact gate requires
valid synchronized PTP/gPTP or GPS metadata.

Livox spherical angles are hundredths of a degree. With `theta` as the polar
angle and `phi` as azimuth, the emitted sensor-frame unit direction is:

```text
theta_rad = (theta / 100) * pi / 180
phi_rad   = (phi   / 100) * pi / 180
x = sin(theta_rad) * cos(phi_rad)
y = sin(theta_rad) * sin(phi_rad)
z = cos(theta_rad)
```

Depth is converted from millimetres to metres. In particular, `depth == 0`
becomes `NO_RETURN` while retaining the raw spherical direction; it is never
reconstructed from `line`, timestamp, or a simulation CSV.

RayBundle publication is coupled to the upstream point-cloud flush. A bundle
is published only when its raw base timestamp and sample count exactly match
the compatibility point-cloud frame. A mismatch is counted and suppresses the
bundle, so apparent alignment cannot be created by independently repacking two
streams. For every aligned frame, diagnostics and RayBundle carry the same
header stamp; the diagnostic's `device_restart_marker` uses the `powerup_cnt`
snapshot captured at that frame's first packet. The same diagnostic also binds
`frame_scan_id`, `frame_pattern_start_index`, and `frame_ray_count` to the
published bundle. PointCloud2, CustomMsg/PCL, IMU, and bag branches remain
upstream compatible.

The latched `/uav1/mid360/hw_spherical_diagnostics` stream reports device type,
serial number, firmware, `powerup_cnt`, type-3 request/response acceptance,
spherical-packet confirmation, raw/spherical packet and sample counts,
zero-depth finite/non-trivial counts, frame-alignment mismatches, temporal
continuity, the raw eight-byte timestamp in hex/little-endian form,
`time_type`, frame-stamp source, synchronization state, host receipt time, and
whether the stream is single-device. The driver deliberately
reports `hw_spherical_candidate_unverified`,
`exact_no_return_direction_supported=false`, and
`fallback_mode=calibrated_fallback`; only the independent capability probe may
promote the common mode after repeated physical restarts.

The integrated hardware launch fixes `xfer_format=0`, preserving
`/livox/lidar` as the PointCloud2 stream paired with this RayBundle path. The
probe pairs each diagnostic/RayBundle frame by exact stamp and then cross-checks
`scan_id`, `pattern_start_index`, and ray count. It clears unpaired halves at
every boot-epoch transition, binds the paired power-up snapshot and
device/serial/firmware identity within each trial, and performs a fresh
diagnostic/ray/paired-frame check before any exact promotion. Its mode
heartbeat is accepted by the hardware preprocessor only from
`/mid360_spherical_capability_probe`; dynamic `sim_exact` is forbidden. The
default 3 s wall-time lease remains active, and timeout or probe shutdown
returns the path to sticky `calibrated_fallback`.

This audited path has two deliberate geometry boundaries:

- It supports one Mid-360 stream. Observing a second device handle marks the
  stream non-single-device and prevents capability qualification.
- The supplied type-3 configuration uses zero Livox extrinsics. Raw directions
  are in `uav1/mid360`; apply measured sensor-to-body/world transforms
  downstream. Non-zero driver extrinsics are outside the same-frame alignment
  contract and must not be enabled without a new audit.

No physical Mid-360, firmware response, or zero-depth `theta`/`phi` behavior
has been verified in this workspace. Software readiness does not establish
`hw_spherical_exact` and must not be cited as hardware evidence.

# Upstream Livox ROS Driver 2 README

The upstream text is retained below for provenance and general reference. Its
`./build.sh` commands are **prohibited in this shared workspace**; use the safe
catkin commands at the top of this file.

Livox ROS Driver 2 is the 2nd-generation driver package used to connect LiDAR products produced by Livox, applicable for ROS (noetic recommended) and ROS2 (foxy or humble recommended).

  **Note :**

  As a debugging tool, Livox ROS Driver is not recommended for mass production but limited to test scenarios. You should optimize the code based on the original source to meet your various needs.

## 1. Preparation

### 1.1 OS requirements

  * Ubuntu 18.04 for ROS Melodic;
  * Ubuntu 20.04 for ROS Noetic and ROS2 Foxy;
  * Ubuntu 22.04 for ROS2 Humble;

  **Tips:**

  Colcon is a build tool used in ROS2.

  How to install colcon: [Colcon installation instructions](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html)

### 1.2 Install ROS & ROS2

For ROS Melodic installation, please refer to:
[ROS Melodic installation instructions](https://wiki.ros.org/melodic/Installation)

For ROS Noetic installation, please refer to:
[ROS Noetic installation instructions](https://wiki.ros.org/noetic/Installation)

For ROS2 Foxy installation, please refer to:
[ROS Foxy installation instructions](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)

For ROS2 Humble installation, please refer to:
[ROS Humble installation instructions](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

Desktop-Full installation is recommend.

## 2. Build & Run Livox ROS Driver 2

### 2.1 Clone Livox ROS Driver 2 source code:

```shell
git clone https://github.com/Livox-SDK/livox_ros_driver2.git ws_livox/src/livox_ros_driver2
```

  **Note :**

  Be sure to clone the source code in a '[work_space]/src/' folder (as shown above), otherwise compilation errors will occur due to the compilation tool restriction.

### 2.2 Build & install the Livox-SDK2

  **Note :**

  Please follow the guidance of installation in the [Livox-SDK2/README.md](https://github.com/Livox-SDK/Livox-SDK2/blob/master/README.md)

### 2.3 Build the Livox ROS Driver 2:

#### For ROS (take Noetic as an example):

```shell
source /opt/ros/noetic/setup.sh
./build.sh ROS1
```

#### For ROS2 Foxy:

```shell
source /opt/ros/foxy/setup.sh
./build.sh ROS2
```

#### For ROS2 Humble:

```shell
source /opt/ros/humble/setup.sh
./build.sh humble
```

### 2.4 Run Livox ROS Driver 2:

#### For ROS:

```shell
source ../../devel/setup.sh
roslaunch livox_ros_driver2 [launch file]
```

in which,

* **livox_ros_driver2** : is the ROS package name of Livox ROS Driver 2;
* **[launch file]** : is the ROS launch file you want to use; the 'launch_ROS1' folder contains several launch samples for your reference;

An rviz launch example for HAP LiDAR would be:

```shell
roslaunch livox_ros_driver2 rviz_HAP.launch
```

#### For ROS2:

```shell
source ../../install/setup.sh
ros2 launch livox_ros_driver2 [launch file]
```

in which,

* **[launch file]** : is the ROS2 launch file you want to use; the 'launch_ROS2' folder contains several launch samples for your reference.

A rviz launch example for HAP LiDAR would be:

```shell
ros2 launch livox_ros_driver2 rviz_HAP_launch.py
```

## 3. Launch file and livox_ros_driver2 internal parameter configuration instructions

### 3.1 Launch file configuration instructions

Launch files of ROS are in the "ws_livox/src/livox_ros_driver2/launch_ROS1" directory and launch files of ROS2 are in the "ws_livox/src/livox_ros_driver2/launch_ROS2" directory. Different launch files have different configuration parameter values and are used in different scenarios:

| launch file name          | Description                                                  |
| ------------------------- | ------------------------------------------------------------ |
| rviz_HAP.launch   | Connect to HAP LiDAR device<br>Publish pointcloud2 format  data<br>Autoload rviz |
| msg_HAP.launch     | Connect to HAP LiDAR device<br>Publish livox customized pointcloud data|
| rviz_MID360.launch        | Connect to MID360 LiDAR device<br>Publish pointcloud2 format data <br>Autoload rviz|
| msg_MID360.launch          | Connect to MID360 LiDAR device<br>Publish livox customized pointcloud data |
| rviz_mixed.launch    | Connect to HAP and MID360 LiDAR device<br>Publish pointcloud2 format data <br>Autoload rviz|
| msg_mixed.launch      | Connect to HAP and MID360 LiDAR device<br>Publish livox customized pointcloud data |

### 3.2 Livox ros driver 2 internal main parameter configuration instructions

All internal parameters of Livox_ros_driver2 are in the launch file. Below are detailed descriptions of the three commonly used parameters :

| Parameter    | Detailed description                                         | Default |
| ------------ | ------------------------------------------------------------ | ------- |
| publish_freq | Set the frequency of point cloud publish <br>Floating-point data type, recommended values 5.0, 10.0, 20.0, 50.0, etc. The maximum publish frequency is 100.0 Hz.| 10.0    |
| multi_topic  | If the LiDAR device has an independent topic to publish pointcloud data<br>0 -- All LiDAR devices use the same topic to publish pointcloud data<br>1 -- Each LiDAR device has its own topic to publish point cloud data | 0       |
| xfer_format  | Set pointcloud format<br>0 -- Livox pointcloud2(PointXYZRTLT) pointcloud format<br>1 -- Livox customized pointcloud format<br>2 -- Standard pointcloud2 (pcl :: PointXYZI) pointcloud format in the PCL library (just for ROS) | 0       |

  **Note :**

  Other parameters not mentioned in this table are not suggested to be changed unless fully understood.

&ensp;&ensp;&ensp;&ensp;***Livox_ros_driver2 pointcloud data detailed description :***

1. Livox pointcloud2 (PointXYZRTLT) point cloud format, as follows :

```c
float32 x               # X axis, unit:m
float32 y               # Y axis, unit:m
float32 z               # Z axis, unit:m
float32 intensity       # the value is reflectivity, 0.0~255.0
uint8   tag             # livox tag
uint8   line            # laser number in lidar
float64 timestamp       # Timestamp of point
```
  **Note :**

  The number of points in the frame may be different, but each point provides a timestamp.

2. Livox customized data package format, as follows :

```c
std_msgs/Header header     # ROS standard message header
uint64          timebase   # The time of first point
uint32          point_num  # Total number of pointclouds
uint8           lidar_id   # Lidar device id number
uint8[3]        rsvd       # Reserved use
CustomPoint[]   points     # Pointcloud data
```

&ensp;&ensp;&ensp;&ensp;Customized Point Cloud (CustomPoint) format in the above customized data package :

```c
uint32  offset_time     # offset time relative to the base time
float32 x               # X axis, unit:m
float32 y               # Y axis, unit:m
float32 z               # Z axis, unit:m
uint8   reflectivity    # reflectivity, 0~255
uint8   tag             # livox tag
uint8   line            # laser number in lidar
```

3. The standard pointcloud2 (pcl :: PointXYZI) format in the PCL library (only ROS can publish):

&ensp;&ensp;&ensp;&ensp;Please refer to the pcl :: PointXYZI data structure in the point_types.hpp file of the PCL library.

## 4. LiDAR config

LiDAR Configurations (such as ip, port, data type... etc.) can be set via a json-style config file. Config files for single HAP, Mid360 and mixed-LiDARs are in the "config" folder. The parameter naming *'user_config_path'* in launch files indicates such json file path.

1. Follow is a configuration example for HAP LiDAR (located in config/HAP_config.json):

```json
{
  "lidar_summary_info" : {
    "lidar_type": 8  # protocol type index, please don't revise this value
  },
  "HAP": {
    "device_type" : "HAP",
    "lidar_ipaddr": "",
    "lidar_net_info" : {
      "cmd_data_port": 56000,  # command port
      "push_msg_port": 0,
      "point_data_port": 57000,
      "imu_data_port": 58000,
      "log_data_port": 59000
    },
    "host_net_info" : {
      "cmd_data_ip" : "192.168.1.5",  # host ip (it can be revised)
      "cmd_data_port": 56000,
      "push_msg_ip": "",
      "push_msg_port": 0,
      "point_data_ip": "192.168.1.5",  # host ip
      "point_data_port": 57000,
      "imu_data_ip" : "192.168.1.5",  # host ip
      "imu_data_port": 58000,
      "log_data_ip" : "",
      "log_data_port": 59000
    }
  },
  "lidar_configs" : [
    {
      "ip" : "192.168.1.100",  # ip of the LiDAR you want to config
      "pcl_data_type" : 1,
      "pattern_mode" : 0,
      "blind_spot_set" : 50,
      "extrinsic_parameter" : {
        "roll": 0.0,
        "pitch": 0.0,
        "yaw": 0.0,
        "x": 0,
        "y": 0,
        "z": 0
      }
    }
  ]
}
```

The parameter attributes in the above json file are described in the following table :

**LiDAR configuration parameter**
| Parameter                  | Type    | Description                                                  | Default         |
| :------------------------- | ------- | ------------------------------------------------------------ | --------------- |
| ip             | String  | Ip of the LiDAR you want to config | 192.168.1.100 |
| pcl_data_type             | Int | Choose the resolution of the point cloud data to send<br>1 -- Cartesian coordinate data (32 bits)<br>2 -- Cartesian coordinate data (16 bits) <br>3 --Spherical coordinate data| 1           |
| pattern_mode                | Int     | Space scan pattern<br>0 -- non-repeating scanning pattern mode<br>1 -- repeating scanning pattern mode <br>2 -- repeating scanning pattern mode (low scanning rate) | 0               |
| blind_spot_set (Only for HAP LiDAR)                 | Int     | Set blind spot<br>Range from 50 cm to 200 cm               | 50               |
| extrinsic_parameter |      | Set extrinsic parameter<br> The data types of "roll" "picth" "yaw" are float <br>  The data types of "x" "y" "z" are int<br>               |

For more infomation about the HAP config, please refer to:
[HAP Config File Description](https://github.com/Livox-SDK/Livox-SDK2/wiki/hap-config-file-description)

2. When connecting multiple LiDARs, add objects corresponding to different LiDARs to the "lidar_configs" array. Examples of mixed-LiDARs config file contents are as follows :

```json
{
  "lidar_summary_info" : {
    "lidar_type": 8  # protocol type index, please don't revise this value
  },
  "HAP": {
    "lidar_net_info" : {  # HAP ports, please don't revise these values
      "cmd_data_port": 56000,  # HAP command port
      "push_msg_port": 0,
      "point_data_port": 57000,
      "imu_data_port": 58000,
      "log_data_port": 59000
    },
    "host_net_info" : {
      "cmd_data_ip" : "192.168.1.5",  # host ip
      "cmd_data_port": 56000,
      "push_msg_ip": "",
      "push_msg_port": 0,
      "point_data_ip": "192.168.1.5",  # host ip
      "point_data_port": 57000,
      "imu_data_ip" : "192.168.1.5",  # host ip
      "imu_data_port": 58000,
      "log_data_ip" : "",
      "log_data_port": 59000
    }
  },
  "MID360": {
    "lidar_net_info" : {  # Mid360 ports, please don't revise these values
      "cmd_data_port": 56100,  # Mid360 command port
      "push_msg_port": 56200,
      "point_data_port": 56300,
      "imu_data_port": 56400,
      "log_data_port": 56500
    },
    "host_net_info" : {
      "cmd_data_ip" : "192.168.1.5",  # host ip
      "cmd_data_port": 56101,
      "push_msg_ip": "192.168.1.5",  # host ip
      "push_msg_port": 56201,
      "point_data_ip": "192.168.1.5",  # host ip
      "point_data_port": 56301,
      "imu_data_ip" : "192.168.1.5",  # host ip
      "imu_data_port": 56401,
      "log_data_ip" : "",
      "log_data_port": 56501
    }
  },
  "lidar_configs" : [
    {
      "ip" : "192.168.1.100",  # ip of the HAP you want to config
      "pcl_data_type" : 1,
      "pattern_mode" : 0,
      "blind_spot_set" : 50,
      "extrinsic_parameter" : {
        "roll": 0.0,
        "pitch": 0.0,
        "yaw": 0.0,
        "x": 0,
        "y": 0,
        "z": 0
      }
    },
    {
      "ip" : "192.168.1.12",  # ip of the Mid360 you want to config
      "pcl_data_type" : 1,
      "pattern_mode" : 0,
      "extrinsic_parameter" : {
        "roll": 0.0,
        "pitch": 0.0,
        "yaw": 0.0,
        "x": 0,
        "y": 0,
        "z": 0
      }
    }
  ]
}
```
3. when multiple nics on the host connect to multiple LiDARs, you need to add objects corresponding to different LiDARs to the lidar_configs array. Run different luanch files separately, and the following is an example of mixing lidar configuration file contents:

**MID360_config1:**
```json
{
  "lidar_summary_info" : {
    "lidar_type": 8  # protocol type index，please don't revise this value
  },
    "MID360": {
        "lidar_net_info": {
            "cmd_data_port": 56100, # command port
            "push_msg_port": 56200,
            "point_data_port": 56300,
            "imu_data_port": 56400,
            "log_data_port": 56500
        },
        "host_net_info": [
            {
                "lidar_ip": ["192.168.1.100"], # Lidar ip
                "host_ip": "192.168.1.5", # host ip
                "cmd_data_port": 56101,
                "push_msg_port": 56201,
                "point_data_port": 56301,
                "imu_data_port": 56401,
                "log_data_port": 56501
            }
        ]
    },
    "lidar_configs": [
        {
            "ip": "192.168.1.100", # ip of the LiDAR you want to config
            "pcl_data_type": 1,
            "pattern_mode": 0,
            "extrinsic_parameter": {
                "roll": 0.0,
                "pitch": 0.0,
                "yaw": 0.0,
                "x": 0,
                "y": 0,
                "z": 0
            }
        }
    ]
}
```
**MID360_config2:**
```json
{
  "lidar_summary_info" : {
    "lidar_type": 8  # protocol type index，please don't revise this value
  },
    "MID360": {
        "lidar_net_info": {
            "cmd_data_port": 56100, # command port
            "push_msg_port": 56200,
            "point_data_port": 56300,
            "imu_data_port": 56400,
            "log_data_port": 56500
        },
        "host_net_info": [
            {
                "lidar_ip": ["192.168.2.100"], # Lidar ip
                "host_ip": "192.168.2.5", # host ip
                "cmd_data_port": 56101,
                "push_msg_port": 56201,
                "point_data_port": 56301,
                "imu_data_port": 56401,
                "log_data_port": 56501
            }
        ]
    },
    "lidar_configs": [
        {
            "ip": "192.168.2.100", # ip of the LiDAR you want to config
            "pcl_data_type": 1,
            "pattern_mode": 0,
            "extrinsic_parameter": {
                "roll": 0.0,
                "pitch": 0.0,
                "yaw": 0.0,
                "x": 0,
                "y": 0,
                "z": 0
            }
        }
    ]
}
```
**Launch1:**
```
<launch>
    <!--user configure parameters for ros start-->
    <arg name="lvx_file_path" default="livox_test.lvx"/>
    <arg name="bd_list" default="100000000000000"/>
    <arg name="xfer_format" default="0"/>
    <arg name="multi_topic" default="1"/>
    <arg name="data_src" default="0"/>
    <arg name="publish_freq" default="10.0"/>
    <arg name="output_type" default="0"/>
    <arg name="rviz_enable" default="true"/>
    <arg name="rosbag_enable" default="false"/>
    <arg name="cmdline_arg" default="$(arg bd_list)"/>
    <arg name="msg_frame_id" default="livox_frame"/>
    <arg name="lidar_bag" default="true"/>
    <arg name="imu_bag" default="true"/>
    <!--user configure parameters for ros end-->

    <param name="xfer_format" value="$(arg xfer_format)"/>
    <param name="multi_topic" value="$(arg multi_topic)"/>
    <param name="data_src" value="$(arg data_src)"/>
    <param name="publish_freq" type="double" value="$(arg publish_freq)"/>
    <param name="output_data_type" value="$(arg output_type)"/>
    <param name="cmdline_str" type="string" value="$(arg bd_list)"/>
    <param name="cmdline_file_path" type="string" value="$(arg lvx_file_path)"/>
    <param name="user_config_path" type="string" value="$(find livox_ros_driver2)/config/MID360_config1.json"/> # Mid360 MID360_config1 name
    <param name="frame_id" type="string" value="$(arg msg_frame_id)"/>
    <param name="enable_lidar_bag" type="bool" value="$(arg lidar_bag)"/>
    <param name="enable_imu_bag" type="bool" value="$(arg imu_bag)"/>

    <node name="livox_lidar_publisher1" pkg="livox_ros_driver2"
          type="livox_ros_driver2_node" required="true"
          output="screen" args="$(arg cmdline_arg)"/>

    <group if="$(arg rviz_enable)">
        <node name="livox_rviz" pkg="rviz" type="rviz" respawn="true"
                args="-d $(find livox_ros_driver2)/config/display_point_cloud_ROS1.rviz"/>
    </group>

    <group if="$(arg rosbag_enable)">
        <node pkg="rosbag" type="record" name="record" output="screen"
                args="-a"/>
    </group>

</launch>
```
**Launch2:**
```
<launch>
    <!--user configure parameters for ros start-->
    <arg name="lvx_file_path" default="livox_test.lvx"/>
    <arg name="bd_list" default="100000000000000"/>
    <arg name="xfer_format" default="0"/>
    <arg name="multi_topic" default="1"/>
    <arg name="data_src" default="0"/>
    <arg name="publish_freq" default="10.0"/>
    <arg name="output_type" default="0"/>
    <arg name="rviz_enable" default="true"/>
    <arg name="rosbag_enable" default="false"/>
    <arg name="cmdline_arg" default="$(arg bd_list)"/>
    <arg name="msg_frame_id" default="livox_frame"/>
    <arg name="lidar_bag" default="true"/>
    <arg name="imu_bag" default="true"/>
    <!--user configure parameters for ros end-->

    <param name="xfer_format" value="$(arg xfer_format)"/>
    <param name="multi_topic" value="$(arg multi_topic)"/>
    <param name="data_src" value="$(arg data_src)"/>
    <param name="publish_freq" type="double" value="$(arg publish_freq)"/>
    <param name="output_data_type" value="$(arg output_type)"/>
    <param name="cmdline_str" type="string" value="$(arg bd_list)"/>
    <param name="cmdline_file_path" type="string" value="$(arg lvx_file_path)"/>
    <param name="user_config_path" type="string" value="$(find livox_ros_driver2)/config/MID360_config2.json"/> # Mid360 MID360_config2 name
    <param name="frame_id" type="string" value="$(arg msg_frame_id)"/>
    <param name="enable_lidar_bag" type="bool" value="$(arg lidar_bag)"/>
    <param name="enable_imu_bag" type="bool" value="$(arg imu_bag)"/>

    <node name="livox_lidar_publisher2" pkg="livox_ros_driver2"
          type="livox_ros_driver2_node" required="true"
          output="screen" args="$(arg cmdline_arg)"/>

    <group if="$(arg rviz_enable)">
        <node name="livox_rviz" pkg="rviz" type="rviz" respawn="true"
                args="-d $(find livox_ros_driver2)/config/display_point_cloud_ROS1.rviz"/>
    </group>

    <group if="$(arg rosbag_enable)">
        <node pkg="rosbag" type="record" name="record" output="screen"
                args="-a"/>
    </group>

</launch>

```

## 5. Supported LiDAR list

* HAP
* Mid360
* (more types are comming soon...)

## 6. FAQ

### 6.1 launch with "livox_lidar_rviz_HAP.launch" but no point cloud display on the grid?

Please check the "Global Options - Fixed Frame" field in the RViz "Display" pannel. Set the field value to "livox_frame" and check the "PointCloud2" option in the pannel.

### 6.2 launch with command "ros2 launch livox_lidar_rviz_HAP_launch.py" but cannot open shared object file "liblivox_sdk_shared.so" ?

Please add '/usr/local/lib' to the env LD_LIBRARY_PATH.

* If you want to add to current terminal:

  ```shell
  export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/lib
  ```

* If you want to add to current user:

  ```shell
  vim ~/.bashrc
  export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/lib
  source ~/.bashrc
  ```
