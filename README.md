# Cartographer ROS for Vega-1 RPLIDAR

This repository is a customized version of [Google Cartographer ROS2](https://github.com/ros2/cartographer_ros.git), specifically adapted and configured for the **Dexmate Vega-1** equipped with **RPLIDAR** as the 2D LiDAR sensor.

### What is Cartographer?

Cartographer is a real-time simultaneous localization and mapping (SLAM) library developed by Google. It provides:
- 2D and 3D SLAM capabilities
- Real-time loop closure detection
- Pure localization mode against pre-built maps
- Submap-based mapping approach for large-scale environments

### Modifications for Vega-1

While the core Cartographer algorithm remains unchanged, this repository includes several Vega-1 specific customizations:

- **RPLIDAR-optimized configurations**: Tuned Lua configuration files (`rplidar_2d_*.lua`) for online SLAM, offline mapping, and localization modes
- **Vega-1 URDF models**: Robot description files (`vega_dummy.urdf`, `vega_full.urdf`) defining the robot's frame structure and sensor placements
- **Custom launch files**: ROS2 launch scripts (`vega_2d*.launch.py`) pre-configured for the Vega-1 robot platform
- **Topic mappings**: Default topic names and transforms aligned with Vega-1's sensor suite and coordinate frames
- **Workflow optimizations**: Streamlined workflows for mapping and localization specific to the Vega-1 deployment scenarios

These modifications ensure out-of-the-box compatibility with the Vega-1 robot's hardware and software stack, eliminating the need for manual configuration when deploying Cartographer SLAM.

## Setup

Clone the repositories:
```bash
mkdir -p carto_ws/src && cd carto_ws/src
git clone https://github.com/ros2/cartographer.git
git clone https://github.com/dexmate-ai/cartographer_ros_vega.git
```

Install dependencies:
```bash
sudo apt install libabsl-dev libceres-dev libcairo2-dev lua5.3 liblua5.3-dev protobuf-compiler libprotobuf-dev
```

Build with `colcon`:
```bash
cd carto_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Usage

To use the system with Vega-1, the following data streams are required:
1. `/scan_pcd` → PointCloud2 for raw scan data
2. `/odom` → Odometry (optional)

Odometry improves point cloud undistortion and increases odometry frequency. Plesae refer to [dexcontrol-rosbridge](https://github.com/dexmate-ai/dexcontrol-rosbridge) for how to publish a wheel odometry from dexmate robots.

**Note:** IMU integration is not tuned, and the URDF extrinsic calibration for IMU uses placeholder values.

Visualization:
```bash
rviz2 -d src/cartographer_ros/cartographer_ros/configuration_files/demo_2d.rviz
```

### Online SLAM

Parameters: [rplidar_2d_online.lua](./cartographer_ros/configuration_files/rplidar_2d_online.lua)

Run:
```bash
ros2 launch cartographer_ros vega_2d.launch.py
```

Move the robot around. A successful run produces results like this:

![2D_online_demo](./images/online_slam_demo.gif)

The registered point cloud (in the map frame) is published on `/registered_scan`. State estimations are updated in the `tf_tree`. See the rviz2 panels for details.

### Localization

**Step 1: Record a bag file**
When topics are available, record the point cloud (and optionally odometry).
**Note:** The Cartographer offline node only accepts specific topic names (e.g., `/points2`). Relaying your topic may be required.

```bash
# optional: if the published topic is not the standard name
ros2 run topic_tools relay /scan_pcd /points2

ros2 bag record /points2
```

Tips: Move slowly and steadily; avoid rapid rotations.

**Step 2: Run offline mapping**
Parameters: [rplidar_2d_offline.lua](./cartographer_ros/configuration_files/rplidar_2d_offline.lua)

```bash
ros2 launch cartographer_ros offline_vega_2d.launch.py bag_filenames:=/path/to/mapping_data/mapping_data_0.mcap
```

This generates a `[bag_name].pbstream` file in the same directory as your bag file.

**Step 3: Online localization**
Parameters: [rplidar_2d_localization.lua](./cartographer_ros/configuration_files/rplidar_2d_localization.lua)

Run:
```bash
ros2 launch cartographer_ros vega_2d_localization.launch.py load_state_filename:=/path/to/your/pbstream
```

Example of a successful run (robot localizes after ~30s):
![2D_localization](./images/offline_localization.gif)

Alternatively, [use this ros2 bag for testing localization](./data/bag_localization.mcap). This was recorded in the same area as the mapping bag above.

For adjusting SLAM behavior (e.g., map size, voxel size, or re-localization speed), see the [tuning guide](https://google-cartographer-ros.readthedocs.io/en/latest/tuning.html).

## Acknowledgement
This repository is based on the following open source code bases:
- [Cartographer ROS2](https://github.com/ros2/cartographer_ros.git)
- [Cartographer](https://github.com/ros2/cartographer.git)
