# LD14P Mapping in Foxglove

This workflow builds a live 2D occupancy map from the LDROBOT LD14P scan and
shows that map, the scan, and the robot frames in Foxglove's 3D panel. The
LD14P is a planar lidar: the scene is viewed in 3D, but the map itself is not a
3D/voxel reconstruction.

The mapping launch is observational. It starts the sensor driver, laser-based
odometry, SLAM Toolbox, and Foxglove Bridge; it does not command robot motion.
Use only a separately validated, supervised motion controller to move the
platform. The committed URDF geometry and sensor mounting pose are placeholders
and must be measured and updated for the physical robot before hardware use.

## Install Dependencies

On the ROS 2 Jazzy robot computer, source Jazzy and install the Foxglove bridge:

```bash
sudo apt install ros-jazzy-foxglove-bridge
```

Add the upstream ROS 2 LDROBOT driver and RF2O laser-odometry package to
`robot/ros2_ws/src`:

```bash
cd robot/ros2_ws/src
git clone https://github.com/ldrobotSensorTeam/ldlidar_sl_ros2.git
git clone --branch humble-devel https://github.com/Adlink-ROS/rf2o_laser_odometry.git
```

Then install package dependencies and build the workspace:

```bash
cd robot/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

The LDROBOT driver supports LD14P and publishes `sensor_msgs/msg/LaserScan` on
`/scan`. The launch uses its 230400 baud setting and `lidar_link` frame. RF2O
estimates `odom -> base_footprint` from successive scans because this repository
does not yet include physical wheel odometry.

## Build A Map

Connect the sensor, verify its serial device, then launch the mapping stack:

```bash
ros2 launch robot_bringup ld14p_mapping.launch.py serial_port:=/dev/ttyUSB0
```

In Foxglove Desktop, connect to `ws://localhost:8765` and add a 3D panel. Set
the fixed frame to `map`, then enable `/map` (`nav_msgs/msg/OccupancyGrid`),
`/scan` (`sensor_msgs/msg/LaserScan`), and `/tf` plus `/tf_static`. Optionally
enable `/robot_description` to show the URDF model. The map is accumulated by
SLAM as scans arrive and the platform moves. Save this panel arrangement as a
Foxglove layout for the presentation.

Drive only with the robot's separately validated controller while watching the
live map. Cover the presentation area slowly, including revisiting the start
area for loop closure. Save a durable occupancy map from the ROS workspace:

```bash
ros2 run nav2_map_server map_saver_cli -f /tmp/ld14p_presentation_map
```

This creates `/tmp/ld14p_presentation_map.yaml` and
`/tmp/ld14p_presentation_map.pgm`. Keep both files together to reopen or share
the captured map. To retain source sensor data for replay, record a rosbag in a
second terminal during the survey:

```bash
ros2 bag record -o ~/ld14p_mapping_run /scan /tf /tf_static /odom_rf2o /map
```

## Network Access

The bridge binds to `127.0.0.1` by default. For Foxglove on another computer on
a trusted network, set `foxglove_address:=0.0.0.0` and connect to
`ws://<robot-ip>:8765`; restrict access with the host firewall and do not expose
the unauthenticated bridge to an untrusted network.