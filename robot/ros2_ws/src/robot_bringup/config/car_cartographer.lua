-- Cartographer for the proof-of-concept RC car: 2D, lidar only.
--
-- The car has no wheel sensors and no IMU, so every bit of motion is worked
-- out by matching each scan against the map so far. That is why online
-- correlative scan matching is on: without odometry the pose extrapolator
-- alone guesses badly, and the search window is what finds the car again.
--
-- Tuned for the LD14P (about 6 scans a second, 8 m range) and slow driving,
-- 0.3 m/s or less: at that speed the car moves about 5 cm between scans,
-- well inside the 15 cm search window. Drive faster, or turn quickly, and
-- the map smears. Started from Cartographer's own revo_lds.lua, the example
-- for a similar cheap spinning lidar.
--
-- Used only by car_mapping.launch.py. Nothing here feeds the real robot's
-- safety distances.

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "base_link",
  -- Nothing else publishes odom on the car, so Cartographer provides it.
  odom_frame = "odom",
  provide_odom_frame = true,
  -- The car rocks on bumps; keep the published pose flat.
  publish_frame_projected_to_2d = true,
  use_pose_extrapolator = true,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  -- 50 Hz rather than the examples' 200 Hz: the pose goes over Wi-Fi to
  -- Foxglove, and a Pi has little to spare.
  pose_publish_period_sec = 20e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D.use_imu_data = false
-- Every scan is one full turn of the lidar; at 6 Hz there are none to spare.
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1
-- Returns closer than this are the car itself, its mount or the tether.
TRAJECTORY_BUILDER_2D.min_range = 0.15
TRAJECTORY_BUILDER_2D.max_range = 8.
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 1.
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.15
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(20.)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1
-- Fewer scans per submap than the default 90: at 6 Hz, 90 would be a long
-- stretch of drift before a submap is finished and can be matched against.
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 35

POSE_GRAPH.optimize_every_n_nodes = 35
POSE_GRAPH.optimization_problem.huber_scale = 1e2
POSE_GRAPH.constraint_builder.min_score = 0.65

return options
