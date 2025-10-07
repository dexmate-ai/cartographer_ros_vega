-- Copyright 2025 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "laser_link",
  published_frame = "laser_link",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = true,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 0,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 1,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,

  publish_tracked_pose = true,
}

-- Enable 2D mapping
MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 8  -- Use more threads for offline processing

-- ==== HIGH-QUALITY OFFLINE FRONTEND SETTINGS ====

-- Basic sensor settings for slamtec s3
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.min_range = 0.1    -- Stricter minimum range filtering
TRAJECTORY_BUILDER_2D.max_range = 30.   -- slamtec s3 maximum range

-- Data accumulation - maintain single frame processing for highest accuracy
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1

-- Fine-grained filtering for high quality
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.015              -- Finer voxel filtering, preserve more details
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 0.25  -- Smaller maximum voxel size
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 350  -- Require more points to ensure quality
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_range = 80.

-- Loop closure filtering - preserve more information for loop detection
TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.max_length = 0.6
TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.min_num_points = 150
TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.max_range = 80.

-- Enable correlative scan matching for better accuracy
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.2
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(30.)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 1e-1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1

-- Enhanced Ceres scan matcher for offline processing
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 20.   -- Increase occupied space weight
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 5.      -- Increase translation constraint
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 1.         -- Increase rotation constraint
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 100  -- Allow more iterations for offline
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.use_nonmonotonic_steps = true

-- Stricter motion filter for denser mapping
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 2.0     -- More frequent data processing
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.03 -- Smaller movement threshold, capture more details
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.5)  -- Smaller angle threshold

-- High-quality submap settings
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 150            -- Larger submaps for better constraints
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.03  -- High-resolution grid map

-- Enhanced probability grid settings
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.hit_probability = 0.57
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.miss_probability = 0.47
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.insert_free_space = true

-- ==== HIGH-QUALITY OFFLINE BACKEND SETTINGS ====

-- More frequent optimization for offline processing
POSE_GRAPH.optimize_every_n_nodes = 25    -- More frequent backend optimization

-- Enhanced constraint building for better loop closure
POSE_GRAPH.constraint_builder.sampling_ratio = 1.0             -- Maximum sampling rate, check all possible constraints
POSE_GRAPH.constraint_builder.max_constraint_distance = 25.    -- Expand constraint search range
POSE_GRAPH.constraint_builder.min_score = 0.4                 -- Slightly lower threshold to get more constraints
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.55
POSE_GRAPH.constraint_builder.log_matches = true              -- Log constraint matching information

-- Enhanced fast correlative scan matcher for constraint building
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 10.
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(45.)
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.branch_and_bound_depth = 8

-- Enhanced Ceres scan matcher for constraint refinement
POSE_GRAPH.constraint_builder.ceres_scan_matcher.occupied_space_weight = 25.
POSE_GRAPH.constraint_builder.ceres_scan_matcher.translation_weight = 5.
POSE_GRAPH.constraint_builder.ceres_scan_matcher.rotation_weight = 5.
POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 20
POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.use_nonmonotonic_steps = true

-- -- High-quality optimization settings
-- POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 1e5
-- POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 1e5
-- POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e5
-- POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e5
-- POSE_GRAPH.optimization_problem.huber_scale = 5e0              -- Smaller Huber scale to reduce outlier influence

-- Extensive final optimization for offline processing
POSE_GRAPH.max_num_final_iterations = 1000                    -- Extensive final optimization iterations
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 150
POSE_GRAPH.optimization_problem.ceres_solver_options.num_threads = 8  -- Multi-threaded optimization
POSE_GRAPH.optimization_problem.ceres_solver_options.use_nonmonotonic_steps = false

-- Enhanced global constraint search for offline scenarios
POSE_GRAPH.global_sampling_ratio = 0.01                       -- Increase global sampling rate
POSE_GRAPH.global_constraint_search_after_n_seconds = 5.      -- More frequent global constraint search
return options