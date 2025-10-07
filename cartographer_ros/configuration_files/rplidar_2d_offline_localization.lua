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

-- Pure localization configuration for RPLIDAR S3 using offline processing
-- This configuration uses rplidar_2d_offline as base and enables pure localization

include "rplidar_2d_offline.lua"

-- Enable pure localization mode
TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 3,
}

-- Optimize more frequently in localization mode for better tracking
POSE_GRAPH.optimize_every_n_nodes = 20

-- Increase constraints for better localization
POSE_GRAPH.constraint_builder.sampling_ratio = 0.5
POSE_GRAPH.constraint_builder.max_constraint_distance = 30.

-- More aggressive loop closure detection for localization
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.55
POSE_GRAPH.constraint_builder.min_score = 0.5

-- Wider search window for initial localization
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 10.
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(45.)

-- Disable global SLAM for pure localization
POSE_GRAPH.global_sampling_ratio = 0.003
POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 1.1e4
POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = 1e5

-- Motion filter for localization stability
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 2.
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.05
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.5)

-- Higher weight on scan matching for better localization accuracy
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 20.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 5.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 5.

return options