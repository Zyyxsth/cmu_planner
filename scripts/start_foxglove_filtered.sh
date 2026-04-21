#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
source /opt/d1_ros2/local_setup.bash

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

# Keep Foxglove focused on the upper autonomy stack.
# Intentionally exclude /tf and /tf_static to avoid a huge transform tree full
# of unrelated vendor robot frames in the remote UI.
TOPIC_WHITELIST="[
  '^/tf$',
  '^/tf_static$',
  '^/terrain_map$',
  '^/terrain_map_ext$',
  '^/path$',
  '^/way_point$',
  '^/exploration_path$',
  '^/global_path$',
  '^/local_path$',
  '^/exploration_finish$',
  '^/runtime$',
  '^/runtime_breakdown$',
  '^/viz_graph_topic$',
  '^/viz_contour_topic$',
  '^/viz_poly_topic$',
  '^/navigation_boundary$',
  '^/goal_point$',
  '^/state_estimation$',
  '^/state_estimation_at_scan$',
  '^/tare_visualizer/marker$',
  '^/tare_visualizer/exploring_subspaces$',
  '^/tare_visualizer/local_path$',
  '^/tare_visualizer/local_planning_horizon$',
  '^/tare_visualizer/uncovered_surface_points$',
  '^/tare_visualizer/viewpoints$',
  '^/tare_visualizer/viewpoint_candidates$',
  '^/viewpoint_vis_cloud$',
  '^/selected_viewpoint_vis_cloud$',
  '^/lookahead_point_cloud$',
  '^/keypose_graph_edge_marker$',
  '^/grid_world_marker$',
  '^/uncovered_cloud$',
  '^/uncovered_frontier_cloud$',
  '^/frontier_cloud$',
  '^/filtered_frontier_cloud$',
  '^/d15020108/rl_controller/fsm$',
  '^/far_reach_goal_status$'
]"

exec ros2 launch foxglove_bridge foxglove_bridge_launch.xml \
  topic_whitelist:="${TOPIC_WHITELIST}" \
  publish_client_count:="false"
