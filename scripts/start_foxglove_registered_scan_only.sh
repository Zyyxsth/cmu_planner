#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
source /opt/d1_ros2/local_setup.bash

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

# Ultra-light Foxglove bridge:
# only expose the bridged ODIN scan used for remote point cloud inspection.
# /registered_scan already uses frame_id=map, so this can work without /tf.
TOPIC_WHITELIST="[
  '^/registered_scan$'
]"

exec ros2 launch foxglove_bridge foxglove_bridge_launch.xml \
  topic_whitelist:="${TOPIC_WHITELIST}" \
  publish_client_count:="false"
