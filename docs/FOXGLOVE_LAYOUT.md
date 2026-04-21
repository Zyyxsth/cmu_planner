# Foxglove Layout For D1H + ODIN Navigation

This note maps the current RViz setup to Foxglove so the same navigation stack can be inspected without relying on RViz.

## Goal

Use Foxglove Desktop to observe the real-robot route-planner stack on a remote machine through `foxglove_bridge`.

Robot-side bridge:

```bash
source /opt/ros/humble/setup.bash
source /opt/d1_ros2/local_setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

Foxglove Desktop connection URL:

```text
ws://10.1.1.36:8765
```

## Fixed Frame

Use `map` as the 3D fixed frame.

That matches the current planner stack:

- `/registered_scan` uses `frame_id=map`
- `/state_estimation` is the bridged odometry for planning
- planner visualization topics are also in `map`

## RViz To Foxglove Mapping

The current RViz config is:

- [vehicle_simulator.rviz](/home/robot/cmu_planner/src/base_autonomy/vehicle_simulator/rviz/vehicle_simulator.rviz)

Recommended Foxglove mapping:

| RViz item | Topic | Foxglove panel |
| --- | --- | --- |
| Vehicle axes | `TF` / `tf_static` | `3D` |
| RegScan | `/registered_scan` | `3D` |
| TerrainMap | `/terrain_map` | `3D` |
| TerrainMapExt | `/terrain_map_ext` | `3D` |
| Path | `/path` | `3D` |
| FreePaths | `/free_paths` | `3D` |
| Waypoint | `/way_point` | `3D` or `Raw Messages` |
| Boundary | `/navigation_boundary` | `3D` |
| OverallMap | `/overall_map` | `3D` |
| ExploredAreas | `/explored_areas` | `3D` |
| Trajectory | `/trajectory` | `3D` |
| VGraph | `/viz_graph_topic` | `3D` |
| Graph decoder markers | `/graph_decoder_viz` | `3D` |
| Image | `/camera/image` | `Image` |
| SemanticImage | `/camera/semantic_image` | `Image` |
| State estimation | `/state_estimation` | `Raw Messages`, `Plot`, `3D` |
| Goal point command | `/goal_point` | `Raw Messages` |

## Recommended First Layout

Start simple. Do not load every heavy topic at once.

### Panel 1: 3D

Add these topics first:

- `TF`
- `/registered_scan`
- `/path`
- `/way_point`
- `/viz_graph_topic`

Then optionally add:

- `/terrain_map`
- `/terrain_map_ext`
- `/overall_map`
- `/free_paths`
- `/graph_decoder_viz`

### Panel 2: Raw Messages

Watch:

- `/state_estimation`
- `/goal_point`
- `/far_reach_goal_status`
- `/d15020108/rl_controller/fsm`

### Panel 3: Plot

Useful signals:

- `/state_estimation.pose.pose.position.x`
- `/state_estimation.pose.pose.position.y`
- `/state_estimation.twist.twist.linear.x`
- `/state_estimation.twist.twist.angular.z`

### Panel 4: Image

Optional:

- `/camera/image`
- `/camera/semantic_image`

## Suggested Bringup Order

### 1. Robot navigation stack

```bash
cd /home/robot/cmu_planner
source ./source_workspace_setup.bash
ros2 launch vehicle_simulator system_real_robot_with_route_planner_d1.launch.py
```

### 2. Foxglove bridge

```bash
source /opt/ros/humble/setup.bash
source /opt/d1_ros2/local_setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

### 3. Foxglove Desktop on remote machine

Connect to:

```text
ws://10.1.1.36:8765
```

## What To Prefer In Foxglove

For a stable remote view, prioritize these topics:

- `/state_estimation`
- `/path`
- `/goal_point`
- `/viz_graph_topic`
- `/graph_decoder_viz`

Only add dense point clouds after the basic layout is stable:

- `/registered_scan`
- `/terrain_map`
- `/terrain_map_ext`
- `/overall_map`

## Exploration Debug Topics

For the current real-robot exploration workflow, the reliable exploration-point topics are:

- `/viewpoint_vis_cloud`
- `/selected_viewpoint_vis_cloud`
- `/lookahead_point_cloud`
- `/uncovered_cloud`
- `/uncovered_frontier_cloud`

Do not treat these as the primary Foxglove exploration-point layers for now:

- `/tare_visualizer/viewpoints`
- `/tare_visualizer/viewpoint_candidates`
- `/tare_visualizer/uncovered_surface_points`

Those `tare_visualizer/*` point-cloud topics may exist in the graph, but on the current stack they are not the dependable source for the viewpoint / selected-viewpoint / lookahead visualization you see in RViz.

## Current Limitation

Remote point cloud visualization can still be limited by Wi-Fi packet loss.

Important observation from local measurement:

- `/registered_scan` is normal on the robot side at about `10 Hz`
- `/viz_graph_topic` is normal on the robot side at about `5 Hz`

So if Foxglove or RViz shows flashing point clouds on a remote machine, the first suspect is the network path, not the local ODIN-to-planner bridge.

## Practical Note

Foxglove is a good replacement for remote monitoring, but for now it should be treated primarily as a visualization frontend.

For sending goals and control commands, the most reliable method is still ROS CLI or existing control scripts unless we later add a dedicated Foxglove publishing workflow.
