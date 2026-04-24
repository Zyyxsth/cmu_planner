import json
import os
import re
import shlex
import tempfile
import xml.etree.ElementTree as ET
from collections import deque
from math import asin, atan2, cos, sin

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import FrontendLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def _rpy_to_matrix(roll, pitch, yaw):
  cr = cos(roll)
  sr = sin(roll)
  cp = cos(pitch)
  sp = sin(pitch)
  cy = cos(yaw)
  sy = sin(yaw)
  return [
      [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
      [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
      [-sp, cp * sr, cp * cr],
  ]


def _matmul3(a, b):
  return [
      [
          a[row][0] * b[0][col] + a[row][1] * b[1][col] + a[row][2] * b[2][col]
          for col in range(3)
      ]
      for row in range(3)
  ]


def _rotate_vector(rotation, vector):
  return [
      rotation[0][0] * vector[0] + rotation[0][1] * vector[1] + rotation[0][2] * vector[2],
      rotation[1][0] * vector[0] + rotation[1][1] * vector[1] + rotation[1][2] * vector[2],
      rotation[2][0] * vector[0] + rotation[2][1] * vector[1] + rotation[2][2] * vector[2],
  ]


def _compose_pose(parent, xyz, rpy):
  local_rotation = _rpy_to_matrix(*rpy)
  translated = _rotate_vector(parent["rotation"], list(xyz))
  return {
      "rotation": _matmul3(parent["rotation"], local_rotation),
      "translation": [
          parent["translation"][0] + translated[0],
          parent["translation"][1] + translated[1],
          parent["translation"][2] + translated[2],
      ],
  }


def _axis_angle_to_matrix(axis, angle):
  x, y, z = axis
  norm = (x * x + y * y + z * z) ** 0.5
  if norm <= 1e-9:
    return _rpy_to_matrix(0.0, 0.0, 0.0)
  x /= norm
  y /= norm
  z /= norm
  c = cos(angle)
  s = sin(angle)
  one_c = 1.0 - c
  return [
      [c + x * x * one_c, x * y * one_c - z * s, x * z * one_c + y * s],
      [y * x * one_c + z * s, c + y * y * one_c, y * z * one_c - x * s],
      [z * x * one_c - y * s, z * y * one_c + x * s, c + z * z * one_c],
  ]


def _matrix_to_rpy(rotation):
  pitch = asin(-max(-1.0, min(1.0, rotation[2][0])))
  roll = atan2(rotation[2][1], rotation[2][2])
  yaw = atan2(rotation[1][0], rotation[0][0])
  return roll, pitch, yaw


def _parse_xyz_rpy(element):
  if element is None:
    return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]
  xyz = [float(value) for value in element.attrib.get("xyz", "0 0 0").split()]
  rpy = [float(value) for value in element.attrib.get("rpy", "0 0 0").split()]
  return xyz, rpy


def _read_mesh_aabb_corners(path):
  with open(path, "rb") as handle:
    data = handle.read()

  mins = [float("inf"), float("inf"), float("inf")]
  maxs = [float("-inf"), float("-inf"), float("-inf")]

  def update(x, y, z):
    mins[0] = min(mins[0], x)
    mins[1] = min(mins[1], y)
    mins[2] = min(mins[2], z)
    maxs[0] = max(maxs[0], x)
    maxs[1] = max(maxs[1], y)
    maxs[2] = max(maxs[2], z)

  if len(data) >= 84:
    triangle_count = int.from_bytes(data[80:84], byteorder="little", signed=False)
    if 84 + triangle_count * 50 == len(data):
      import struct

      offset = 84
      for _ in range(triangle_count):
        offset += 12
        for _ in range(3):
          x, y, z = struct.unpack("<fff", data[offset:offset + 12])
          update(x, y, z)
          offset += 12
        offset += 2
    else:
      for line in data.decode("utf-8", errors="ignore").splitlines():
        parts = line.strip().split()
        if len(parts) == 4 and parts[0].lower() == "vertex":
          update(float(parts[1]), float(parts[2]), float(parts[3]))

  return [
      [x, y, z]
      for x in (mins[0], maxs[0])
      for y in (mins[1], maxs[1])
      for z in (mins[2], maxs[2])
  ]


def _load_d1_visuals(package_share):
  urdf_path = os.path.join(package_share, "D1", "urdf", "D1_description.urdf")
  mesh_dir = os.path.join(package_share, "D1", "meshes")
  root = ET.parse(urdf_path).getroot()

  joint_defaults = [
      (re.compile(r".*_hip_joint$"), 0.0),
      (re.compile(r".*_thigh_joint$"), 0.4),
      (re.compile(r".*_calf_joint$"), -1.2),
      (re.compile(r".*_foot_joint$"), 0.0),
  ]

  def joint_default_angle(joint_name):
    for pattern, value in joint_defaults:
      if pattern.match(joint_name):
        return value
    return 0.0

  links = {link.attrib["name"]: link for link in root.findall("link")}
  children = {}
  for joint in root.findall("joint"):
    parent = joint.find("parent").attrib["link"]
    child = joint.find("child").attrib["link"]
    xyz, rpy = _parse_xyz_rpy(joint.find("origin"))
    axis = [0.0, 0.0, 1.0]
    if joint.find("axis") is not None:
      axis = [float(value) for value in joint.find("axis").attrib.get("xyz", "0 0 1").split()]
    children.setdefault(parent, []).append(
        {
            "joint_name": joint.attrib["name"],
            "child": child,
            "xyz": xyz,
            "rpy": rpy,
            "axis": axis,
            "angle": joint_default_angle(joint.attrib["name"]),
        })

  base_pose = {
      "rotation": _rpy_to_matrix(0.0, 0.0, 0.0),
      "translation": [0.0, 0.0, 0.0],
  }
  link_poses = {"base_link": base_pose}
  queue = deque(["base_link"])
  while queue:
    parent = queue.popleft()
    for child in children.get(parent, []):
      child_pose = _compose_pose(link_poses[parent], child["xyz"], child["rpy"])
      child_pose["rotation"] = _matmul3(
          child_pose["rotation"], _axis_angle_to_matrix(child["axis"], child["angle"]))
      link_poses[child["child"]] = child_pose
      queue.append(child["child"])

  visuals = []
  scene_min_z = float("inf")
  for link_name, pose in link_poses.items():
    link = links.get(link_name)
    if link is None:
      continue
    visual = link.find("visual")
    if visual is None:
      continue
    mesh = visual.find("./geometry/mesh")
    if mesh is None:
      continue

    visual_xyz, visual_rpy = _parse_xyz_rpy(visual.find("origin"))
    visual_pose = _compose_pose(pose, visual_xyz, visual_rpy)
    roll, pitch, yaw = _matrix_to_rpy(visual_pose["rotation"])
    mesh_uri = os.path.join(mesh_dir, os.path.basename(mesh.attrib["filename"]))
    visuals.append(
        {
            "name": link_name,
            "mesh_uri": mesh_uri,
            "translation": visual_pose["translation"],
            "rpy": [roll, pitch, yaw],
        })

    for corner in _read_mesh_aabb_corners(mesh_uri):
      point = _rotate_vector(visual_pose["rotation"], corner)
      point = [
          point[0] + visual_pose["translation"][0],
          point[1] + visual_pose["translation"][1],
          point[2] + visual_pose["translation"][2],
      ]
      scene_min_z = min(scene_min_z, point[2])

  # Unity simulation publishes sensor-frame odometry with vehicleHeight ~= 0.75 m.
  # The Gazebo model origin is aligned with that sensor frame, so the visual shell
  # must extend down close to ground contact instead of hanging below the sensor by
  # only a small amount.
  visual_offset_z = -0.69 - scene_min_z

  visual_lines = []
  for visual in visuals:
    translation = [
        visual["translation"][0],
        visual["translation"][1],
        visual["translation"][2] + visual_offset_z,
    ]
    roll, pitch, yaw = visual["rpy"]
    visual_lines.extend(
        [
            f'        <visual name="d1_{visual["name"]}">\n',
            (
                "          <pose>"
                f"{translation[0]:.6f} {translation[1]:.6f} {translation[2]:.6f} "
                f"{roll:.6f} {pitch:.6f} {yaw:.6f}"
                "</pose>\n"
            ),
            "          <geometry>\n",
            "            <mesh>\n",
            f'              <uri>file://{visual["mesh_uri"]}</uri>\n',
            "            </mesh>\n",
            "          </geometry>\n",
            "          <visibility_flags>0x02</visibility_flags>\n",
            "        </visual>\n",
        ])

  return "".join(visual_lines)


def _load_scene_boxes(package_share):
  metadata_path = os.path.join(
      package_share, "mesh", "whitebox_stair_test", "whitebox_stair_test.json")
  with open(metadata_path, "r", encoding="utf-8") as handle:
    metadata = json.load(handle)

  visual_lines = []
  for obj in metadata["objects"]:
    size_x = obj["x_max"] - obj["x_min"]
    size_y = obj["y_max"] - obj["y_min"]
    size_z = obj["z_max"] - obj["z_min"]
    pose_x = (obj["x_min"] + obj["x_max"]) * 0.5
    pose_y = (obj["y_min"] + obj["y_max"]) * 0.5
    pose_z = (obj["z_min"] + obj["z_max"]) * 0.5

    if obj["name"] == "floor":
      ambient = "0.54 0.58 0.64 1"
      diffuse = "0.54 0.58 0.64 1"
      specular = "0.08 0.08 0.08 1"
    else:
      ambient = "0.82 0.28 0.18 1"
      diffuse = "0.82 0.28 0.18 1"
      specular = "0.12 0.12 0.12 1"

    visual_lines.extend(
        [
            f'        <collision name="{obj["name"]}_collision">\n',
            f"          <pose>{pose_x:.6f} {pose_y:.6f} {pose_z:.6f} 0 0 0</pose>\n",
            "          <geometry>\n",
            "            <box>\n",
            f"              <size>{size_x:.6f} {size_y:.6f} {size_z:.6f}</size>\n",
            "            </box>\n",
            "          </geometry>\n",
            "        </collision>\n",
            f'        <visual name="{obj["name"]}_visual">\n',
            f"          <pose>{pose_x:.6f} {pose_y:.6f} {pose_z:.6f} 0 0 0</pose>\n",
            "          <geometry>\n",
            "            <box>\n",
            f"              <size>{size_x:.6f} {size_y:.6f} {size_z:.6f}</size>\n",
            "            </box>\n",
            "          </geometry>\n",
            "          <material>\n",
            f"            <ambient>{ambient}</ambient>\n",
            f"            <diffuse>{diffuse}</diffuse>\n",
            f"            <specular>{specular}</specular>\n",
            "          </material>\n",
            "          <visibility_flags>0x01</visibility_flags>\n",
            "        </visual>\n",
        ])

  return "".join(visual_lines)


def _write_world_file(context, *_args, **_kwargs):
  package_share = get_package_share_directory("vehicle_simulator")
  world_path = os.path.join(tempfile.gettempdir(), "cmu_planner_whitebox_gazebo.sdf")
  gui_config_path = os.path.join(tempfile.gettempdir(), "cmu_planner_whitebox_gazebo_gui.config")
  gui = LaunchConfiguration("gazebo_gui").perform(context)
  sensor_visualize = "true" if gui == "true" else "false"
  d1_visuals = _load_d1_visuals(package_share)
  scene_boxes = _load_scene_boxes(package_share)

  world_sdf = f"""<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="whitebox_stair_test">
    <gravity>0 0 -9.8</gravity>

    <plugin filename="ignition-gazebo-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="ignition-gazebo-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="ignition-gazebo-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="ignition-gazebo-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.98 0.95 0.90 1</diffuse>
      <specular>0.35 0.35 0.35 1</specular>
      <direction>-0.4 0.2 -0.9</direction>
    </light>

    <model name="whitebox_scene">
      <static>true</static>
      <link name="scene_link">
{scene_boxes.rstrip()}
      </link>
    </model>

    <model name="stairbot">
      <pose>0 0 0.75 0 0 0</pose>
      <static>true</static>

      <link name="chassis">
        <pose>-0.28 0 -0.06 0 0 0</pose>
        <gravity>false</gravity>
        <kinematic>true</kinematic>
        <inertial>
          <mass>8.0</mass>
          <inertia>
            <ixx>0.12</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.18</iyy>
            <iyz>0</iyz>
            <izz>0.22</izz>
          </inertia>
        </inertial>
{d1_visuals.rstrip()}
      </link>

      <link name="lidar_link">
        <pose>0 0 0 0 0 0</pose>
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>0.025</radius>
              <length>0.04</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0.05 0.05 0.05 1</ambient>
            <diffuse>0.05 0.05 0.05 1</diffuse>
          </material>
          <visibility_flags>0x02</visibility_flags>
        </visual>
        <sensor name="scan_visualizer" type="lidar">
          <topic>lidar_visual</topic>
          <update_rate>10</update_rate>
          <always_on>true</always_on>
          <visualize>{sensor_visualize}</visualize>
          <lidar>
            <visibility_mask>0x01</visibility_mask>
            <scan>
              <horizontal>
                <samples>180</samples>
                <resolution>1</resolution>
                <min_angle>-3.14159</min_angle>
                <max_angle>3.14159</max_angle>
              </horizontal>
              <vertical>
                <samples>1</samples>
                <resolution>1</resolution>
                <min_angle>0.0</min_angle>
                <max_angle>0.0</max_angle>
              </vertical>
            </scan>
            <range>
              <min>0.08</min>
              <max>20.0</max>
              <resolution>0.02</resolution>
            </range>
          </lidar>
        </sensor>
        <sensor name="gpu_lidar" type="gpu_lidar">
          <topic>lidar</topic>
          <update_rate>10</update_rate>
          <always_on>true</always_on>
          <visualize>{sensor_visualize}</visualize>
          <lidar>
            <visibility_mask>0x01</visibility_mask>
            <scan>
              <horizontal>
                <samples>720</samples>
                <resolution>1</resolution>
                <min_angle>-3.14159</min_angle>
                <max_angle>3.14159</max_angle>
              </horizontal>
              <vertical>
                <samples>16</samples>
                <resolution>1</resolution>
                <min_angle>-0.26</min_angle>
                <max_angle>0.26</max_angle>
              </vertical>
            </scan>
            <range>
              <min>0.08</min>
              <max>20.0</max>
              <resolution>0.02</resolution>
            </range>
          </lidar>
        </sensor>
      </link>

      <joint name="lidar_joint" type="fixed">
        <parent>chassis</parent>
        <child>lidar_link</child>
      </joint>
    </model>
  </world>
</sdf>
"""

  with open(world_path, "w", encoding="ascii") as handle:
    handle.write(world_sdf)

  gz_args = f"-r {'-s ' if gui == 'false' else ''}"
  if gui == "true":
    base_gui_config = os.path.expanduser("~/.ignition/gazebo/6/gui.config")
    if not os.path.exists(base_gui_config):
      base_gui_config = "/usr/share/ignition/ignition-gazebo6/gui/gui.config"
    with open(base_gui_config, "r", encoding="utf-8") as handle:
      gui_config = handle.read().rstrip()
    if "VisualizeLidar" not in gui_config:
      gui_config += """

<plugin filename="VisualizeLidar" name="Visualize Lidar">
  <ignition-gui>
    <title>Visualize Lidar</title>
    <property type="bool" key="showTitleBar">true</property>
    <property type="bool" key="resizable">true</property>
    <property type="double" key="x">25</property>
    <property type="double" key="y">140</property>
    <property type="double" key="width">360</property>
    <property type="double" key="height">220</property>
    <property type="string" key="state">floating</property>
  </ignition-gui>
</plugin>
"""
    with open(gui_config_path, "w", encoding="utf-8") as handle:
      handle.write(gui_config + "\n")
    gz_args += f"--gui-config {shlex.quote(gui_config_path)} "
  gz_args += shlex.quote(world_path)

  return [
      SetParameter(name="use_sim_time", value=True),
      IncludeLaunchDescription(
          PythonLaunchDescriptionSource(
              os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")),
          launch_arguments={"gz_args": gz_args, "on_exit_shutdown": "true"}.items(),
      ),
  ]


def generate_launch_description():
  route_planner_config = LaunchConfiguration("route_planner_config")
  world_name = LaunchConfiguration("world_name")
  vehicle_height = LaunchConfiguration("vehicleHeight")
  camera_offset_z = LaunchConfiguration("cameraOffsetZ")
  vehicle_x = LaunchConfiguration("vehicleX")
  vehicle_y = LaunchConfiguration("vehicleY")
  terrain_z = LaunchConfiguration("terrainZ")
  vehicle_yaw = LaunchConfiguration("vehicleYaw")
  check_terrain_conn = LaunchConfiguration("checkTerrainConn")

  start_local_planner = IncludeLaunchDescription(
      FrontendLaunchDescriptionSource(
          os.path.join(get_package_share_directory("local_planner"), "launch", "local_planner.launch")),
      launch_arguments={
          "cameraOffsetZ": camera_offset_z,
          "goalX": vehicle_x,
          "goalY": vehicle_y,
      }.items(),
  )

  start_terrain_analysis = IncludeLaunchDescription(
      FrontendLaunchDescriptionSource(
          os.path.join(get_package_share_directory("terrain_analysis"), "launch", "terrain_analysis.launch")))

  start_terrain_analysis_ext = IncludeLaunchDescription(
      FrontendLaunchDescriptionSource(
          os.path.join(get_package_share_directory("terrain_analysis_ext"), "launch", "terrain_analysis_ext.launch")),
      launch_arguments={"checkTerrainConn": check_terrain_conn}.items(),
  )

  start_vehicle_simulator = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
          os.path.join(get_package_share_directory("vehicle_simulator"), "launch", "vehicle_simulator.launch")),
      launch_arguments={
          "useSimTime": "true",
          "vehicleHeight": vehicle_height,
          "vehicleX": vehicle_x,
          "vehicleY": vehicle_y,
          "terrainZ": terrain_z,
          "vehicleYaw": vehicle_yaw,
      }.items(),
  )

  start_registered_scan = Node(
      package="vehicle_simulator",
      executable="registeredScanFromOdom",
      output="screen",
      parameters=[{
          "point_cloud_topic": "/lidar/points",
          "odom_topic": "/state_estimation",
          "registered_scan_topic": "/registered_scan",
          "state_estimation_at_scan_topic": "/state_estimation_at_scan",
          "output_frame": "map",
          "sensor_frame": "sensor_at_scan",
          "publish_state_estimation_at_scan": False,
          "publish_scan_tf": False,
          "sensor_offset_x": 0.0,
          "sensor_offset_y": 0.0,
          "sensor_offset_z": 0.0,
      }],
  )

  start_sensor_scan_generation = IncludeLaunchDescription(
      FrontendLaunchDescriptionSource(
          os.path.join(get_package_share_directory("sensor_scan_generation"), "launch", "sensor_scan_generation.launch")))

  start_visualization_tools = IncludeLaunchDescription(
      FrontendLaunchDescriptionSource(
          os.path.join(get_package_share_directory("visualization_tools"), "launch", "visualization_tools.launch")),
      launch_arguments={"world_name": world_name}.items(),
  )

  start_joy = Node(
      package="joy",
      executable="joy_node",
      name="ps3_joy",
      output="screen",
      parameters=[{"dev": "/dev/input/js0", "deadzone": 0.12, "autorepeat_rate": 0.0}],
  )

  start_far_planner = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
          os.path.join(get_package_share_directory("far_planner"), "launch", "far_planner.launch")),
      launch_arguments={"config": route_planner_config, "use_sim_time": "true"}.items(),
  )

  start_bridge = Node(
      package="ros_gz_bridge",
      executable="parameter_bridge",
      output="screen",
      arguments=[
          "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
          "/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
          ["/world/", world_name, "/set_pose@ros_gz_interfaces/srv/SetEntityPose"],
      ],
  )

  start_pose_adapter = Node(
      package="vehicle_simulator",
      executable="poseStampedToGazeboSetPose",
      output="screen",
      parameters=[{
          "input_topic": "/unity_sim/set_model_state",
          "service_name": ["/world/", world_name, "/set_pose"],
          "model_name": "stairbot",
      }],
  )

  return LaunchDescription([
      DeclareLaunchArgument("route_planner_config", default_value="indoor", description=""),
      DeclareLaunchArgument("world_name", default_value="whitebox_stair_test", description=""),
      DeclareLaunchArgument("vehicleHeight", default_value="0.75", description=""),
      DeclareLaunchArgument("cameraOffsetZ", default_value="0.041", description=""),
      DeclareLaunchArgument("vehicleX", default_value="0.0", description=""),
      DeclareLaunchArgument("vehicleY", default_value="0.0", description=""),
      DeclareLaunchArgument("terrainZ", default_value="0.0", description=""),
      DeclareLaunchArgument("vehicleYaw", default_value="0.0", description=""),
      DeclareLaunchArgument("checkTerrainConn", default_value="true", description=""),
      DeclareLaunchArgument("gazebo_gui", default_value="true", description=""),
      SetParameter(name="use_sim_time", value=True),
      OpaqueFunction(function=_write_world_file),
      start_bridge,
      start_pose_adapter,
      start_local_planner,
      start_terrain_analysis,
      start_terrain_analysis_ext,
      start_vehicle_simulator,
      start_registered_scan,
      start_sensor_scan_generation,
      start_visualization_tools,
      start_joy,
      start_far_planner,
  ])
