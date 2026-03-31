#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "rmw/types.h"
#include "rmw/qos_profiles.h"

using namespace std;

// Point clouds
pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudIn(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr laserCLoudInSensorFrame(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZRGB>::Ptr laserCloudColored(new pcl::PointCloud<pcl::PointXYZRGB>());

double robotX = 0;
double robotY = 0;
double robotZ = 0;
double roll = 0;
double pitch = 0;
double yaw = 0;

bool newTransformToMap = false;

nav_msgs::msg::Odometry odometryIn;
shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> pubOdometryPointer;
tf2::Stamped<tf2::Transform> transformToMap;
geometry_msgs::msg::TransformStamped transformTfGeom ; 

unique_ptr<tf2_ros::TransformBroadcaster> tfBroadcasterPointer;
shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pubLaserCloud;
shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pubColoredRegisteredScan;

// Robot ID and color settings
int robotId = 0;
bool useRobotColor = true;
bool publishColoredRegisteredScan = true;

// Predefined colors for different robots (RGB)
// Robot 0: Red, Robot 1: Green, Robot 2: Blue, Robot 3: Yellow, Robot 4: Cyan, Robot 5: Magenta
const uint8_t ROBOT_COLORS[][3] = {
  {255, 0, 0},     // Robot 0: Red
  {0, 255, 0},     // Robot 1: Green
  {0, 0, 255},     // Robot 2: Blue
  {255, 255, 0},   // Robot 3: Yellow
  {0, 255, 255},   // Robot 4: Cyan
  {255, 0, 255},   // Robot 5: Magagenta
  {255, 128, 0},   // Robot 6: Orange
  {128, 0, 255},   // Robot 7: Purple
  {255, 192, 203}, // Robot 8: Pink
  {0, 128, 128},   // Robot 9: Teal
};
const int NUM_ROBOT_COLORS = sizeof(ROBOT_COLORS) / sizeof(ROBOT_COLORS[0]);

void laserCloudAndOdometryHandler(const nav_msgs::msg::Odometry::ConstSharedPtr odometry,
                                  const sensor_msgs::msg::PointCloud2::ConstSharedPtr laserCloud2)
{
  laserCloudIn->clear();
  laserCLoudInSensorFrame->clear();
  laserCloudColored->clear();

  pcl::fromROSMsg(*laserCloud2, *laserCloudIn);

  odometryIn = *odometry;

  transformToMap.setOrigin(
      tf2::Vector3(odometryIn.pose.pose.position.x, odometryIn.pose.pose.position.y, odometryIn.pose.pose.position.z));
  transformToMap.setRotation(tf2::Quaternion(odometryIn.pose.pose.orientation.x, odometryIn.pose.pose.orientation.y,
                                            odometryIn.pose.pose.orientation.z, odometryIn.pose.pose.orientation.w));

  int laserCloudInNum = laserCloudIn->points.size();

  pcl::PointXYZ p1;
  pcl::PointXYZRGB p_colored;
  tf2::Vector3 vec;

  // Get color for this robot
  uint8_t r = 255, g = 255, b = 255;  // Default: white
  if (useRobotColor) {
    int colorIdx = robotId % NUM_ROBOT_COLORS;
    r = ROBOT_COLORS[colorIdx][0];
    g = ROBOT_COLORS[colorIdx][1];
    b = ROBOT_COLORS[colorIdx][2];
  }

  // Create colored point cloud in world frame (registered_scan)
  for (int i = 0; i < laserCloudInNum; i++)
  {
    p1 = laserCloudIn->points[i];
    
    // For colored point cloud (world frame)
    p_colored.x = p1.x;
    p_colored.y = p1.y;
    p_colored.z = p1.z;
    p_colored.r = r;
    p_colored.g = g;
    p_colored.b = b;
    p_colored.a = 255;
    laserCloudColored->points.push_back(p_colored);

    // Transform to sensor frame for sensor_scan
    vec.setX(p1.x);
    vec.setY(p1.y);
    vec.setZ(p1.z);
    vec = transformToMap.inverse() * vec;
    p1.x = vec.x();
    p1.y = vec.y();
    p1.z = vec.z();
    laserCLoudInSensorFrame->points.push_back(p1);
  }

  odometryIn.header.stamp = laserCloud2->header.stamp;
  odometryIn.header.frame_id = "map";
  odometryIn.child_frame_id = "sensor_at_scan";
  pubOdometryPointer->publish(odometryIn);

  transformToMap.frame_id_ = "map";
  transformTfGeom = tf2::toMsg(transformToMap);
  transformTfGeom.header.stamp = laserCloud2->header.stamp;
  transformTfGeom.child_frame_id = "sensor_at_scan";
  tfBroadcasterPointer->sendTransform(transformTfGeom);

  // Publish sensor_scan (sensor frame, no color)
  sensor_msgs::msg::PointCloud2 scan_data;
  pcl::toROSMsg(*laserCLoudInSensorFrame, scan_data);
  scan_data.header.stamp = laserCloud2->header.stamp;
  scan_data.header.frame_id = "sensor_at_scan";
  pubLaserCloud->publish(scan_data);

  // Publish colored registered_scan (world frame, with color)
  if (publishColoredRegisteredScan) {
    sensor_msgs::msg::PointCloud2 colored_scan_data;
    pcl::toROSMsg(*laserCloudColored, colored_scan_data);
    colored_scan_data.header.stamp = laserCloud2->header.stamp;
    colored_scan_data.header.frame_id = "map";
    pubColoredRegisteredScan->publish(colored_scan_data);
  }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto nh = rclcpp::Node::make_shared("sensor_scan");
  std::string stateEstimationTopic = "/state_estimation";
  std::string registeredScanTopic = "/registered_scan";
  std::string stateEstimationAtScanTopic = "/state_estimation_at_scan";
  std::string sensorScanTopic = "/sensor_scan";
  std::string coloredRegisteredScanTopic = "/registered_scan_colored";

  // ROS message filters
  message_filters::Subscriber<nav_msgs::msg::Odometry> subOdometry;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> subLaserCloud;

  typedef message_filters::sync_policies::ApproximateTime<nav_msgs::msg::Odometry, sensor_msgs::msg::PointCloud2> syncPolicy;
  typedef message_filters::Synchronizer<syncPolicy> Sync;
  boost::shared_ptr<Sync> sync_;
  
  // Define qos_profile as the pre-defined rmw_qos_profile_sensor_data, but with depth equal to 1.
  rmw_qos_profile_t qos_profile=
  {
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    1,
    RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
    RMW_QOS_POLICY_DURABILITY_VOLATILE,
    RMW_QOS_DEADLINE_DEFAULT,
    RMW_QOS_LIFESPAN_DEFAULT,
    RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
    RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
    false
  };

  nh->declare_parameter<std::string>("stateEstimationTopic",
                                     stateEstimationTopic);
  nh->declare_parameter<std::string>("registeredScanTopic",
                                     registeredScanTopic);
  nh->declare_parameter<std::string>("stateEstimationAtScanTopic",
                                     stateEstimationAtScanTopic);
  nh->declare_parameter<std::string>("sensorScanTopic", sensorScanTopic);
  nh->declare_parameter<std::string>("coloredRegisteredScanTopic", coloredRegisteredScanTopic);
  nh->declare_parameter<int>("robot_id", robotId);
  nh->declare_parameter<bool>("use_robot_color", useRobotColor);
  nh->declare_parameter<bool>("publish_colored_registered_scan", publishColoredRegisteredScan);
  
  nh->get_parameter("stateEstimationTopic", stateEstimationTopic);
  nh->get_parameter("registeredScanTopic", registeredScanTopic);
  nh->get_parameter("stateEstimationAtScanTopic", stateEstimationAtScanTopic);
  nh->get_parameter("sensorScanTopic", sensorScanTopic);
  nh->get_parameter("coloredRegisteredScanTopic", coloredRegisteredScanTopic);
  nh->get_parameter("robot_id", robotId);
  nh->get_parameter("use_robot_color", useRobotColor);
  nh->get_parameter("publish_colored_registered_scan", publishColoredRegisteredScan);

  RCLCPP_INFO(nh->get_logger(), "Sensor scan generation started for robot %d", robotId);
  if (useRobotColor) {
    int colorIdx = robotId % NUM_ROBOT_COLORS;
    RCLCPP_INFO(nh->get_logger(), "Using RGB color: R=%d, G=%d, B=%d", 
                ROBOT_COLORS[colorIdx][0], ROBOT_COLORS[colorIdx][1], ROBOT_COLORS[colorIdx][2]);
    RCLCPP_INFO(nh->get_logger(), "Publishing colored registered scan to: %s", coloredRegisteredScanTopic.c_str());
  }

  subOdometry.subscribe(nh, stateEstimationTopic, qos_profile);
  subLaserCloud.subscribe(nh, registeredScanTopic, qos_profile);
  sync_.reset(new Sync(syncPolicy(100), subOdometry, subLaserCloud));
  sync_->registerCallback(std::bind(laserCloudAndOdometryHandler, placeholders::_1, placeholders::_2));
  pubOdometryPointer =
      nh->create_publisher<nav_msgs::msg::Odometry>(
          stateEstimationAtScanTopic, 5);

  tfBroadcasterPointer = std::make_unique<tf2_ros::TransformBroadcaster>(*nh);

  pubLaserCloud =
      nh->create_publisher<sensor_msgs::msg::PointCloud2>(sensorScanTopic, 2);

  pubColoredRegisteredScan =
      nh->create_publisher<sensor_msgs::msg::PointCloud2>(coloredRegisteredScanTopic, 2);

  rclcpp::spin(nh);

  return 0;
}
