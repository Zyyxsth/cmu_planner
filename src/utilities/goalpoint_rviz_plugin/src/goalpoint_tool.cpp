#include <goalpoint_tool.hpp>

#include <string>
#include <cstdlib>

#include <rviz_common/display_context.hpp>
#include <rviz_common/logging.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <rviz_common/properties/qos_profile_property.hpp>

namespace goalpoint_rviz_plugin
{
GoalpointTool::GoalpointTool()
: rviz_default_plugins::tools::PoseTool(), qos_profile_(5)
{
  shortcut_key_ = 'w';

  topic_property_ = new rviz_common::properties::StringProperty("Topic", "goalpoint", "The topic on which to publish navigation waypionts.",
                                       getPropertyContainer(), SLOT(updateTopic()), this);

  goal_z_mode_property_ = new rviz_common::properties::StringProperty(
    "Goal Z Mode",
    "floor1",
    "Goal z selection: current, floor1, floor2, or custom. floor1/floor2 are sensor-frame goal heights.",
    getPropertyContainer());

  custom_goal_z_property_ = new rviz_common::properties::FloatProperty(
    "Custom Goal Z",
    0.75,
    "Absolute map-frame z used when Goal Z Mode is custom.",
    getPropertyContainer());
  
  qos_profile_property_ = new rviz_common::properties::QosProfileProperty(
    topic_property_, qos_profile_);
}

GoalpointTool::~GoalpointTool() = default;

void GoalpointTool::onInitialize()
{
  rviz_default_plugins::tools::PoseTool::onInitialize();
  qos_profile_property_->initialize(
    [this](rclcpp::QoS profile) {this->qos_profile_ = profile;});
  setName("Goalpoint");
  updateTopic();
  vehicle_z = 0;
}

void GoalpointTool::updateTopic()
{
  rclcpp::Node::SharedPtr raw_node =
    context_->getRosNodeAbstraction().lock()->get_raw_node();
  sub_ = raw_node->template create_subscription<nav_msgs::msg::Odometry>("/state_estimation", 5 ,std::bind(&GoalpointTool::odomHandler,this,std::placeholders::_1));
  
  pub_ = raw_node->template create_publisher<geometry_msgs::msg::PointStamped>("/goal_point", qos_profile_);
  pub_joy_ = raw_node->template create_publisher<sensor_msgs::msg::Joy>("/joy", qos_profile_);
  clock_ = raw_node->get_clock();
}

void GoalpointTool::odomHandler(const nav_msgs::msg::Odometry::ConstSharedPtr odom)
{
  vehicle_z = odom->pose.pose.position.z;
}

double GoalpointTool::resolveGoalZ() const
{
  const std::string mode = goal_z_mode_property_->getStdString();
  if (mode == "current" || mode == "vehicle") {
    return vehicle_z;
  }
  if (mode == "floor1" || mode == "floor_1" || mode == "1") {
    return 0.75;
  }
  if (mode == "floor2" || mode == "floor_2" || mode == "2") {
    return 3.75;
  }
  if (mode == "custom") {
    return custom_goal_z_property_->getFloat();
  }

  char * end = nullptr;
  const double parsed_z = std::strtod(mode.c_str(), &end);
  if (end != mode.c_str() && *end == '\0') {
    return parsed_z;
  }
  RVIZ_COMMON_LOG_WARNING_STREAM(
    "GoalpointTool: unknown Goal Z Mode '" << mode << "', falling back to current vehicle z.");
  return vehicle_z;
}

void GoalpointTool::onPoseSet(double x, double y, double theta)
{
  (void)theta;
  sensor_msgs::msg::Joy joy;

  joy.axes.push_back(0);
  joy.axes.push_back(0);
  joy.axes.push_back(-1.0);
  joy.axes.push_back(0);
  joy.axes.push_back(1.0);
  joy.axes.push_back(1.0);
  joy.axes.push_back(0);
  joy.axes.push_back(0);

  joy.buttons.push_back(0);
  joy.buttons.push_back(0);
  joy.buttons.push_back(0);
  joy.buttons.push_back(0);
  joy.buttons.push_back(0);
  joy.buttons.push_back(0);
  joy.buttons.push_back(0);
  joy.buttons.push_back(1);
  joy.buttons.push_back(0);
  joy.buttons.push_back(0);
  joy.buttons.push_back(0);

  joy.header.stamp = clock_->now();
  joy.header.frame_id = "goalpoint_tool";
  pub_joy_->publish(joy);

  geometry_msgs::msg::PointStamped goalpoint;
  goalpoint.header.frame_id = "map";
  goalpoint.header.stamp = joy.header.stamp;
  goalpoint.point.x = x;
  goalpoint.point.y = y;
  goalpoint.point.z = resolveGoalZ();

  pub_->publish(goalpoint);
  usleep(10000);
  pub_->publish(goalpoint);
}

GoalpointFloor2Tool::GoalpointFloor2Tool()
: GoalpointTool()
{
  shortcut_key_ = 'e';
}

void GoalpointFloor2Tool::onInitialize()
{
  GoalpointTool::onInitialize();
  setName("Goalpoint Floor2");
}

double GoalpointFloor2Tool::resolveGoalZ() const
{
  return 3.75;
}
}

#include <pluginlib/class_list_macros.hpp> 
PLUGINLIB_EXPORT_CLASS(goalpoint_rviz_plugin::GoalpointTool, rviz_common::Tool)
PLUGINLIB_EXPORT_CLASS(goalpoint_rviz_plugin::GoalpointFloor2Tool, rviz_common::Tool)
