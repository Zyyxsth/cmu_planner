#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros_gz_interfaces/msg/entity.hpp"
#include "ros_gz_interfaces/srv/set_entity_pose.hpp"

class PoseStampedToGazeboSetPose : public rclcpp::Node
{
public:
  PoseStampedToGazeboSetPose()
  : Node("pose_stamped_to_gazebo_set_pose")
  {
    input_topic_ =
      declare_parameter<std::string>("input_topic", "/unity_sim/set_model_state");
    service_name_ =
      declare_parameter<std::string>("service_name", "/world/whitebox_stair_test/set_pose");
    model_name_ = declare_parameter<std::string>("model_name", "stairbot");

    client_ = create_client<ros_gz_interfaces::srv::SetEntityPose>(service_name_);
    subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      input_topic_, 20,
      std::bind(&PoseStampedToGazeboSetPose::poseHandler, this, std::placeholders::_1));
  }

private:
  void poseHandler(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
  {
    if (!client_->service_is_ready() || request_pending_) {
      return;
    }

    auto request = std::make_shared<ros_gz_interfaces::srv::SetEntityPose::Request>();
    request->entity.name = model_name_;
    request->entity.type = ros_gz_interfaces::msg::Entity::MODEL;
    request->pose = msg->pose;

    request_pending_ = true;
    client_->async_send_request(
      request,
      [this](rclcpp::Client<ros_gz_interfaces::srv::SetEntityPose>::SharedFuture future) {
        request_pending_ = false;
        const auto response = future.get();
        if (!response->success) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Gazebo set_pose rejected pose update for model '%s'.", model_name_.c_str());
        }
      });
  }

  std::string input_topic_;
  std::string service_name_;
  std::string model_name_;
  bool request_pending_ = false;

  rclcpp::Client<ros_gz_interfaces::srv::SetEntityPose>::SharedPtr client_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseStampedToGazeboSetPose>());
  rclcpp::shutdown();
  return 0;
}
