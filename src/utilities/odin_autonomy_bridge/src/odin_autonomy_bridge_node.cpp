#include <memory>
#include <string>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

class OdinAutonomyBridge : public rclcpp::Node {
 public:
  OdinAutonomyBridge() : Node("odin_autonomy_bridge_node") {
    declare_parameter<std::string>("input_odom_topic", "/odin1/odometry");
    declare_parameter<std::string>("input_cloud_topic", "/odin1/cloud_slam");
    declare_parameter<std::string>("input_imu_topic", "/odin1/imu");
    declare_parameter<std::string>("output_state_estimation_topic", "/state_estimation");
    declare_parameter<std::string>("output_registered_scan_topic", "/registered_scan");
    declare_parameter<std::string>("output_imu_topic", "/imu/data");
    declare_parameter<std::string>("world_frame", "map");
    declare_parameter<bool>("rewrite_frame_id", true);

    input_odom_topic_ = get_parameter("input_odom_topic").as_string();
    input_cloud_topic_ = get_parameter("input_cloud_topic").as_string();
    input_imu_topic_ = get_parameter("input_imu_topic").as_string();
    output_state_estimation_topic_ =
        get_parameter("output_state_estimation_topic").as_string();
    output_registered_scan_topic_ =
        get_parameter("output_registered_scan_topic").as_string();
    output_imu_topic_ = get_parameter("output_imu_topic").as_string();
    world_frame_ = get_parameter("world_frame").as_string();
    rewrite_frame_id_ = get_parameter("rewrite_frame_id").as_bool();

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
        output_state_estimation_topic_, 20);
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        output_registered_scan_topic_, 10);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(output_imu_topic_, 50);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        input_odom_topic_, 20,
        std::bind(&OdinAutonomyBridge::OdomCallback, this,
                  std::placeholders::_1));
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        input_cloud_topic_, 10,
        std::bind(&OdinAutonomyBridge::CloudCallback, this,
                  std::placeholders::_1));
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        input_imu_topic_, 50,
        std::bind(&OdinAutonomyBridge::ImuCallback, this,
                  std::placeholders::_1));

    RCLCPP_INFO(
        get_logger(),
        "Odin bridge started. odom:%s cloud:%s imu:%s -> state:%s scan:%s imu_out:%s",
        input_odom_topic_.c_str(), input_cloud_topic_.c_str(),
        input_imu_topic_.c_str(), output_state_estimation_topic_.c_str(),
        output_registered_scan_topic_.c_str(), output_imu_topic_.c_str());
  }

 private:
  void OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    auto out = *msg;
    if (rewrite_frame_id_) {
      out.header.frame_id = world_frame_;
    }
    odom_pub_->publish(out);
  }

  void CloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    auto out = *msg;
    if (rewrite_frame_id_) {
      out.header.frame_id = world_frame_;
    }
    cloud_pub_->publish(out);
  }

  void ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    imu_pub_->publish(*msg);
  }

  std::string input_odom_topic_;
  std::string input_cloud_topic_;
  std::string input_imu_topic_;
  std::string output_state_estimation_topic_;
  std::string output_registered_scan_topic_;
  std::string output_imu_topic_;
  std::string world_frame_;
  bool rewrite_frame_id_{true};

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OdinAutonomyBridge>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
