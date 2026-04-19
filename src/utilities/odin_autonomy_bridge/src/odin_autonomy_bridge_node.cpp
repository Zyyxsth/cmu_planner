#include <memory>
#include <string>
#include <vector>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

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
  static bool HasField(
      const sensor_msgs::msg::PointCloud2 & msg,
      const std::string & field_name) {
    for (const auto & field : msg.fields) {
      if (field.name == field_name) {
        return true;
      }
    }
    return false;
  }

  static sensor_msgs::msg::PointCloud2 AddDefaultIntensityField(
      const sensor_msgs::msg::PointCloud2 & msg) {
    sensor_msgs::msg::PointCloud2 out;
    out.header = msg.header;
    out.height = msg.height;
    out.width = msg.width;
    out.is_bigendian = msg.is_bigendian;
    out.is_dense = msg.is_dense;

    sensor_msgs::PointCloud2Modifier modifier(out);
    modifier.setPointCloud2Fields(
        4,
        "x", 1, sensor_msgs::msg::PointField::FLOAT32,
        "y", 1, sensor_msgs::msg::PointField::FLOAT32,
        "z", 1, sensor_msgs::msg::PointField::FLOAT32,
        "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
    modifier.resize(static_cast<size_t>(msg.width) * static_cast<size_t>(msg.height));

    sensor_msgs::PointCloud2ConstIterator<float> in_x(msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> in_y(msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> in_z(msg, "z");
    sensor_msgs::PointCloud2Iterator<float> out_x(out, "x");
    sensor_msgs::PointCloud2Iterator<float> out_y(out, "y");
    sensor_msgs::PointCloud2Iterator<float> out_z(out, "z");
    sensor_msgs::PointCloud2Iterator<float> out_intensity(out, "intensity");

    const size_t point_count =
        static_cast<size_t>(msg.width) * static_cast<size_t>(msg.height);
    for (size_t i = 0; i < point_count;
         ++i, ++in_x, ++in_y, ++in_z, ++out_x, ++out_y, ++out_z, ++out_intensity) {
      *out_x = *in_x;
      *out_y = *in_y;
      *out_z = *in_z;
      *out_intensity = 0.0f;
    }

    return out;
  }

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
    if (!HasField(out, "intensity")) {
      out = AddDefaultIntensityField(out);
      if (rewrite_frame_id_) {
        out.header.frame_id = world_frame_;
      }
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
