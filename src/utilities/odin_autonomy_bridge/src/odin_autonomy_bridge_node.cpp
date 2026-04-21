#include <cmath>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"

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
    declare_parameter<bool>("alias_rgb_field_as_intensity", true);
    declare_parameter<bool>("cloud_transform_with_latest_odom", false);
    declare_parameter<bool>("enable_odom_bridge", true);
    declare_parameter<bool>("enable_cloud_bridge", true);
    declare_parameter<bool>("enable_imu_bridge", true);

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
    alias_rgb_field_as_intensity_ =
        get_parameter("alias_rgb_field_as_intensity").as_bool();
    cloud_transform_with_latest_odom_ =
        get_parameter("cloud_transform_with_latest_odom").as_bool();
    enable_odom_bridge_ = get_parameter("enable_odom_bridge").as_bool();
    enable_cloud_bridge_ = get_parameter("enable_cloud_bridge").as_bool();
    enable_imu_bridge_ = get_parameter("enable_imu_bridge").as_bool();

    if (enable_odom_bridge_ || cloud_transform_with_latest_odom_) {
      odom_callback_group_ =
          create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      rclcpp::SubscriptionOptions odom_options;
      odom_options.callback_group = odom_callback_group_;
      odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
          input_odom_topic_, 20,
          std::bind(&OdinAutonomyBridge::OdomCallback, this,
                    std::placeholders::_1),
          odom_options);
    }

    if (enable_odom_bridge_) {
      odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
          output_state_estimation_topic_, 20);
    }

    if (enable_cloud_bridge_) {
      cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
          output_registered_scan_topic_, 10);
      cloud_callback_group_ =
          create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      rclcpp::SubscriptionOptions cloud_options;
      cloud_options.callback_group = cloud_callback_group_;
      cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
          input_cloud_topic_, 10,
          std::bind(&OdinAutonomyBridge::CloudCallback, this,
                    std::placeholders::_1),
          cloud_options);
    }

    if (enable_imu_bridge_) {
      imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(output_imu_topic_, 50);
      imu_callback_group_ =
          create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      rclcpp::SubscriptionOptions imu_options;
      imu_options.callback_group = imu_callback_group_;
      imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
          input_imu_topic_, 50,
          std::bind(&OdinAutonomyBridge::ImuCallback, this,
                    std::placeholders::_1),
          imu_options);
    }

    RCLCPP_INFO(
        get_logger(),
        "Odin bridge started. odom:%s cloud:%s imu:%s -> state:%s scan:%s imu_out:%s alias_rgb_field_as_intensity:%s cloud_transform_with_latest_odom:%s enable_odom:%s enable_cloud:%s enable_imu:%s",
        input_odom_topic_.c_str(), input_cloud_topic_.c_str(),
        input_imu_topic_.c_str(), output_state_estimation_topic_.c_str(),
        output_registered_scan_topic_.c_str(), output_imu_topic_.c_str(),
        alias_rgb_field_as_intensity_ ? "true" : "false",
        cloud_transform_with_latest_odom_ ? "true" : "false",
        enable_odom_bridge_ ? "true" : "false",
        enable_cloud_bridge_ ? "true" : "false",
        enable_imu_bridge_ ? "true" : "false");
  }

 private:
  static const sensor_msgs::msg::PointField * FindField(
      const sensor_msgs::msg::PointCloud2 & msg,
      const std::string & field_name) {
    for (const auto & field : msg.fields) {
      if (field.name == field_name) {
        return &field;
      }
    }
    return nullptr;
  }

  static bool HasField(
      const sensor_msgs::msg::PointCloud2 & msg,
      const std::string & field_name) {
    return FindField(msg, field_name) != nullptr;
  }

  static bool HasFloat32XYZFields(const sensor_msgs::msg::PointCloud2 & msg) {
    const auto * x_field = FindField(msg, "x");
    const auto * y_field = FindField(msg, "y");
    const auto * z_field = FindField(msg, "z");
    if (x_field == nullptr || y_field == nullptr || z_field == nullptr) {
      return false;
    }

    return x_field->datatype == sensor_msgs::msg::PointField::FLOAT32 &&
           y_field->datatype == sensor_msgs::msg::PointField::FLOAT32 &&
           z_field->datatype == sensor_msgs::msg::PointField::FLOAT32 &&
           x_field->count == 1 && y_field->count == 1 && z_field->count == 1;
  }

  static bool CanAliasRgbFieldAsIntensity(
      const sensor_msgs::msg::PointCloud2 & msg) {
    const auto * x_field = FindField(msg, "x");
    const auto * y_field = FindField(msg, "y");
    const auto * z_field = FindField(msg, "z");
    const auto * rgb_field = FindField(msg, "rgb");
    if (rgb_field == nullptr) {
      rgb_field = FindField(msg, "rgba");
    }

    if (x_field == nullptr || y_field == nullptr || z_field == nullptr ||
        rgb_field == nullptr) {
      return false;
    }

    return x_field->datatype == sensor_msgs::msg::PointField::FLOAT32 &&
           y_field->datatype == sensor_msgs::msg::PointField::FLOAT32 &&
           z_field->datatype == sensor_msgs::msg::PointField::FLOAT32 &&
           rgb_field->datatype == sensor_msgs::msg::PointField::FLOAT32 &&
           x_field->count == 1 && y_field->count == 1 && z_field->count == 1 &&
           rgb_field->count == 1 && x_field->offset == 0 &&
           y_field->offset == 4 && z_field->offset == 8 &&
           rgb_field->offset == 12;
  }

  static void AliasRgbFieldAsIntensity(sensor_msgs::msg::PointCloud2 * msg) {
    for (auto & field : msg->fields) {
      if (field.name == "rgb" || field.name == "rgba") {
        field.name = "intensity";
        break;
      }
    }
  }

  static sensor_msgs::msg::PointCloud2 AddDefaultIntensityField(
      const sensor_msgs::msg::PointCloud2 & msg) {
    const auto * x_field = FindField(msg, "x");
    const auto * y_field = FindField(msg, "y");
    const auto * z_field = FindField(msg, "z");
    if (x_field == nullptr || y_field == nullptr || z_field == nullptr) {
      return msg;
    }

    sensor_msgs::msg::PointCloud2 out;
    out.header = msg.header;
    out.height = msg.height;
    out.width = msg.width;
    out.is_bigendian = msg.is_bigendian;
    out.is_dense = msg.is_dense;

    out.fields.resize(4);
    out.fields[0].name = "x";
    out.fields[0].offset = 0;
    out.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    out.fields[0].count = 1;
    out.fields[1].name = "y";
    out.fields[1].offset = 4;
    out.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    out.fields[1].count = 1;
    out.fields[2].name = "z";
    out.fields[2].offset = 8;
    out.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    out.fields[2].count = 1;
    out.fields[3].name = "intensity";
    out.fields[3].offset = 12;
    out.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
    out.fields[3].count = 1;

    out.point_step = 16;
    out.row_step = out.point_step * out.width;
    out.data.resize(static_cast<size_t>(out.row_step) * out.height);

    const size_t point_count =
        static_cast<size_t>(msg.width) * static_cast<size_t>(msg.height);
    const auto * in_ptr = msg.data.data();
    auto * out_ptr = out.data.data();
    for (size_t i = 0; i < point_count; ++i) {
      std::memcpy(out_ptr + 0, in_ptr + x_field->offset, sizeof(float));
      std::memcpy(out_ptr + 4, in_ptr + y_field->offset, sizeof(float));
      std::memcpy(out_ptr + 8, in_ptr + z_field->offset, sizeof(float));
      std::memset(out_ptr + 12, 0, sizeof(float));
      in_ptr += msg.point_step;
      out_ptr += out.point_step;
    }

    return out;
  }

  void OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    {
      std::lock_guard<std::mutex> lock(latest_odom_mutex_);
      latest_odom_translation_x_ = msg->pose.pose.position.x;
      latest_odom_translation_y_ = msg->pose.pose.position.y;
      latest_odom_translation_z_ = msg->pose.pose.position.z;
      latest_odom_qx_ = msg->pose.pose.orientation.x;
      latest_odom_qy_ = msg->pose.pose.orientation.y;
      latest_odom_qz_ = msg->pose.pose.orientation.z;
      latest_odom_qw_ = msg->pose.pose.orientation.w;
      latest_odom_available_ = true;
    }

    if (!enable_odom_bridge_) {
      return;
    }

    if (rewrite_frame_id_) {
      msg->header.frame_id = world_frame_;
    }
    odom_pub_->publish(*msg);
  }

  bool TransformCloudWithLatestOdom(sensor_msgs::msg::PointCloud2 * msg) {
    if (!HasFloat32XYZFields(*msg)) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Cloud transform requested but x/y/z float32 fields are unavailable.");
      return false;
    }

    double tx = 0.0;
    double ty = 0.0;
    double tz = 0.0;
    double qx = 0.0;
    double qy = 0.0;
    double qz = 0.0;
    double qw = 1.0;
    {
      std::lock_guard<std::mutex> lock(latest_odom_mutex_);
      if (!latest_odom_available_) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "Dropping cloud because no odometry has been cached yet.");
        return false;
      }
      tx = latest_odom_translation_x_;
      ty = latest_odom_translation_y_;
      tz = latest_odom_translation_z_;
      qx = latest_odom_qx_;
      qy = latest_odom_qy_;
      qz = latest_odom_qz_;
      qw = latest_odom_qw_;
    }

    const auto * x_field = FindField(*msg, "x");
    const auto * y_field = FindField(*msg, "y");
    const auto * z_field = FindField(*msg, "z");

    const double q_norm = std::sqrt(
        qx * qx + qy * qy + qz * qz + qw * qw);
    if (q_norm <= 1e-9) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Dropping cloud because odometry quaternion is invalid.");
      return false;
    }

    qx /= q_norm;
    qy /= q_norm;
    qz /= q_norm;
    qw /= q_norm;

    const double r00 = 1.0 - 2.0 * (qy * qy + qz * qz);
    const double r01 = 2.0 * (qx * qy - qz * qw);
    const double r02 = 2.0 * (qx * qz + qy * qw);
    const double r10 = 2.0 * (qx * qy + qz * qw);
    const double r11 = 1.0 - 2.0 * (qx * qx + qz * qz);
    const double r12 = 2.0 * (qy * qz - qx * qw);
    const double r20 = 2.0 * (qx * qz - qy * qw);
    const double r21 = 2.0 * (qy * qz + qx * qw);
    const double r22 = 1.0 - 2.0 * (qx * qx + qy * qy);

    const size_t point_count =
        static_cast<size_t>(msg->width) * static_cast<size_t>(msg->height);
    auto * ptr = msg->data.data();
    for (size_t i = 0; i < point_count; ++i) {
      float x = 0.0f;
      float y = 0.0f;
      float z = 0.0f;
      std::memcpy(&x, ptr + x_field->offset, sizeof(float));
      std::memcpy(&y, ptr + y_field->offset, sizeof(float));
      std::memcpy(&z, ptr + z_field->offset, sizeof(float));

      const double wx = r00 * x + r01 * y + r02 * z + tx;
      const double wy = r10 * x + r11 * y + r12 * z + ty;
      const double wz = r20 * x + r21 * y + r22 * z + tz;

      const float wx_f = static_cast<float>(wx);
      const float wy_f = static_cast<float>(wy);
      const float wz_f = static_cast<float>(wz);
      std::memcpy(ptr + x_field->offset, &wx_f, sizeof(float));
      std::memcpy(ptr + y_field->offset, &wy_f, sizeof(float));
      std::memcpy(ptr + z_field->offset, &wz_f, sizeof(float));
      ptr += msg->point_step;
    }

    msg->header.frame_id = world_frame_;
    return true;
  }

  void CloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    auto out = *msg;

    if (cloud_transform_with_latest_odom_) {
      if (!TransformCloudWithLatestOdom(&out)) {
        return;
      }
    } else if (rewrite_frame_id_) {
      out.header.frame_id = world_frame_;
    }
    if (!HasField(out, "intensity")) {
      if (alias_rgb_field_as_intensity_ && CanAliasRgbFieldAsIntensity(out)) {
        AliasRgbFieldAsIntensity(&out);
        cloud_pub_->publish(out);
      } else {
        auto intensity_out = AddDefaultIntensityField(out);
        if (!cloud_transform_with_latest_odom_ && rewrite_frame_id_) {
          intensity_out.header.frame_id = world_frame_;
        }
        cloud_pub_->publish(intensity_out);
      }
      return;
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
  bool alias_rgb_field_as_intensity_{true};
  bool cloud_transform_with_latest_odom_{false};
  bool enable_odom_bridge_{true};
  bool enable_cloud_bridge_{true};
  bool enable_imu_bridge_{true};

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

  rclcpp::CallbackGroup::SharedPtr odom_callback_group_;
  rclcpp::CallbackGroup::SharedPtr cloud_callback_group_;
  rclcpp::CallbackGroup::SharedPtr imu_callback_group_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  std::mutex latest_odom_mutex_;
  bool latest_odom_available_{false};
  double latest_odom_translation_x_{0.0};
  double latest_odom_translation_y_{0.0};
  double latest_odom_translation_z_{0.0};
  double latest_odom_qx_{0.0};
  double latest_odom_qy_{0.0};
  double latest_odom_qz_{0.0};
  double latest_odom_qw_{1.0};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OdinAutonomyBridge>();
  rclcpp::executors::MultiThreadedExecutor executor(
      rclcpp::ExecutorOptions(), 3);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
