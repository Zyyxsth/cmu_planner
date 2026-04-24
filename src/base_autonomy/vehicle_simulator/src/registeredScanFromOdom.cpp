#include <cmath>
#include <deque>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"

class RegisteredScanFromOdom : public rclcpp::Node
{
public:
  RegisteredScanFromOdom()
  : Node("registered_scan_from_odom")
  {
    point_cloud_topic_ =
      declare_parameter<std::string>("point_cloud_topic", "/lidar/points");
    odom_topic_ =
      declare_parameter<std::string>("odom_topic", "/state_estimation");
    registered_scan_topic_ =
      declare_parameter<std::string>("registered_scan_topic", "/registered_scan");
    state_estimation_at_scan_topic_ =
      declare_parameter<std::string>("state_estimation_at_scan_topic", "/state_estimation_at_scan");
    output_frame_ = declare_parameter<std::string>("output_frame", "map");
    sensor_frame_ = declare_parameter<std::string>("sensor_frame", "sensor_at_scan");
    publish_state_estimation_at_scan_ =
      declare_parameter<bool>("publish_state_estimation_at_scan", false);
    publish_scan_tf_ = declare_parameter<bool>("publish_scan_tf", false);
    odom_buffer_sec_ = declare_parameter<double>("odom_buffer_sec", 2.0);
    max_odom_sync_gap_sec_ = declare_parameter<double>("max_odom_sync_gap_sec", 0.10);
    sensor_offset_x_ = declare_parameter<double>("sensor_offset_x", 0.0);
    sensor_offset_y_ = declare_parameter<double>("sensor_offset_y", 0.0);
    sensor_offset_z_ = declare_parameter<double>("sensor_offset_z", 0.0);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 20,
      [this](const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        odom_buffer_.push_back(*msg);
        has_odom_ = true;
        const double newest_stamp = rclcpp::Time(msg->header.stamp).seconds();
        while (!odom_buffer_.empty()) {
          const double oldest_stamp = rclcpp::Time(odom_buffer_.front().header.stamp).seconds();
          if (newest_stamp - oldest_stamp <= odom_buffer_sec_) {
            break;
          }
          odom_buffer_.pop_front();
        }
      });

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      point_cloud_topic_, rclcpp::SensorDataQoS(),
      std::bind(&RegisteredScanFromOdom::cloudHandler, this, std::placeholders::_1));

    auto registered_scan_qos = rclcpp::QoS(rclcpp::KeepLast(5));
    registered_scan_qos.reliable();
    registered_scan_qos.durability_volatile();
    registered_scan_pub_ =
      create_publisher<sensor_msgs::msg::PointCloud2>(registered_scan_topic_, registered_scan_qos);
    if (publish_state_estimation_at_scan_) {
      state_estimation_at_scan_pub_ =
        create_publisher<nav_msgs::msg::Odometry>(state_estimation_at_scan_topic_, 10);
    }
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

private:
  void cloudHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg)
  {
    if (!has_odom_) {
      return;
    }

    nav_msgs::msg::Odometry odom_at_scan;
    if (!lookupOdomAtStamp(cloud_msg->header.stamp, odom_at_scan)) {
      return;
    }

    const auto & pose = odom_at_scan.pose.pose;
    tf2::Quaternion rotation(
      pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    tf2::Transform sensor_to_map(rotation);
    sensor_to_map.setOrigin(
      tf2::Vector3(pose.position.x, pose.position.y, pose.position.z) +
      tf2::quatRotate(rotation, tf2::Vector3(sensor_offset_x_, sensor_offset_y_, sensor_offset_z_)));

    pcl::PointCloud<pcl::PointXYZI> registered_cloud;
    registered_cloud.reserve(static_cast<size_t>(cloud_msg->width) * cloud_msg->height);

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*cloud_msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*cloud_msg, "z");

    const auto point_count =
      static_cast<size_t>(cloud_msg->width) * static_cast<size_t>(cloud_msg->height);
    for (size_t i = 0; i < point_count; ++i, ++iter_x, ++iter_y, ++iter_z) {
      if (!std::isfinite(*iter_x) || !std::isfinite(*iter_y) || !std::isfinite(*iter_z)) {
        continue;
      }

      const auto mapped_point = sensor_to_map * tf2::Vector3(*iter_x, *iter_y, *iter_z);

      pcl::PointXYZI point;
      point.x = static_cast<float>(mapped_point.x());
      point.y = static_cast<float>(mapped_point.y());
      point.z = static_cast<float>(mapped_point.z());
      point.intensity = 0.0f;
      registered_cloud.push_back(point);
    }

    registered_cloud.width = static_cast<uint32_t>(registered_cloud.size());
    registered_cloud.height = 1;
    registered_cloud.is_dense = false;

    sensor_msgs::msg::PointCloud2 registered_scan_msg;
    pcl::toROSMsg(registered_cloud, registered_scan_msg);
    registered_scan_msg.header.stamp = cloud_msg->header.stamp;
    registered_scan_msg.header.frame_id = output_frame_;
    registered_scan_pub_->publish(registered_scan_msg);

    if (publish_state_estimation_at_scan_ && state_estimation_at_scan_pub_) {
      odom_at_scan.header.stamp = cloud_msg->header.stamp;
      odom_at_scan.header.frame_id = output_frame_;
      odom_at_scan.child_frame_id = sensor_frame_;
      state_estimation_at_scan_pub_->publish(odom_at_scan);
    }

    if (publish_scan_tf_) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header.stamp = cloud_msg->header.stamp;
      transform.header.frame_id = output_frame_;
      transform.child_frame_id = sensor_frame_;
      transform.transform.translation.x = pose.position.x;
      transform.transform.translation.y = pose.position.y;
      transform.transform.translation.z = pose.position.z;
      transform.transform.rotation = pose.orientation;
      tf_broadcaster_->sendTransform(transform);
    }
  }

  bool lookupOdomAtStamp(
    const builtin_interfaces::msg::Time & stamp,
    nav_msgs::msg::Odometry & odom_out)
  {
    if (odom_buffer_.empty()) {
      return false;
    }

    const rclcpp::Time target_time(stamp);
    const double target_sec = target_time.seconds();

    if (odom_buffer_.size() == 1) {
      const double only_gap =
        std::abs(rclcpp::Time(odom_buffer_.front().header.stamp).seconds() - target_sec);
      if (only_gap > max_odom_sync_gap_sec_) {
        return false;
      }
      odom_out = odom_buffer_.front();
      return true;
    }

    for (size_t i = 1; i < odom_buffer_.size(); ++i) {
      const auto & prev = odom_buffer_[i - 1];
      const auto & next = odom_buffer_[i];
      const double prev_sec = rclcpp::Time(prev.header.stamp).seconds();
      const double next_sec = rclcpp::Time(next.header.stamp).seconds();

      if (target_sec < prev_sec || target_sec > next_sec) {
        continue;
      }

      const double span = std::max(1e-6, next_sec - prev_sec);
      const double ratio = std::clamp((target_sec - prev_sec) / span, 0.0, 1.0);

      odom_out = prev;
      odom_out.header.stamp = stamp;

      odom_out.pose.pose.position.x =
        prev.pose.pose.position.x +
        ratio * (next.pose.pose.position.x - prev.pose.pose.position.x);
      odom_out.pose.pose.position.y =
        prev.pose.pose.position.y +
        ratio * (next.pose.pose.position.y - prev.pose.pose.position.y);
      odom_out.pose.pose.position.z =
        prev.pose.pose.position.z +
        ratio * (next.pose.pose.position.z - prev.pose.pose.position.z);

      tf2::Quaternion q_prev, q_next;
      tf2::fromMsg(prev.pose.pose.orientation, q_prev);
      tf2::fromMsg(next.pose.pose.orientation, q_next);
      q_prev.normalize();
      q_next.normalize();
      odom_out.pose.pose.orientation = tf2::toMsg(q_prev.slerp(q_next, ratio));

      odom_out.twist.twist.linear.x =
        prev.twist.twist.linear.x +
        ratio * (next.twist.twist.linear.x - prev.twist.twist.linear.x);
      odom_out.twist.twist.linear.y =
        prev.twist.twist.linear.y +
        ratio * (next.twist.twist.linear.y - prev.twist.twist.linear.y);
      odom_out.twist.twist.linear.z =
        prev.twist.twist.linear.z +
        ratio * (next.twist.twist.linear.z - prev.twist.twist.linear.z);
      odom_out.twist.twist.angular.x =
        prev.twist.twist.angular.x +
        ratio * (next.twist.twist.angular.x - prev.twist.twist.angular.x);
      odom_out.twist.twist.angular.y =
        prev.twist.twist.angular.y +
        ratio * (next.twist.twist.angular.y - prev.twist.twist.angular.y);
      odom_out.twist.twist.angular.z =
        prev.twist.twist.angular.z +
        ratio * (next.twist.twist.angular.z - prev.twist.twist.angular.z);
      return true;
    }

    const auto & front = odom_buffer_.front();
    const auto & back = odom_buffer_.back();
    const double front_gap = std::abs(rclcpp::Time(front.header.stamp).seconds() - target_sec);
    const double back_gap = std::abs(rclcpp::Time(back.header.stamp).seconds() - target_sec);
    if (std::min(front_gap, back_gap) > max_odom_sync_gap_sec_) {
      return false;
    }

    odom_out = (front_gap <= back_gap) ? front : back;
    odom_out.header.stamp = stamp;
    return true;
  }

  std::string point_cloud_topic_;
  std::string odom_topic_;
  std::string registered_scan_topic_;
  std::string state_estimation_at_scan_topic_;
  std::string output_frame_;
  std::string sensor_frame_;
  bool publish_state_estimation_at_scan_ = false;
  bool publish_scan_tf_ = false;
  double sensor_offset_x_ = 0.0;
  double sensor_offset_y_ = 0.0;
  double sensor_offset_z_ = 0.0;
  double odom_buffer_sec_ = 2.0;
  double max_odom_sync_gap_sec_ = 0.10;
  bool has_odom_ = false;
  std::deque<nav_msgs::msg::Odometry> odom_buffer_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr registered_scan_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr state_estimation_at_scan_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RegisteredScanFromOdom>());
  rclcpp::shutdown();
  return 0;
}
