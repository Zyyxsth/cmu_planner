#include <cmath>
#include <memory>
#include <string>

#include "ception_msgs/msg/imu_euler.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "motion_msgs/msg/leg_motors.hpp"
#include "motion_msgs/msg/motion_ctrl.hpp"
#include "motion_msgs/msg/robot_status.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

class D1CompatBridge : public rclcpp::Node {
 public:
  D1CompatBridge() : Node("d1_compat_bridge_node") {
    declare_parameter<std::string>("input_motion_cmd_topic", "diablo/MotionCmd");
    declare_parameter<std::string>("input_odom_topic", "/d1/odom");
    declare_parameter<std::string>("input_imu_topic", "/d1/imu");
    declare_parameter<std::string>("output_twist_topic", "/d1/cmd_vel");
    declare_parameter<std::string>("output_twist_stamped_topic", "/d1/cmd_vel_stamped");
    declare_parameter<std::string>("output_state_estimation_topic", "/state_estimation");
    declare_parameter<std::string>("output_robot_status_topic", "diablo/sensor/Body_state");
    declare_parameter<std::string>("output_leg_motors_topic", "diablo/sensor/Motors");
    declare_parameter<std::string>("output_imu_euler_topic", "/diablo/sensor/ImuEuler");
    declare_parameter<std::string>("vehicle_frame", "vehicle");
    declare_parameter<double>("nominal_leg_length", 0.40);
    declare_parameter<int>("robot_mode_ready_value", 3);
    declare_parameter<int>("ctrl_mode_value", 0);
    declare_parameter<bool>("use_odom_orientation_for_euler", true);

    input_motion_cmd_topic_ = get_parameter("input_motion_cmd_topic").as_string();
    input_odom_topic_ = get_parameter("input_odom_topic").as_string();
    input_imu_topic_ = get_parameter("input_imu_topic").as_string();
    output_twist_topic_ = get_parameter("output_twist_topic").as_string();
    output_twist_stamped_topic_ =
        get_parameter("output_twist_stamped_topic").as_string();
    output_state_estimation_topic_ =
        get_parameter("output_state_estimation_topic").as_string();
    output_robot_status_topic_ =
        get_parameter("output_robot_status_topic").as_string();
    output_leg_motors_topic_ =
        get_parameter("output_leg_motors_topic").as_string();
    output_imu_euler_topic_ =
        get_parameter("output_imu_euler_topic").as_string();
    vehicle_frame_ = get_parameter("vehicle_frame").as_string();
    nominal_leg_length_ = get_parameter("nominal_leg_length").as_double();
    robot_mode_ready_value_ = get_parameter("robot_mode_ready_value").as_int();
    ctrl_mode_value_ = get_parameter("ctrl_mode_value").as_int();
    use_odom_orientation_for_euler_ =
        get_parameter("use_odom_orientation_for_euler").as_bool();

    motion_cmd_sub_ = create_subscription<motion_msgs::msg::MotionCtrl>(
        input_motion_cmd_topic_, 10,
        std::bind(&D1CompatBridge::MotionCmdCallback, this,
                  std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        input_odom_topic_, 20,
        std::bind(&D1CompatBridge::OdomCallback, this, std::placeholders::_1));
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        input_imu_topic_, 20,
        std::bind(&D1CompatBridge::ImuCallback, this, std::placeholders::_1));

    twist_pub_ = create_publisher<geometry_msgs::msg::Twist>(output_twist_topic_, 10);
    twist_stamped_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
        output_twist_stamped_topic_, 10);
    state_estimation_pub_ = create_publisher<nav_msgs::msg::Odometry>(
        output_state_estimation_topic_, 20);
    robot_status_pub_ = create_publisher<motion_msgs::msg::RobotStatus>(
        output_robot_status_topic_, 10);
    leg_motors_pub_ = create_publisher<motion_msgs::msg::LegMotors>(
        output_leg_motors_topic_, 10);
    imu_euler_pub_ = create_publisher<ception_msgs::msg::IMUEuler>(
        output_imu_euler_topic_, 20);

    RCLCPP_INFO(
        get_logger(),
        "D1 compat bridge started. cmd:%s odom:%s imu:%s -> state:%s",
        input_motion_cmd_topic_.c_str(), input_odom_topic_.c_str(),
        input_imu_topic_.c_str(), output_state_estimation_topic_.c_str());
  }

 private:
  void MotionCmdCallback(const motion_msgs::msg::MotionCtrl::SharedPtr msg) {
    geometry_msgs::msg::Twist twist_msg;
    twist_msg.linear.x = msg->value.forward;
    twist_msg.angular.z = msg->value.left;
    twist_pub_->publish(twist_msg);

    geometry_msgs::msg::TwistStamped twist_stamped_msg;
    twist_stamped_msg.header.stamp = now();
    twist_stamped_msg.header.frame_id = vehicle_frame_;
    twist_stamped_msg.twist = twist_msg;
    twist_stamped_pub_->publish(twist_stamped_msg);
  }

  void OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    auto output = *msg;
    state_estimation_pub_->publish(output);

    motion_msgs::msg::RobotStatus robot_status;
    robot_status.header = msg->header;
    robot_status.ctrl_mode_msg = static_cast<uint8_t>(ctrl_mode_value_);
    robot_status.robot_mode_msg = static_cast<uint8_t>(robot_mode_ready_value_);
    robot_status.error_msg = 0;
    robot_status.warning_msg = 0;
    robot_status_pub_->publish(robot_status);

    motion_msgs::msg::LegMotors leg_motors;
    leg_motors.header = msg->header;
    leg_motors.left_leg_length = nominal_leg_length_;
    leg_motors.right_leg_length = nominal_leg_length_;
    leg_motors_pub_->publish(leg_motors);

    if (use_odom_orientation_for_euler_) {
      PublishEulerFromQuaternion(msg->header, msg->pose.pose.orientation.x,
                                 msg->pose.pose.orientation.y,
                                 msg->pose.pose.orientation.z,
                                 msg->pose.pose.orientation.w);
    }
  }

  void ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    PublishEulerFromQuaternion(msg->header, msg->orientation.x,
                               msg->orientation.y, msg->orientation.z,
                               msg->orientation.w);
  }

  void PublishEulerFromQuaternion(const std_msgs::msg::Header &header, double x,
                                  double y, double z, double w) {
    tf2::Quaternion quat(x, y, z, w);
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    ception_msgs::msg::IMUEuler imu_euler;
    imu_euler.header = header;
    imu_euler.roll = roll;
    imu_euler.pitch = pitch;
    imu_euler.yaw = yaw;
    imu_euler_pub_->publish(imu_euler);
  }

  std::string input_motion_cmd_topic_;
  std::string input_odom_topic_;
  std::string input_imu_topic_;
  std::string output_twist_topic_;
  std::string output_twist_stamped_topic_;
  std::string output_state_estimation_topic_;
  std::string output_robot_status_topic_;
  std::string output_leg_motors_topic_;
  std::string output_imu_euler_topic_;
  std::string vehicle_frame_;
  double nominal_leg_length_{0.40};
  int robot_mode_ready_value_{3};
  int ctrl_mode_value_{0};
  bool use_odom_orientation_for_euler_{true};

  rclcpp::Subscription<motion_msgs::msg::MotionCtrl>::SharedPtr motion_cmd_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr
      twist_stamped_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr state_estimation_pub_;
  rclcpp::Publisher<motion_msgs::msg::RobotStatus>::SharedPtr robot_status_pub_;
  rclcpp::Publisher<motion_msgs::msg::LegMotors>::SharedPtr leg_motors_pub_;
  rclcpp::Publisher<ception_msgs::msg::IMUEuler>::SharedPtr imu_euler_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<D1CompatBridge>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
