#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>

#include "ception_msgs/msg/imu_euler.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "motion_msgs/msg/leg_motors.hpp"
#include "motion_msgs/msg/motion_ctrl.hpp"
#include "motion_msgs/msg/robot_status.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

class D1CompatBridge : public rclcpp::Node {
 public:
  D1CompatBridge() : Node("d1_compat_bridge_node") {
    declare_parameter<std::string>("input_motion_cmd_topic", "diablo/MotionCmd");
    declare_parameter<std::string>("input_joint_states_topic", "joint_states");
    declare_parameter<std::string>("input_imu_topic", "imu_sensor_broadcaster/imu");
    declare_parameter<std::string>("input_fsm_topic", "rl_controller/fsm");
    declare_parameter<std::string>("input_motors_status_topic",
                                   "system_status_broadcaster/motors_status");
    declare_parameter<std::string>("input_odom_topic", "");
    declare_parameter<std::string>("output_twist_topic", "command/cmd_twist");
    declare_parameter<std::string>("output_fsm_topic", "command/cmd_key");
    declare_parameter<std::string>("output_state_estimation_topic", "");
    declare_parameter<std::string>("output_robot_status_topic",
                                   "diablo/sensor/Body_state");
    declare_parameter<std::string>("output_leg_motors_topic",
                                   "diablo/sensor/Motors");
    declare_parameter<std::string>("output_imu_euler_topic",
                                   "/diablo/sensor/ImuEuler");
    declare_parameter<std::string>("vehicle_frame", "vehicle");
    declare_parameter<std::string>("standup_key", "rl_1");
    declare_parameter<std::string>("standdown_key", "transform_down");
    declare_parameter<double>("nominal_leg_length", 0.40);
    declare_parameter<int>("robot_mode_ready_value", 3);
    declare_parameter<int>("robot_mode_not_ready_value", 0);
    declare_parameter<int>("ctrl_mode_value", 0);
    declare_parameter<double>("status_timeout_sec", 1.0);
    declare_parameter<double>("status_publish_rate_hz", 20.0);
    declare_parameter<bool>("publish_twist_from_motion_cmd", true);

    input_motion_cmd_topic_ =
        get_parameter("input_motion_cmd_topic").as_string();
    input_joint_states_topic_ =
        get_parameter("input_joint_states_topic").as_string();
    input_imu_topic_ = get_parameter("input_imu_topic").as_string();
    input_fsm_topic_ = get_parameter("input_fsm_topic").as_string();
    input_motors_status_topic_ =
        get_parameter("input_motors_status_topic").as_string();
    input_odom_topic_ = get_parameter("input_odom_topic").as_string();
    output_twist_topic_ = get_parameter("output_twist_topic").as_string();
    output_fsm_topic_ = get_parameter("output_fsm_topic").as_string();
    output_state_estimation_topic_ =
        get_parameter("output_state_estimation_topic").as_string();
    output_robot_status_topic_ =
        get_parameter("output_robot_status_topic").as_string();
    output_leg_motors_topic_ =
        get_parameter("output_leg_motors_topic").as_string();
    output_imu_euler_topic_ =
        get_parameter("output_imu_euler_topic").as_string();
    vehicle_frame_ = get_parameter("vehicle_frame").as_string();
    standup_key_ = get_parameter("standup_key").as_string();
    standdown_key_ = get_parameter("standdown_key").as_string();
    nominal_leg_length_ = get_parameter("nominal_leg_length").as_double();
    robot_mode_ready_value_ = get_parameter("robot_mode_ready_value").as_int();
    robot_mode_not_ready_value_ =
        get_parameter("robot_mode_not_ready_value").as_int();
    ctrl_mode_value_ = get_parameter("ctrl_mode_value").as_int();
    status_timeout_sec_ = get_parameter("status_timeout_sec").as_double();
    status_publish_rate_hz_ =
        std::max(1.0, get_parameter("status_publish_rate_hz").as_double());
    publish_twist_from_motion_cmd_ =
        get_parameter("publish_twist_from_motion_cmd").as_bool();

    motion_cmd_sub_ = create_subscription<motion_msgs::msg::MotionCtrl>(
        input_motion_cmd_topic_, 20,
        std::bind(&D1CompatBridge::MotionCmdCallback, this,
                  std::placeholders::_1));
    joint_states_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        input_joint_states_topic_, 50,
        std::bind(&D1CompatBridge::JointStatesCallback, this,
                  std::placeholders::_1));
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        input_imu_topic_, 50,
        std::bind(&D1CompatBridge::ImuCallback, this, std::placeholders::_1));
    fsm_sub_ = create_subscription<std_msgs::msg::String>(
        input_fsm_topic_, 20,
        std::bind(&D1CompatBridge::FsmCallback, this, std::placeholders::_1));
    motors_status_sub_ =
        create_subscription<diagnostic_msgs::msg::DiagnosticStatus>(
            input_motors_status_topic_, 20,
            std::bind(&D1CompatBridge::MotorsStatusCallback, this,
                      std::placeholders::_1));

    if (!input_odom_topic_.empty() && !output_state_estimation_topic_.empty()) {
      odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
          input_odom_topic_, 20,
          std::bind(&D1CompatBridge::OdomCallback, this,
                    std::placeholders::_1));
      state_estimation_pub_ = create_publisher<nav_msgs::msg::Odometry>(
          output_state_estimation_topic_, 20);
    }

    if (publish_twist_from_motion_cmd_ && !output_twist_topic_.empty()) {
      twist_stamped_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
          output_twist_topic_, 20);
    }
    fsm_pub_ =
        create_publisher<std_msgs::msg::String>(output_fsm_topic_, 10);
    robot_status_pub_ = create_publisher<motion_msgs::msg::RobotStatus>(
        output_robot_status_topic_, 10);
    leg_motors_pub_ = create_publisher<motion_msgs::msg::LegMotors>(
        output_leg_motors_topic_, 10);
    imu_euler_pub_ = create_publisher<ception_msgs::msg::IMUEuler>(
        output_imu_euler_topic_, 20);

    status_timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / status_publish_rate_hz_),
        std::bind(&D1CompatBridge::PublishStatus, this));

    RCLCPP_INFO(
        get_logger(),
        "D1 compat bridge started. cmd:%s joints:%s imu:%s fsm:%s motors:%s twist:%s",
        input_motion_cmd_topic_.c_str(), input_joint_states_topic_.c_str(),
        input_imu_topic_.c_str(), input_fsm_topic_.c_str(),
        input_motors_status_topic_.c_str(), output_twist_topic_.c_str());
  }

 private:
  void MotionCmdCallback(const motion_msgs::msg::MotionCtrl::SharedPtr msg) {
    if (msg->mode_mark) {
      HandleModeCommand(*msg);
      return;
    }

    if (!publish_twist_from_motion_cmd_ || !twist_stamped_pub_) {
      return;
    }

    geometry_msgs::msg::TwistStamped twist_msg;
    twist_msg.header.stamp = now();
    twist_msg.header.frame_id = vehicle_frame_;
    twist_msg.twist.linear.x = msg->value.forward;
    twist_msg.twist.linear.y = 0.0;
    twist_msg.twist.linear.z = 0.0;
    twist_msg.twist.angular.x = 0.0;
    twist_msg.twist.angular.y = 0.0;
    twist_msg.twist.angular.z = msg->value.left;
    twist_stamped_pub_->publish(twist_msg);
  }

  void JointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    latest_joint_states_ = *msg;
    last_joint_state_time_ = now();
    joint_name_to_index_.clear();
    for (size_t i = 0; i < msg->name.size(); ++i) {
      joint_name_to_index_[msg->name[i]] = i;
    }
  }

  void ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    latest_imu_ = *msg;
    last_imu_time_ = now();
    PublishEulerFromQuaternion(msg->header, msg->orientation.x,
                               msg->orientation.y, msg->orientation.z,
                               msg->orientation.w);
  }

  void FsmCallback(const std_msgs::msg::String::SharedPtr msg) {
    latest_fsm_ = *msg;
    last_fsm_time_ = now();
  }

  void MotorsStatusCallback(
      const diagnostic_msgs::msg::DiagnosticStatus::SharedPtr msg) {
    latest_motors_status_ = *msg;
    last_motors_status_time_ = now();
  }

  void OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    if (state_estimation_pub_) {
      state_estimation_pub_->publish(*msg);
    }
  }

  void HandleModeCommand(const motion_msgs::msg::MotionCtrl &msg) {
    std::string key_to_publish;
    if (msg.mode.stand_mode) {
      key_to_publish = standup_key_;
    } else {
      key_to_publish = standdown_key_;
    }

    if (key_to_publish.empty() || key_to_publish == last_published_key_) {
      return;
    }

    std_msgs::msg::String key_msg;
    key_msg.data = key_to_publish;
    fsm_pub_->publish(key_msg);
    last_published_key_ = key_to_publish;
  }

  void PublishStatus() {
    const auto current_time = now();

    motion_msgs::msg::RobotStatus robot_status;
    robot_status.header.stamp = current_time;
    robot_status.header.frame_id = vehicle_frame_;
    robot_status.ctrl_mode_msg = static_cast<uint8_t>(ctrl_mode_value_);
    robot_status.robot_mode_msg = static_cast<uint8_t>(
        HasFreshState(current_time) ? robot_mode_ready_value_
                                    : robot_mode_not_ready_value_);
    robot_status.error_msg =
        latest_motors_status_.level >= diagnostic_msgs::msg::DiagnosticStatus::ERROR
            ? 1u
            : 0u;
    robot_status.warning_msg =
        latest_motors_status_.level == diagnostic_msgs::msg::DiagnosticStatus::WARN
            ? 1u
            : 0u;
    robot_status_pub_->publish(robot_status);

    motion_msgs::msg::LegMotors leg_motors;
    leg_motors.header = robot_status.header;
    PopulateLegMotors(leg_motors);
    leg_motors_pub_->publish(leg_motors);
  }

  bool HasFreshState(const rclcpp::Time &current_time) const {
    const bool imu_fresh =
        last_imu_time_.nanoseconds() > 0 &&
        (current_time - last_imu_time_).seconds() <= status_timeout_sec_;
    const bool joints_fresh =
        last_joint_state_time_.nanoseconds() > 0 &&
        (current_time - last_joint_state_time_).seconds() <= status_timeout_sec_;
    return imu_fresh || joints_fresh;
  }

  void PopulateLegMotors(motion_msgs::msg::LegMotors &leg_motors) const {
    leg_motors.left_hip_pos = GetJointPosition("FL_hip_joint");
    leg_motors.left_hip_vel = GetJointVelocity("FL_hip_joint");
    leg_motors.left_hip_iq = GetJointEffort("FL_hip_joint");

    leg_motors.left_knee_pos = GetJointPosition("FL_calf_joint");
    leg_motors.left_knee_vel = GetJointVelocity("FL_calf_joint");
    leg_motors.left_knee_iq = GetJointEffort("FL_calf_joint");

    leg_motors.left_wheel_pos = GetJointPosition("FL_foot_joint");
    leg_motors.left_wheel_vel = GetJointVelocity("FL_foot_joint");
    leg_motors.left_wheel_iq = GetJointEffort("FL_foot_joint");

    leg_motors.right_hip_pos = GetJointPosition("FR_hip_joint");
    leg_motors.right_hip_vel = GetJointVelocity("FR_hip_joint");
    leg_motors.right_hip_iq = GetJointEffort("FR_hip_joint");

    leg_motors.right_knee_pos = GetJointPosition("FR_calf_joint");
    leg_motors.right_knee_vel = GetJointVelocity("FR_calf_joint");
    leg_motors.right_knee_iq = GetJointEffort("FR_calf_joint");

    leg_motors.right_wheel_pos = GetJointPosition("FR_foot_joint");
    leg_motors.right_wheel_vel = GetJointVelocity("FR_foot_joint");
    leg_motors.right_wheel_iq = GetJointEffort("FR_foot_joint");

    leg_motors.left_leg_length = nominal_leg_length_;
    leg_motors.right_leg_length = nominal_leg_length_;
  }

  double GetJointPosition(const std::string &name) const {
    return GetJointValue(name, latest_joint_states_.position);
  }

  double GetJointVelocity(const std::string &name) const {
    return GetJointValue(name, latest_joint_states_.velocity);
  }

  double GetJointEffort(const std::string &name) const {
    return GetJointValue(name, latest_joint_states_.effort);
  }

  double GetJointValue(const std::string &name,
                       const std::vector<double> &values) const {
    const auto iter = joint_name_to_index_.find(name);
    if (iter == joint_name_to_index_.end()) {
      return 0.0;
    }
    const size_t index = iter->second;
    if (index >= values.size()) {
      return 0.0;
    }
    return values[index];
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
  std::string input_joint_states_topic_;
  std::string input_imu_topic_;
  std::string input_fsm_topic_;
  std::string input_motors_status_topic_;
  std::string input_odom_topic_;
  std::string output_twist_topic_;
  std::string output_fsm_topic_;
  std::string output_state_estimation_topic_;
  std::string output_robot_status_topic_;
  std::string output_leg_motors_topic_;
  std::string output_imu_euler_topic_;
  std::string vehicle_frame_;
  std::string standup_key_;
  std::string standdown_key_;
  std::string last_published_key_;

  double nominal_leg_length_{0.40};
  int robot_mode_ready_value_{3};
  int robot_mode_not_ready_value_{0};
  int ctrl_mode_value_{0};
  double status_timeout_sec_{1.0};
  double status_publish_rate_hz_{20.0};
  bool publish_twist_from_motion_cmd_{true};

  sensor_msgs::msg::JointState latest_joint_states_;
  sensor_msgs::msg::Imu latest_imu_;
  std_msgs::msg::String latest_fsm_;
  diagnostic_msgs::msg::DiagnosticStatus latest_motors_status_;
  std::unordered_map<std::string, size_t> joint_name_to_index_;

  rclcpp::Time last_joint_state_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_imu_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_fsm_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_motors_status_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<motion_msgs::msg::MotionCtrl>::SharedPtr motion_cmd_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
      joint_states_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr fsm_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr
      motors_status_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr
      twist_stamped_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr fsm_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr state_estimation_pub_;
  rclcpp::Publisher<motion_msgs::msg::RobotStatus>::SharedPtr robot_status_pub_;
  rclcpp::Publisher<motion_msgs::msg::LegMotors>::SharedPtr leg_motors_pub_;
  rclcpp::Publisher<ception_msgs::msg::IMUEuler>::SharedPtr imu_euler_pub_;

  rclcpp::TimerBase::SharedPtr status_timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<D1CompatBridge>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
