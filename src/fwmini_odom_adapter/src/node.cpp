#include <algorithm>
#include <array>
#include <chrono>
#include <memory>
#include <stdexcept>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "yhs_can_interfaces/msg/chassis_info_fb.hpp"
#include "fwmini_odom_adapter/core.hpp"

namespace fwmini_odom_adapter {
class Adapter : public rclcpp::Node {
 public:
  Adapter() : Node("fwmini_odom_adapter") {
    timeout_ = declare_parameter("feedback_timeout", 0.2);
    max_dt_ = declare_parameter("max_dt", 0.2);
    future_tolerance_ = declare_parameter("future_tolerance", 0.05);
    if (!std::isfinite(timeout_) || timeout_ <= 0 ||
        !std::isfinite(max_dt_) || max_dt_ <= 0 || max_dt_ > timeout_ ||
        !std::isfinite(future_tolerance_) || future_tolerance_ < 0)
      throw std::invalid_argument("invalid feedback timing limits (max_dt <= feedback_timeout)");
    profile_ = declare_parameter<std::string>("feedback_profile", "mps_degps");
    units_ = std::make_unique<Units>(profile_);
    integrator_ = std::make_unique<Integrator>(max_dt_);
    publish_tf_ = declare_parameter("publish_tf", true);
    // Fixed canonical frames: manufacturer frame_id is deliberately not copied.
    pose_variance_ = covariance("pose_variance", {0.01, 0.01, 1e6, 1e6, 1e6, 0.02});
    twist_variance_ = covariance("twist_variance", {0.01, 0.01, 1e6, 1e6, 1e6, 0.02});
    pub_ = create_publisher<nav_msgs::msg::Odometry>(
      declare_parameter<std::string>("odom_topic", "/wheel/odom"), rclcpp::QoS(10));
    diag_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
    if (publish_tf_) tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    sub_ = create_subscription<yhs_can_interfaces::msg::ChassisInfoFb>(
      declare_parameter<std::string>("feedback_topic", "/chassis_info_fb"), rclcpp::QoS(10),
      [this](yhs_can_interfaces::msg::ChassisInfoFb::ConstSharedPtr msg) { receive(*msg); });
    timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() { diagnose(); });
  }
 private:
  std::array<double, 36> covariance(const std::string &name, std::vector<double> defaults) {
    const auto values = declare_parameter<std::vector<double>>(name, defaults);
    if (values.size() != 6) throw std::invalid_argument(name + " must have six diagonal variances");
    std::array<double, 36> out{};
    for (size_t i = 0; i < 6; ++i) {
      if (!std::isfinite(values[i]) || values[i] <= 0)
        throw std::invalid_argument(name + " must be finite and positive");
      out[i * 7] = values[i];
    }
    return out;
  }
  static double steady_now() {
    return std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  }
  void reject(const std::string &reason) {
    issue_ = reason;
    integrator_->break_continuity();
    // Rejected feedback never refreshes last_valid_.
  }
  void receive(const yhs_can_interfaces::msg::ChassisInfoFb &msg) {
    const auto ros_now = now();
    const double wall = steady_now();
    if (have_ros_now_ && ros_now.nanoseconds() < last_ros_now_) {
      integrator_->break_continuity();
      last_stamp_ = -1; // ROS replay clock epoch changed; keep continuous pose.
      valid_ = false;
    }
    have_ros_now_ = true;
    last_ros_now_ = ros_now.nanoseconds();
    if (msg.header.stamp.sec < 0 || msg.header.stamp.nanosec >= 1000000000u ||
        (msg.header.stamp.sec == 0 && msg.header.stamp.nanosec == 0)) {
      reject("invalid feedback stamp"); return;
    }
    const auto stamp = rclcpp::Time(msg.header.stamp, get_clock()->get_clock_type());
    const double age = (ros_now - stamp).seconds();
    if (age > timeout_ || age < -future_tolerance_) {
      reject("stale, zero or future feedback stamp"); return;
    }
    if (stamp.nanoseconds() <= last_stamp_) { reject("non-increasing feedback stamp"); return; }
    const auto v = units_->decode({msg.ctrl_fb.ctrl_fb_x_linear,
                                   msg.ctrl_fb.ctrl_fb_y_linear,
                                   msg.ctrl_fb.ctrl_fb_z_angular});
    if (!finite(v)) { reject("non-finite feedback"); return; }
    const bool gap = valid_ &&
      (wall - last_valid_ >= timeout_ || (stamp.nanoseconds() - last_stamp_) * 1e-9 > max_dt_);
    if (!valid_ || gap) integrator_->break_continuity();
    if (!integrator_->update(v, stamp.seconds())) { reject("invalid integration sample"); return; }
    valid_ = true;
    last_stamp_ = stamp.nanoseconds();
    last_valid_ = wall;
    issue_ = gap ? "feedback gap: rebaselined; unobserved displacement unknown" : "";
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = msg.header.stamp;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    const auto pose = integrator_->pose();
    odom.pose.pose.position.x = pose.x;
    odom.pose.pose.position.y = pose.y;
    odom.pose.pose.orientation.z = std::sin(pose.yaw / 2.);
    odom.pose.pose.orientation.w = std::cos(pose.yaw / 2.);
    odom.pose.covariance = pose_variance_;
    odom.twist.twist.linear.x = v.x;
    odom.twist.twist.linear.y = v.y;
    odom.twist.twist.angular.z = v.z;
    odom.twist.covariance = twist_variance_;
    pub_->publish(odom);
    if (tf_) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header = odom.header;
      transform.child_frame_id = odom.child_frame_id;
      transform.transform.translation.x = pose.x;
      transform.transform.translation.y = pose.y;
      transform.transform.rotation = odom.pose.pose.orientation;
      tf_->sendTransform(transform);
    }
  }
  void diagnose() {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "fwmini_odom_adapter/feedback";
    status.hardware_id = "fwmini";
    const double age = valid_ ? steady_now() - last_valid_ : -1.;
    const bool healthy = valid_ && age < timeout_;
    status.level = healthy ? (issue_.empty() ? 0 : 1) : 2;
    status.message = !valid_ ? "waiting for valid feedback" :
      !healthy ? "feedback timeout; odom/TF publication stopped" :
      issue_.empty() ? "feedback healthy" : issue_;
    for (const auto &pair : std::vector<std::pair<std::string, std::string>>{
        {"feedback_age_s", std::to_string(age)}, {"profile", profile_}, {"last_issue", issue_}}) {
      diagnostic_msgs::msg::KeyValue value;
      value.key = pair.first; value.value = pair.second;
      status.values.push_back(value);
    }
    array.status.push_back(status);
    diag_->publish(array);
  }
  double timeout_{0.2}, max_dt_{0.2}, future_tolerance_{0.05}, last_valid_{0};
  int64_t last_stamp_{-1}, last_ros_now_{0};
  bool valid_{false}, publish_tf_{true}, have_ros_now_{false};
  std::string profile_, issue_;
  std::array<double, 36> pose_variance_{}, twist_variance_{};
  std::unique_ptr<Units> units_;
  std::unique_ptr<Integrator> integrator_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_;
  rclcpp::Subscription<yhs_can_interfaces::msg::ChassisInfoFb>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<fwmini_odom_adapter::Adapter>());
  } catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("fwmini_odom_adapter"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
