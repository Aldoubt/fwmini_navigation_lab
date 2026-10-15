#include <chrono>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "fwmini_interfaces/msg/tag_detection.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tag_follow_controller/core.hpp"

namespace tag_follow_controller {
class Node : public rclcpp::Node {
 public:
  Node() : rclcpp::Node("tag_follow_controller") {
    Config c;
    c.desired = declare_parameter("desired_distance", 0.8);
    c.min_distance = declare_parameter("min_distance", 0.3);
    c.max_distance = declare_parameter("max_distance", 5.0);
    c.linear_gain = declare_parameter("linear_gain", 0.4);
    c.angular_gain = declare_parameter("angular_gain", 1.0);
    c.max_linear = declare_parameter("max_linear_x", 0.2);
    c.max_angular = declare_parameter("max_angular_z", 0.4);
    c.distance_deadband = declare_parameter("distance_deadband", 0.05);
    c.angle_deadband = declare_parameter("angle_deadband", 0.03);
    c.turn_threshold = declare_parameter("turn_threshold", 0.3);
    c.timeout = declare_parameter("tag_timeout", 0.2);
    timeout_ = c.timeout;
    controller_ = std::make_unique<Controller>(c);
    extrinsics_ = declare_parameter("extrinsics_verified", false);
    target_ = declare_parameter<int>("target_id", 0);
    if (target_ < 0) throw std::invalid_argument("target_id must be nonnegative");
    buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    listener_ = std::make_unique<tf2_ros::TransformListener>(*buffer_);
    pub_ = create_publisher<geometry_msgs::msg::Twist>(
      declare_parameter<std::string>("command_topic", "/cmd_vel"), 1);
    sub_ = create_subscription<fwmini_interfaces::msg::TagDetection>(
      declare_parameter<std::string>("detection_topic", "/tag_detection"), 1,
      [this](fwmini_interfaces::msg::TagDetection::ConstSharedPtr msg) { receive(*msg); });
    service_ = create_service<std_srvs::srv::SetBool>("/fwmini/enable_tag_follow",
      [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
             std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
        const bool accepted = !req->data || extrinsics_;
        controller_->enable(accepted && req->data);
        last_stamp_ = -1;
        publish();
        res->success = accepted;
        res->message = accepted ? "State changed; fresh tag required" :
          "Set extrinsics_verified only after measuring the camera mount and verifying TF";
      });
    timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() { publish(); });
  }
 private:
  static double steady() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
  }
  void invalidate() { controller_->invalidate(); publish(); }
  void receive(const fwmini_interfaces::msg::TagDetection &msg) {
    const auto ros_now = now();
    if (have_clock_ && ros_now.nanoseconds() < last_clock_) last_stamp_ = -1;
    have_clock_ = true; last_clock_ = ros_now.nanoseconds();
    if (!extrinsics_ || !msg.valid || msg.id != target_ || msg.header.frame_id.empty() ||
        msg.header.stamp.sec < 0 || msg.header.stamp.nanosec >= 1000000000u ||
        (msg.header.stamp.sec == 0 && msg.header.stamp.nanosec == 0)) {
      invalidate(); return;
    }
    const auto stamp = rclcpp::Time(msg.header.stamp, get_clock()->get_clock_type());
    const double age = (ros_now - stamp).seconds();
    if (age < -0.05 || age > timeout_ || stamp.nanoseconds() <= last_stamp_) {
      invalidate(); return;
    }
    geometry_msgs::msg::PointStamped point;
    point.header = msg.header; point.point = msg.pose.position;
    try {
      const auto transform = buffer_->lookupTransform("base_link", msg.header.frame_id, stamp);
      geometry_msgs::msg::PointStamped body;
      tf2::doTransform(point, body, transform);
      if (!std::isfinite(body.point.z) ||
          !controller_->observe(body.point.x, body.point.y, steady())) { invalidate(); return; }
      last_stamp_ = stamp.nanoseconds();
      observation_stamp_ = stamp.nanoseconds();
    } catch (const tf2::TransformException &e) {
      invalidate();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Camera TF unavailable: %s", e.what());
    }
  }
  void publish() {
    if (observation_stamp_ > 0) {
      const double age = (now().nanoseconds() - observation_stamp_) * 1e-9;
      if (age < -0.05 || age >= timeout_) controller_->invalidate();
    }
    const auto command = controller_->output(steady());
    geometry_msgs::msg::Twist msg;
    msg.linear.x = command.x; msg.linear.y = command.y; msg.angular.z = command.z;
    pub_->publish(msg);
  }
  int target_{0};
  bool extrinsics_{false}, have_clock_{false};
  int64_t last_stamp_{-1}, last_clock_{0}, observation_stamp_{-1};
  double timeout_{0.2};
  std::unique_ptr<Controller> controller_;
  std::unique_ptr<tf2_ros::Buffer> buffer_;
  std::unique_ptr<tf2_ros::TransformListener> listener_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::Subscription<fwmini_interfaces::msg::TagDetection>::SharedPtr sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try { rclcpp::spin(std::make_shared<tag_follow_controller::Node>()); }
  catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("tag_follow_controller"), "%s", e.what());
    rclcpp::shutdown(); return 1;
  }
  rclcpp::shutdown(); return 0;
}
