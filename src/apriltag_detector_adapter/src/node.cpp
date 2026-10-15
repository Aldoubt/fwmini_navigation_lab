#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "apriltag_msgs/msg/april_tag_detection_array.hpp"
#include "fwmini_interfaces/msg/tag_detection.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "apriltag_detector_adapter/core.hpp"

namespace apriltag_detector_adapter {
class Adapter : public rclcpp::Node {
 public:
  Adapter() : Node("apriltag_detector_adapter") {
    target_ = declare_parameter<int>("target_id", 0);
    family_ = declare_parameter<std::string>("family", "tag36h11");
    frame_ = declare_parameter<std::string>("tag_frame", "fwmini_observed_tag");
    timeout_ = declare_parameter("observation_timeout", 0.2);
    if (!std::isfinite(timeout_) || timeout_ <= 0 || family_.empty() || frame_.empty())
      throw std::invalid_argument("invalid observation config");
    gate_ = std::make_unique<QualityGate>(target_,
      declare_parameter<int>("max_hamming", 0), declare_parameter("min_decision_margin", 30.0));
    buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    listener_ = std::make_unique<tf2_ros::TransformListener>(*buffer_);
    pub_ = create_publisher<fwmini_interfaces::msg::TagDetection>(
      declare_parameter<std::string>("output_topic", "/tag_detection"), 1);
    sub_ = create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
      declare_parameter<std::string>("detections_topic", "/fwmini/tags/raw_detections"), 1,
      [this](apriltag_msgs::msg::AprilTagDetectionArray::ConstSharedPtr msg) { receive(*msg); });
    timer_ = create_wall_timer(std::chrono::milliseconds(10), [this]() { try_pose(); });
  }
 private:
  static double steady() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
  }
  void invalid(const std::string &reason) {
    fwmini_interfaces::msg::TagDetection msg;
    msg.header.stamp = now();
    msg.id = target_; msg.family = family_; msg.valid = false; msg.reason = reason;
    pub_->publish(msg);
    pending_.reset();
  }
  void receive(const apriltag_msgs::msg::AprilTagDetectionArray &array) {
    pending_.reset();
    if (array.header.frame_id.empty() || array.header.stamp.sec < 0 ||
        array.header.stamp.nanosec >= 1000000000u ||
        (array.header.stamp.sec == 0 && array.header.stamp.nanosec == 0)) {
      invalid("invalid detection header"); return;
    }
    const auto stamp = rclcpp::Time(array.header.stamp, get_clock()->get_clock_type());
    const double age = (now() - stamp).seconds();
    if (age < -0.05 || age > timeout_) { invalid("stale/future detection"); return; }
    std::optional<fwmini_interfaces::msg::TagDetection> selected;
    for (const auto &d : array.detections) {
      if (d.family != family_ || !gate_->accept(d.id, d.hamming, d.decision_margin)) continue;
      if (selected) { invalid("ambiguous duplicate target"); return; }
      fwmini_interfaces::msg::TagDetection out;
      out.header = array.header; out.id = d.id; out.family = d.family;
      out.decision_margin = d.decision_margin; out.hamming = d.hamming;
      selected = out;
    }
    if (!selected) { invalid("target absent or quality rejected"); return; }
    pending_ = selected;
    receipt_ = steady();
    try_pose(); // TF can arrive after detections: retry without blocking executor.
  }
  void try_pose() {
    if (!pending_) return;
    const auto stamp = rclcpp::Time(pending_->header.stamp, get_clock()->get_clock_type());
    const double age = (now() - stamp).seconds();
    if (steady() - receipt_ >= timeout_ || age > timeout_ || age < -0.05) {
      invalid("pose TF unavailable before observation deadline"); return;
    }
    try {
      const auto transform = buffer_->lookupTransform(pending_->header.frame_id, frame_, stamp);
      const auto &p = transform.transform.translation;
      const auto &q = transform.transform.rotation;
      if (!valid_pose(p.x, p.y, p.z, q.x, q.y, q.z, q.w)) {
        invalid("invalid optical-frame tag pose"); return;
      }
      pending_->pose.position.x = p.x; pending_->pose.position.y = p.y;
      pending_->pose.position.z = p.z; pending_->pose.orientation = q;
      pending_->valid = true; pending_->reason = "observed";
      pub_->publish(*pending_);
      pending_.reset();
    } catch (const tf2::TransformException &) {
      // Never use latest TF for a different image timestamp.
    }
  }
  int target_{0};
  double timeout_{0.2}, receipt_{0};
  std::string family_, frame_;
  std::optional<fwmini_interfaces::msg::TagDetection> pending_;
  std::unique_ptr<QualityGate> gate_;
  std::unique_ptr<tf2_ros::Buffer> buffer_;
  std::unique_ptr<tf2_ros::TransformListener> listener_;
  rclcpp::Publisher<fwmini_interfaces::msg::TagDetection>::SharedPtr pub_;
  rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try { rclcpp::spin(std::make_shared<apriltag_detector_adapter::Adapter>()); }
  catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("apriltag_detector_adapter"), "%s", e.what());
    rclcpp::shutdown(); return 1;
  }
  rclcpp::shutdown(); return 0;
}
