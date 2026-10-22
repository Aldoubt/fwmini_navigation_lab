#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "yhs_can_interfaces/msg/ctrl_cmd.hpp"
#include "fwmini_chassis_adapter/core.hpp"

using namespace std::chrono_literals;
namespace fwmini_chassis_adapter {
class Adapter : public rclcpp::Node {
 public:
  Adapter() : Node("fwmini_chassis_adapter") {
    Limits limits{declare_parameter("max_linear_x", 0.3),
                  declare_parameter("max_linear_y", 0.3),
                  declare_parameter("max_angular_z", 0.5),
                  declare_parameter("cmd_timeout", 0.2)};
    gate_ = std::make_unique<CommandGate>(limits);
    // Opt-in trial mode: enabling never resurrects an old command.
    // Timeout-to-zero, limits, and explicit disable remain active.
    gate_->enable(declare_parameter("initial_motion_enabled", false));
    units_ = std::make_unique<Units>(declare_parameter<std::string>("vendor_linear_unit", "mps"),
                                    declare_parameter<std::string>("vendor_angular_unit", "degps"));
    gear_ = declare_parameter<int>("control_gear", 0);
    if (gear_ < 0 || gear_ > 15) throw std::invalid_argument("control_gear must be 0..15");
    const double rate = declare_parameter("publish_rate", 50.0);
    if (!std::isfinite(rate) || rate < 10 || rate > 1000)
      throw std::invalid_argument("publish_rate must be 10..1000 Hz");
    pub_ = create_publisher<yhs_can_interfaces::msg::CtrlCmd>(
      declare_parameter<std::string>("vendor_command_topic", "/ctrl_cmd"), rclcpp::QoS(1));
    sub_ = create_subscription<geometry_msgs::msg::Twist>(
      declare_parameter<std::string>("command_topic", "/cmd_vel"), rclcpp::QoS(1),
      [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
        if (!gate_->receive({msg->linear.x, msg->linear.y, msg->angular.z}, steady_now()))
          RCLCPP_WARN(get_logger(), "Rejected non-finite command; output cleared");
      });
    service_ = create_service<std_srvs::srv::SetBool>("/fwmini/enable_motion",
      [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
             std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
        gate_->enable(req->data);
        publish();  // Immediate zero, including when enabling: need a fresh command.
        res->success = true;
        res->message = req->data ? "Enabled; waiting for fresh cmd_vel" : "Disabled; output zero";
      });
    timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1. / rate)), [this]() { publish(); });
  }
  void stop() { gate_->enable(false); publish(); }
 private:
  static double steady_now() {
    return std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  }
  void publish() {
    const auto v = units_->encode(gate_->output(steady_now()));
    yhs_can_interfaces::msg::CtrlCmd msg;
    msg.ctrl_cmd_gear = static_cast<uint8_t>(gear_);
    msg.ctrl_cmd_x_linear = static_cast<float>(v.x);
    msg.ctrl_cmd_y_linear = static_cast<float>(v.y);
    msg.ctrl_cmd_z_angular = static_cast<float>(v.z);
    pub_->publish(msg);
  }
  int gear_{0};
  std::unique_ptr<CommandGate> gate_;
  std::unique_ptr<Units> units_;
  rclcpp::Publisher<yhs_can_interfaces::msg::CtrlCmd>::SharedPtr pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<fwmini_chassis_adapter::Adapter>();
    rclcpp::spin(node);  // Single-threaded executor serializes gate state.
    if (rclcpp::ok()) node->stop();
  } catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("fwmini_chassis_adapter"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
