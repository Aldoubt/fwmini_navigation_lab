#include <chrono>
#include <csignal>
#include <atomic>
#include <cstdlib>
#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>
#include <iomanip>
#include <sstream>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "yhs_can_interfaces/msg/ctrl_cmd.hpp"
#include "yhs_can_interfaces/msg/chassis_info_fb.hpp"
#include "yhs_can_interfaces/msg/io_fb.hpp"
#include "fwmini_chassis_adapter/core.hpp"

namespace fwmini_chassis_adapter {
class Adapter : public rclcpp::Node {
 public:
  Adapter() : Node("fwmini_chassis_adapter") {
    // Same-host second instances cannot even create a CtrlCmd publisher.
    const char *domain=std::getenv("ROS_DOMAIN_ID");
    lock_=::open((std::string("/tmp/fwmini_ctrl_cmd_")+(domain ? domain : "0")+".lock").c_str(), O_CREAT|O_RDWR, 0600);
    if (lock_ < 0 || ::flock(lock_, LOCK_EX|LOCK_NB) != 0) throw std::runtime_error("another FW-mini CtrlCmd owner exists");
    Limits limits{declare_parameter("max_linear_x", 0.05), declare_parameter("max_linear_y", 0.03),
                  declare_parameter("max_angular_z", 0.15), declare_parameter("cmd_timeout", 0.2)};
    Timing timing{declare_parameter("feedback_timeout", 0.2), declare_parameter("switch_timeout", 3.0),
                  declare_parameter("confirm_duration", 0.06), static_cast<int>(declare_parameter("confirm_samples", 3))};
    gate_=std::make_unique<Controller>(limits,timing);
    gate_->epoch=static_cast<uint64_t>(std::chrono::steady_clock::now().time_since_epoch().count());
    units_=std::make_unique<Units>(declare_parameter<std::string>("vendor_linear_unit","mps"),
                                  declare_parameter<std::string>("vendor_angular_unit","degps"));
    if (declare_parameter("control_gear",0) != 0) throw std::invalid_argument("startup control_gear must be 0; use mode-selection services");
    if (declare_parameter("publish_rate",50.0) != 50.0) throw std::invalid_argument("FW-mini CtrlCmd rate must remain 50 Hz");
    pub_=create_publisher<yhs_can_interfaces::msg::CtrlCmd>("/ctrl_cmd",rclcpp::QoS(1));
    status_=create_publisher<std_msgs::msg::String>("/fwmini/control_status",rclcpp::QoS(1).transient_local());
    sub_=create_subscription<geometry_msgs::msg::TwistStamped>("/fwmini/cmd_vel_guarded",rclcpp::QoS(1),
      [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr msg) {
        const auto &v=msg->twist;
        // Epoch prevents old buffered or in-flight guard messages from being replayed.
        if (msg->header.frame_id != "fwmini_epoch_"+std::to_string(gate_->epoch)) return;
        const double age=(now()-rclcpp::Time(msg->header.stamp)).seconds();
        if (!std::isfinite(age) || age<0 || age>=0.2) { gate_->fault("guard_command_stale"); notify(); return; }
        last_guard_epoch_=gate_->epoch; last_guard_stamp_=steady_now();
        if (!std::isfinite(v.linear.z) || !std::isfinite(v.angular.x) || !std::isfinite(v.angular.y) ||
            v.linear.z != 0 || v.angular.x != 0 || v.angular.y != 0) gate_->fault("unsupported_velocity_axis");
        else gate_->receive({v.linear.x,v.linear.y,v.angular.z},gate_->epoch,steady_now());
        notify();
      });
    feedback_=create_subscription<yhs_can_interfaces::msg::ChassisInfoFb>("/chassis_info_fb",rclcpp::QoS(10),
      [this](yhs_can_interfaces::msg::ChassisInfoFb::ConstSharedPtr msg) {
        const auto stamp=rclcpp::Time(msg->header.stamp).nanoseconds();
        const double age=(now()-rclcpp::Time(msg->header.stamp)).seconds();
        if (stamp<=last_feedback_stamp_ || age<0 || age>=0.2) { gate_->fault("invalid_ctrl_feedback_stamp"); notify(); return; }
        last_feedback_stamp_=stamp;
        const auto &fb=msg->ctrl_fb;
        // Immediate safety check too, even if independent IO stream has stalled.
        if (msg->io_fb.io_fb_estop) gate_->fault("emergency_stop");
        if (msg->io_fb.io_fb_joypad_ctrl) gate_->fault("control_authority_lost");
        gate_->feedback(fb.ctrl_fb_gear,{fb.ctrl_fb_x_linear,fb.ctrl_fb_y_linear,fb.ctrl_fb_z_angular*pi/180.},steady_now());
        notify();
      });
    io_=create_subscription<yhs_can_interfaces::msg::IoFb>("/fwmini/io_fb",rclcpp::QoS(1),
      [this](yhs_can_interfaces::msg::IoFb::ConstSharedPtr msg) {
        gate_->io(msg->io_fb_estop,msg->io_fb_joypad_ctrl,steady_now());
        notify();
      });
    external_=create_subscription<std_msgs::msg::Bool>("/fwmini/external_stop",rclcpp::QoS(1).transient_local(),
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) { gate_->external_stop(msg->data); notify(); });
    guard_fault_=create_subscription<std_msgs::msg::String>("/fwmini/guard_fault",rclcpp::QoS(10),
      [this](std_msgs::msg::String::ConstSharedPtr msg) {
        std::string reason="guard_rejected_command";
        for (const auto *candidate : {"unsupported_velocity_axis", "nonfinite_command", "adapter_status_stale",
             "upstream_command_timeout", "invalid_adapter_status", "source_handoff_requires_reenable", "autonomous_source_not_enabled"})
          if (msg->data == candidate) reason=std::string("guard:")+candidate;
        gate_->fault(reason); notify();
      });
    enable_=create_service<std_srvs::srv::SetBool>("/fwmini/enable_motion",
      [this](const std_srvs::srv::SetBool::Request::SharedPtr req,std_srvs::srv::SetBool::Response::SharedPtr res) {
        if (req->data) {
          if (gate_->state == State::READY && (last_guard_epoch_ != gate_->epoch || steady_now()-last_guard_stamp_ >= 0.2)) {
            gate_->reason="enable_requires_fresh_guard_epoch"; res->success=false;
          } else res->success=gate_->enable(steady_now());
        }
        else { gate_->disable(); res->success=true; }
        notify(); res->message=snapshot();
      });
    four_=mode_service("/fwmini/select_four_wheel_steer","four_wheel_steer");
    lateral_=mode_service("/fwmini/select_lateral","lateral");
    stop_=create_service<std_srvs::srv::Trigger>("/fwmini/stop",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,std_srvs::srv::Trigger::Response::SharedPtr res) {
        gate_->fault("external_stop_service"); notify(); res->success=true; res->message=snapshot();
      });
    query_=create_service<std_srvs::srv::Trigger>("/fwmini/get_control_state",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,std_srvs::srv::Trigger::Response::SharedPtr res) {
        notify(); res->success=true; res->message=snapshot();
      });
    timer_=create_wall_timer(std::chrono::milliseconds(20),[this]() { publish(); });
    publish();
  }
  ~Adapter() override { if (lock_>=0) ::close(lock_); }
  void stop() {
    gate_->disable("node_stopping");
    for (int i=0;i<3;++i) {
      publish();
      if (i<2) std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  }
 private:
  static double steady_now() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
  }
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr mode_service(const std::string &service,const std::string &mode) {
    return create_service<std_srvs::srv::Trigger>(service,
      [this,mode](const std_srvs::srv::Trigger::Request::SharedPtr,std_srvs::srv::Trigger::Response::SharedPtr res) {
        res->success=gate_->request(mode,steady_now()); publish(); res->message=snapshot();
      });
  }
  std::string snapshot() const {
    const auto v=gate_->output(); const double t=steady_now();
    std::ostringstream out; out<<std::setprecision(12);
    out<<"{\"state\":\""<<name(gate_->state)<<"\",\"requested_mode\":\""<<gate_->mode
       <<"\",\"target_gear\":"<<gate_->target<<",\"actual_gear\":"<<gate_->actual
       <<",\"guard_epoch_seen\":"<<last_guard_epoch_
       <<",\"guard_feedback_age_sec\":"<<(last_guard_stamp_>0?t-last_guard_stamp_:-1)
       <<",\"output_gear\":"<<gate_->output_gear()<<",\"feedback_age_sec\":"<<gate_->feedback_age(t)
       <<",\"io_feedback_age_sec\":"<<gate_->io_age(t)<<",\"motion_allowed\":"<<(gate_->state==State::ACTIVE?"true":"false")
       <<",\"command_mps_radps\":["<<v.x<<","<<v.y<<","<<v.z<<"],\"epoch\":"<<gate_->epoch
       <<",\"reason\":\""<<gate_->reason<<"\",\"last_mode_switch_result\":\""<<gate_->switch_result<<"\"}";
    return out.str();
  }
  void notify() {
    gate_->tick(steady_now());
    // A new transition gets an immediate zero; repeated fault feedback must
    // not turn the 50 Hz CtrlCmd timer into an uncontrolled publish loop.
    if (gate_->epoch != last_published_epoch_) publish();
  }
  void update_status() {
    std_msgs::msg::String status; status.data=snapshot(); status_->publish(status);
  }
  void publish() {
    // CtrlCmd protocol period is 10..50 ms. Transitions clear the core
    // immediately, but even priority zero frames respect the 10 ms minimum.
    const double remaining=0.010-(steady_now()-last_send_stamp_);
    if (remaining>0) std::this_thread::sleep_for(std::chrono::duration<double>(remaining));
    gate_->tick(steady_now());
    const bool conflict=count_publishers("/ctrl_cmd")>1;
    if (conflict) gate_->fault("duplicate_ctrl_cmd_publisher");
    const auto v=units_->encode(gate_->output());
    yhs_can_interfaces::msg::CtrlCmd msg;
    msg.ctrl_cmd_gear=static_cast<uint8_t>(gate_->output_gear());
    msg.ctrl_cmd_x_linear=static_cast<float>(v.x); msg.ctrl_cmd_y_linear=static_cast<float>(v.y); msg.ctrl_cmd_z_angular=static_cast<float>(v.z);
    pub_->publish(msg);
    last_send_stamp_=steady_now();
    last_published_epoch_=gate_->epoch;
    update_status();
    if (conflict) throw std::runtime_error("duplicate_ctrl_cmd_publisher; adapter relinquishing ownership");
  }
  int lock_{-1}; int64_t last_feedback_stamp_{0}; uint64_t last_published_epoch_{0}, last_guard_epoch_{0}; double last_guard_stamp_{0}, last_send_stamp_{0};
  std::unique_ptr<Controller> gate_; std::unique_ptr<Units> units_;
  rclcpp::Publisher<yhs_can_interfaces::msg::CtrlCmd>::SharedPtr pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_;
  rclcpp::Subscription<yhs_can_interfaces::msg::ChassisInfoFb>::SharedPtr feedback_;
  rclcpp::Subscription<yhs_can_interfaces::msg::IoFb>::SharedPtr io_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr external_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr guard_fault_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr four_,lateral_,stop_,query_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}
std::atomic_bool running{true};
void interrupt(int) { running=false; }
int main(int argc,char **argv) {
  rclcpp::init(argc,argv,rclcpp::InitOptions(),rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT,interrupt); std::signal(SIGTERM,interrupt);
  int result=0;
  try {
    auto node=std::make_shared<fwmini_chassis_adapter::Adapter>();
    while (running && rclcpp::ok()) { rclcpp::spin_some(node); std::this_thread::sleep_for(std::chrono::milliseconds(2)); }
    if (rclcpp::ok()) node->stop();
  } catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("fwmini_chassis_adapter"),"%s",e.what()); result=1;
  }
  rclcpp::shutdown(); return result;
}
