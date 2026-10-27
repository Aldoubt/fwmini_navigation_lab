#pragma once
#include <algorithm>
#include <cstdint>
#include <cmath>
#include <stdexcept>
#include <string>

namespace fwmini_chassis_adapter {
constexpr double pi = 3.14159265358979323846;
struct Velocity { double x{0}, y{0}, z{0}; };
struct Limits { double x{0.3}, y{0.3}, z{0.5}, timeout{0.2}; };
inline bool finite(Velocity v) {
  return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}
// The ONLY boundary conversion. The reviewed YHS ROS API is m/s and deg/s.
// mmps is for a different, explicitly verified vendor ROS API, not CAN packing.
class Units {
 public:
  Units(const std::string &linear, const std::string &angular) {
    if (linear == "mps") linear_scale_ = 1.;
    else if (linear == "mmps") linear_scale_ = 1000.;
    else throw std::invalid_argument("vendor_linear_unit must be mps or mmps");
    if (angular == "degps") angular_scale_ = 180. / pi;
    else if (angular == "radps") angular_scale_ = 1.;
    else throw std::invalid_argument("vendor_angular_unit must be degps or radps");
  }
  Velocity encode(Velocity v) const {
    return {v.x * linear_scale_, v.y * linear_scale_, v.z * angular_scale_};
  }
 private:
  double linear_scale_{1}, angular_scale_{1};
};
// CtrlCmd V1.0.4 p15: 0 disable, 6 4T4D, 8 lateral (not SteeringCtrlCmd 5/7).
enum class State { DISABLED, STOPPING, SWITCHING, READY, ACTIVE, FAULT };
inline const char *name(State state) {
  switch (state) {
    case State::DISABLED: return "DISABLED";
    case State::STOPPING: return "STOPPING";
    case State::SWITCHING: return "SWITCHING";
    case State::READY: return "READY";
    case State::ACTIVE: return "ACTIVE";
    default: return "FAULT";
  }
}
struct Timing {
  double feedback_timeout{0.2}, switch_timeout{3.0}, confirm_duration{0.06};
  int confirm_samples{3};
};
class Controller {
 public:
  explicit Controller(Limits limits = {}, Timing timing = {}) : limits_(limits), timing_(timing) {
    for (double value : {limits.x, limits.y, limits.z, limits.timeout,
                        timing.feedback_timeout, timing.switch_timeout, timing.confirm_duration})
      if (!std::isfinite(value) || value <= 0) throw std::invalid_argument("positive finite limits/timing required");
    if (timing.confirm_samples < 2 || timing.confirm_duration >= timing.switch_timeout)
      throw std::invalid_argument("invalid feedback confirmation timing");
  }
  State state{State::DISABLED};
  std::string mode{"none"}, reason{"startup_disabled"}, switch_result{"NOT_REQUESTED"};
  int target{0}, actual{-1};
  uint64_t epoch{0};
  Velocity command{};
  bool request(const std::string &requested, double now) {
    if (requested != "four_wheel_steer" && requested != "lateral") { reason="invalid_mode"; return false; }
    clear();
    mode=requested; target=(requested == "lateral" ? 8 : 6);
    state=State::STOPPING; phase_start_=now; reset_streak();
    switch_result="PENDING_STOP_CONFIRMATION"; reason="waiting_for_stopped_feedback";
    return true;
  }
  void disable(const std::string &why="operator_disabled") {
    if (state == State::STOPPING || state == State::SWITCHING) switch_result="CANCELLED:"+why;
    clear(); target=0; state=State::DISABLED; reason=why; reset_streak();
  }
  void fault(const std::string &why) {
    if (state == State::FAULT) return;  // Preserve the first cause until explicit recovery.
    if (state != State::FAULT) {
      if (state == State::STOPPING || state == State::SWITCHING) switch_result="FAILED:"+why;
      clear(); state=State::FAULT; reset_streak();
    }
    reason=why;
  }
  void io(bool estop, bool remote, double now) {
    io_stamp_=now; io_seen_=true; estop_=estop; remote_=remote;
    if (estop) fault("emergency_stop");
    else if (remote) fault("control_authority_lost");
  }
  void external_stop(bool stop) {
    external_stop_=stop;
    if (stop) fault("external_stop");
  }
  void feedback(int gear, Velocity velocity, double now) {
    if (!finite(velocity)) { fault("nonfinite_feedback"); return; }
    const bool continuous=feedback_seen_ && now > feedback_stamp_ &&
                          now-feedback_stamp_ < timing_.feedback_timeout;
    actual=gear; feedback_stamp_=now; feedback_seen_=true;
    if (!continuous) reset_streak();
    if (state == State::DISABLED || state == State::FAULT) return;
    if (state == State::STOPPING && !io_seen_) return;
    if (!health(now)) { fault(health_reason(now)); return; }
    const bool stopped=std::abs(velocity.x)<=0.01 && std::abs(velocity.y)<=0.01 && std::abs(velocity.z)<=0.02;
    if (state == State::STOPPING || state == State::SWITCHING) {
      const bool match=stopped && (state == State::STOPPING || actual == target);
      if (!match) { reset_streak(); return; }
      if (streak_++ == 0) streak_start_=now;
      if (streak_ < timing_.confirm_samples || now-streak_start_ < timing_.confirm_duration) return;
      reset_streak();
      clear();
      if (state == State::STOPPING) {
        state=State::SWITCHING; reason="waiting_for_target_gear"; switch_result="PENDING_GEAR_CONFIRMATION";
      } else {
        state=State::READY; reason="ready_requires_explicit_enable"; switch_result="SUCCESS";
      }
    } else if (actual != target) fault("gear_mismatch");
  }
  bool enable(double now) {
    tick(now);
    if (state != State::READY) { if (state != State::FAULT) reason="enable_requires_READY"; return false; }
    if (!health(now) || actual != target) { fault(health_reason(now)); return false; }
    clear(); state=State::ACTIVE; enabled_at_=now; command_stamp_=now; reason="waiting_for_new_command";
    return true;
  }
  bool receive(Velocity velocity, uint64_t command_epoch, double now) {
    tick(now);
    if (command_epoch != epoch || state != State::ACTIVE) return false;
    if (!finite(velocity)) { fault("nonfinite_command"); return false; }
    if ((target == 6 && velocity.y != 0.0) || (target == 8 && velocity.z != 0.0)) {
      fault("unsupported_velocity_axis"); return false;
    }
    command={std::clamp(velocity.x,-limits_.x,limits_.x),std::clamp(velocity.y,-limits_.y,limits_.y),
             std::clamp(velocity.z,-limits_.z,limits_.z)};
    command_stamp_=now; reason="active";
    return true;
  }
  void tick(double now) {
    if (state == State::DISABLED || state == State::FAULT) return;
    if ((state == State::STOPPING || state == State::SWITCHING) && now-phase_start_ >= timing_.switch_timeout) {
      fault("mode_switch_timeout"); return;
    }
    // Before the first valid feedback we remain at gear 0 and never switch.
    if (state == State::STOPPING && (!feedback_seen_ || !io_seen_)) return;
    if (!health(now)) { fault(health_reason(now)); return; }
    if (state == State::ACTIVE && (now-command_stamp_ >= limits_.timeout || now < command_stamp_))
      fault("command_timeout");
  }
  Velocity output() const { return state == State::ACTIVE ? command : Velocity{}; }
  int output_gear() const { return (state == State::SWITCHING || state == State::READY || state == State::ACTIVE) ? target : 0; }
  double feedback_age(double now) const { return feedback_seen_ ? now-feedback_stamp_ : -1; }
  double io_age(double now) const { return io_seen_ ? now-io_stamp_ : -1; }
  double enabled_at() const { return enabled_at_; }
 private:
  void clear() { command={}; ++epoch; }
  void reset_streak() { streak_=0; streak_start_=0; }
  std::string health_reason(double now) const {
    if (external_stop_) return "external_stop";
    if (estop_) return "emergency_stop";
    if (remote_) return "control_authority_lost";
    if (!io_seen_ || now-io_stamp_ < 0 || now-io_stamp_ >= timing_.feedback_timeout) return "io_feedback_stale";
    return "ctrl_feedback_stale";
  }
  bool health(double now) const {
    return !external_stop_ && !estop_ && !remote_ && io_seen_ && feedback_seen_ &&
      now-io_stamp_ >= 0 && now-io_stamp_ < timing_.feedback_timeout &&
      now-feedback_stamp_ >= 0 && now-feedback_stamp_ < timing_.feedback_timeout;
  }
  Limits limits_; Timing timing_;
  bool feedback_seen_{false}, io_seen_{false}, estop_{false}, remote_{false}, external_stop_{false};
  double feedback_stamp_{0}, io_stamp_{0}, phase_start_{0}, streak_start_{0}, command_stamp_{0}, enabled_at_{0};
  int streak_{0};
};
}  // namespace fwmini_chassis_adapter
