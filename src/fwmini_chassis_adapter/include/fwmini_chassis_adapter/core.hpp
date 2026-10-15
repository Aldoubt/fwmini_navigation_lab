#pragma once
#include <algorithm>
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
class CommandGate {
 public:
  explicit CommandGate(Limits limits) : limits_(limits) {
    for (double value : {limits.x, limits.y, limits.z, limits.timeout})
      if (!std::isfinite(value) || value <= 0)
        throw std::invalid_argument("limits and cmd_timeout must be finite and positive");
  }
  void enable(bool enabled) {
    enabled_ = enabled;
    received_ = false;  // Never replay a command from before enable/disable.
    command_ = {};
  }
  bool receive(Velocity v, double steady_time) {
    if (!finite(v) || !std::isfinite(steady_time)) {
      received_ = false;
      command_ = {};
      return false;
    }
    command_ = {std::clamp(v.x, -limits_.x, limits_.x),
                std::clamp(v.y, -limits_.y, limits_.y),
                std::clamp(v.z, -limits_.z, limits_.z)};
    stamp_ = steady_time;
    received_ = true;
    return true;
  }
  Velocity output(double steady_time) const {
    const double age = steady_time - stamp_;
    if (!enabled_ || !received_ || !std::isfinite(age) ||
        age < 0 || age >= limits_.timeout) return {};
    return command_;
  }
 private:
  Limits limits_;
  bool enabled_{false}, received_{false};
  Velocity command_;
  double stamp_{0};
};
}  // namespace fwmini_chassis_adapter
