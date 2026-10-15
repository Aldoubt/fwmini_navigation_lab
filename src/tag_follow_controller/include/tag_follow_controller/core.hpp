#pragma once
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace tag_follow_controller {
struct Command { double x{0}, y{0}, z{0}; };
struct Config {
  double desired{0.8}, min_distance{0.3}, max_distance{5.0};
  double linear_gain{0.4}, angular_gain{1.0};
  double max_linear{0.2}, max_angular{0.4};
  double distance_deadband{0.05}, angle_deadband{0.03}, turn_threshold{0.3}, timeout{0.2};
};
class Controller {
 public:
  explicit Controller(Config c) : c_(c) {
    for (double v : {c.desired, c.min_distance, c.max_distance, c.linear_gain,
                     c.angular_gain, c.max_linear, c.max_angular, c.timeout})
      if (!std::isfinite(v) || v <= 0) throw std::invalid_argument("invalid follow parameters");
    for (double v : {c.distance_deadband, c.angle_deadband, c.turn_threshold})
      if (!std::isfinite(v) || v < 0) throw std::invalid_argument("invalid deadband");
    if (c.min_distance >= c.desired || c.max_distance <= c.desired ||
        c.turn_threshold <= c.angle_deadband)
      throw std::invalid_argument("inconsistent follow distance/angle limits");
  }
  void enable(bool enabled) { enabled_ = enabled; invalidate(); }
  void invalidate() { valid_ = false; }
  bool observe(double x, double y, double steady_time) {
    const double distance = std::hypot(x, y);
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(steady_time) ||
        x <= 0 || distance < c_.min_distance || distance > c_.max_distance) {
      invalidate(); return false;
    }
    x_ = x; y_ = y; receipt_ = steady_time; valid_ = true; return true;
  }
  Command output(double steady_time) const {
    const double age = steady_time - receipt_;
    if (!enabled_ || !valid_ || !std::isfinite(age) || age < 0 || age >= c_.timeout) return {};
    const double angle = std::atan2(y_, x_);
    const double error = std::hypot(x_, y_) - c_.desired;
    Command out;
    if (std::abs(angle) > c_.angle_deadband)
      out.z = std::clamp(c_.angular_gain * angle, -c_.max_angular, c_.max_angular);
    // Forward-only demo: stop when close; turn first if target is far off axis.
    if (error > c_.distance_deadband && std::abs(angle) <= c_.turn_threshold)
      out.x = std::min(c_.max_linear, c_.linear_gain * error);
    return out;
  }
 private:
  Config c_;
  bool enabled_{false}, valid_{false};
  double x_{0}, y_{0}, receipt_{0};
};
}
