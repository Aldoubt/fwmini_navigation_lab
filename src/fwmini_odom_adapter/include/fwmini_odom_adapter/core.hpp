#pragma once
#include <cmath>
#include <stdexcept>
#include <string>
namespace fwmini_odom_adapter {
constexpr double pi = 3.14159265358979323846;
struct Velocity { double x{0}, y{0}, z{0}; };
struct Pose { double x{0}, y{0}, yaw{0}; };
inline bool finite(Velocity v) {
  return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}
// Legacy reviewed_yhs_ros2 profile compensates OLD decoder only; default uses corrected SI m/s.
class Units {
 public:
  explicit Units(const std::string &profile) {
    if (profile == "reviewed_yhs_ros2") { y_scale_ = 0.1; angular_scale_ = pi / 180.; }
    else if (profile == "mps_degps") { angular_scale_ = pi / 180.; }
    else if (profile == "mmps_degps") {
      linear_scale_ = 0.001; angular_scale_ = pi / 180.;
    } else if (profile != "si") throw std::invalid_argument("unknown feedback_profile");
  }
  Velocity decode(Velocity v) const {
    return {v.x * linear_scale_, v.y * linear_scale_ * y_scale_, v.z * angular_scale_};
  }
 private:
  double linear_scale_{1}, y_scale_{1}, angular_scale_{1};
};
class Integrator {
 public:
  explicit Integrator(double max_dt) : max_dt_(max_dt) {
    if (!std::isfinite(max_dt) || max_dt <= 0) throw std::invalid_argument("invalid max_dt");
  }
  // Sample-and-hold previous body velocity over [last_stamp, stamp].
  // A gap rebaselines without inventing motion through missing feedback.
  bool update(Velocity v, double stamp) {
    if (!finite(v) || !std::isfinite(stamp) || stamp < 0) return false;
    if (!initialized_) { baseline(v, stamp); return true; }
    const double dt = stamp - stamp_;
    if (dt <= 0) return false;
    if (dt <= max_dt_) {
      const double c = std::cos(pose_.yaw), s = std::sin(pose_.yaw);
      pose_.x += (previous_.x * c - previous_.y * s) * dt;
      pose_.y += (previous_.x * s + previous_.y * c) * dt;
      pose_.yaw = std::atan2(std::sin(pose_.yaw + previous_.z * dt),
                             std::cos(pose_.yaw + previous_.z * dt));
    }
    baseline(v, stamp);
    return true;
  }
  void break_continuity() { initialized_ = false; } // Keep pose; next sample is baseline.
  const Pose &pose() const { return pose_; }
 private:
  void baseline(Velocity v, double stamp) { previous_ = v; stamp_ = stamp; initialized_ = true; }
  Pose pose_;
  Velocity previous_;
  double stamp_{0}, max_dt_;
  bool initialized_{false};
};
}
