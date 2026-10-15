#include <iostream>
#include <stdexcept>
#include "apriltag_detector_adapter/core.hpp"
#include "tag_follow_controller/core.hpp"
#include "fwmini_chassis_adapter/core.hpp"
void check(bool value) { if (!value) throw std::runtime_error("chain integration failed"); }
int main() {
  apriltag_detector_adapter::QualityGate detector(0, 0, 30);
  tag_follow_controller::Controller follower(tag_follow_controller::Config{});
  fwmini_chassis_adapter::CommandGate chassis({0.3, 0.3, 0.5, 0.2});
  fwmini_chassis_adapter::Units units("mps", "degps");
  check(detector.accept(0, 0, 100));
  follower.enable(true); chassis.enable(true);
  // Synthetic optical position (right 0.12 m, forward 1.2 m).
  // Test mount maps optical z -> base x and optical x -> base -y.
  follower.observe(1.2, -0.12, 1);
  const auto follow = follower.output(1.05);
  chassis.receive({follow.x, follow.y, follow.z}, 1.05);
  const auto vendor = units.encode(chassis.output(1.06));
  check(vendor.x > 0 && vendor.x <= 0.2 && vendor.y == 0 && vendor.z < 0);
  follower.invalidate();
  const auto lost = follower.output(1.07);
  chassis.receive({lost.x, lost.y, lost.z}, 1.07);
  check(units.encode(chassis.output(1.08)).x == 0);
  follower.observe(1.2, 0, 2);
  check(follower.output(2.21).x == 0); // camera/tag timeout
  chassis.receive({0.2, 0, 0}, 3);
  check(chassis.output(3.21).x == 0); // follower process/output timeout
  std::cout << "PASS chain: tag quality -> body position -> follow -> gate -> vendor units, loss/watchdogs\n";
}
