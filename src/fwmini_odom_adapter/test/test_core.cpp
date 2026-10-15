#include <iostream>
#include <limits>
#include "fwmini_odom_adapter/core.hpp"
using namespace fwmini_odom_adapter;
void check(bool ok, const char *reason) {
  if (!ok) throw std::runtime_error(reason);
}
void near(double a, double b) { check(std::abs(a-b) < 1e-9, "numeric mismatch"); }
int main() {
  Units reviewed("reviewed_yhs_ros2");
  auto v = reviewed.decode({1, 10, 90});
  near(v.x, 1); near(v.y, 1); near(v.z, pi/2);
  v = Units("mmps_degps").decode({1000, -500, 180});
  near(v.x, 1); near(v.y, -0.5); near(v.z, pi);
  v = Units("mps_degps").decode({1, 1, 90});
  near(v.y, 1);
  v = Units("si").decode({1, 1, 0.5});
  near(v.z, 0.5);
  Integrator odom(0.2);
  check(odom.update({1, 2, 0.5}, 1), "baseline failed");
  check(odom.update({1, 2, 0.5}, 1.1), "integration failed");
  near(odom.pose().x, 0.1); near(odom.pose().y, 0.2); near(odom.pose().yaw, 0.05);
  check(!odom.update({1, 2, 0.5}, 1.1), "duplicate stamp accepted");
  check(!odom.update({1, 2, 0.5}, 0.9), "reversed stamp accepted");
  check(!odom.update({std::numeric_limits<double>::infinity(), 0, 0}, 1.2), "Inf accepted");
  check(odom.update({1, 2, 0.5}, 2), "gap baseline failed");
  near(odom.pose().x, 0.1); near(odom.pose().y, 0.2);
  odom.break_continuity();
  check(odom.update({1, 0, 0}, 0.1), "new clock epoch failed");
  near(odom.pose().x, 0.1);
  Integrator rotated(2);
  rotated.update({0, 0, pi/2}, 1);
  rotated.update({0, 1, 0}, 2);
  rotated.update({0, 1, 0}, 3);
  near(rotated.pose().x, -1); near(rotated.pose().y, 0);
  Integrator lateral(0.2);
  lateral.update({0, -1, 0}, 1); lateral.update({0, -1, 0}, 1.1);
  near(lateral.pose().x, 0); near(lateral.pose().y, -0.1);
  bool threw = false;
  try { Units bad("unknown"); } catch (const std::invalid_argument &) { threw = true; }
  check(threw, "unknown profile accepted");
  std::cout << "PASS odom: lateral/rotated SE2, feedback units/correction, gaps, stamps, invalid input\n";
}
