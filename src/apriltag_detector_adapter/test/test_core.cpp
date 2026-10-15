#include <iostream>
#include <limits>
#include "apriltag_detector_adapter/core.hpp"
using namespace apriltag_detector_adapter;
void check(bool condition) { if (!condition) throw std::runtime_error("quality gate failed"); }
int main() {
  QualityGate gate(0, 0, 30);
  check(gate.accept(0, 0, 50));
  check(!gate.accept(1, 0, 50)); check(!gate.accept(0, 1, 50));
  check(!gate.accept(0, -1, 50)); check(!gate.accept(0, 0, 29));
  check(!gate.accept(0, 0, std::numeric_limits<double>::quiet_NaN()));
  check(valid_pose(0, 0, 1, 0, 0, 0, 1));
  check(!valid_pose(0, 0, -1, 0, 0, 0, 1));
  check(!valid_pose(0, 0, 1, 0, 0, 0, 0));
  check(!valid_pose(std::numeric_limits<double>::infinity(), 0, 1, 0, 0, 0, 1));
  std::cout << "PASS detector: target, hamming, margin, finite forward pose and quaternion\n";
}
