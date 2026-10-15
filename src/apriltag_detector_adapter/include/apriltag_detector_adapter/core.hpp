#pragma once
#include <cmath>
#include <stdexcept>
namespace apriltag_detector_adapter {
class QualityGate {
 public:
  QualityGate(int target, int hamming, double margin) :
      target_(target), hamming_(hamming), margin_(margin) {
    if (target < 0 || hamming < 0 || !std::isfinite(margin) || margin < 0)
      throw std::invalid_argument("invalid detector quality parameters");
  }
  bool accept(int id, int hamming, double margin) const {
    return id == target_ && hamming >= 0 && hamming <= hamming_ &&
           std::isfinite(margin) && margin >= margin_;
  }
 private:
  int target_, hamming_;
  double margin_;
};
inline bool valid_pose(double x, double y, double z,
                       double qx, double qy, double qz, double qw) {
  const double norm = qx*qx + qy*qy + qz*qz + qw*qw;
  return std::isfinite(x) && std::isfinite(y) && std::isfinite(z) && z > 0 &&
    std::isfinite(norm) && std::abs(norm - 1.) < 0.01;
}
}
