#include <iostream>
#include <limits>
#include "tag_follow_controller/core.hpp"
using namespace tag_follow_controller;
void check(bool condition) { if (!condition) throw std::runtime_error("follow test failed"); }
int main() {
  Controller follow(Config{});
  follow.observe(2, 0, 1); check(follow.output(1).x == 0);
  follow.enable(true); check(follow.output(1).x == 0);
  follow.observe(2, 0, 1); check(follow.output(1.1).x == 0.2);
  check(follow.output(1.21).x == 0);
  follow.observe(2, 0.2, 2); check(follow.output(2).z > 0);
  follow.observe(2, -0.2, 2); check(follow.output(2).z < 0);
  follow.observe(1, 1, 2); check(follow.output(2).x == 0); check(follow.output(2).z == 0.4);
  follow.observe(0.8, 0, 2); check(follow.output(2).x == 0);
  follow.observe(0.4, 0, 2); check(follow.output(2).x == 0); // no reverse
  check(!follow.observe(0.1, 0, 2)); check(follow.output(2).z == 0);
  check(!follow.observe(-1, 0, 2)); check(!follow.observe(6, 0, 2));
  check(!follow.observe(std::numeric_limits<double>::quiet_NaN(), 0, 2));
  follow.observe(2, 0, 2); follow.invalidate(); check(follow.output(2).x == 0);
  follow.observe(2, 0, 2); follow.enable(false); check(follow.output(2).x == 0);
  std::cout << "PASS follow: enable, distance, bearing signs, turn-first, loss, timeout and invalid data\n";
}
