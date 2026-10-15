#include <iostream>
#include <limits>
#include "fwmini_chassis_adapter/core.hpp"
using namespace fwmini_chassis_adapter;
void check(bool ok, const char *reason) {
  if (!ok) throw std::runtime_error(reason);
}
void near(double a, double b) { check(std::abs(a-b) < 1e-9, "numeric mismatch"); }
int main() {
  CommandGate gate({0.3, 0.2, 0.5, 0.2});
  gate.receive({1, -1, 2}, 1);
  near(gate.output(1).x, 0);  // Disabled by default.
  gate.enable(true);
  near(gate.output(1).x, 0);  // Old command never resurrects.
  check(gate.receive({1, -1, 2}, 1), "receive failed");
  const auto v = gate.output(1.1);
  near(v.x, 0.3); near(v.y, -0.2); near(v.z, 0.5);
  near(gate.output(1.21).z, 0);  // timeout
  near(gate.output(0.9).x, 0);   // clock regression
  gate.receive({0.1, 0.1, 0.1}, 2);
  gate.enable(false);
  near(gate.output(2).x, 0);
  gate.enable(true);
  near(gate.output(2).x, 0);
  gate.receive({0.1, 0.1, 0.1}, 2);
  check(!gate.receive({std::numeric_limits<double>::quiet_NaN(), 0, 0}, 2.01),
        "NaN accepted");
  near(gate.output(2.02).x, 0);
  Units yhs("mps", "degps");
  auto encoded = yhs.encode({0.3, -0.2, pi/2});
  near(encoded.x, 0.3); near(encoded.y, -0.2); near(encoded.z, 90);
  Units raw("mmps", "radps");
  encoded = raw.encode({0.3, -0.2, 0.5});
  near(encoded.x, 300); near(encoded.y, -200); near(encoded.z, 0.5);
  bool threw = false;
  try { CommandGate bad({-1, 1, 1, 1}); } catch (const std::invalid_argument &) { threw = true; }
  check(threw, "invalid limits accepted");
  threw = false;
  try { Units bad("unknown", "degps"); } catch (const std::invalid_argument &) { threw = true; }
  check(threw, "unknown units accepted");
  std::cout << "PASS command: enable, fresh command, limits, watchdog, invalid input, unit conversion\n";
}
