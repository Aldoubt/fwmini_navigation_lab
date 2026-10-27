#include <iostream>
#include <limits>
#include "fwmini_chassis_adapter/core.hpp"
using namespace fwmini_chassis_adapter;
void check(bool ok,const char *why) { if (!ok) throw std::runtime_error(why); }
void near(double a,double b) { check(std::abs(a-b)<1e-9,"numeric mismatch"); }
void zero(const Controller &c) { const auto v=c.output(); near(v.x,0); near(v.y,0); near(v.z,0); }
void sample(Controller &c,int gear,double t,Velocity v={}) { c.io(false,false,t); c.feedback(gear,v,t); c.tick(t); }
void ready(Controller &c,const std::string &mode,double t) {
  check(c.request(mode,t),"request"); zero(c); check(c.output_gear()==0,"stop before shift");
  for (int i=1;i<=5;++i) sample(c,0,t+i*.02);
  check(c.state==State::SWITCHING,"stopped streak");
  check(c.output_gear()==(mode=="lateral"?8:6),"CtrlCmd gear"); zero(c);
  for (int i=6;i<=10;++i) sample(c,c.target,t+i*.02);
  check(c.state==State::READY,"matching streak");
}
int main() {
  Controller c;
  check(c.state==State::DISABLED && c.output_gear()==0,"startup"); zero(c); check(!c.enable(1),"READY required");
  ready(c,"four_wheel_steer",1); check(c.enable(1.21),"enable"); zero(c); const auto old=c.epoch;
  check(c.receive({.05,0,.1},c.epoch,1.22),"4T4D velocity"); near(c.output().z,.1);
  check(!c.receive({.01,.01,0},c.epoch,1.23),"reject vy"); check(c.state==State::FAULT,"latch fault"); zero(c);
  ready(c,"lateral",2); check(c.enable(2.21),"enable lateral");
  check(!c.receive({.05,0,.1},old,2.22),"old epoch"); zero(c);
  check(c.receive({.02,-.03,0},c.epoch,2.23),"lateral"); near(c.output().y,-.03);
  check(!c.receive({0,0,.01},c.epoch,2.24),"reject wz"); zero(c);
  ready(c,"lateral",3); c.enable(3.21); c.receive({.03,.03,0},c.epoch,3.22); c.disable(); zero(c);
  check(!c.enable(3.23),"disable invalidates readiness");
  ready(c,"four_wheel_steer",4); c.enable(4.21); c.receive({.02,0,0},c.epoch,4.22);
  c.io(true,false,4.23); zero(c); check(c.reason=="emergency_stop","estop");
  ready(c,"four_wheel_steer",5); c.enable(5.21); c.receive({.02,0,0},c.epoch,5.22);
  c.io(false,true,5.23); zero(c); check(c.reason=="control_authority_lost","authority");
  ready(c,"four_wheel_steer",6); c.enable(6.21); c.receive({.02,0,0},c.epoch,6.22);
  sample(c,6,6.38); sample(c,6,6.43); zero(c); check(c.reason=="command_timeout","command watchdog");
  ready(c,"lateral",7); c.enable(7.21); c.receive({0,.02,0},c.epoch,7.22); c.tick(7.5);
  zero(c); check(c.reason=="io_feedback_stale","feedback watchdog");
  ready(c,"four_wheel_steer",8); c.enable(8.21); sample(c,8,8.22); zero(c); check(c.reason=="gear_mismatch","gear");
  ready(c,"four_wheel_steer",9); c.enable(9.21);
  c.receive({std::numeric_limits<double>::quiet_NaN(),0,0},c.epoch,9.22); zero(c); check(c.state==State::FAULT,"NaN");
  Controller timeout; timeout.request("lateral",10); timeout.tick(13.01);
  check(timeout.state==State::FAULT && timeout.switch_result=="FAILED:mode_switch_timeout","switch timeout"); zero(timeout);
  Controller moving; moving.request("lateral",20);
  for (int i=1;i<=30;++i) sample(moving,0,20+i*.02,{.1,0,0});
  check(moving.state==State::STOPPING && moving.output_gear()==0,"moving cannot shift"); zero(moving);
  Controller mismatch; mismatch.request("lateral",30);
  for (int i=1;i<=5;++i) sample(mismatch,0,30+i*.02);
  for (int i=6;i<=150;++i) { sample(mismatch,(i%2?8:6),30+i*.02); zero(mismatch); }
  check(mismatch.state==State::FAULT,"intermittent gear cannot confirm");
  ready(c,"lateral",40); c.enable(40.21); c.external_stop(true); zero(c); check(c.reason=="external_stop","external stop");
  Units u("mps","degps"); auto v=u.encode({.05,.03,pi/2}); near(v.x,.05); near(v.y,.03); near(v.z,90);
  bool threw=false; try { Controller bad({-1,1,1,1}); } catch (const std::invalid_argument &) { threw=true; }
  check(threw,"config");
  std::cout<<"PASS: startup, modes 6/8, stop-before-shift, feedback streak, timeouts, axes, epochs, disable, estop, authority, units\n";
}
