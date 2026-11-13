// motion_model.hpp -- our best guess of how fast the robot is ACTUALLY moving,
// given only the voltages we commanded. No encoders, so this is all we have
// between camera frames.  Model per wheel:
//     target speed = KV * (|cmd| - deadband) * sign(cmd)     (0 inside deadband)
//     speed       += (target - speed) * dt / (tau + dt)      (first-order lag)
#pragma once
#include <cmath>

namespace trk {

class DriveModel {
 public:
  DriveModel(double kv, double tau, double deadbandMv, double trackIn)
      : kv_(kv), tau_(tau), db_(deadbandMv), track_(trackIn) {}

  void step(double cmdLeftMv, double cmdRightMv, double dt) {
    const double a = dt / (tau_ + dt);
    vl_ += a * (target(cmdLeftMv) - vl_);
    vr_ += a * (target(cmdRightMv) - vr_);
  }
  void reset() { vl_ = vr_ = 0; }

  double v() const { return 0.5 * (vl_ + vr_); }          // forward [in/s]
  double vl() const { return vl_; }                        // left wheel [in/s]
  double vr() const { return vr_; }                        // right wheel [in/s]
  double w() const { return (vr_ - vl_) / track_; }        // turn rate [rad/s], + = left

  // Inverse: what wheel voltages give (v, w)?  Adds the deadband back in.
  void commandFor(double v, double w, double& leftMv, double& rightMv) const {
    const double vl = v - 0.5 * w * track_, vr = v + 0.5 * w * track_;
    leftMv = inverse(vl);
    rightMv = inverse(vr);
  }

 private:
  double target(double cmd) const {
    const double mag = std::fabs(cmd) - db_;
    if (mag <= 0) return 0;
    return std::copysign(kv_ * mag, cmd);
  }
  double inverse(double wheelV) const {
    if (std::fabs(wheelV) < 1e-6) return 0;
    return std::copysign(std::fabs(wheelV) / kv_ + db_, wheelV);
  }
  double kv_, tau_, db_, track_;
  double vl_ = 0, vr_ = 0;
};

} // namespace trk
