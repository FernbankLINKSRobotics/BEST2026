// arm_control.hpp -- potentiometer angles + position control for the arm (axle 2) and the
// FlapJack (axle 3). Pure C++ (no PROS) so it can be tested on a laptop.
//
// KIT POTENTIOMETER (Consumable Kit): rotary, LINEAR, 16 kOhm, 300 deg, panel mount, ~6 mm shaft.
//   Wired to a 3-wire port (ends to +5 V / ground, wiper to signal) the V5 reads 0..4095,
//   so 300 deg / 4095 = 0.0733 deg per count. Geared 1:1 (16T-16T) it reads the axle angle.
//   Keep every joint's full travel INSIDE the pot's 300 deg and away from its ends:
//   install the gear so the pot reads about 2048 in the MIDDLE of the joint's travel.
//
// Angles (degrees):
//   ARM      0 = arm pointing straight FORWARD and level, +90 = straight up,
//            more than 90 = folded back over the robot (the 24 in start cube), negative = down.
//   FLAPJACK "ground angle": 0 = fork LEVEL with the floor, + = tip raised.
//            If the axle-3 pot's body rides on the arm, it measures the FlapJack RELATIVE to the
//            arm, and ground = arm + relative. (Set FJ_POT_RELATIVE_TO_ARM in the config.)
#pragma once
#include <cmath>

namespace arm {

constexpr double POT_DEG_PER_COUNT = 300.0 / 4095.0;

// A raw value pinned at either end almost always means an unplugged or broken pot.
inline bool potOk(int raw, int lo = 30, int hi = 4065) { return raw >= lo && raw <= hi; }

inline double potDeg(int raw, int rawAtZero, int sign) {
  return sign * (raw - rawAtZero) * POT_DEG_PER_COUNT;
}

// Position controller for one joint driven by an MC55: P + D + gravity feed-forward.
struct Joint {
  double kp;        // mV per degree of error
  double kd;        // mV per (degree / second)
  double kg;        // mV to hold the joint against gravity when level (scaled by cos(angle))
  int    maxMv;     // output limit
  double minDeg, maxDeg;   // soft limits for the TARGET
  double tolDeg;    // "arrived" tolerance

  double target = 0;
  double lastErr = 0;
  bool   haveLast = false;
  bool   faulted = false;    // runaway detected: output stays 0 until reset()
  double growTime = 0;

  void setTarget(double t) { target = std::fmin(maxDeg, std::fmax(minDeg, t)); }
  bool atTarget(double angle) const { return std::fabs(target - angle) <= tolDeg; }

  // angle = current joint angle, gravAngleDeg = angle that sets the gravity load (0 = level)
  int output(double angle, double dt, double gravAngleDeg) {
    const double err = target - angle;
    const double dErr = haveLast && dt > 0 ? (err - lastErr) / dt : 0.0;
    // RUNAWAY CHECK: pushing hard but the error keeps GROWING for 0.4 s means the motor and the
    // pot disagree about direction (wrong SIGN or motor polarity) -- stop instead of crashing.
    if (haveLast && std::fabs(err) > 15.0 && std::fabs(err) > std::fabs(lastErr)) growTime += dt; else growTime = 0;
    if (growTime > 0.4) faulted = true;
    lastErr = err; haveLast = true;
    if (faulted) return 0;
    double mv = kp * err + kd * dErr + kg * std::cos(gravAngleDeg * 3.14159265358979 / 180.0);
    if (mv > maxMv) mv = maxMv;
    if (mv < -maxMv) mv = -maxMv;
    return static_cast<int>(mv);
  }
  void reset() { haveLast = false; faulted = false; growTime = 0; }
};

} // namespace arm
