// ir_test -- LINKS 701 bench test for the two IR wheel encoders (BEST IR Sensor Kit + wheel decals).
// Build: `pros c new ir_test` (PROS 4), replace src/main.cpp with this file, `pros mu`.
// Readout on the V5 Brain screen AND the controller; CSV lines in the PROS terminal.
//
// PORTS (same as the main robot code)
//   G = LEFT receiver    H = LEFT emitter (driven HIGH)
//   E = RIGHT receiver   F = RIGHT emitter (driven HIGH)
//   Smart 2 = left drive (reversed), Smart 1 = right drive
//
// CONTROLS
//   A      : switch receiver mode DIGITAL <-> ANALOG (we don't know which the kit sensor outputs)
//   X      : zero both counts (then turn a wheel exactly one revolution -> expect 2 x STRIPES = 48)
//   B      : reset the analog min/max and threshold
//   Y (hold) : drive both wheels forward at TEST_MV  (robot ON BLOCKS!) -- checks counting at speed
//   Left stick : drive slowly (40 %) for a floor test
//
// WHAT GOOD LOOKS LIKE
//   * Turning one wheel by hand one full revolution adds 48 counts on that side and 0 on the other.
//   * In DIGITAL mode the raw value flips 0/1 cleanly as stripes pass.
//   * In ANALOG mode, max - min ("span") is several hundred counts or more; edges are counted
//     with hysteresis around the middle of the span.
//   * With Y held (on blocks), left and right counts per second agree within a few percent.
#include "main.h"
#include <cmath>
#include <cstdio>

// ---------------- settings ----------------
constexpr char RX_L = 'G', TX_L = 'H', RX_R = 'E', TX_R = 'F';
constexpr int PORT_LEFT_DRIVE = -2, PORT_RIGHT_DRIVE = 1;
constexpr int STRIPES = 24;                 // black stripes on the decal (make_wheel_decal.py)
constexpr double WHEEL_DIAM_IN = 10.0;
constexpr int TEST_MV = 4000;
constexpr double IN_PER_EDGE = 3.14159265358979 * WHEEL_DIAM_IN / (2.0 * STRIPES);

static pros::Controller master(pros::E_CONTROLLER_MASTER);
static pros::Motor leftDrive(PORT_LEFT_DRIVE), rightDrive(PORT_RIGHT_DRIVE);
static pros::adi::Port rxL(RX_L, pros::E_ADI_DIGITAL_IN), rxR(RX_R, pros::E_ADI_DIGITAL_IN);
static pros::adi::DigitalOut txL(TX_L, true), txR(TX_R, true);

struct Channel {
  pros::adi::Port* port;
  volatile long count = 0;
  volatile int raw = 0;
  int mn = 4095, mx = 0;
  bool state = false, haveState = false;
  volatile int dirSign = 1;
};
static Channel L{&rxL}, R{&rxR};
static volatile bool analogMode = false;

static void sample(Channel& c) {
  const int v = c.port->get_value();
  c.raw = v;
  bool s;
  if (!analogMode) {
    s = v != 0;
  } else {
    if (v < c.mn) c.mn = v;
    if (v > c.mx) c.mx = v;
    const int span = c.mx - c.mn;
    if (span < 150) return;                          // not enough contrast yet: don't count
    const int mid = (c.mx + c.mn) / 2, hyst = span / 8;
    if (!c.haveState) s = v > mid;
    else s = c.state ? (v > mid - hyst) : (v > mid + hyst);
  }
  if (c.haveState && s != c.state) c.count += c.dirSign;
  c.state = s; c.haveState = true;
}

static void pollTask() {                             // 1 ms: no edge missed at full speed
  while (true) { sample(L); sample(R); pros::delay(1); }
}

static void setMode(bool analog) {
  analogMode = analog;
  const auto cfg = analog ? pros::E_ADI_ANALOG_IN : pros::E_ADI_DIGITAL_IN;
  rxL.set_config(cfg); rxR.set_config(cfg);
  for (Channel* c : {&L, &R}) { c->mn = 4095; c->mx = 0; c->haveState = false; }
}

void initialize() {
  pros::screen::print(pros::E_TEXT_MEDIUM, 0, "LINKS 701 IR encoder test");
  static pros::Task t(pollTask, "ir-poll");
}
void disabled() {}
void competition_initialize() {}
void autonomous() {}

void opcontrol() {
  int loop = 0;
  long lastL = 0, lastR = 0;
  std::uint32_t lastRate = pros::millis();
  double rateL = 0, rateR = 0;
  while (true) {
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) setMode(!analogMode);
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) { L.count = 0; R.count = 0; }
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) setMode(analogMode);

    int lmv = 0, rmv = 0;
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) { lmv = rmv = TEST_MV; }
    else {
      const double f = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) / 127.0 * 0.4;
      const double t = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X) / 127.0 * 0.4;
      lmv = static_cast<int>((f + t) * 8000); rmv = static_cast<int>((f - t) * 8000);
    }
    leftDrive.move_voltage(lmv); rightDrive.move_voltage(rmv);
    // single-channel sensor: direction comes from the command (by hand, counts go UP)
    L.dirSign = lmv < 0 ? -1 : 1; R.dirSign = rmv < 0 ? -1 : 1;

    if (pros::millis() - lastRate >= 500) {          // counts per second
      const double s = (pros::millis() - lastRate) / 1000.0;
      rateL = (L.count - lastL) / s; rateR = (R.count - lastR) / s;
      lastL = L.count; lastR = R.count; lastRate = pros::millis();
    }

    if (loop % 10 == 0) {                            // Brain screen, 10 Hz
      pros::screen::print(pros::E_TEXT_MEDIUM, 1, "mode: %s   (A switch, X zero, B reset)   ", analogMode ? "ANALOG " : "DIGITAL");
      pros::screen::print(pros::E_TEXT_MEDIUM, 2, "LEFT  raw %4d  count %6ld  rev %6.2f  %6.1f in   ", (int)L.raw, (long)L.count,
                          L.count / (2.0 * STRIPES), L.count * IN_PER_EDGE);
      pros::screen::print(pros::E_TEXT_MEDIUM, 3, "RIGHT raw %4d  count %6ld  rev %6.2f  %6.1f in   ", (int)R.raw, (long)R.count,
                          R.count / (2.0 * STRIPES), R.count * IN_PER_EDGE);
      pros::screen::print(pros::E_TEXT_MEDIUM, 4, "rate L %6.1f  R %6.1f edges/s   diff %5.1f %%   ", rateL, rateR,
                          (rateL + rateR) != 0 ? 200.0 * (rateL - rateR) / (std::fabs(rateL) + std::fabs(rateR) + 1e-9) : 0.0);
      if (analogMode)
        pros::screen::print(pros::E_TEXT_MEDIUM, 5, "analog span  L %4d..%4d   R %4d..%4d   ", L.mn, L.mx, R.mn, R.mx);
      else
        pros::screen::print(pros::E_TEXT_MEDIUM, 5, "expect %d counts per wheel revolution            ", 2 * STRIPES);
      std::printf("IR,%lu,%s,%d,%ld,%d,%ld,%.1f,%.1f\n", (unsigned long)pros::millis(), analogMode ? "A" : "D",
                  (int)L.raw, (long)L.count, (int)R.raw, (long)R.count, rateL, rateR);
    }
    switch (loop % 15) {                             // controller: one line per 50 ms
      case 0:  master.print(0, 0, "%s L%5ld R%5ld  ", analogMode ? "AN" : "DG", (long)L.count, (long)R.count); break;
      case 5:  master.print(1, 0, "raw L%4d R%4d   ", (int)L.raw, (int)R.raw); break;
      case 10: master.print(2, 0, "rev L%5.2f R%5.2f ", L.count / (2.0 * STRIPES), R.count / (2.0 * STRIPES)); break;
    }
    ++loop;
    pros::delay(10);
  }
}
