// main.cpp -- PROS glue for the LINKS Dining Room tracker.
// Everything robot-specific (ports, motors, buttons, the arm hook) is here.
// All the math lives in include/tracker/*.hpp and is tested on a laptop first.
//
// CONTROLLER MAP (driver-control period; BEST has no field-controlled auton):
//   Left stick Y  / Right stick X : arcade drive
//   A        : start Dining Room autonomous, pivot RIGHT to search
//   Y        : start Dining Room autonomous, pivot LEFT to search
//   B        : ABANDON autonomous (Rule 3.3.2.2.E) -> driver has the robot
//   X        : toggle calibration dump (prints tag corners to the terminal)
//   UP       : toggle SD-card logging
//   (optional switches, see config.hpp: plate switch on the FlapJack, table-edge
//    whisker on the arm, front bumper on the base)
//   DOWN     : motion-model test step (Exercise E5) -- robot drives STRAIGHT
//              at the next test voltage for 1.2 s, then coasts. Put a table
//              tag on the floor/platform 40 in ahead first. Needs an SD card.
// The driver must be HANDS-FREE while the robot is in the Dining Room under
// autonomous control (Rule 3.3.2.2.A). Pressing B ends that.
#include "main.h"
#include <cstdio>
#include <cstring>
#include <vector>
#include "tracker/config.hpp"
#include "tracker/dining_auto.hpp"

using namespace trk;

// ------------------------------ devices ------------------------------
static pros::Controller master(pros::E_CONTROLLER_MASTER);
static pros::Motor leftDrive(cfg::PORT_LEFT_DRIVE);     // MC55 on a Smart Port
static pros::Motor rightDrive(cfg::PORT_RIGHT_DRIVE);   // negative port = reversed
static pros::Motor arm(cfg::PORT_ARM_MOTOR);
static pros::Motor flapJack(cfg::PORT_FLAPJACK_MOTOR);
static pros::ADIMotor finger(cfg::THREE_WIRE_CLAW);
static pros::AIVision vision(cfg::PORT_AIVISION);

static const CameraModel CAM{cfg::FX, cfg::FY, cfg::CX, cfg::CY, cfg::CAM_HEIGHT_IN,
                             deg2rad(cfg::CAM_PITCH_DEG), cfg::CAM_X_IN, cfg::CAM_Y_IN};
static DiningAuto diningAuto(CAM);

// ------------------------------ helpers ------------------------------
static int clampMv(double mv) {
  if (mv > cfg::MC55_MAX_MV) mv = cfg::MC55_MAX_MV;
  if (mv < -cfg::MC55_MAX_MV) mv = -cfg::MC55_MAX_MV;
  return static_cast<int>(mv);
}

static volatile int lastLeftMv = 0, lastRightMv = 0;   // encoder task uses the sign for direction

static void setDrive(double leftMv, double rightMv) {
  lastLeftMv = clampMv(leftMv);
  lastRightMv = clampMv(rightMv);
  leftDrive.move_voltage(lastLeftMv);
  rightDrive.move_voltage(lastRightMv);
}

// ------------------------------ OPTIONAL: IR wheel encoders (Plan B) ------------------------------
// One BEST IR sensor per drive wheel, looking at a striped disc. Single channel,
// so direction comes from the sign of the last command. ASSUMES a digital
// output that flips as each stripe edge passes -- bench-test your IR kit first;
// if it is analog, threshold get_value() with hysteresis here instead.
class IrWheelCounter {
 public:
  explicit IrWheelCounter(char port) : in_(port) {}
  void poll(int dirSign) {
    const bool s = in_.get_value() != 0;
    if (s != last_) { count_ += (dirSign >= 0) ? 1 : -1; last_ = s; }
  }
  long count() const { return count_; }
 private:
  pros::adi::DigitalIn in_;
  bool last_ = false;
  volatile long count_ = 0;
};
static IrWheelCounter* encLeft = nullptr;
static IrWheelCounter* encRight = nullptr;

static void encoderTask() {          // 1 ms polling so no edge is missed at full speed
  while (true) {
    encLeft->poll(lastLeftMv);
    encRight->poll(lastRightMv);
    pros::delay(1);
  }
}

// Read every AprilTag the sensor currently sees. Returns false if the frame
// is IDENTICAL to the previous one (sensor hasn't produced a new image yet),
// so we never feed the same measurement to the filter twice.
static bool readTags(std::vector<RawTag>& out) {
  static std::vector<RawTag> prev;
  out.clear();
  const std::uint32_t now = pros::millis();
  std::vector<pros::AIVision::Object> objs = vision.get_all_objects();
  for (const auto& o : objs) {
    if (!pros::AIVision::is_type(o, pros::AivisionDetectType::tag)) continue;
    RawTag t;
    t.id = o.id;
    t.u[0] = o.object.tag.x0; t.v[0] = o.object.tag.y0;
    t.u[1] = o.object.tag.x1; t.v[1] = o.object.tag.y1;
    t.u[2] = o.object.tag.x2; t.v[2] = o.object.tag.y2;
    t.u[3] = o.object.tag.x3; t.v[3] = o.object.tag.y3;
    t.t_ms = now;
    out.push_back(t);
  }
  bool same = (out.size() == prev.size());
  for (size_t i = 0; same && i < out.size(); ++i) {
    same = out[i].id == prev[i].id &&
           std::memcmp(out[i].u, prev[i].u, sizeof(out[i].u)) == 0 &&
           std::memcmp(out[i].v, prev[i].v, sizeof(out[i].v)) == 0;
  }
  prev = out;
  return !same || out.empty();   // an empty frame is still information
}

// ------------------------------ OPTIONAL: contact switches ------------------------------
// Omron SS-01GL13PT microswitches (Consumable Kit). Debounced: a switch must read
// the same for 3 loops (30 ms) before we believe it changed.
class Switch {
 public:
  explicit Switch(char port) : in_(port) {}
  bool update() {
    const bool raw = in_.get_value() == cfg::SWITCH_PRESSED_LEVEL;
    if (raw == stable_) count_ = 0;
    else if (++count_ >= 3) { stable_ = raw; count_ = 0; }
    return stable_;
  }
  bool pressed() const { return stable_; }
 private:
  pros::adi::DigitalIn in_;
  bool stable_ = false;
  int count_ = 0;
};
static Switch* plateSw = nullptr;
static Switch* whiskerSw = nullptr;
static Switch* bumperSw = nullptr;

// ------------------------------ SD logging ------------------------------
static FILE* logFile = nullptr;

static void startLog() {
  if (!pros::usd::is_installed()) { master.print(2, 0, "NO SD CARD     "); return; }
  char name[40];
  std::snprintf(name, sizeof name, "/usd/trk_%lu.csv", (unsigned long)pros::millis());
  logFile = std::fopen(name, "w");
  if (logFile)
    std::fprintf(logFile, "t_ms,state,cmdL,cmdR,v,w,haveMeas,accepted,measX,measY,"
                          "sigAlong,sigCross,estX,estY,sigma,ex,ey\n");
}
static void stopLog() { if (logFile) { std::fclose(logFile); logFile = nullptr; } }

static void logLine(const AutoOutput& out) {
  if (!logFile) return;
  const auto& t = diningAuto.telemetry();
  const auto& e = diningAuto.ekf();
  std::fprintf(logFile, "%lu,%s,%.0f,%.0f,%.2f,%.3f,%d,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
               (unsigned long)pros::millis(), stateName(t.state), out.leftMv, out.rightMv, t.v, t.w,
               (int)t.haveMeas, (int)t.accepted, t.meas.x, t.meas.y, t.measAlong, t.measCross,
               e.x, e.y, t.sigma, t.ex, t.ey);
}

// Calibration dump: one line per tag, copy from the PROS terminal into a CSV.
static void calDump(const std::vector<RawTag>& tags) {
  for (const auto& t : tags)
    std::printf("CAL,%lu,%d,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f\n", (unsigned long)t.t_ms, t.id,
                t.u[0], t.v[0], t.u[1], t.v[1], t.u[2], t.v[2], t.u[3], t.v[3]);
}

// ------------------------------ motion-model test (Exercise E5) ------------------------------
static const int SYSID_MV[] = {2000, 3000, 4000, 5000, 6000, 8000};
static int sysidIndex = 0;
static bool sysidActive = false;
static std::uint32_t sysidT0 = 0;
static int sysidMv = 0;
static FILE* sysidFile = nullptr;

static void sysidStart() {
  if (!pros::usd::is_installed()) { master.print(2, 0, "NO SD CARD     "); return; }
  sysidMv = SYSID_MV[sysidIndex];
  sysidIndex = (sysidIndex + 1) % (int)(sizeof(SYSID_MV) / sizeof(SYSID_MV[0]));
  char name[48];
  std::snprintf(name, sizeof name, "/usd/sysid_%d_%lu.csv", sysidMv, (unsigned long)pros::millis());
  sysidFile = std::fopen(name, "w");
  if (!sysidFile) return;
  std::fprintf(sysidFile, "t_ms,cmd_mV,id,measX,measY\n");
  sysidT0 = pros::millis();
  sysidActive = true;
}

// Returns the drive command for this loop and logs any table-tag sightings.
static int sysidStep(bool newFrame, const std::vector<RawTag>& tags) {
  const std::uint32_t t = pros::millis() - sysidT0;
  const int cmd = (t < 1200) ? sysidMv : 0;
  if (newFrame) {
    for (const auto& tag : tags) {
      TableTagMeas m = measureTableTag(CAM, tag, cfg::TABLE_TAG_Z_IN, cfg::TAG_QUAD_IN, cfg::SIDE_TOL_FRAC);
      if (m.ok) std::fprintf(sysidFile, "%lu,%d,%d,%.2f,%.2f\n", (unsigned long)t, cmd, m.id, m.center.x, m.center.y);
    }
  }
  if (t > 2400) { std::fclose(sysidFile); sysidFile = nullptr; sysidActive = false; }
  return cmd;
}

// ------------------------------ arm hook ------------------------------
// Robot-specific: lower the fork / release the plate. Return true when DONE.
// Called every loop while the state machine is in PLACE. Non-blocking!
// TODO(team): replace with your arm (small motor 1) + fork wrist (small motor 2).
static bool placeHook(bool firstCall) {
  static std::uint32_t t0 = 0;
  if (firstCall) t0 = pros::millis();
  const std::uint32_t dt = pros::millis() - t0;
  // Example timing only -- e.g. drive the wrist down for 400 ms, then stop.
  // With a plate switch, "done" = the clamp has let go of the plate (+150 ms).
  if (cfg::USE_PLATE_SWITCH) {
    static std::uint32_t releasedAt = 0;
    if (firstCall) releasedAt = 0;
    if (!plateSw->pressed() && releasedAt == 0) releasedAt = pros::millis();
    if (releasedAt != 0 && pros::millis() - releasedAt > 150) return true;
    return dt > 2000;   // safety: never wait forever
  }
  return dt > 400;
}

// ------------------------------ PROS entry points ------------------------------
void initialize() {
  vision.reset();
  vision.enable_detection_types(pros::AivisionModeType::tags);
  vision.set_tag_family(pros::AivisionTagFamily::tag_21H7);   // Circle21h7 (Rule 3.3.2.2.A.B)
  leftDrive.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);         // MC55: coast or brake only
  rightDrive.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  if (cfg::USE_PLATE_SWITCH) plateSw = new Switch(cfg::ADI_PLATE_SWITCH);
  if (cfg::USE_EDGE_WHISKER) whiskerSw = new Switch(cfg::ADI_EDGE_WHISKER);
  if (cfg::USE_FRONT_BUMPER) bumperSw = new Switch(cfg::ADI_FRONT_BUMPER);
  if (cfg::USE_IR_ENCODER) {
    encLeft = new IrWheelCounter(cfg::ADI_ENC_LEFT);
    encRight = new IrWheelCounter(cfg::ADI_ENC_RIGHT);
    static pros::Task enc(encoderTask, "ir-enc");
  }
  pros::screen::print(pros::E_TEXT_MEDIUM, 1, "LINKS 701 tracker ready%s", cfg::USE_IR_ENCODER ? " (IR enc)" : "");
}

void disabled() {}
void competition_initialize() {}
void autonomous() {}   // BEST does not use VEX field control; auto is button-launched.

void opcontrol() {
  bool calMode = false, wasPlacing = false;
  int loop = 0;
  std::vector<RawTag> tags;
  while (true) {
    // ---- buttons ----
    // ---- switches ----
    if (plateSw) plateSw->update();
    const bool whisk = whiskerSw ? whiskerSw->update() : false;
    const bool bump = bumperSw ? bumperSw->update() : false;
    const bool plateOk = !plateSw || plateSw->pressed();   // no switch fitted = assume OK

    const bool startR = master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A);
    const bool startL = master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y);
    if ((startR || startL) && !plateOk) master.print(2, 0, "NO PLATE - not started");
    else if (startR) diningAuto.start(-1);
    else if (startL) diningAuto.start(+1);
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) diningAuto.abort("driver abandoned");
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) calMode = !calMode;
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) { if (logFile) stopLog(); else startLog(); }
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN) && !diningAuto.running() && !sysidActive)
      sysidStart();

    // ---- camera (every Nth loop) ----
    bool newFrame = false;
    if (loop % cfg::VISION_EVERY_N_LOOPS == 0) {
      newFrame = readTags(tags);
      if (calMode && newFrame) calDump(tags);
    }

    // ---- drive ----
    const double dt = cfg::LOOP_MS / 1000.0;
    const bool placing = diningAuto.state() == AutoState::PLACE;
    const bool placeDone = placing && placeHook(placing && !wasPlacing);
    wasPlacing = placing;
    if (cfg::USE_IR_ENCODER) {     // hand the state machine the wheel travel since last loop
      static long prevL = 0, prevR = 0;
      const long nl = encLeft->count(), nr = encRight->count();
      diningAuto.feedOdometry((nl - prevL) * cfg::ENC_IN_PER_EDGE, (nr - prevR) * cfg::ENC_IN_PER_EDGE);
      prevL = nl; prevR = nr;
    }
    if (whiskerSw || bumperSw) diningAuto.feedSwitches(whisk, bump);
    AutoOutput out = diningAuto.step(dt, newFrame ? &tags : nullptr, placeDone);

    if (diningAuto.running()) {
      setDrive(out.leftMv, out.rightMv);           // hands off the sticks!
    } else if (sysidActive) {
      const int cmd = sysidStep(newFrame, tags);
      setDrive(cmd, cmd);
    } else {
      const double fwd = -1 * master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) / 127.0; // had to reverse
      const double turn = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X) / 127.0;
      setDrive((fwd + turn) * cfg::MC55_MAX_MV, (fwd - turn) * cfg::MC55_MAX_MV);
      if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
        arm.move(100);
      } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
        arm.move(-100);
      } else {
        arm.move(0);
      }
      if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
        flapJack.move(100);
      } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
        flapJack.move(-100);
      } else {
        flapJack.move(0);
      }
      if (master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
        
        finger.set_value(60);
      } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
        finger.set_value(-60);
      }
    }
    logLine(out);

    // ---- screens (controller screen updates are slow: only every 100 ms) ----
    if (loop % 10 == 0) {
      const auto& t = diningAuto.telemetry();
      pros::screen::print(pros::E_TEXT_MEDIUM, 2, "state %-9s sig %5.2f  ", stateName(t.state), t.sigma);
      pros::screen::print(pros::E_TEXT_MEDIUM, 3, "ex %6.1f ey %6.1f tags %d  ", t.ex, t.ey, (int)tags.size());
      if (t.state == AutoState::ABORT)
        pros::screen::print(pros::E_TEXT_MEDIUM, 4, "abort: %-24s", t.abortReason);
      master.print(0, 0, "%-8s %s%s   ", stateName(t.state), calMode ? "CAL " : "", logFile ? "LOG" : "");
    }
    ++loop;
    pros::delay(cfg::LOOP_MS);
  }
}
