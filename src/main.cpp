// main.cpp -- LINKS 701 robot program (PROS 4): driver controls v2 + Dining Room auto + camera self-check.
// Controls follow Best_2026_Controller_Controls_v2.xlsx (the controls group's sheet). Summary:
//   Left stick   : full-speed one-stick arcade (has priority)   Right stick : ~1/2-speed arcade
//   L1 / L2      : arm up / down (holds)                         R1 / R2     : FlapJack tip up / down (holds)
//   D-pad alone  : claw  LEFT open | RIGHT closed | UP finger up | DOWN finger down
//   B + L1/L2    : arm to the next setpoint above / below        B + R1/R2   : FlapJack to the next angle
//   B + Dup      : PLATE CARRY  (arm level, FJ 0, claw closed -- also the auto start pose)
//   B + Ddown    : PLATE CHECK  (arm fully down, FJ +15, claw closed)
//   A + Dup      : DOOR OPEN    A + Dleft : TOP 3 BREAD    A + Dright : MEAT LEVEL 2    A + Ddown : LEVEL 1
//   Y + Dleft / Y + Dright : start the Dining Room auto (left / right search). Camera self-check runs first.
//   A + X + B + Dup + Ddown held 0.5 s : STOW (pre-match). While X is held, A/B + D-pad macros are blocked.
//   During auto: any NEW press from 1 s after the start cancels it. Hands free (Rule 3.3.2.2.A).
// Controller screen: line 1 = mode / macro, line 2 = FlapJack angle, line 3 = warnings (timed warnings, else
// POT! / RUN! for a joint). Driver cheat sheet + controls sheet v3 match this file.
#include "main.h"
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>
#include "tracker/config.hpp"
#include "tracker/mount.hpp"
#include "tracker/dining_auto.hpp"
#include "tracker/arm_control.hpp"
#include "tracker/color_ref.hpp"

using namespace trk;

// ------------------------------ devices ------------------------------
static pros::Controller master(pros::E_CONTROLLER_MASTER);
static pros::Motor leftDrive(cfg::PORT_LEFT_DRIVE);     // MC55 on a Smart Port
static pros::Motor rightDrive(cfg::PORT_RIGHT_DRIVE);   // negative port = reversed
static pros::Motor armMotor(cfg::PORT_ARM);             // MC55, main 4-bar arm
static pros::Motor flapjackMotor(cfg::PORT_FLAPJACK);   // MC55, FlapJack end (wrist)
static pros::adi::Port clawServo(cfg::ADI_CLAW_SERVO, pros::E_ADI_LEGACY_SERVO);
static pros::AIVision vision(cfg::PORT_AIVISION);
static pros::adi::AnalogIn armPot(cfg::ADI_ARM_POT);
static pros::adi::AnalogIn fjPot(cfg::ADI_FJ_POT);

static const CameraModel CAM = cameraFromConfig();     // lens position from the riser mount (config.hpp)
static DiningAuto diningAuto(CAM);

// ------------------------------ buttons: read ONCE per loop ------------------------------
enum Btn { BL1, BL2, BR1, BR2, BUP, BDN, BLT, BRT, BA, BB, BX, BY, NBTN };
static const pros::controller_digital_e_t BTN_ID[NBTN] = {
    pros::E_CONTROLLER_DIGITAL_L1, pros::E_CONTROLLER_DIGITAL_L2, pros::E_CONTROLLER_DIGITAL_R1,
    pros::E_CONTROLLER_DIGITAL_R2, pros::E_CONTROLLER_DIGITAL_UP, pros::E_CONTROLLER_DIGITAL_DOWN,
    pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT, pros::E_CONTROLLER_DIGITAL_A,
    pros::E_CONTROLLER_DIGITAL_B, pros::E_CONTROLLER_DIGITAL_X, pros::E_CONTROLLER_DIGITAL_Y};
static bool held[NBTN], pressed[NBTN];
static void readButtons() {
  for (int i = 0; i < NBTN; ++i) {
    held[i] = master.get_digital(BTN_ID[i]);
    pressed[i] = master.get_digital_new_press(BTN_ID[i]);   // one call per button per loop
  }
}
static bool anyPressed() { for (int i = 0; i < NBTN; ++i) if (pressed[i]) return true; return false; }

// ------------------------------ controller screen text ------------------------------
static char line1[20] = "Ready", line3[20] = "";
static std::uint32_t line3Until = 0;
static void say(const char* s) { std::snprintf(line1, sizeof line1, "%s", s); }
static void warn(const char* s, int ms = 3000) { std::snprintf(line3, sizeof line3, "%s", s); line3Until = pros::millis() + ms; }

// ------------------------------ claw servo ------------------------------
// Every command goes through here so we can SEE whether PROS accepted it (rc = PROS_ERR -> rejected).
static int servoLastCmd = 0, servoLastRc = 0, servoLastErr = 0;
static void claw(int v) {
  errno = 0;
  servoLastCmd = v;
  servoLastRc = clawServo.set_value(v);
  servoLastErr = errno;
}
constexpr int NO_CLAW = -1000;

// ------------------------------ drive ------------------------------
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
static void driverDrive() {
  auto shape = [](int raw) {
    if (std::abs(raw) < cfg::STICK_DEADBAND) return 0.0;
    const double x = raw / 127.0;
    return cfg::SQUARED_INPUTS ? x * std::fabs(x) : x;
  };
  const int ly = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), lx = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
  const int ry = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y), rx = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
  double fwd, turn;
  if (std::abs(ly) > cfg::LEFT_PRIORITY_DB || std::abs(lx) > cfg::LEFT_PRIORITY_DB) {   // left stick has priority
    fwd = shape(ly); turn = cfg::TURN_SCALE * shape(lx);
  } else if (std::abs(ry) >= cfg::STICK_DEADBAND || std::abs(rx) >= cfg::STICK_DEADBAND) {   // slow drive
    fwd = cfg::RIGHT_STICK_SCALE * shape(ry); turn = cfg::RIGHT_STICK_SCALE * cfg::TURN_SCALE * shape(rx);
  } else {
    fwd = shape(ly); turn = cfg::TURN_SCALE * shape(lx);
  }
  double l = fwd + turn, r = fwd - turn;
  const double m = std::fmax(1.0, std::fmax(std::fabs(l), std::fabs(r)));   // full stick + turn still turns
  setDrive(l / m * cfg::MC55_MAX_MV, r / m * cfg::MC55_MAX_MV);
}

// ------------------------------ IR wheel encoders ------------------------------
// One BEST IR sensor per drive wheel, looking at the printed stripe decal. Single channel, so direction
// comes from the sign of the last command. ASSUMES a digital output (bench-test with ir_test first).
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
static pros::adi::DigitalOut* irEmitter = nullptr;
static pros::adi::DigitalOut* irEmitterR = nullptr;
static void encoderTask() {          // 1 ms polling so no edge is missed at full speed
  while (true) {
    encLeft->poll(lastLeftMv);
    if (encRight) encRight->poll(lastRightMv);
    pros::delay(1);
  }
}

// ------------------------------ contact switches ------------------------------
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

// ------------------------------ camera: tags + colour boxes from ONE read ------------------------------
static bool colorsOn = false;
static void setColorDetection(bool on) {
  if (on == colorsOn) return;
  if (on) vision.enable_detection_types(pros::AivisionModeType::colors);
  else vision.disable_detection_types(pros::AivisionModeType::colors);
  colorsOn = on;
}
// Returns false if the frame is IDENTICAL to the previous one (no new image yet).
static bool readVision(std::vector<RawTag>& tags, std::vector<Blob>& blobs) {
  static long prevHash = 0;
  tags.clear(); blobs.clear();
  long h = 17;
  const std::uint32_t now = pros::millis();
  for (const auto& o : vision.get_all_objects()) {
    if (pros::AIVision::is_type(o, pros::AivisionDetectType::tag)) {
      RawTag t; t.id = o.id; t.t_ms = now;
      t.u[0] = o.object.tag.x0; t.v[0] = o.object.tag.y0; t.u[1] = o.object.tag.x1; t.v[1] = o.object.tag.y1;
      t.u[2] = o.object.tag.x2; t.v[2] = o.object.tag.y2; t.u[3] = o.object.tag.x3; t.v[3] = o.object.tag.y3;
      tags.push_back(t);
      h = h * 131 + o.id * 7 + o.object.tag.x0 * 3 + o.object.tag.y0 * 5 + o.object.tag.x2 + o.object.tag.y2 * 11;
    } else if (pros::AIVision::is_type(o, pros::AivisionDetectType::color) && o.id >= 1 && o.id <= 4) {
      const auto& c = o.object.color;
      blobs.push_back({o.id, c.xoffset + 0.5 * c.width, c.yoffset + 0.5 * c.height, (double)c.width, (double)c.height});
      h = h * 131 + o.id + c.xoffset * 3 + c.yoffset * 5 + c.width * 7;
    }
  }
  const bool fresh = (h != prevHash) || (tags.empty() && blobs.empty());
  prevHash = h;
  return fresh;
}

// ------------------------------ SD logging ------------------------------
static FILE* logFile = nullptr;
static void startLog() {
  if (!pros::usd::is_installed()) return;
  char name[40];
  std::snprintf(name, sizeof name, "/usd/trk_%lu.csv", (unsigned long)pros::millis());
  logFile = std::fopen(name, "w");
  if (logFile)
    std::fprintf(logFile, "t_ms,state,cmdL,cmdR,v,w,haveMeas,accepted,measX,measY,"
                          "sigAlong,sigCross,estX,estY,sigma,ex,ey\n");
}
static void logLine(const AutoOutput& out) {
  if (!logFile) return;
  const auto& t = diningAuto.telemetry();
  const auto& e = diningAuto.ekf();
  std::fprintf(logFile, "%lu,%s,%.0f,%.0f,%.2f,%.3f,%d,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
               (unsigned long)pros::millis(), stateName(t.state), out.leftMv, out.rightMv, t.v, t.w,
               (int)t.haveMeas, (int)t.accepted, t.meas.x, t.meas.y, t.measAlong, t.measCross,
               e.x, e.y, t.sigma, t.ex, t.ey);
}

// ------------------------------ motion-model test (Exercise E5, SYSID_MODE) ------------------------------
static const int SYSID_MV[] = {2000, 3000, 4000, 5000, 6000, 8000};
static int sysidIndex = 0, sysidMv = 0;
static bool sysidActive = false;
static std::uint32_t sysidT0 = 0;
static FILE* sysidFile = nullptr;
static void sysidStart() {
  if (!pros::usd::is_installed()) { warn("NO SD CARD"); return; }
  sysidMv = SYSID_MV[sysidIndex];
  sysidIndex = (sysidIndex + 1) % (int)(sizeof(SYSID_MV) / sizeof(SYSID_MV[0]));
  char name[48];
  std::snprintf(name, sizeof name, "/usd/sysid_%d_%lu.csv", sysidMv, (unsigned long)pros::millis());
  sysidFile = std::fopen(name, "w");
  if (!sysidFile) return;
  std::fprintf(sysidFile, "t_ms,cmd_mV,id,measX,measY\n");
  sysidT0 = pros::millis();
  sysidActive = true;
  say("SYSID run");
}
static int sysidStep(bool newFrame, const std::vector<RawTag>& tags, const CameraModel& cam) {
  const std::uint32_t t = pros::millis() - sysidT0;
  const int cmd = (t < 1200) ? sysidMv : 0;
  if (newFrame)
    for (const auto& tag : tags) {
      TableTagMeas m = measureTableTag(cam, tag, cfg::TABLE_TAG_Z_IN, cfg::TAG_QUAD_IN, cfg::SIDE_TOL_FRAC);
      if (m.ok) std::fprintf(sysidFile, "%lu,%d,%d,%.2f,%.2f\n", (unsigned long)t, cmd, m.id, m.center.x, m.center.y);
    }
  if (t > 2400) { std::fclose(sysidFile); sysidFile = nullptr; sysidActive = false; say("SYSID done"); }
  return cmd;
}

// ------------------------------ arm + FlapJack (pots on axles 2 and 3) ------------------------------
static arm::Joint armJ{cfg::ARM_KP, cfg::ARM_KD, cfg::ARM_KG, cfg::ARM_MAX_MV, cfg::ARM_MIN_DEG, cfg::ARM_MAX_DEG, cfg::ARM_TOL_DEG};
static arm::Joint fjJ{cfg::FJ_KP, cfg::FJ_KD, cfg::FJ_KG, cfg::FJ_MAX_MV, cfg::FJ_MIN_DEG - 360, cfg::FJ_MAX_DEG + 360, cfg::FJ_TOL_DEG};
static double armDeg = 0, fjGroundDeg = 0, fjRelDeg = 0, fjGroundTarget = 0;
static bool armPotOk = false, fjPotOk = false, armInit = false;

static void readArm() {
  const int ra = armPot.get_value(), rf = fjPot.get_value();
  armPotOk = cfg::USE_ARM_POT && arm::potOk(ra);
  fjPotOk = cfg::USE_FJ_POT && arm::potOk(rf);
  armDeg = arm::potDeg(ra, cfg::ARM_RAW_AT_LEVEL, cfg::ARM_SIGN);
  fjRelDeg = arm::potDeg(rf, cfg::FJ_RAW_AT_LEVEL, cfg::FJ_SIGN);
  fjGroundDeg = cfg::FJ_POT_RELATIVE_TO_ARM ? fjRelDeg + armDeg : fjRelDeg;
  if (!armInit) { armJ.setTarget(armDeg); fjGroundTarget = fjGroundDeg; armInit = true; }   // no jump at enable
}
static void setFj(double g) { fjGroundTarget = std::fmin(cfg::FJ_MAX_DEG, std::fmax(cfg::FJ_MIN_DEG, g)); }
// armIn / fjIn: -1, 0, +1 = manual rate input (driver), 0 in auto.
static void runArm(int armIn, int fjIn, double dt) {
  int armMv, fjMv;
  if (armPotOk) {
    if (armIn) armJ.setTarget(armJ.target + armIn * cfg::ARM_RATE_DPS * dt);
    armMv = armJ.output(armDeg, dt, armDeg);
  } else {
    armJ.reset(); armMv = armIn * cfg::ARM_MV;
  }
  if (fjPotOk) {
    if (fjIn) setFj(fjGroundTarget + fjIn * cfg::FJ_RATE_DPS * dt);
    const double relTarget = (cfg::FJ_AUTO_LEVEL && cfg::FJ_POT_RELATIVE_TO_ARM) ? fjGroundTarget - armDeg
                           : (cfg::FJ_POT_RELATIVE_TO_ARM ? fjGroundTarget - armJ.target : fjGroundTarget);
    fjJ.setTarget(relTarget);
    fjMv = fjJ.output(fjRelDeg, dt, fjGroundDeg);
  } else {
    fjJ.reset(); fjMv = fjIn * cfg::FLAPJACK_MV;
  }
  armMotor.move_voltage(armMv);
  flapjackMotor.move_voltage(fjMv);
}

// ------------------------------ driver macros (controls v2) ------------------------------
struct Macro { const char* name; double arm; double fj; int claw; };
static const Macro M_PLATE_CARRY = {"Plate carry", cfg::ARM_PLATE_CARRY_DEG, 0.0, cfg::CLAW_CLOSED};
static const Macro M_PLATE_CHECK = {"Plate check", cfg::ARM_PLATE_CHECK_DEG, cfg::FJ_PLATE_CHECK_DEG, cfg::CLAW_CLOSED};
static const Macro M_DOOR_OPEN   = {"Door open", cfg::ARM_DOOR_OPEN_DEG, cfg::FJ_DOOR_OPEN_DEG, cfg::CLAW_CLOSED};
static const Macro M_TOP3_BREAD  = {"Top 3 bread", cfg::ARM_TOP3_BREAD_DEG, 0.0, cfg::CLAW_OPEN};
static const Macro M_MEAT_L2     = {"Meat level 2", cfg::ARM_MEAT_L2_DEG, 0.0, cfg::CLAW_OPEN};
static const Macro M_LEVEL1      = {"Level 1", cfg::ARM_LEVEL1_DEG, 0.0, cfg::CLAW_OPEN};
static const Macro M_STOW        = {"Stowing...", cfg::ARM_STOW_DEG, cfg::FJ_LEVEL_DEG, cfg::CLAW_OPEN};
// B+L1/L2 setpoints = the arm heights the macros use
static const Macro* const ARM_SETPOINTS[] = {&M_STOW, &M_PLATE_CARRY, &M_PLATE_CHECK, &M_DOOR_OPEN, &M_TOP3_BREAD, &M_MEAT_L2, &M_LEVEL1};
static bool macroActive = false, stowing = false;

static void runMacro(const Macro& m) {
  armJ.setTarget(m.arm);
  setFj(m.fj);
  if (m.claw != NO_CLAW) claw(m.claw);
  macroActive = true;
  stowing = (&m == &M_STOW);
  say(m.name);
}
static void stepArmSetpoint(int dir) {          // +1 = next above, -1 = next below (skip within 3 deg)
  const Macro* best = nullptr;
  for (const Macro* m : ARM_SETPOINTS) {
    const double d = (m->arm - armDeg) * dir;
    if (d >= cfg::SETPOINT_MIN_STEP_DEG && (!best || d < (best->arm - armDeg) * dir)) best = m;
  }
  if (!best) { warn(dir > 0 ? "No setpoint above" : "No setpoint below"); return; }
  armJ.setTarget(best->arm); macroActive = true; stowing = false; say(best->name);
}
static void stepFjSetpoint(int dir) {
  if (!fjPotOk) { warn("FJ N/A"); return; }
  double best = NAN;
  for (double s : cfg::FJ_SETPOINTS_DEG) {
    const double d = (s - fjGroundDeg) * dir;
    if (d >= cfg::SETPOINT_MIN_STEP_DEG && (std::isnan(best) || d < (best - fjGroundDeg) * dir)) best = s;
  }
  if (std::isnan(best)) { warn(dir > 0 ? "FJ at top" : "FJ at bottom"); return; }
  setFj(best); macroActive = true; stowing = false;
}

// ------------------------------ camera self-check (forearm decal) ------------------------------
static double refD[3] = {0, 0, 0};              // camera change since the baseline: d_pitch, d_yaw, d_roll [rad]
static bool refHave = false;
static char ccMsg[48] = "not run";
enum class CC { OFF, ABOVE, DOWN, SETTLE, COLLECT };
static CC cc = CC::OFF;
static int ccDir = 0, ccN = 0, ccSquares = 0;
static std::uint32_t ccT0 = 0, ccSettleT = 0;
static double ccAcc[3] = {0, 0, 0};
static std::uint32_t autoStartMs = 0;
static bool autoWasRunning = false;

static bool haveDecalBaseline() {
  for (int k = 0; k < 4; ++k) if (cfg::REF_DECAL_U[k] != 0 || cfg::REF_DECAL_V[k] != 0) return true;
  return false;
}
static void startAutoNow(int dir) {
  diningAuto.start(dir);
  armJ.setTarget(cfg::ARM_AUTO_CARRY_DEG);   // carry low: the plate must not hide the tables
  setFj(cfg::FJ_LEVEL_DEG);
  macroActive = false;
  say(dir > 0 ? "AUTO search L" : "AUTO search R");
}
static void ccStop() { cc = CC::OFF; setColorDetection(false); }
static void requestAuto(int dir) {              // Y + Dleft / Dright
  autoStartMs = pros::millis();
  if (cfg::USE_DECAL_REF && cfg::SELFCHECK_ON_START && armPotOk && haveDecalBaseline()) {
    cc = CC::ABOVE; ccDir = dir; ccN = 0; ccT0 = pros::millis(); ccAcc[0] = ccAcc[1] = ccAcc[2] = 0;
    setColorDetection(true);
    setFj(cfg::FJ_LEVEL_DEG);
    if (armDeg > cfg::ARM_CAMCHECK_DEG + 3.0) { cc = CC::DOWN; armJ.setTarget(cfg::ARM_CAMCHECK_DEG); }
    else armJ.setTarget(cfg::ARM_CAMCHECK_DEG + cfg::ARM_CAMCHECK_FROM_ABOVE_DEG);   // come down onto it
    macroActive = false;
    say("Cam check");
  } else {
    startAutoNow(dir);
  }
}
static void ccStep(bool newFrame, const std::vector<Blob>& blobs) {
  if (cc == CC::OFF) return;
  const std::uint32_t now = pros::millis();
  if (now - ccT0 > (std::uint32_t)cfg::CAMCHECK_TIMEOUT_MS) {
    std::snprintf(ccMsg, sizeof ccMsg, "SKIPPED (timeout, %d frames)", ccN);
    warn("CAM CHECK SKIPPED");
    ccStop(); startAutoNow(ccDir);              // keep the last correction
    return;
  }
  const double err = std::fabs(armDeg - cfg::ARM_CAMCHECK_DEG);
  switch (cc) {
    case CC::ABOVE:
      if (armDeg >= armJ.target - 1.5) { cc = CC::DOWN; armJ.setTarget(cfg::ARM_CAMCHECK_DEG); }
      break;
    case CC::DOWN:
      if (err <= cfg::CAMCHECK_SETTLE_DEG) { cc = CC::SETTLE; ccSettleT = now; }
      break;
    case CC::SETTLE:
      if (err > cfg::CAMCHECK_SETTLE_DEG) cc = CC::DOWN;
      else if (now - ccSettleT >= (std::uint32_t)cfg::CAMCHECK_SETTLE_MS) cc = CC::COLLECT;
      break;
    case CC::COLLECT: {
      if (!newFrame) break;
      const DecalResult R = decalCheck(CAM, cfg::CAM_SIDE, blobs.data(), (int)blobs.size(), armDeg, true,
                                       cfg::REF_DECAL_U, cfg::REF_DECAL_V, cfg::REF_DECAL_DEG, cfg::REF_SEARCH_PX);
      ccSquares = R.squares;
      if (R.sideWarn) {                         // the camera sees the OTHER face: config corner is wrong
        std::snprintf(ccMsg, sizeof ccMsg, "CAMERA ON OTHER CORNER? check CAM_SIDE");
        warn("CAM SIDE WRONG", 6000); say("Auto cancelled");
        ccStop();
        return;
      }
      if (R.ok) { for (int j = 0; j < 3; ++j) ccAcc[j] += R.d[j]; ++ccN; }
      if (ccN >= cfg::CAMCHECK_FRAMES) {
        for (int j = 0; j < 3; ++j) refD[j] = ccAcc[j] / ccN;   // relative to the baseline -> replaces the old value
        refHave = true;
        diningAuto.setCamera(applyEuler(CAM, refD));
        std::snprintf(ccMsg, sizeof ccMsg, "dP %+.2f dY %+.2f dR %+.2f deg", rad2deg(refD[0]), rad2deg(refD[1]), rad2deg(refD[2]));
        std::printf("CAMCHECK,%lu,%.3f,%.3f,%.3f,%d\n", (unsigned long)now, rad2deg(refD[0]), rad2deg(refD[1]), rad2deg(refD[2]), ccN);
        ccStop(); startAutoNow(ccDir);
      }
      break;
    }
    default: break;
  }
}

// ------------------------------ auto place hook ------------------------------
// Called every loop while the state machine is in PLACE. Non-blocking. Returns true when DONE.
static bool placeHook(bool firstCall) {
  static int phase = 0;
  static std::uint32_t tPhase = 0, releasedAt = 0;
  if (firstCall) { phase = 0; tPhase = pros::millis(); releasedAt = 0; armJ.setTarget(cfg::ARM_AUTO_PLACE_DEG); setFj(cfg::FJ_LEVEL_DEG); }
  if (phase == 0) {                                   // 1) lower the plate onto the table
    if ((armPotOk && armJ.atTarget(armDeg)) || pros::millis() - tPhase > 1500) { phase = 1; tPhase = pros::millis(); claw(cfg::CLAW_OPEN); }
    return false;
  }
  if (phase == 1 && pros::millis() - tPhase > 300) { phase = 2; armJ.setTarget(cfg::ARM_AUTO_CARRY_DEG); }   // 2) lift back
  const std::uint32_t dt = pros::millis() - tPhase;
  if (cfg::USE_PLATE_SWITCH) {
    if (!plateSw->pressed() && releasedAt == 0) releasedAt = pros::millis();
    if (releasedAt != 0 && pros::millis() - releasedAt > 150) return true;
    return dt > 2000;
  }
  return dt > 400;
}

// ------------------------------ PROS entry points ------------------------------
void initialize() {
  vision.reset();
  vision.enable_detection_types(pros::AivisionModeType::tags);
  vision.set_tag_family(pros::AivisionTagFamily::tag_21H7);   // Circle21h7 (Rule 3.3.2.2.A.B)
  if (cfg::USE_DECAL_REF)                                      // the 4 printed decal colours (ids 1..4)
    for (int i = 0; i < 4; ++i)
      vision.set_color({(std::uint8_t)(i + 1), cfg::COLOR_RGB[i][0], cfg::COLOR_RGB[i][1], cfg::COLOR_RGB[i][2],
                        cfg::COLOR_HUE_RANGE, cfg::COLOR_SAT_RANGE});
  leftDrive.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);         // MC55: coast or brake only
  rightDrive.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  armMotor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  flapjackMotor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  clawServo.set_config(pros::E_ADI_LEGACY_SERVO);
  claw(cfg::CLAW_CLOSED);
  if (cfg::USE_PLATE_SWITCH) plateSw = new Switch(cfg::ADI_PLATE_SWITCH);
  if (cfg::USE_EDGE_WHISKER) whiskerSw = new Switch(cfg::ADI_EDGE_WHISKER);
  if (cfg::USE_FRONT_BUMPER) bumperSw = new Switch(cfg::ADI_FRONT_BUMPER);
  if (cfg::USE_IR_ENCODER) {
    irEmitter = new pros::adi::DigitalOut(cfg::ADI_IR_EMITTER, true);
    encLeft = new IrWheelCounter(cfg::ADI_IR_RECEIVER);
    if (cfg::USE_RIGHT_ENCODER) {
      irEmitterR = new pros::adi::DigitalOut(cfg::ADI_IR_TX_RIGHT, true);
      encRight = new IrWheelCounter(cfg::ADI_ENC_RIGHT_RX);
    }
    static pros::Task enc(encoderTask, "ir-enc");
  }
  pros::screen::print(pros::E_TEXT_MEDIUM, 1, "LINKS 701 ready  camera %s corner, lens X %.1f Y %+.1f Z %.1f",
                      cfg::CAM_SIDE > 0 ? "LEFT" : "RIGHT", CAM.xoff, CAM.yoff, CAM.height);
}

void disabled() {}
void competition_initialize() {}
void autonomous() {}   // BEST does not use VEX field control; auto is button-launched.

void opcontrol() {
  bool wasPlacing = false;
  int loop = 0;
  std::uint32_t stowT0 = 0;
  std::vector<RawTag> tags;
  std::vector<Blob> blobs;
  if (cfg::AUTO_SD_LOG) startLog();
  while (true) {
    const double dt = cfg::LOOP_MS / 1000.0;
    readButtons();
    if (plateSw) plateSw->update();
    const bool whisk = whiskerSw ? whiskerSw->update() : false;
    const bool bump = bumperSw ? bumperSw->update() : false;
    const bool plateOk = !plateSw || plateSw->pressed();   // no switch fitted = assume OK
    readArm();

    // ---- camera (every Nth loop) ----
    bool newFrame = false;
    if (loop % cfg::VISION_EVERY_N_LOOPS == 0) newFrame = readVision(tags, blobs);

    const bool autoBusy = diningAuto.running() || cc != CC::OFF;
    // ---- auto cancel: any NEW press from 1 s after the start ----
    if (autoBusy && pros::millis() - autoStartMs > (std::uint32_t)cfg::AUTO_CANCEL_GRACE_MS && anyPressed()) {
      if (cc != CC::OFF) ccStop();
      if (diningAuto.running()) diningAuto.abort("driver cancelled");
      armJ.setTarget(armDeg); setFj(fjGroundDeg);  // hold where we are
      say("Auto cancelled");
      for (int i = 0; i < NBTN; ++i) pressed[i] = false;   // this press only cancels
    }
    ccStep(newFrame, blobs);

    // ---- driver inputs (ignored while the auto / self-check runs) ----
    int armIn = 0, fjIn = 0;
    if (!diningAuto.running() && cc == CC::OFF && !sysidActive) {
      const bool faceHeld = held[BA] || held[BB] || held[BX] || held[BY];
      // STOW: A + X + B + Dup + Ddown held together for 0.5 s (pre-match only)
      if (held[BA] && held[BX] && held[BB] && held[BUP] && held[BDN]) {
        if (stowT0 == 0) stowT0 = pros::millis();
        else if (pros::millis() - stowT0 >= (std::uint32_t)cfg::STOW_HOLD_MS && !stowing) runMacro(M_STOW);
      } else stowT0 = 0;
      const int dIdx = pressed[BUP] ? BUP : pressed[BDN] ? BDN : pressed[BLT] ? BLT : pressed[BRT] ? BRT : -1;
      if (dIdx >= 0) {
        if (!faceHeld) {                                                   // claw positions
          if (dIdx == BLT) { claw(cfg::CLAW_OPEN); say("Claw open"); }
          if (dIdx == BRT) { claw(cfg::CLAW_CLOSED); say("Claw closed"); }
          if (dIdx == BUP) { claw(cfg::CLAW_FINGER_UP); say("Finger up"); }
          if (dIdx == BDN) { claw(cfg::CLAW_FINGER_DOWN); say("Finger down"); }
        } else if (held[BY] && !held[BA] && !held[BB] && !held[BX]) {     // Y + Dleft / Dright: auto
          if (dIdx == BLT || dIdx == BRT) {
            const int dir = dIdx == BLT ? +1 : -1;
            if (!plateOk) warn("NO PLATE");
            else if (cfg::SYSID_MODE) sysidStart();
            else requestAuto(dir);
          }
        } else if (held[BX]) {
          // X held: A/B + D-pad macros blocked (stow combo passes through them)
        } else if (held[BB] && !held[BA]) {                               // B presets
          if (dIdx == BUP) runMacro(M_PLATE_CARRY);
          if (dIdx == BDN) runMacro(M_PLATE_CHECK);
        } else if (held[BA] && !held[BB]) {                               // A pickup heights
          if (dIdx == BUP) runMacro(M_DOOR_OPEN);
          if (dIdx == BLT) runMacro(M_TOP3_BREAD);
          if (dIdx == BRT) runMacro(M_MEAT_L2);
          if (dIdx == BDN) runMacro(M_LEVEL1);
        }
      }
      if (held[BB] && !held[BA] && !held[BX]) {                           // B + L/R: setpoint stepping
        if (pressed[BL1]) stepArmSetpoint(+1);
        if (pressed[BL2]) stepArmSetpoint(-1);
        if (pressed[BR1]) stepFjSetpoint(+1);
        if (pressed[BR2]) stepFjSetpoint(-1);
      } else {
        // a NEW L/R press cancels a running macro: start from where the joint is now
        if (pressed[BL1] || pressed[BL2]) { armJ.setTarget(armDeg); macroActive = stowing = false; say("Manual"); }
        if (pressed[BR1] || pressed[BR2]) { setFj(fjGroundDeg); macroActive = stowing = false; say("Manual"); }
        armIn = held[BL1] ? 1 : held[BL2] ? -1 : 0;
        fjIn = held[BR1] ? 1 : held[BR2] ? -1 : 0;
      }
      if (stowing && armPotOk && armJ.atTarget(armDeg)) say("Stowed, ready");
    }

    // ---- drive ----
    const bool placing = diningAuto.state() == AutoState::PLACE;
    const bool placeDone = placing && placeHook(placing && !wasPlacing);
    wasPlacing = placing;
    if (cfg::USE_IR_ENCODER) {
      static long prevL = 0, prevR = 0;
      const long nl = encLeft->count();
      if (encRight) {
        const long nr = encRight->count();
        diningAuto.feedOdometry((nl - prevL) * cfg::ENC_IN_PER_EDGE, (nr - prevR) * cfg::ENC_IN_PER_EDGE);
        prevR = nr;
      } else {
        diningAuto.feedOdometryLeftOnly((nl - prevL) * cfg::ENC_IN_PER_EDGE);
      }
      prevL = nl;
    }
    if (whiskerSw || bumperSw) diningAuto.feedSwitches(whisk, bump);
    AutoOutput out = diningAuto.step(dt, newFrame ? &tags : nullptr, placeDone);
    if (diningAuto.running()) setDrive(out.leftMv, out.rightMv);   // hands off the sticks!
    else if (sysidActive) { const int cmd = sysidStep(newFrame, tags, refHave ? applyEuler(CAM, refD) : CAM); setDrive(cmd, cmd); }
    else if (cc != CC::OFF) setDrive(0, 0);                         // self-check: robot still
    else driverDrive();
    if (autoWasRunning && !diningAuto.running()) {                   // auto just ended
      const auto& t = diningAuto.telemetry();
      if (t.state == AutoState::DONE) say("Auto done");
      else if (std::strcmp(t.abortReason, "driver cancelled") != 0) { say("Auto stopped"); warn(t.abortReason, 5000); }
      armJ.setTarget(armDeg); setFj(fjGroundDeg);
    }
    autoWasRunning = diningAuto.running();
    runArm(armIn, fjIn, dt);
    logLine(out);

    // ---- screens ----
    if (loop % 10 == 0) {
      const auto& t = diningAuto.telemetry();
      pros::screen::print(pros::E_TEXT_MEDIUM, 2, "state %-9s sig %5.2f  ex %6.1f ey %6.1f tags %d   ", stateName(t.state), t.sigma, t.ex, t.ey, (int)tags.size());
      pros::screen::print(pros::E_TEXT_MEDIUM, 3, "arm %6.1f->%6.1f %s  fj %6.1f->%6.1f %s   ", armDeg, armJ.target,
                          armPotOk ? (armJ.faulted ? "RUN!" : "") : "POT!", fjGroundDeg, fjGroundTarget, fjPotOk ? (fjJ.faulted ? "RUN!" : "") : "POT!");
      pros::screen::print(pros::E_TEXT_MEDIUM, 4, "cam check: %-40s", cc != CC::OFF ? "running" : ccMsg);
      pros::screen::print(pros::E_TEXT_MEDIUM, 5, "decal %d/4 squares  baseline %s  colours %s   ", ccSquares,
                          haveDecalBaseline() ? "set" : "NOT SET", colorsOn ? "ON" : "off");
      pros::screen::print(pros::E_TEXT_MEDIUM, 6, "claw cmd %4d rc %d err %d  log %s   ", servoLastCmd,
                          servoLastRc == PROS_ERR ? -1 : servoLastRc, servoLastErr, logFile ? "ON" : "off");
      if (t.state == AutoState::ABORT) pros::screen::print(pros::E_TEXT_MEDIUM, 7, "abort: %-30s", t.abortReason);
    }
    if (line3[0] && pros::millis() > line3Until) line3[0] = 0;
    switch (loop % 15) {                                      // controller: one line per 50 ms
      case 0:  master.print(0, 0, "%-19s", line1); break;
      case 5:
        if (fjPotOk) master.print(1, 0, "FJ %+4.0f deg         ", fjGroundDeg);
        else master.print(1, 0, "FJ N/A              ");
        break;
      case 10: {                                            // timed warning first, else a standing joint fault
        const char* w = line3[0] ? line3 : armJ.faulted ? "RUN! arm" : fjJ.faulted ? "RUN! FJ"
                      : (cfg::USE_ARM_POT && !armPotOk) ? "POT! arm" : (cfg::USE_FJ_POT && !fjPotOk) ? "POT! FJ" : "";
        master.print(2, 0, "%-19s", w);
        break;
      }
    }
    ++loop;
    pros::delay(cfg::LOOP_MS);
  }
}
