// pot_test -- LINKS 701 potentiometer test + set-point teaching for the arm (axle 2) and FlapJack (axle 3).
// Build: `pros c new pot_test` (PROS 4), replace src/main.cpp with this file, copy include/arm_control.hpp
// into the project's include/ folder, then `pros mu`. Values to paste into the main robot's
// include/tracker/config.hpp are printed in the PROS terminal (`pros terminal`) and on the Brain screen.
//
// KIT POT: rotary, linear, 16 kOhm, 300 deg, ~6 mm shaft -> 0..4095 counts, 0.0733 deg/count.
// Geared 1:1 with 16T gears, so the pot turns exactly with its axle.
// Wiring (Screw Terminal Sensor Interface Cable): pot ends to red (+5 V) and black (ground), wiper (middle
// pin) to white (signal). Install the gear so the pot reads ~2048 in the MIDDLE of the joint's travel and
// never reaches 0 or 4095 anywhere in the travel (the screen warns "END!" within 150 counts of an end).
//
// PORTS: A = arm pot (axle 2)  B = FlapJack pot (axle 3)  Smart 4 = arm motor  Smart 5 = FlapJack motor
//
// STEP 1 - MANUAL (start-up mode): L1/L2 drive the arm, R1/R2 drive the FlapJack, plain voltage.
//          Watch the raw numbers: raising the arm / tipping the fork up should move them smoothly.
// STEP 2 - LEVEL: set the arm straight forward and level AND the fork level (phone level), press A.
//          This records ARM_RAW_AT_LEVEL and FJ_RAW_AT_LEVEL.
// STEP 3 - SIGNS: raise the arm into the STOW pose and press X+LEFT (stores STOW and sets ARM_SIGN so
//          STOW reads positive). Tip the fork UP from level, hold Y and tap UP (sets FJ_SIGN).
// TEACH the arm presets used by the match program (controls v2 macros + the Dining Room auto):
//          UP / DOWN (no X) = choose which preset (name on line 3 of the controller)
//          X+RIGHT = store the CURRENT arm angle into the chosen preset
//          X+LEFT  = store STOW (folded into the 24 in start cube). The first time, ARM_SIGN is set so
//                    that STOW comes out positive (arm up).
//          Hold Y and tap UP with the fork tipped UP to set FJ_SIGN (fork-up = positive).
//          PLATE CARRY and CAMCHECK are 0 deg (arm level) by definition -- they come from step 2 (LEVEL).
// GO TO (closed loop): RIGHT (no X) = go to the chosen preset, LEFT (no X) = go to STOW.
//          While closed loop is on, L1/L2 and R1/R2 move the targets (like the main robot), the
//          FlapJack keeps its angle to the ground (auto-level).
// B      : STOP -- motors off, back to manual. Also clears a RUNAWAY fault.
// RUNAWAY: if a joint pushes hard and its error keeps growing for 0.4 s, it stops and shows RUN!
//          That means the SIGN or the motor direction is backwards -- fix it before trusting presets.
#include "main.h"
#include <cmath>
#include <cstdio>
#include "arm_control.hpp"

// ---------------- settings (copy the same numbers into the main robot config.hpp) ----------------
constexpr char ADI_ARM_POT = 'A', ADI_FJ_POT = 'B';
constexpr int PORT_ARM = 4, PORT_FLAPJACK = 5;
constexpr bool FJ_POT_RELATIVE_TO_ARM = true;
constexpr int MANUAL_ARM_MV = 6000, MANUAL_FJ_MV = 5000;
constexpr double ARM_KP = 180.0, ARM_KD = 6.0, ARM_KG = 1200.0; constexpr int ARM_MAX_MV = 8000;
constexpr double FJ_KP = 150.0, FJ_KD = 4.0, FJ_KG = 0.0; constexpr int FJ_MAX_MV = 6000;
constexpr double ARM_MIN_DEG = -55.0, ARM_MAX_DEG = 150.0, FJ_MIN_DEG = -90.0, FJ_MAX_DEG = 90.0;
constexpr double RATE_DPS = 60.0;

static pros::Controller master(pros::E_CONTROLLER_MASTER);
static pros::Motor armMotor(PORT_ARM), fjMotor(PORT_FLAPJACK);
static pros::adi::AnalogIn armPot(ADI_ARM_POT), fjPot(ADI_FJ_POT);

// taught values (start from the main code's placeholders)
static int armRawLevel = 2048, fjRawLevel = 2048, armSign = 1, fjSign = 1;
constexpr int NPRESET = 8;
static double preset[NPRESET] = {140.0, -50.0, 35.0, 15.0, -10.0, -40.0, -40.0, -46.0};
static const char* presetName[NPRESET] = {"STOW", "PLATE_CHECK", "DOOR_OPEN", "TOP3_BREAD", "MEAT_L2", "LEVEL1", "AUTO_CARRY", "AUTO_PLACE"};
static const char* presetCfg[NPRESET] = {"ARM_STOW_DEG", "ARM_PLATE_CHECK_DEG", "ARM_DOOR_OPEN_DEG", "ARM_TOP3_BREAD_DEG",
                                         "ARM_MEAT_L2_DEG", "ARM_LEVEL1_DEG", "ARM_AUTO_CARRY_DEG", "ARM_AUTO_PLACE_DEG"};
static int chosen = 1;
static bool stowTaught = false;

static void printConfig() {
  std::printf("\n// ---- paste into include/tracker/config.hpp (main robot) ----\n");
  std::printf("constexpr int  ARM_RAW_AT_LEVEL  = %d;\nconstexpr int  ARM_SIGN          = %d;\n", armRawLevel, armSign);
  std::printf("constexpr int  FJ_RAW_AT_LEVEL   = %d;\nconstexpr int  FJ_SIGN           = %d;\n", fjRawLevel, fjSign);
  for (int i = 0; i < NPRESET; ++i) std::printf("constexpr double %-20s = %.1f;\n", presetCfg[i], preset[i]);
  std::printf("\n");
}

void initialize() { pros::screen::print(pros::E_TEXT_MEDIUM, 0, "LINKS 701 potentiometer test"); }
void disabled() {}
void competition_initialize() {}
void autonomous() {}

void opcontrol() {
  arm::Joint armJ{ARM_KP, ARM_KD, ARM_KG, ARM_MAX_MV, ARM_MIN_DEG, ARM_MAX_DEG, 2.0};
  arm::Joint fjJ{FJ_KP, FJ_KD, FJ_KG, FJ_MAX_MV, -400, 400, 2.0};
  bool closedLoop = false;
  double fjGroundTarget = 0;
  const char* last = "manual";
  int loop = 0;
  const double dt = 0.01;
  while (true) {
    const int ra = armPot.get_value(), rf = fjPot.get_value();
    const bool okA = arm::potOk(ra), okF = arm::potOk(rf);
    const double armDeg = arm::potDeg(ra, armRawLevel, armSign);
    const double fjRel = arm::potDeg(rf, fjRawLevel, fjSign);
    const double fjGround = FJ_POT_RELATIVE_TO_ARM ? fjRel + armDeg : fjRel;
    const bool xHeld = master.get_digital(pros::E_CONTROLLER_DIGITAL_X);
    const bool yHeld = master.get_digital(pros::E_CONTROLLER_DIGITAL_Y);
    const bool dL = master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT);
    const bool dU = master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP);
    const bool dD = master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN);
    const bool dR = master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT);

    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
      closedLoop = false; armJ.reset(); fjJ.reset(); last = "STOP";
    }
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {      // LEVEL reference
      armRawLevel = ra; fjRawLevel = rf; last = "LEVEL saved"; printConfig();
    }
    if (yHeld && dU) {                                                     // FlapJack sign: fork tipped UP now
      fjSign = (rf - fjRawLevel) >= 0 ? 1 : -1; last = "FJ sign set"; printConfig();
    } else if (xHeld && (dL || dR)) {                                      // TEACH a preset
      int which = dL ? 0 : chosen;
      if (which == 0 && !stowTaught) {                                     // STOW = arm up -> must read positive
        armSign = (ra - armRawLevel) >= 0 ? 1 : -1; stowTaught = true;
      }
      preset[which] = arm::potDeg(ra, armRawLevel, armSign);
      last = presetName[which]; printConfig();
    } else if (!xHeld && (dU || dD)) {                                     // choose a preset
      chosen = (chosen + (dU ? 1 : NPRESET - 1)) % NPRESET; last = presetName[chosen];
    } else if (!xHeld && (dL || dR)) {                                     // GO TO a preset
      const int which = dL ? 0 : chosen;
      closedLoop = true; armJ.reset(); fjJ.reset();
      armJ.setTarget(preset[which]); fjGroundTarget = 0.0; last = presetName[which];
    }

    const int lIn = master.get_digital(pros::E_CONTROLLER_DIGITAL_L1) ? 1 : master.get_digital(pros::E_CONTROLLER_DIGITAL_L2) ? -1 : 0;
    const int rIn = master.get_digital(pros::E_CONTROLLER_DIGITAL_R1) ? 1 : master.get_digital(pros::E_CONTROLLER_DIGITAL_R2) ? -1 : 0;
    int armMv = lIn * MANUAL_ARM_MV, fjMv = rIn * MANUAL_FJ_MV;
    if (closedLoop && okA) {
      if (lIn) armJ.setTarget(armJ.target + lIn * RATE_DPS * dt);
      armMv = armJ.output(armDeg, dt, armDeg);
    }
    if (closedLoop && okF) {
      if (rIn) fjGroundTarget = std::fmin(FJ_MAX_DEG, std::fmax(FJ_MIN_DEG, fjGroundTarget + rIn * RATE_DPS * dt));
      fjJ.setTarget(FJ_POT_RELATIVE_TO_ARM ? fjGroundTarget - armDeg : fjGroundTarget);  // auto-level
      fjMv = fjJ.output(fjRel, dt, fjGround);
    }
    armMotor.move_voltage(armMv);
    fjMotor.move_voltage(fjMv);

    if (loop % 10 == 0) {
      auto endWarn = [](int r) { return (r < 150 || r > 3945) ? "END!" : "    "; };
      pros::screen::print(pros::E_TEXT_MEDIUM, 1, "mode %-7s last: %-12s", closedLoop ? "CLOSED" : "MANUAL", last);
      pros::screen::print(pros::E_TEXT_MEDIUM, 2, "ARM  raw %4d %s %s  angle %7.1f  target %7.1f %s  ", ra, okA ? "  " : "!!",
                          endWarn(ra), armDeg, armJ.target, armJ.faulted ? "RUN!" : "");
      pros::screen::print(pros::E_TEXT_MEDIUM, 3, "FJ   raw %4d %s %s  rel %6.1f  ground %6.1f -> %6.1f %s  ", rf, okF ? "  " : "!!",
                          endWarn(rf), fjRel, fjGround, fjGroundTarget, fjJ.faulted ? "RUN!" : "");
      pros::screen::print(pros::E_TEXT_MEDIUM, 4, "LEVEL arm %d fj %d  SIGN arm %+d fj %+d   ", armRawLevel, fjRawLevel, armSign, fjSign);
      pros::screen::print(pros::E_TEXT_MEDIUM, 5, "chosen %-11s %6.1f | STOW %.1f CHK %.1f DOOR %.1f T3 %.1f   ", presetName[chosen], preset[chosen], preset[0], preset[1], preset[2], preset[3]);
      pros::screen::print(pros::E_TEXT_MEDIUM, 7, "MEAT %.1f LVL1 %.1f AUTO_CARRY %.1f AUTO_PLACE %.1f   ", preset[4], preset[5], preset[6], preset[7]);
      pros::screen::print(pros::E_TEXT_MEDIUM, 6, "out arm %5d mV  fj %5d mV   ", armMv, fjMv);
      std::printf("POT,%lu,%d,%d,%.1f,%.1f,%.1f,%d,%d\n", (unsigned long)pros::millis(), ra, rf, armDeg, fjRel, fjGround, armMv, fjMv);
    }
    switch (loop % 15) {                                                   // controller: one line per 50 ms
      case 0:  master.print(0, 0, "ARM %6.1f %s   ", armDeg, armJ.faulted ? "RUN!" : (okA ? "" : "POT!")); break;
      case 5:  master.print(1, 0, "FJ  %6.1f %s   ", fjGround, fjJ.faulted ? "RUN!" : (okF ? "" : "POT!")); break;
      case 10: master.print(2, 0, "%c %-11s%6.1f", closedLoop ? 'C' : 'M', presetName[chosen], preset[chosen]); break;
    }
    ++loop;
    pros::delay(10);
  }
}
