// dining_auto.hpp -- the autonomous Dining Room routine as a state machine.
// Pure C++ (no PROS): main.cpp feeds it camera frames and applies its motor
// commands; test/host_sim.cpp feeds it SIMULATED frames. Same code, both places.
//
//   IDLE -> ENTER -> SEARCH -> APPROACH -> COMMIT -> PLACE -> BACKOUT -> DONE
//                       \__________\___________\______________> ABORT
#pragma once
#include <cmath>
#include <vector>
#include "config.hpp"
#include "geometry.hpp"
#include "tag_measure.hpp"
#include "target_ekf.hpp"
#include "motion_model.hpp"
#include "field_map.hpp"

namespace trk {

enum class AutoState { IDLE, ENTER, SEARCH, APPROACH, COMMIT, PLACE, BACKOUT, DONE, ABORT };

inline const char* stateName(AutoState s) {
  switch (s) {
    case AutoState::IDLE: return "IDLE";       case AutoState::ENTER: return "ENTER";
    case AutoState::SEARCH: return "SEARCH";   case AutoState::APPROACH: return "APPROACH";
    case AutoState::COMMIT: return "COMMIT";   case AutoState::PLACE: return "PLACE";
    case AutoState::BACKOUT: return "BACKOUT"; case AutoState::DONE: return "DONE";
    default: return "ABORT";
  }
}

struct AutoOutput {
  double leftMv = 0, rightMv = 0;
  bool requestPlace = false;      // true -> run the arm/fork release hook
};

// Everything worth logging, refreshed every step.
struct AutoTelemetry {
  AutoState state = AutoState::IDLE;
  bool haveMeas = false, accepted = false;
  Vec2 meas;                     // last table measurement (robot frame)
  double measAlong = 0, measCross = 0;
  double ex = 0, ey = 0;         // plate-to-table error (robot frame)
  double sigma = 0;              // EKF largest 1-sigma [in]
  double v = 0, w = 0;           // motion-model speed estimate
  const char* abortReason = "";
  bool   whiskerUsed = false;  // the edge whisker fix was accepted this run
  double whiskerInnov = 0;     // how far off the estimate was when it clicked [in]
  double tableYawDeg = 0;      // table orientation in robot frame, mod 90 [deg]
};

// Along the robot's centerline (Y = 0), where does a square table outline begin?
// (cx, cy) = table center in robot frame, psi = table edge direction (any multiple
// of 90 deg gives the same square), half = half-width. False if the centerline misses.
inline bool tableEdgeOnCenterline(double cx, double cy, double psi, double half, double& xEdge) {
  const double c = std::cos(psi), s = std::sin(psi);
  double lo = -1e9, hi = 1e9;
  auto slab = [&](double A, double B) {         // |A*t + B| <= half
    if (std::fabs(A) < 1e-9) return std::fabs(B) <= half;
    double t1 = (-half - B) / A, t2 = (half - B) / A;
    if (t1 > t2) { const double tmp = t1; t1 = t2; t2 = tmp; }
    lo = std::fmax(lo, t1); hi = std::fmin(hi, t2);
    return true;
  };
  if (!slab(c, -cx * c - cy * s)) return false;
  if (!slab(-s, cx * s - cy * c)) return false;
  if (lo > hi) return false;
  xEdge = lo;
  return true;
}

class DiningAuto {
 public:
  explicit DiningAuto(const CameraModel& cam)
      : cam_(cam), drive_(cfg::KV_INPS_PER_MV, cfg::TAU_S, cfg::DEADBAND_MV, cfg::TRACK_WIDTH_IN) {}

  // searchDir: +1 = pivot LEFT while searching, -1 = pivot RIGHT.
  void start(int searchDir) {
    ekf_.reset(); drive_.reset();
    searchDir_ = (searchDir >= 0) ? 1 : -1;
    tState_ = 0; tTotal_ = 0; travel_ = 0; lastL_ = lastR_ = 0; rejects_ = 0; reinits_ = 0; odoV_ = odoW_ = 0; haveOdo_ = false;
    yaw4_ = 0; haveYaw_ = false; whiskerUsed_ = false; whiskerRise_ = bumperRise_ = false; creep_ = 0;
    tel_ = AutoTelemetry{};
    go(AutoState::ENTER);
  }
  void abort(const char* why) { tel_.abortReason = why; go(AutoState::ABORT); }
  bool running() const {
    return state_ != AutoState::IDLE && state_ != AutoState::DONE && state_ != AutoState::ABORT;
  }
  AutoState state() const { return state_; }
  const AutoTelemetry& telemetry() const { return tel_; }
  const TargetEKF& ekf() const { return ekf_; }
  int reinits() const { return reinits_; }

  // OPTIONAL: wheel travel since the last step() from IR encoders [in].
  // Call before step(). If never called, the motion model is used instead.
  void feedOdometry(double dLeftIn, double dRightIn) {
    odoL_ = dLeftIn; odoR_ = dRightIn; haveOdo_ = true;
  }

  // OPTIONAL: contact switches, current state (true = pressed). Call before step().
  void feedSwitches(bool whisker, bool bumper) {
    whiskerEnabled_ = true;
    whiskerRise_ = whisker && !whiskerPrev_; whiskerPrev_ = whisker;
    bumperRise_  = bumper && !bumperPrev_;   bumperPrev_  = bumper;
  }

  // Call every loop. 'frame' = tags read THIS loop (nullptr if camera not polled).
  // 'placeDone' = the release hook has finished.
  AutoOutput step(double dt, const std::vector<RawTag>* frame, bool placeDone) {
    AutoOutput out;
    tel_.haveMeas = tel_.accepted = false;
    if (!running() && state_ != AutoState::PLACE) { lastL_ = lastR_ = 0; return out; }
    tState_ += dt; tTotal_ += dt;

    // 1) PREDICT: the robot moved since last loop, so the table moved in our frame.
    drive_.step(lastL_, lastR_, dt);
    double v = drive_.v(), w = drive_.w();
    double sv = std::fmax(cfg::SIG_V_MIN, cfg::SIG_V_FRAC * std::fabs(v));
    double sw = std::fmax(cfg::SIG_W_MIN, cfg::SIG_W_FRAC * std::fabs(w));
    if (haveOdo_) {   // measured wheel travel beats a guessed one
      // Ticks arrive in whole edges, so per-loop v and w are jumpy -- but the
      // rounding error never accumulates (it is at most one edge in total), so
      // only wheel slip goes into the process noise. Scale it by the smooth
      // model speed, not by the jumpy measured one.
      // Smooth over ~40 ms so one-edge jumps don't jerk the steering.
      // (An exponential average keeps the total distance, it just spreads it out.)
      const double a = dt / (0.04 + dt);
      odoV_ += a * (0.5 * (odoL_ + odoR_) / dt - odoV_);
      odoW_ += a * ((odoR_ - odoL_) / (cfg::TRACK_WIDTH_IN * dt) - odoW_);
      v = odoV_;
      w = odoW_;
      sv = std::fmax(cfg::SIG_ENC_V_MIN, cfg::SIG_ENC_FRAC * std::fabs(drive_.v()));
      sw = std::fmax(cfg::SIG_ENC_W_MIN, cfg::SIG_ENC_FRAC * std::fabs(drive_.w()));
      haveOdo_ = false;
    }
    ekf_.predict(v, w, dt, sv, sw);
    yaw4_ = wrapPi(yaw4_ - 4.0 * w * dt);   // the table turns the other way in our frame
    travel_ += std::fabs(v) * dt;

    // 2) UPDATE: fold in any table tags we can see (not while blind/placing).
    if (frame && (state_ == AutoState::ENTER || state_ == AutoState::SEARCH ||
                  state_ == AutoState::APPROACH)) {
      handleFrame(*frame, v, w);
    }

    // 2b) CONTACT SWITCHES
    if (bumperRise_ && (state_ == AutoState::ENTER || state_ == AutoState::SEARCH ||
                        state_ == AutoState::APPROACH || state_ == AutoState::COMMIT))
      abort("front bumper hit");
    if (whiskerRise_ && !whiskerUsed_ && ekf_.valid && haveYaw_ &&
        (state_ == AutoState::APPROACH || state_ == AutoState::COMMIT)) {
      handleWhisker();
    }
    whiskerRise_ = bumperRise_ = false;

    // 3) DECIDE + ACT
    if (tTotal_ > cfg::AUTO_TIMEOUT_S) abort("auto timeout");
    double vCmd = 0, wCmd = 0;
    const double ex = ekf_.x - cfg::PLACE_X_IN, ey = ekf_.y - cfg::PLACE_Y_IN;
    tel_.ex = ex; tel_.ey = ey; tel_.sigma = ekf_.valid ? ekf_.sigmaMax() : 0;
    tel_.v = v; tel_.w = w;

    switch (state_) {
      case AutoState::ENTER:
        vCmd = cfg::ENTER_V_INPS;
        if (ekf_.valid) go(AutoState::APPROACH);
        else if (travel_ >= cfg::ENTER_DIST_IN) go(AutoState::SEARCH);
        break;

      case AutoState::SEARCH:
        wCmd = searchDir_ * cfg::SEARCH_W_RADPS;
        if (ekf_.valid) go(AutoState::APPROACH);
        else if (tState_ > cfg::SEARCH_TIMEOUT_S) abort("no table tag found");
        break;

      case AutoState::APPROACH: {
        if (ekf_.sinceUpdate > cfg::LOST_TIMEOUT_S) { abort("track lost"); break; }
        if (ekf_.sigmaMax() > cfg::MAX_SIGMA_IN)    { abort("estimate too uncertain"); break; }
        // Point the robot's centerline (which carries the plate) at the table.
        const double bearing = std::atan2(ey, ekf_.x);
        wCmd = clampAbs(cfg::KP_TURN * bearing, cfg::W_MAX_RADPS);
        if (std::fabs(bearing) < deg2rad(cfg::TURN_IN_PLACE_DEG)) {
          vCmd = clamp(cfg::KP_DRIVE * ex, cfg::V_MIN_INPS, cfg::V_MAX_INPS) * std::cos(bearing);
        }
        const bool aligned = std::fabs(ey) < cfg::COMMIT_MAX_EY_IN;
        if (ex < cfg::COMMIT_EX_IN && aligned) go(AutoState::COMMIT);
        else if (!tagWillStayVisible() && aligned) go(AutoState::COMMIT);
        else if (!tagWillStayVisible() && ex < cfg::COMMIT_EX_IN) abort("lost view while misaligned");
        break;
      }

      case AutoState::COMMIT:
        // Blind: the plate is about to cover the tag. Drive straight and let the
        // EKF "coast" on the motion model until it says the plate is on target.
        vCmd = cfg::V_MIN_INPS + 2.0;
        // Keep steering at where the coasting estimate says the table is.
        wCmd = clampAbs(cfg::KP_TURN * std::atan2(ey, std::fmax(ekf_.x, 1.0)), cfg::W_MAX_RADPS);
        // Stop commanding early: the sled keeps rolling about v*tau after we cut power.
        if (ex <= drive_.v() * cfg::TAU_S) {
          if (whiskerEnabled_ && !whiskerUsed_) {
            // We think we've arrived, but the whisker never felt the table edge, and it
            // is mounted so that it ALWAYS touches first. So we're short: creep on.
            vCmd = cfg::V_MIN_INPS;
            creep_ += std::fabs(v) * dt;
            if (creep_ > cfg::WHISKER_MAX_CREEP_IN) abort("whisker never touched");
          } else {
            go(AutoState::PLACE);
          }
        }
        break;

      case AutoState::PLACE:
        out.requestPlace = (std::fabs(drive_.v()) < 0.5);   // wait until we have stopped rolling
        if (placeDone) { travel_ = 0; go(AutoState::BACKOUT); }
        break;

      case AutoState::BACKOUT:
        vCmd = -cfg::ENTER_V_INPS;
        if (travel_ >= cfg::BACKOUT_DIST_IN) go(AutoState::DONE);
        break;

      default: break;
    }

    if (!running() && state_ != AutoState::PLACE) { vCmd = wCmd = 0; }
    double l = 0, r = 0;
    if (vCmd != 0 || wCmd != 0) drive_.commandFor(vCmd, wCmd, l, r);
    l = clampAbs(l, cfg::MC55_MAX_MV); r = clampAbs(r, cfg::MC55_MAX_MV);
    lastL_ = l; lastR_ = r;
    out.leftMv = l; out.rightMv = r;
    tel_.state = state_;
    return out;
  }

 private:
  void go(AutoState s) { state_ = s; tState_ = 0; }

  void handleFrame(const std::vector<RawTag>& frame, double v, double w) {
    const double lat = cfg::VISION_LATENCY_MS / 1000.0;
    const double pitchSig = deg2rad(cfg::SIG_PITCH_DEG);
    bool haveBest = false; TableTagMeas best; MeasNoise bestN{0, 0};
    for (const auto& t : frame) {
      if (!fieldmap::isTableId(t.id)) continue;
      if (ekf_.valid && t.id != ekf_.id) continue;           // locked: ignore other tables
      TableTagMeas m = measureTableTag(cam_, t, cfg::TABLE_TAG_Z_IN, cfg::TAG_QUAD_IN,
                                       cfg::SIDE_TOL_FRAC);
      if (!m.ok) continue;
      m.center = moveIntoCurrentFrame(m.center, v, w, lat);  // undo sensor latency
      MeasNoise n = tableMeasNoise(cam_, m.horizDist, cfg::TABLE_TAG_Z_IN, cfg::SIG_PX,
                                   pitchSig, cfg::SIG_FLOOR_IN);
      if (!haveBest || m.horizDist < best.horizDist) { best = m; bestN = n; haveBest = true; }
    }
    if (!haveBest) return;
    // Table orientation, mod 90 deg: average 4*yaw on the unit circle so that
    // 0, 90, 180, 270 deg all agree -- this works whatever the corner order is.
    {
      const double m4 = 4.0 * best.yaw, a = haveYaw_ ? 0.3 : 1.0;
      yaw4_ = std::atan2((1 - a) * std::sin(yaw4_) + a * std::sin(m4),
                         (1 - a) * std::cos(yaw4_) + a * std::cos(m4));
      haveYaw_ = true;
      tel_.tableYawDeg = rad2deg(yaw4_ / 4.0);
    }
    tel_.haveMeas = true; tel_.meas = best.center;
    tel_.measAlong = bestN.along; tel_.measCross = bestN.cross;
    if (!ekf_.valid) {                                       // first sighting: lock on
      ekf_.init(best.id, best.center, bestN.along, bestN.cross);
      tel_.accepted = true;
    } else {
      tel_.accepted = ekf_.update(best.center, bestN.along, bestN.cross, cfg::GATE_CHI2);
      // Several good-looking measurements in a row all disagree with us?
      // Then WE are probably wrong (filter diverged), not the camera. Restart.
      rejects_ = tel_.accepted ? 0 : rejects_ + 1;
      if (rejects_ >= cfg::REINIT_AFTER_REJECTS) {
        ekf_.init(best.id, best.center, bestN.along, bestN.cross);
        rejects_ = 0; ++reinits_;
      }
    }
  }

  // The whisker just clicked: the table's front edge is at WHISKER_X - LAG along
  // our centerline. Compare with where our estimate puts that edge, and correct.
  void handleWhisker() {
    const double psi = yaw4_ / 4.0, half = cfg::TABLE_HALF_IN, eps = 0.05;
    double h, hy;
    if (!tableEdgeOnCenterline(ekf_.x, ekf_.y, psi, half, h)) return;          // we think we miss
    if (!tableEdgeOnCenterline(ekf_.x, ekf_.y + eps, psi, half, hy)) hy = h;
    const double H1 = (hy - h) / eps;                                          // dh/dy (numeric)
    const double z = cfg::WHISKER_X_IN - cfg::WHISKER_LAG_IN;
    tel_.whiskerInnov = z - h;
    // A physical touch is more trustworthy than a drifted estimate (e.g. a bumped
    // camera), so accept anything inside a plausible window instead of a sigma gate.
    if (std::fabs(z - h) > cfg::WHISKER_WINDOW_IN) return;
    if (ekf_.updateScalar(z, h, 1.0, H1, cfg::SIG_WHISKER_IN, 1e9)) {
      whiskerUsed_ = true; tel_.whiskerUsed = true;
    }
  }

  // Will the table tag still be fully visible a little closer in?
  // Checks (a) all 4 corners stay inside the image and (b) the carried plate
  // does not block the line of sight to any corner.
  bool tagWillStayVisible() const {
    const double look = 2.0;  // inches of travel we look ahead
    const double h = 0.5 * cfg::TAG_QUAD_IN;
    const double plateR = 0.5 * cfg::PLATE_DIAMETER_IN;
    const double tPlate = (cam_.height - cfg::PLATE_CARRY_Z_IN) / (cam_.height - cfg::TABLE_TAG_Z_IN);
    for (int i = 0; i < 4; ++i) {
      const double cx = ekf_.x - look + ((i & 1) ? h : -h);
      const double cy = ekf_.y + ((i & 2) ? h : -h);
      Pixel p;
      if (!pointToPixel(cam_, cx, cy, cfg::TABLE_TAG_Z_IN, p)) return false;
      if (!inImage(p, cfg::IMG_W, cfg::IMG_H, 4.0)) return false;
      // where the sight line crosses the plate's height
      const double px = cam_.xoff + tPlate * (cx - cam_.xoff);
      const double py = cam_.yoff + tPlate * (cy - cam_.yoff);
      if (std::hypot(px - cfg::PLACE_X_IN, py - cfg::PLACE_Y_IN) < plateR + 0.25) return false;
    }
    return true;
  }

  static double clamp(double x, double lo, double hi) { return x < lo ? lo : (x > hi ? hi : x); }
  static double clampAbs(double x, double m) { return clamp(x, -m, m); }

  CameraModel cam_;
  DriveModel drive_;
  TargetEKF ekf_;
  AutoState state_ = AutoState::IDLE;
  AutoTelemetry tel_;
  int searchDir_ = 1;
  int rejects_ = 0;
  double yaw4_ = 0; bool haveYaw_ = false;
  bool whiskerPrev_ = false, whiskerRise_ = false, whiskerUsed_ = false;
  bool bumperPrev_ = false, bumperRise_ = false;
  bool whiskerEnabled_ = false;
  double creep_ = 0;
  bool haveOdo_ = false;
  double odoL_ = 0, odoR_ = 0, odoV_ = 0, odoW_ = 0;
  int reinits_ = 0;
  double tState_ = 0, tTotal_ = 0, travel_ = 0, lastL_ = 0, lastR_ = 0;
};

} // namespace trk
