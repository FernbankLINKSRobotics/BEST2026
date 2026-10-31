// host_sim.cpp -- run the REAL tracker code on a laptop against a simulated
// robot, camera and Dining Room. No robot or PROS needed.
//
// Build:  g++ -std=c++17 -O2 -I../include host_sim.cpp -o host_sim
// Run:    ./host_sim trace 7          -> trace.csv   (one run, every loop)
//         ./host_sim mc 300           -> mc.csv      (300 random runs, final errors)
//         ./host_sim mc 300 2.0       -> same, but camera pitch mis-calibrated by +2 deg
//         ./host_sim mc 300 0 0.03    -> motion model fitted to within +/-3% (after Exercise E5)
//         ./host_sim mc 300 0 0.15 22 30 1 -> camera at 22 in, 30 deg pitch, IR encoders ON
//         ./host_sim mc 300 0 0.15 18 50 0 1 -> table-edge whisker ON (no encoders)
//
// The simulator deliberately makes the "real" robot DIFFERENT from the model
// in config.hpp (speed gain, lag, deadband, left/right mismatch, sled rocking,
// pixel noise, dropouts, latency, occasional garbage detections). If the
// tracker survives this, it has a fighting chance on carpet.
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <random>
#include <vector>
#include "tracker/dining_auto.hpp"

using namespace trk;

struct Truth { double X, Y, th, vl, vr; };

struct SimParams {
  double kvScale, tau, deadband, rightScale;   // how the real drive differs
  double pitchBiasDeg;                          // calibration error we inject
};

static const double TABLE_X = 36.0, TABLE_Y = 36.0;      // Table:2 (nearest the +X,+Y doorway)


int main(int argc, char** argv) {
  const char* mode = argc > 1 ? argv[1] : "trace";
  const int n = argc > 2 ? std::atoi(argv[2]) : 1;
  const double pitchBias = argc > 3 ? std::atof(argv[3]) : 0.0;   // camera pitch calibration error [deg]
  const double kvSpread  = argc > 4 ? std::atof(argv[4]) : 0.15;  // motion-model speed error (0.15 = +/-15%)
  const double camH      = argc > 5 ? std::atof(argv[5]) : cfg::CAM_HEIGHT_IN;
  const double camPitch  = argc > 6 ? std::atof(argv[6]) : cfg::CAM_PITCH_DEG;
  const bool   useEnc    = argc > 7 ? std::atoi(argv[7]) != 0 : false;
  const bool   useWhisk  = argc > 8 ? std::atoi(argv[8]) != 0 : false;
  const bool trace = std::strcmp(mode, "trace") == 0;
  FILE* f = std::fopen(trace ? "trace.csv" : "mc.csv", "w");
  if (trace)
    std::fprintf(f, "t,state,trueX,trueY,estX,estY,sig,measX,measY,accepted,haveFrame,X,Y,th\n");
  else
    std::fprintf(f, "run,outcome,err_in,along_in,lateral_in,t_s,abort,whisker\n");

  const CameraModel camModel{cfg::FX, cfg::FY, cfg::CX, cfg::CY, camH,
                             deg2rad(camPitch), cfg::CAM_X_IN, cfg::CAM_Y_IN};
  int success = 0;
  const int runs = trace ? 1 : n;
  for (int run = 0; run < runs; ++run) {
    std::mt19937 rng(trace ? 1000 + n : 1000 + run);
    std::uniform_real_distribution<double> U(0, 1);
    std::normal_distribution<double> N(0, 1);
    SimParams sp{1.0 - kvSpread + 2 * kvSpread * U(rng),
                 cfg::TAU_S * (1.0 - kvSpread + 2 * kvSpread * U(rng)), 1000 + 400 * U(rng),
                 0.95 + 0.10 * U(rng), pitchBias};

    // Start: robot fully behind the +Y doorway tape (y = 66), centered in the
    // doorway (x = 54), facing -Y (into the room). Plate front edge ~ at the line.
    Truth tr{54.0 + 2.0 * N(rng), 66.0 + cfg::PLACE_X_IN + 5.5, deg2rad(-90 + 3.0 * N(rng)), 0, 0};

    DiningAuto autoRoutine(camModel);
    autoRoutine.start(-1);   // table is to the robot's right -> pivot right
    std::deque<std::pair<double, Truth>> hist;   // for sensor latency
    double t = 0, nextFrame = 0, pitchWobble = 0;
    // IR encoder simulation: real wheel travel, a per-run slip bias, edge quantization
    const double slip = 1.0 + 0.03 * N(rng);
    double distL = 0, distR = 0; long ticksL = 0, ticksR = 0;
    const double whiskLag = cfg::WHISKER_LAG_IN + 0.08 * N(rng);   // real click point varies a little
    bool whiskOn = false;
    bool placed = false; double placeErr = -1, along = 0, lateral = 0; int placeTimer = 0;
    const double dt = cfg::LOOP_MS / 1000.0;

    AutoOutput cmd;
    while (t < 20.0) {
      // ---- true robot dynamics (differs from the model!) ----
      auto wheel = [&](double mv, double vNow, double scale) {
        double mag = std::fabs(mv) - sp.deadband;
        double target = mag > 0 ? std::copysign(cfg::KV_INPS_PER_MV * sp.kvScale * scale * mag, mv) : 0;
        return vNow + (target - vNow) * dt / (sp.tau + dt);
      };
      tr.vl = wheel(cmd.leftMv, tr.vl, 1.0);
      tr.vr = wheel(cmd.rightMv, tr.vr, sp.rightScale);
      distL += tr.vl * dt * slip; distR += tr.vr * dt * slip;
      if (useEnc) {
        long nl = (long)std::floor(distL / cfg::ENC_IN_PER_EDGE), nr = (long)std::floor(distR / cfg::ENC_IN_PER_EDGE);
        autoRoutine.feedOdometry((nl - ticksL) * cfg::ENC_IN_PER_EDGE, (nr - ticksR) * cfg::ENC_IN_PER_EDGE);
        ticksL = nl; ticksR = nr;
      }
      if (useWhisk) {   // true table front edge along our centerline vs. the whisker
        const double c0 = std::cos(tr.th), s0 = std::sin(tr.th);
        const double rx =  c0 * (TABLE_X - tr.X) + s0 * (TABLE_Y - tr.Y);
        const double ry = -s0 * (TABLE_X - tr.X) + c0 * (TABLE_Y - tr.Y);
        double xe;
        if (tableEdgeOnCenterline(rx, ry, -tr.th, cfg::TABLE_HALF_IN, xe) && xe <= cfg::WHISKER_X_IN - whiskLag) whiskOn = true;
        autoRoutine.feedSwitches(whiskOn, false);
      }
      double v = 0.5 * (tr.vl + tr.vr) * (1 + 0.03 * N(rng));
      double w = (tr.vr - tr.vl) / cfg::TRACK_WIDTH_IN + 0.01 * N(rng);
      tr.X += v * std::cos(tr.th) * dt;
      tr.Y += v * std::sin(tr.th) * dt;
      tr.th += w * dt;
      pitchWobble = 0.95 * pitchWobble + 0.05 * deg2rad(0.6) * 4.4 * N(rng);  // ~0.6 deg rms rocking
      hist.push_back({t, tr});
      while (hist.size() > 200) hist.pop_front();

      // ---- simulated AI Vision Sensor: 30 fps, 50 ms latency ----
      std::vector<RawTag> frame; bool haveFrame = false;
      if (t >= nextFrame) {
        nextFrame += 1.0 / 30.0;
        haveFrame = true;
        Truth past = hist.front().second;
        for (auto& h : hist) if (h.first <= t - cfg::VISION_LATENCY_MS / 1000.0) past = h.second;
        CameraModel camTrue = camModel;
        camTrue.pitch += deg2rad(sp.pitchBiasDeg) + pitchWobble;
        // table tag corners -> robot frame (of 'past') -> pixels
        const double c = std::cos(past.th), s = std::sin(past.th), hq = 0.5 * cfg::TAG_QUAD_IN;
        RawTag tag; tag.id = 7; bool visible = true;
        const double cu[4] = {-hq, hq, hq, -hq}, cv[4] = {-hq, -hq, hq, hq};
        for (int i = 0; i < 4 && visible; ++i) {
          double fx = TABLE_X + cu[i], fy = TABLE_Y + cv[i];
          double rx =  c * (fx - past.X) + s * (fy - past.Y);
          double ry = -s * (fx - past.X) + c * (fy - past.Y);
          // carried plate blocks the view?
          double tt = (camTrue.height - cfg::PLATE_CARRY_Z_IN) / (camTrue.height - cfg::TABLE_TAG_Z_IN);
          double px = camTrue.xoff + tt * (rx - camTrue.xoff), py = camTrue.yoff + tt * (ry - camTrue.yoff);
          if (std::hypot(px - cfg::PLACE_X_IN, py - cfg::PLACE_Y_IN) < 0.5 * cfg::PLATE_DIAMETER_IN) visible = false;
          Pixel p;
          if (!pointToPixel(camTrue, rx, ry, cfg::TABLE_TAG_Z_IN, p) || !inImage(p, 320, 240, 2)) visible = false;
          tag.u[i] = std::round(p.u + cfg::SIG_PX * N(rng));
          tag.v[i] = std::round(p.v + cfg::SIG_PX * N(rng));
        }
        if (visible) {
          double vmin = 1e9, vmax = -1e9;
          for (int i = 0; i < 4; ++i) { vmin = std::fmin(vmin, tag.v[i]); vmax = std::fmax(vmax, tag.v[i]); }
          bool decodable = (vmax - vmin) >= 10.0;          // ~2 px per module, 5 modules tall
          bool dropout = U(rng) < 0.15 || (std::fabs(w) > 1.0 && U(rng) < 0.5);
          if (U(rng) < 0.02) for (int i = 0; i < 4; ++i) { tag.u[i] += 18; tag.v[i] -= 6; }  // garbage
          if (decodable && !dropout) frame.push_back(tag);
        }
      }

      cmd = autoRoutine.step(dt, haveFrame ? &frame : nullptr, placeTimer > 30);
      if (cmd.requestPlace) {
        if (!placed && placeTimer == 30) {   // measure after the robot has fully stopped  // plate center lands where the robot says PLACE_X is
          double pxF = tr.X + cfg::PLACE_X_IN * std::cos(tr.th);
          double pyF = tr.Y + cfg::PLACE_X_IN * std::sin(tr.th);
          placeErr = std::hypot(pxF - TABLE_X, pyF - TABLE_Y);
          double ex = TABLE_X - pxF, ey = TABLE_Y - pyF;
          along = ex * std::cos(tr.th) + ey * std::sin(tr.th);
          lateral = -ex * std::sin(tr.th) + ey * std::cos(tr.th);
          placed = true;
        }
        ++placeTimer;
      }
      if (trace) {
        const double c = std::cos(tr.th), s = std::sin(tr.th);
        double trX =  c * (TABLE_X - tr.X) + s * (TABLE_Y - tr.Y);
        double trY = -s * (TABLE_X - tr.X) + c * (TABLE_Y - tr.Y);
        const auto& tel = autoRoutine.telemetry(); const auto& e = autoRoutine.ekf();
        std::fprintf(f, "%.3f,%s,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d,%d,%.3f,%.3f,%.4f\n", t,
                     stateName(autoRoutine.state()), trX, trY, e.valid ? e.x : NAN, e.valid ? e.y : NAN,
                     e.valid ? e.sigmaMax() : NAN, tel.haveMeas ? tel.meas.x : NAN,
                     tel.haveMeas ? tel.meas.y : NAN, (int)tel.accepted, (int)haveFrame, tr.X, tr.Y, tr.th);
      }
      t += dt;
      if (!autoRoutine.running()) break;
    }
    const auto st = autoRoutine.state();
    const bool ok = placed && placeErr <= 0.5 * cfg::PLATE_DIAMETER_IN - 0.5 * std::sqrt(2.0) * cfg::TAG_QUAD_IN;
    if (ok) ++success;
    if (!trace)
      std::fprintf(f, "%d,%s,%.3f,%.3f,%.3f,%.2f,%s,%d\n", run, placed ? (ok ? "scored" : "missed") : "aborted",
                   placeErr, along, lateral, t, st == AutoState::ABORT ? autoRoutine.telemetry().abortReason : "",
                   (int)autoRoutine.telemetry().whiskerUsed);
  }
  std::fclose(f);
  std::printf("%s: %d/%d runs scored (plate fully covers the 4.25 in square)\n", mode, success, runs);
  return 0;
}
