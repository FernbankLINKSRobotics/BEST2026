// mount.hpp -- where the lens is, from the 3/4 in tee-plug camera mount (config.hpp "Camera mount").
// Pure C++ (no PROS). Used by the robot code, the camera test, the simulator and the laptop tests, so
// every program builds the SAME camera model from the same numbers.
#pragma once
#include <cmath>
#include "config.hpp"
#include "geometry.hpp"

namespace trk {

struct LensPos { double x, y, z; };

// side: +1 = LEFT front tee, -1 = RIGHT front tee. pitchDeg: + = down. yawDeg: + = turned left.
// The pitch pivot sits straight above the tee (yaw axis); the lens is LENS_FWD/LENS_UP from the pivot
// when the camera looks level, and swings around the pivot as the pitch changes.
inline LensPos lensFromMount(int side, double pitchDeg, double yawDeg) {
  const double p = deg2rad(pitchDeg), y = deg2rad(yawDeg);
  const double fwd = cfg::LENS_FWD_OF_PIVOT_IN * std::cos(p) + cfg::LENS_UP_OF_PIVOT_IN * std::sin(p);
  const double up = -cfg::LENS_FWD_OF_PIVOT_IN * std::sin(p) + cfg::LENS_UP_OF_PIVOT_IN * std::cos(p);
  const double pivZ = cfg::PLUG_RIM_Z_IN + cfg::PITCH_PIVOT_ABOVE_RIM_IN;
  return {cfg::TEE_X_IN + fwd * std::cos(y), side * cfg::TEE_Y_IN + fwd * std::sin(y), pivZ + up};
}

inline double camYawDeg(int side) { return side > 0 ? cfg::CAM_YAW_LEFT_DEG : cfg::CAM_YAW_RIGHT_DEG; }

// The camera model the robot uses, for a given corner (default: the one in config.hpp).
inline CameraModel cameraFromConfig(int side = cfg::CAM_SIDE, double pitchDeg = cfg::CAM_PITCH_DEG,
                                    double yawDeg = -999.0) {
  if (yawDeg < -900.0) yawDeg = camYawDeg(side);
  LensPos L = lensFromMount(side, pitchDeg, yawDeg);
  if (cfg::CAM_POS_MEASURED && side == cfg::CAM_SIDE) L = {cfg::CAM_X_MEAS_IN, cfg::CAM_Y_MEAS_IN, cfg::CAM_HEIGHT_MEAS_IN};
  CameraModel c{cfg::FX, cfg::FY, cfg::CX, cfg::CY, L.z, deg2rad(pitchDeg), L.x, L.y,
                deg2rad(yawDeg), deg2rad(cfg::CAM_ROLL_DEG)};
  return c;
}

}  // namespace trk
