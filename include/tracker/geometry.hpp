// geometry.hpp -- pinhole camera model + ray/plane intersection.
// Pure C++ (no PROS), so it can be unit-tested on a laptop.
#pragma once
#include <cmath>

namespace trk {

constexpr double PI = 3.14159265358979323846;
inline double deg2rad(double d) { return d * PI / 180.0; }
inline double rad2deg(double r) { return r * 180.0 / PI; }
inline double wrapPi(double a) {
  while (a > PI) a -= 2 * PI;
  while (a < -PI) a += 2 * PI;
  return a;
}

struct Vec2  { double x = 0, y = 0; };
struct Pixel { double u = 0, v = 0; };

// Camera orientation in the ROBOT frame (X fwd, Y left, Z up), applied as
//   yaw   (+ = turned LEFT),  pitch (+ = tilted DOWN),  roll (about the lens axis;
//   +/-90 deg = sensor mounted sideways / "portrait"; the sign depends on cable side).
// With yaw = roll = 0:  forward f = (cos p, 0, -sin p), right r = (0, -1, 0),
//                        down d = (-sin p, 0, -cos p).
// Image coordinates are the SENSOR's own: u to the sensor's right, v to its bottom
// (320 x 240 even when the sensor is mounted sideways).
struct CameraModel {
  double fx, fy, cx, cy;   // intrinsics [px]
  double height;           // lens height above floor [in]
  double pitch;            // [rad], + = down
  double xoff, yoff;       // lens position in robot frame [in]
  double yaw = 0;          // [rad], + = turned left
  double roll = 0;         // [rad]
  // Optional: camera axes given directly (set by the reference-tag correction). When useAxes is
  // true, ax[0..2] = forward, ax[3..5] = sensor-right, ax[6..8] = sensor-down (robot frame).
  bool useAxes = false;
  double ax[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
};

struct CamAxes { double f[3], r[3], d[3]; };
inline CamAxes camAxes(const CameraModel& c) {
  if (c.useAxes) {
    CamAxes A;
    for (int i = 0; i < 3; ++i) { A.f[i] = c.ax[i]; A.r[i] = c.ax[3 + i]; A.d[i] = c.ax[6 + i]; }
    return A;
  }
  const double cy = std::cos(c.yaw), sy = std::sin(c.yaw), cp = std::cos(c.pitch), sp = std::sin(c.pitch);
  const double f[3] = {cp * cy, cp * sy, -sp};
  const double r0[3] = {sy, -cy, 0.0};
  const double d0[3] = {f[1] * r0[2] - f[2] * r0[1], f[2] * r0[0] - f[0] * r0[2], f[0] * r0[1] - f[1] * r0[0]};
  const double cr = std::cos(c.roll), sr = std::sin(c.roll);
  CamAxes A;
  for (int i = 0; i < 3; ++i) {
    A.f[i] = f[i];
    A.r[i] = cr * r0[i] + sr * d0[i];
    A.d[i] = -sr * r0[i] + cr * d0[i];
  }
  return A;
}

// Pixel -> point where its viewing ray hits the horizontal plane Z = zPlane.
// Returns false if the ray points at or above the horizon (never hits).
inline bool pixelToPlaneZ(const CameraModel& c, Pixel p, double zPlane, Vec2& out) {
  const double a = (p.u - c.cx) / c.fx;          // how far right of center (sensor axes)
  const double b = (p.v - c.cy) / c.fy;          // how far below center
  const CamAxes A = camAxes(c);
  const double Rx = A.f[0] + a * A.r[0] + b * A.d[0];   // ray = f + a*r + b*d
  const double Ry = A.f[1] + a * A.r[1] + b * A.d[1];
  const double Rz = A.f[2] + a * A.r[2] + b * A.d[2];
  if (Rz >= -1e-6) return false;                 // looking level or up: no hit
  const double t = (zPlane - c.height) / Rz;     // > 0 when plane is below lens
  out.x = c.xoff + t * Rx;
  out.y = c.yoff + t * Ry;
  return true;
}

// Robot-frame 3D point -> pixel (used by the simulator and by the
// "will the tag still be visible?" check). False if behind the lens.
inline bool pointToPixel(const CameraModel& c, double X, double Y, double Z, Pixel& out) {
  const double dx = X - c.xoff, dy = Y - c.yoff, dz = Z - c.height;
  const CamAxes A = camAxes(c);
  const double zc = dx * A.f[0] + dy * A.f[1] + dz * A.f[2];   // along forward axis
  const double xc = dx * A.r[0] + dy * A.r[1] + dz * A.r[2];   // along sensor-right axis
  const double yc = dx * A.d[0] + dy * A.d[1] + dz * A.d[2];   // along sensor-down axis
  if (zc <= 1e-6) return false;
  out.u = c.cx + c.fx * xc / zc;
  out.v = c.cy + c.fy * yc / zc;
  return true;
}

inline bool inImage(Pixel p, double w, double h, double margin = 2.0) {
  return p.u >= margin && p.u <= w - margin && p.v >= margin && p.v <= h - margin;
}

} // namespace trk
