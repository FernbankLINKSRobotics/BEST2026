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

// Camera axes, expressed in the ROBOT frame (X fwd, Y left, Z up):
//   forward f = ( cos p, 0, -sin p)   (p = pitch, + = tilted down)
//   right   r = ( 0,    -1,  0    )
//   down    d = (-sin p, 0, -cos p)
// Image coordinates: u grows to the right, v grows downward.
struct CameraModel {
  double fx, fy, cx, cy;   // intrinsics [px]
  double height;           // lens height above floor [in]
  double pitch;            // [rad], + = down
  double xoff, yoff;       // lens position in robot frame [in]
};

// Pixel -> point where its viewing ray hits the horizontal plane Z = zPlane.
// Returns false if the ray points at or above the horizon (never hits).
inline bool pixelToPlaneZ(const CameraModel& c, Pixel p, double zPlane, Vec2& out) {
  const double a = (p.u - c.cx) / c.fx;          // how far right of center
  const double b = (p.v - c.cy) / c.fy;          // how far below center
  const double s = std::sin(c.pitch), k = std::cos(c.pitch);
  const double Rx = k - b * s;                   // ray = f + a*r + b*d
  const double Ry = -a;
  const double Rz = -s - b * k;
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
  const double s = std::sin(c.pitch), k = std::cos(c.pitch);
  const double zc = dx * k - dz * s;             // along forward axis
  const double xc = -dy;                         // along right axis
  const double yc = -dx * s - dz * k;            // along down axis
  if (zc <= 1e-6) return false;
  out.u = c.cx + c.fx * xc / zc;
  out.v = c.cy + c.fy * yc / zc;
  return true;
}

inline bool inImage(Pixel p, double w, double h, double margin = 2.0) {
  return p.u >= margin && p.u <= w - margin && p.v >= margin && p.v <= h - margin;
}

} // namespace trk
