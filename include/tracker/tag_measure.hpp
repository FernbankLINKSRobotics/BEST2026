// tag_measure.hpp -- turn 4 tag corners (pixels) into a robot-frame
// measurement of the table center, plus an honest noise estimate.
#pragma once
#include <cmath>
#include <cstdint>
#include "geometry.hpp"

namespace trk {

// One AprilTag detection, exactly as the sensor reports it.
struct RawTag {
  int id = -1;
  double u[4] = {0, 0, 0, 0};   // corner x pixels (x0..x3)
  double v[4] = {0, 0, 0, 0};   // corner y pixels (y0..y3)
  std::uint32_t t_ms = 0;       // brain time when we READ it
};

struct TableTagMeas {
  bool   ok = false;      // passed all sanity checks
  int    id = -1;
  Vec2   center;          // table-tag center in robot frame [in]
  double yaw = 0;         // tag edge direction in robot frame [rad] (verify corner order!)
  double side[4] = {0, 0, 0, 0}; // projected side lengths [in]
  double pixW = 0, pixH = 0;     // bounding size in the image [px]
  double horizDist = 0;          // lens-to-tag distance along the floor [in]
  const char* reject = "";       // why ok == false (for logging)
};

// Ray-plane back-projection of all four corners onto the table plane.
// A REAL table tag projects to a 4.25" square. Anything else (wall tag,
// misdetection, wrong pitch) gives the wrong side lengths -> rejected.
inline TableTagMeas measureTableTag(const CameraModel& cam, const RawTag& t,
                                    double zPlane, double quadIn, double sideTolFrac) {
  TableTagMeas m;
  m.id = t.id;
  Vec2 p[4];
  double umin = 1e9, umax = -1e9, vmin = 1e9, vmax = -1e9;
  for (int i = 0; i < 4; ++i) {
    if (!pixelToPlaneZ(cam, Pixel{t.u[i], t.v[i]}, zPlane, p[i])) { m.reject = "above horizon"; return m; }
    umin = std::fmin(umin, t.u[i]); umax = std::fmax(umax, t.u[i]);
    vmin = std::fmin(vmin, t.v[i]); vmax = std::fmax(vmax, t.v[i]);
  }
  m.pixW = umax - umin;
  m.pixH = vmax - vmin;
  for (int i = 0; i < 4; ++i) {
    const Vec2& a = p[i];
    const Vec2& b = p[(i + 1) % 4];
    m.side[i] = std::hypot(b.x - a.x, b.y - a.y);
    if (std::fabs(m.side[i] - quadIn) > sideTolFrac * quadIn) { m.reject = "side length"; return m; }
  }
  m.center.x = 0.25 * (p[0].x + p[1].x + p[2].x + p[3].x);
  m.center.y = 0.25 * (p[0].y + p[1].y + p[2].y + p[3].y);
  m.yaw = std::atan2(p[1].y - p[0].y, p[1].x - p[0].x);
  m.horizDist = std::hypot(m.center.x - cam.xoff, m.center.y - cam.yoff);
  m.ok = true;
  return m;
}

// 1-sigma noise of a table-center measurement, split into
//   along  = along the line of sight (range)   -- the BIG one
//   cross  = sideways to the line of sight      -- small
// Physics: a small angle error da at depression angle a moves the floor
// hit point by  dd = da * (d^2 + h^2) / h   (d = floor distance, h = height drop).
struct MeasNoise { double along, cross; };

inline MeasNoise tableMeasNoise(const CameraModel& cam, double horizDist, double zPlane,
                                double sigPx, double sigPitchRad, double floorIn) {
  const double h  = cam.height - zPlane;
  const double d  = horizDist;
  const double slant = std::sqrt(d * d + h * h);
  const double pxEff = sigPx / std::sqrt(2.0);            // 4 corners averaged (partly correlated)
  const double angV = std::hypot(pxEff / cam.fy, sigPitchRad);
  const double angH = pxEff / cam.fx;
  MeasNoise n;
  n.along = std::fmax(floorIn, angV * (d * d + h * h) / h);
  n.cross = std::fmax(floorIn, angH * slant);
  return n;
}

// Shift a point seen dt seconds ago into the robot frame of NOW,
// given the robot moved at forward speed v and turn rate w meanwhile.
inline Vec2 moveIntoCurrentFrame(Vec2 p, double v, double w, double dt) {
  const double c = std::cos(w * dt), s = std::sin(w * dt);
  const double xm = p.x - v * dt;
  return Vec2{ c * xm + s * p.y, -s * xm + c * p.y };
}

} // namespace trk
