// color_ref.hpp -- camera self-check from the printed 4-colour DECAL on the forearm's side faces.
// (No AprilTags on the robot -- hub ruling. Printed paper glued on = decal; rule 1.3.2.3.B allows decal
// colour to be used functionally.) Each side face of ForeArm V2 carries a 2x2 grid of CYAN / MAGENTA /
// ORANGE / VIOLET squares. The AI Vision Sensor reports each square as a colour box, so one decal gives FOUR
// points -> camera pitch, yaw and roll change since the baseline. Every square has its own colour, so
// matching needs no guessing. The decal's handedness in the image confirms which corner the camera is on.
// Pure C++ (no PROS).
#pragma once
#include <cmath>
#include "config.hpp"
#include "geometry.hpp"

namespace trk {

struct Blob { int color; double u, v, w, h; };

inline const int* decalLayout(int side) { return side > 0 ? cfg::DECAL_LEFT : cfg::DECAL_RIGHT; }
inline double forearmFaceY(int side) { return side > 0 ? cfg::FOREARM_FACE_Y_LEFT_IN : cfg::FOREARM_FACE_Y_RIGHT_IN; }

// Robot-frame centre of square k (layout index, see config.hpp) of the decal on the LEFT (+1) or RIGHT (-1)
// face, for arm angle armDeg (0 = level forward, + = up). The decal lies flat on the face (plane Y = const).
inline void decalSquarePos(double armDeg, int side, int k, double out[3],
                           double along = cfg::DECAL_ALONG_IN, double perp = cfg::DECAL_PERP_IN,
                           double pitch = cfg::DECAL_PITCH_IN, double pivX = cfg::ARM_PIVOT_X_IN,
                           double pivZ = cfg::ARM_PIVOT_Z_IN) {
  const double a = deg2rad(armDeg), ux = std::cos(a), uz = std::sin(a), nx = -std::sin(a), nz = std::cos(a);
  const double s = along + ((k % 2 == 0) ? -0.5 : 0.5) * pitch;   // toward axle 2 / toward axle 3
  const double t = perp + ((k < 2) ? 0.5 : -0.5) * pitch;         // top edge / bottom edge
  out[0] = pivX + s * ux + t * nx;
  out[1] = forearmFaceY(side);
  out[2] = pivZ + s * uz + t * nz;
}

// Signed area of the triangle CYAN -> MAGENTA -> ORANGE in the image. Its SIGN is the decal's handedness as
// this camera sees it; looking at the other face from the other corner flips it.
inline double decalHandedness(const double u[4], const double v[4], const int* layout) {
  int iC = -1, iM = -1, iO = -1;
  for (int k = 0; k < 4; ++k) {
    if (layout[k] == cfg::COL_CYAN) iC = k;
    if (layout[k] == cfg::COL_MAGENTA) iM = k;
    if (layout[k] == cfg::COL_ORANGE) iO = k;
  }
  if (iC < 0 || iM < 0 || iO < 0) return 0;
  return (u[iM] - u[iC]) * (v[iO] - v[iC]) - (v[iM] - v[iC]) * (u[iO] - u[iC]);
}

// For each square k, pick the box of colour layout[k] nearest to its predicted pixel (pu[k], pv[k]) and within
// searchPx. If a colour shows up more than once, the nearest wins. idx[k] = blob index or -1. Returns the count.
inline int matchDecal(const Blob* b, int nb, const int* layout, const double pu[4], const double pv[4],
                      double searchPx, int idx[4]) {
  int n = 0;
  for (int k = 0; k < 4; ++k) {
    idx[k] = -1; double best = searchPx;
    for (int i = 0; i < nb; ++i) {
      if (b[i].color != layout[k]) continue;
      const double d = std::hypot(b[i].u - pu[k], b[i].v - pv[k]);
      if (d < best) { best = d; idx[k] = i; }
    }
    if (idx[k] >= 0) ++n;
  }
  return n;
}

// One square observation: its 3D position now (robot frame), its 3D position when the baseline was
// recorded, the baseline pixel, and the pixel seen now.
struct MarkerObs { double pNow[3], pRef[3], refU, refV, u, v; };

// Solve the camera's pitch / yaw (and roll when nParams == 3) change from up to 8 observations.
// Expected pixel = baseline pixel + (model with the change, now) - (original model, at baseline),
// so errors in the measured camera / decal POSITION mostly cancel. Gauss-Newton, numeric slopes.
// d[0] = d_pitch, d[1] = d_yaw, d[2] = d_roll [rad]. rmsPx (optional) = fit error after the solve.
inline bool solveCameraEuler(const CameraModel& cam0, const MarkerObs* obs, int n, int nParams, double d[3],
                             double* rmsPx = nullptr) {
  if (n > 8) n = 8;
  if (2 * n < nParams + 1) return false;
  auto resid = [&](const double dd[3], double* r) {
    CameraModel c = cam0; c.pitch += dd[0]; c.yaw += dd[1]; c.roll += dd[2];
    for (int i = 0; i < n; ++i) {
      Pixel a, b;
      if (!pointToPixel(c, obs[i].pNow[0], obs[i].pNow[1], obs[i].pNow[2], a) ||
          !pointToPixel(cam0, obs[i].pRef[0], obs[i].pRef[1], obs[i].pRef[2], b)) return false;
      r[2 * i] = obs[i].u - (obs[i].refU + a.u - b.u);
      r[2 * i + 1] = obs[i].v - (obs[i].refV + a.v - b.v);
    }
    return true;
  };
  d[0] = d[1] = d[2] = 0;
  const int m = 2 * n; double r[16], rp[16];
  for (int it = 0; it < 6; ++it) {
    if (!resid(d, r)) return false;
    double J[16][3];
    for (int k = 0; k < nParams; ++k) {
      double dp[3] = {d[0], d[1], d[2]}; dp[k] += 1e-4;
      if (!resid(dp, rp)) return false;
      for (int i = 0; i < m; ++i) J[i][k] = -(rp[i] - r[i]) / 1e-4;   // d(expected)/d(param)
    }
    double A[3][3] = {{0}}, g[3] = {0, 0, 0};
    for (int a = 0; a < nParams; ++a) {
      for (int i = 0; i < m; ++i) g[a] += J[i][a] * r[i];
      for (int bb = 0; bb < nParams; ++bb) for (int i = 0; i < m; ++i) A[a][bb] += J[i][a] * J[i][bb];
    }
    double step[3] = {0, 0, 0};
    if (nParams == 2) {
      const double det = A[0][0] * A[1][1] - A[0][1] * A[1][0];
      if (std::fabs(det) < 1e-12) return false;
      step[0] = (g[0] * A[1][1] - A[0][1] * g[1]) / det; step[1] = (A[0][0] * g[1] - A[1][0] * g[0]) / det;
    } else {
      auto det3 = [](const double M[3][3]) {
        return M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1]) - M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0]) +
               M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0]);
      };
      const double det = det3(A);
      if (std::fabs(det) < 1e-12) return false;
      for (int col = 0; col < 3; ++col) {
        double M[3][3];
        for (int r2 = 0; r2 < 3; ++r2) for (int c2 = 0; c2 < 3; ++c2) M[r2][c2] = (c2 == col) ? g[r2] : A[r2][c2];
        step[col] = det3(M) / det;
      }
    }
    for (int k = 0; k < 3; ++k) d[k] += step[k];
  }
  if (rmsPx) {
    if (!resid(d, r)) return false;
    double s2 = 0; for (int i = 0; i < m; ++i) s2 += r[i] * r[i];
    *rmsPx = std::sqrt(s2 / m);
  }
  return true;
}

inline CameraModel applyEuler(const CameraModel& cam0, const double d[3]) {
  CameraModel c = cam0; c.pitch += d[0]; c.yaw += d[1]; c.roll += d[2]; return c;
}

// ---------- one complete decal check (robot, camera test, simulator and tests all call this) ----------
struct DecalResult {
  int squares = 0;          // squares matched (0..4)
  bool sideWarn = false;    // handedness says the camera sees the OTHER face (camera corner wrong in config)
  bool ok = false;          // d[] is valid
  double d[3] = {0, 0, 0};  // d_pitch, d_yaw, d_roll [rad]
  double rmsPx = 0;
  double u[4] = {0, 0, 0, 0}, v[4] = {0, 0, 0, 0};   // matched square centres (layout order)
  double pu[4] = {0, 0, 0, 0}, pv[4] = {0, 0, 0, 0}; // where they were expected
};

// cam0 = the camera model the baseline was recorded with. haveBase = false: predict from the model only
// (first look, before a baseline exists) and do not solve.
inline DecalResult decalCheck(const CameraModel& cam0, int side, const Blob* blobs, int nb, double armDeg,
                              bool haveBase, const double refU[4], const double refV[4], double refDeg,
                              double searchPx, int nParams = cfg::DECAL_SOLVE_ROLL ? 3 : 2) {
  DecalResult R;
  const int* lay = decalLayout(side);
  double pNow[4][3], pRef[4][3];
  for (int k = 0; k < 4; ++k) {
    decalSquarePos(armDeg, side, k, pNow[k]);
    decalSquarePos(haveBase ? refDeg : armDeg, side, k, pRef[k]);
    Pixel a, b;
    if (!pointToPixel(cam0, pNow[k][0], pNow[k][1], pNow[k][2], a) ||
        !pointToPixel(cam0, pRef[k][0], pRef[k][1], pRef[k][2], b)) return R;
    R.pu[k] = haveBase ? refU[k] + a.u - b.u : a.u;
    R.pv[k] = haveBase ? refV[k] + a.v - b.v : a.v;
  }
  int idx[4];
  R.squares = matchDecal(blobs, nb, lay, R.pu, R.pv, searchPx, idx);
  for (int k = 0; k < 4; ++k) if (idx[k] >= 0) { R.u[k] = blobs[idx[k]].u; R.v[k] = blobs[idx[k]].v; }
  if (R.squares == 4) {   // handedness: observed vs predicted
    const double hObs = decalHandedness(R.u, R.v, lay), hExp = decalHandedness(R.pu, R.pv, lay);
    R.sideWarn = hObs * hExp < 0;
  }
  if (!haveBase || R.sideWarn || R.squares < 3) return R;
  MarkerObs ob[4]; int n = 0;
  for (int k = 0; k < 4; ++k) {
    if (idx[k] < 0) continue;
    for (int j = 0; j < 3; ++j) { ob[n].pNow[j] = pNow[k][j]; ob[n].pRef[j] = pRef[k][j]; }
    ob[n].refU = refU[k]; ob[n].refV = refV[k]; ob[n].u = R.u[k]; ob[n].v = R.v[k]; ++n;
  }
  if (!solveCameraEuler(cam0, ob, n, nParams, R.d, &R.rmsPx)) return R;
  R.ok = rad2deg(std::fabs(R.d[0])) <= cfg::REF_MAX_DEG && rad2deg(std::fabs(R.d[1])) <= cfg::REF_MAX_DEG &&
         R.rmsPx <= cfg::REF_MAX_RMS_PX;
  return R;
}

}  // namespace trk
