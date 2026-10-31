// target_ekf.hpp -- 2-state Extended Kalman Filter that tracks ONE table
// center in the ROBOT frame.  State = (x, y): where the table is, relative
// to us.  When we drive, the table "moves" in our frame -> predict().
// When the camera sees the tag, we blend that in       -> update().
#pragma once
#include <cmath>
#include "geometry.hpp"

namespace trk {

class TargetEKF {
 public:
  double x = 0, y = 0;            // estimate [in]
  double P[2][2] = {{0, 0}, {0, 0}}; // covariance [in^2]
  bool   valid = false;
  int    id = -1;                 // which tag we are locked to
  double sinceUpdate = 1e9;       // seconds since last accepted measurement
  double lastD2 = 0;              // last Mahalanobis distance^2 (for logging)

  void reset() { valid = false; id = -1; sinceUpdate = 1e9; }

  // Start tracking from a first measurement z with noise (along, cross).
  void init(int tagId, Vec2 z, double sAlong, double sCross) {
    x = z.x; y = z.y; id = tagId;
    double R[2][2]; noiseToXY(z, sAlong, sCross, R);
    P[0][0] = 2 * R[0][0]; P[0][1] = P[1][0] = 2 * R[0][1]; P[1][1] = 2 * R[1][1];
    valid = true; sinceUpdate = 0;
  }

  // Robot drove at forward speed v [in/s] and turned at w [rad/s] for dt [s].
  // sv, sw = 1-sigma uncertainty in v and w.
  void predict(double v, double w, double dt, double sv, double sw) {
    if (!valid) return;
    const double c = std::cos(w * dt), s = std::sin(w * dt);
    const double xm = x - v * dt;
    const double nx =  c * xm + s * y;
    const double ny = -s * xm + c * y;
    // Jacobians: F = d(new)/d(old), G = d(new)/d(v,w)
    const double F[2][2] = {{ c, s}, {-s, c}};
    const double G[2][2] = {{-c * dt, dt * ny}, { s * dt, -dt * nx}};
    double FP[2][2], Pn[2][2];
    for (int i = 0; i < 2; ++i)
      for (int j = 0; j < 2; ++j)
        FP[i][j] = F[i][0] * P[0][j] + F[i][1] * P[1][j];
    for (int i = 0; i < 2; ++i)
      for (int j = 0; j < 2; ++j)
        Pn[i][j] = FP[i][0] * F[j][0] + FP[i][1] * F[j][1]
                 + G[i][0] * G[j][0] * sv * sv + G[i][1] * G[j][1] * sw * sw;
    x = nx; y = ny;
    P[0][0] = Pn[0][0]; P[0][1] = P[1][0] = 0.5 * (Pn[0][1] + Pn[1][0]); P[1][1] = Pn[1][1];
    sinceUpdate += dt;
  }

  // Blend in a measurement z. Returns false (and changes nothing) if the
  // measurement is so far from the prediction that it is probably garbage.
  bool update(Vec2 z, double sAlong, double sCross, double gateChi2) {
    if (!valid) return false;
    double R[2][2]; noiseToXY(z, sAlong, sCross, R);
    const double S00 = P[0][0] + R[0][0], S01 = P[0][1] + R[0][1], S11 = P[1][1] + R[1][1];
    const double det = S00 * S11 - S01 * S01;
    if (det <= 1e-12) return false;
    const double ex = z.x - x, ey = z.y - y;        // innovation
    lastD2 = (S11 * ex * ex - 2 * S01 * ex * ey + S00 * ey * ey) / det;
    if (lastD2 > gateChi2) return false;            // outlier gate
    const double i00 = S11 / det, i01 = -S01 / det, i11 = S00 / det;
    const double K00 = P[0][0] * i00 + P[0][1] * i01, K01 = P[0][0] * i01 + P[0][1] * i11;
    const double K10 = P[1][0] * i00 + P[1][1] * i01, K11 = P[1][0] * i01 + P[1][1] * i11;
    x += K00 * ex + K01 * ey;
    y += K10 * ex + K11 * ey;
    const double P00 = (1 - K00) * P[0][0] - K01 * P[1][0];
    const double P01 = (1 - K00) * P[0][1] - K01 * P[1][1];
    const double P11 = -K10 * P[0][1] + (1 - K11) * P[1][1];
    P[0][0] = P00; P[0][1] = P[1][0] = P01; P[1][1] = P11;
    sinceUpdate = 0;
    return true;
  }

  // Blend in ONE number z that we predicted would be h = (our formula of x, y),
  // where H0 = dh/dx and H1 = dh/dy. Used for the table-edge whisker.
  bool updateScalar(double z, double h, double H0, double H1, double sigma, double gateChi2) {
    if (!valid) return false;
    const double PH0 = P[0][0] * H0 + P[0][1] * H1;      // P * H^T
    const double PH1 = P[1][0] * H0 + P[1][1] * H1;
    const double S = H0 * PH0 + H1 * PH1 + sigma * sigma;
    const double innov = z - h;
    lastD2 = innov * innov / S;
    if (lastD2 > gateChi2) return false;
    const double K0 = PH0 / S, K1 = PH1 / S;
    x += K0 * innov; y += K1 * innov;
    const double P00 = P[0][0] - K0 * PH0, P01 = P[0][1] - K0 * PH1, P11 = P[1][1] - K1 * PH1;
    P[0][0] = P00; P[0][1] = P[1][0] = P01; P[1][1] = P11;
    return true;
  }

  // Largest 1-sigma axis of the uncertainty ellipse [in].
  double sigmaMax() const {
    const double tr = P[0][0] + P[1][1];
    const double det = P[0][0] * P[1][1] - P[0][1] * P[0][1];
    const double l1 = 0.5 * tr + std::sqrt(std::fmax(0.0, 0.25 * tr * tr - det));
    return std::sqrt(std::fmax(0.0, l1));
  }

 private:
  // Rotate (along, cross) noise into an (x, y) covariance matrix.
  static void noiseToXY(Vec2 z, double sA, double sC, double R[2][2]) {
    const double r = std::hypot(z.x, z.y);
    const double ca = (r > 1e-6) ? z.x / r : 1.0, sa = (r > 1e-6) ? z.y / r : 0.0;
    R[0][0] = ca * ca * sA * sA + sa * sa * sC * sC;
    R[1][1] = sa * sa * sA * sA + ca * ca * sC * sC;
    R[0][1] = R[1][0] = ca * sa * (sA * sA - sC * sC);
  }
};

} // namespace trk
