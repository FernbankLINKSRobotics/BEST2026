// color_ref_test.cpp -- laptop check of the forearm-decal camera self-check.
// g++ -std=c++17 -O2 -I../include color_ref_test.cpp -o color_ref_test && ./color_ref_test [nParams 2|3] [px noise]
#include <cstdio>
#include <cmath>
#include <random>
#include <cstdlib>
#include <algorithm>
#include "tracker/config.hpp"
#include "tracker/mount.hpp"
#include "tracker/color_ref.hpp"
using namespace trk;
static int NP = cfg::DECAL_SOLVE_ROLL ? 3 : 2;
static double SIG = 1.0;

// Project the decal on face 'face' with camera 'cam' at arm angle a -> 4 colour boxes (+ pixel noise).
static int makeBlobs(const CameraModel& cam, int face, double a, std::mt19937& rng, double sig, Blob* out) {
  std::normal_distribution<double> N(0, 1);
  int n = 0; const int* lay = decalLayout(face);
  for (int k = 0; k < 4; ++k) {
    double p[3]; decalSquarePos(a, face, k, p);
    if ((cam.yoff - p[1]) * face <= 0) continue;          // that face points away from this camera
    Pixel q; if (!pointToPixel(cam, p[0], p[1], p[2], q) || !inImage(q, 320, 240, 3)) continue;
    out[n++] = {lay[k], q.u + sig * N(rng), q.v + sig * N(rng), 18, 18};
  }
  return n;
}

int main(int argc, char** argv) {
  if (argc > 1) NP = std::atoi(argv[1]);
  if (argc > 2) SIG = std::atof(argv[2]);
  std::printf("solving %d parameters, %.1f px noise per square, %d frames\n", NP, SIG, cfg::CAMCHECK_FRAMES);
  int fails = 0;
  std::mt19937 rng(7);
  for (int side : {1, -1}) {
    const CameraModel cam = cameraFromConfig(side);
    std::printf("== camera on the %s corner: lens X %.2f Y %+.2f Z %.2f, pitch %.0f down, yaw %+.0f\n", side > 0 ? "LEFT" : "RIGHT",
                cam.xoff, cam.yoff, cam.height, rad2deg(cam.pitch), rad2deg(cam.yaw));
    // visibility of BOTH faces' decals at CHECK and the arm-angle window
    for (int face : {1, -1}) {
      Blob b[4]; const int n = makeBlobs(cam, face, cfg::ARM_CAMCHECK_DEG, rng, 0, b);
      std::printf("   %s-face decal at CHECK %.0f: %d/4 squares in view", face > 0 ? "LEFT " : "RIGHT", cfg::ARM_CAMCHECK_DEG, n);
      if (n == 4) std::printf(" (spacing %.1f / %.1f px)", std::hypot(b[0].u - b[1].u, b[0].v - b[1].v), std::hypot(b[0].u - b[2].u, b[0].v - b[2].v));
      std::printf("\n");
      if (face == side && n != 4) ++fails;
    }
    int lo = 999, hi = -999;
    for (int a = -60; a <= 20; ++a) { Blob b[4]; if (makeBlobs(cam, side, a, rng, 0, b) == 4) { lo = std::min(lo, a); hi = std::max(hi, a); } }
    std::printf("   own decal fully in view for arm %d .. %d deg\n", lo, hi);
    // baseline at CHECK from the nominal camera
    double refU[4], refV[4];
    { Blob b[4]; makeBlobs(cam, side, cfg::ARM_CAMCHECK_DEG, rng, 0, b); for (int k = 0; k < 4; ++k) { refU[k] = b[k].u; refV[k] = b[k].v; } }
    // bumps: 8 frames, 1 px noise, averaged
    const double cases[][3] = {{2, 0, 0}, {0, -3, 0}, {2, -3, 1.5}, {-1.5, 4, -1}};
    for (auto& dc : cases) {
      CameraModel truth = cam; truth.pitch += deg2rad(dc[0]); truth.yaw += deg2rad(dc[1]); truth.roll += deg2rad(dc[2]);
      double acc[3] = {0, 0, 0}; int good = 0;
      for (int f = 0; f < cfg::CAMCHECK_FRAMES; ++f) {
        Blob b[8]; const int nb = makeBlobs(truth, side, cfg::ARM_CAMCHECK_DEG, rng, SIG, b);
        DecalResult R = decalCheck(cam, side, b, nb, cfg::ARM_CAMCHECK_DEG, true, refU, refV, cfg::ARM_CAMCHECK_DEG, cfg::REF_SEARCH_PX, NP);
        if (R.ok) { for (int j = 0; j < 3; ++j) acc[j] += R.d[j]; ++good; }
      }
      if (!good) { std::printf("   bump %+.1f %+.1f %+.1f: NO SOLUTION\n", dc[0], dc[1], dc[2]); ++fails; continue; }
      for (int j = 0; j < 3; ++j) acc[j] /= good;
      CameraModel fixed = applyEuler(cam, acc);
      Pixel px; pointToPixel(truth, 24, 0, cfg::TABLE_TAG_Z_IN, px); Vec2 a0, a1;
      pixelToPlaneZ(cam, px, cfg::TABLE_TAG_Z_IN, a0); pixelToPlaneZ(fixed, px, cfg::TABLE_TAG_Z_IN, a1);
      const double eB = std::hypot(a0.x - 24, a0.y), eA = std::hypot(a1.x - 24, a1.y);
      std::printf("   bump dP %+4.1f dY %+4.1f dR %+4.1f -> found %+5.2f %+5.2f %+5.2f (%d fr) | table pt 24 in ahead: %.2f -> %.2f in\n",
                  dc[0], dc[1], dc[2], rad2deg(acc[0]), rad2deg(acc[1]), rad2deg(acc[2]), good, eB, eA);
      if (eA > 0.4) ++fails;
    }
    // handedness: this corner's camera on this face vs the other corner's camera on the other face
    {
      Blob b[4]; makeBlobs(cam, side, cfg::ARM_CAMCHECK_DEG, rng, 0, b);
      CameraModel other = cameraFromConfig(-side); Blob o[4]; makeBlobs(other, -side, cfg::ARM_CAMCHECK_DEG, rng, 0, o);
      double u[4], v[4], uo[4], vo[4];
      for (int k = 0; k < 4; ++k) { u[k] = b[k].u; v[k] = b[k].v; uo[k] = o[k].u; vo[k] = o[k].v; }
      const double h1 = decalHandedness(u, v, decalLayout(side)), h2 = decalHandedness(uo, vo, decalLayout(-side));
      std::printf("   handedness this corner %+.0f, other corner %+.0f -> %s\n", h1, h2, h1 * h2 < 0 ? "distinguishable" : "SAME (bad)");
      if (h1 * h2 >= 0) ++fails;
      // wrong corner in config: blobs from the other camera, fed to this config, with a wide search
      DecalResult R = decalCheck(cam, side, o, 4, cfg::ARM_CAMCHECK_DEG, false, refU, refV, cfg::ARM_CAMCHECK_DEG, 400.0);
      std::printf("   camera really on the other corner: %d squares matched, sideWarn=%d\n", R.squares, (int)R.sideWarn);
      if (R.squares == 4 && !R.sideWarn) ++fails;
    }
    // the known limit: a pot error looks like a pitch change
    {
      Blob b[8]; const int nb = makeBlobs(cam, side, cfg::ARM_CAMCHECK_DEG - 1.0, rng, 0, b);   // arm really 1 deg lower
      DecalResult R = decalCheck(cam, side, b, nb, cfg::ARM_CAMCHECK_DEG, true, refU, refV, cfg::ARM_CAMCHECK_DEG, cfg::REF_SEARCH_PX);
      std::printf("   pot reads 1.0 deg high (camera untouched) -> solver reports dPitch %+5.2f dYaw %+5.2f dRoll %+5.2f deg\n",
                  rad2deg(R.d[0]), rad2deg(R.d[1]), rad2deg(R.d[2]));
    }
  }
  std::printf(fails ? "FAILED %d\n" : "ALL PASSED\n", fails);
  return fails;
}
