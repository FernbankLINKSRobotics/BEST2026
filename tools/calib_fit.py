#!/usr/bin/env python3
"""
calib_fit.py -- fit the camera numbers in config.hpp from real tag sightings.

WHAT YOU NEED (Section 9 of the packet):
  1. A printed table tag (100% scale) taped flat on a board at a KNOWN height
     (zPlane, e.g. on a real table = 3.625 in).
  2. A spots file, session.csv:     spot,x_in,y_in,z_in,yaw_deg
       x_in, y_in = tag CENTER in the ROBOT frame (X fwd, Y left, from the floor
       point under the drive-axle midpoint).  yaw_deg = tag rotation (0 = square to robot).
  3. For each spot N, a text file spot_N.txt holding the "CAL,..." lines you copied
     from the PROS terminal (press X on the controller to start the dump).

RUN:   python calib_fit.py session.csv            (folder containing spot_*.txt)
       python calib_fit.py --selftest             (makes fake data and checks the fit)

OUTPUT: fitted FX, FY, CX, CAM_PITCH_DEG, plus the RMS pixel error and the corner
        order the sensor uses (needed to trust the 'yaw' value).
        CY and CAM_HEIGHT_IN are HELD FIXED: with all tags on one flat plane, CY and
        pitch (and FY and height) trade off against each other, so the fit cannot
        tell them apart. That's OK -- what matters is that back-projected positions
        come out right, and the selftest checks exactly that. MEASURE the lens height
        with a tape and enter it below as CAM_HEIGHT_MEASURED.
Requires: numpy, scipy   (pip install numpy scipy)
"""
import csv, itertools, math, os, sys
import numpy as np
from scipy.optimize import least_squares

QUAD = 4.25          # inches, white inner square
CAM_X, CAM_Y = 2.0, 0.0   # lens position in robot frame -- MEASURE and edit these
CAM_HEIGHT_MEASURED = 18.0  # lens center above floor, from a tape measure
CY_FIXED = 120.0            # image center row


def project(params, pts):
    """pts: (N,3) robot-frame points -> (N,2) pixels. Same math as geometry.hpp."""
    fx, fy, cx, cy, pitch_deg, height = params
    p = math.radians(pitch_deg)
    dx, dy, dz = pts[:, 0] - CAM_X, pts[:, 1] - CAM_Y, pts[:, 2] - height
    zc = dx * math.cos(p) - dz * math.sin(p)
    xc = -dy
    yc = -dx * math.sin(p) - dz * math.cos(p)
    return np.stack([cx + fx * xc / zc, cy + fy * yc / zc], axis=1)


def tag_corners(x, y, z, yaw_deg):
    h, a = QUAD / 2, math.radians(yaw_deg)
    local = [(-h, -h), (h, -h), (h, h), (-h, h)]   # walk around the square
    return np.array([[x + u * math.cos(a) - v * math.sin(a),
                      y + u * math.sin(a) + v * math.cos(a), z] for u, v in local])


def load(session_path):
    folder = os.path.dirname(os.path.abspath(session_path))
    obs = []   # (corners3d (4,3), pixels (4,2))
    with open(session_path) as f:
        for row in csv.DictReader(f):
            c3 = tag_corners(float(row["x_in"]), float(row["y_in"]), float(row["z_in"]), float(row["yaw_deg"]))
            fn = os.path.join(folder, f"spot_{row['spot']}.txt")
            for line in open(fn):
                if not line.startswith("CAL,"):
                    continue
                v = [float(t) for t in line.strip().split(",")[3:11]]
                obs.append((c3, np.array(v).reshape(4, 2)))
    return obs


def match(params, c3, px):
    """Sensor corner order is unknown: pick the permutation with least error."""
    pred = project(params, c3)
    best = min(itertools.permutations(range(4)),
               key=lambda perm: sum(np.sum((px[list(perm)] - pred) ** 2, axis=1)))
    return best


def fit(obs, guess=(212.3, 195.8, 160.0, 22.0)):
    """Free: fx, fy, cx, pitch.  Fixed: cy, height."""
    free = np.array(guess, float)
    full = lambda f: np.array([f[0], f[1], f[2], CY_FIXED, f[3], CAM_HEIGHT_MEASURED])
    for _ in range(3):                         # re-match corners after each fit
        perms = [match(full(free), c3, px) for c3, px in obs]
        def resid(fr):
            pr = full(fr)
            return np.concatenate([(project(pr, c3) - px[list(perm)]).ravel()
                                   for (c3, px), perm in zip(obs, perms)])
        free = least_squares(resid, free, x_scale=[50, 50, 10, 2]).x
    rms = math.sqrt(np.mean(resid(free) ** 2))
    return full(free), rms, perms


def backproject(params, u, v, z):
    """Pixel -> floor-plane point (same math as pixelToPlaneZ in geometry.hpp)."""
    fx, fy, cx, cy, pitch_deg, height = params
    p = math.radians(pitch_deg)
    a, b = (u - cx) / fx, (v - cy) / fy
    Rx, Ry, Rz = math.cos(p) - b * math.sin(p), -a, -math.sin(p) - b * math.cos(p)
    t = (z - height) / Rz
    return CAM_X + t * Rx, CAM_Y + t * Ry


def report(params, rms, perms):
    fx, fy, cx, cy, pitch, h = params
    print("Paste into config.hpp:")
    print(f"  constexpr double FX = {fx:.1f};\n  constexpr double FY = {fy:.1f};")
    print(f"  constexpr double CX = {cx:.1f};\n  constexpr double CY = {cy:.1f};   // held fixed")
    print(f"  constexpr double CAM_PITCH_DEG = {pitch:.2f};\n  constexpr double CAM_HEIGHT_IN = {h:.2f};   // measured")
    print(f"RMS reprojection error: {rms:.2f} px   (good: < 1.0, suspicious: > 2.0)")
    common = max(set(perms), key=perms.count)
    print(f"Most common corner order (sensor index for physical corners -,- / +,- / +,+ / -,+): {common}")


def selftest():
    rng = np.random.default_rng(1)
    truth = np.array([205.0, 199.0, 163.0, 117.0, 23.5, 17.6])
    spots = [(x, y, 3.625, rng.uniform(-20, 20)) for x in (16, 20, 24, 28, 32) for y in (-8, 0, 8)]
    obs = []
    for s in spots:
        c3 = tag_corners(*s)
        px = project(truth, c3)
        for _ in range(15):
            noisy = np.round(px + rng.normal(0, 0.8, px.shape))
            obs.append((c3, noisy[[2, 3, 0, 1]]))   # pretend the sensor starts at a different corner
    global CAM_HEIGHT_MEASURED
    CAM_HEIGHT_MEASURED = 17.6 + 0.1          # tape-measure reading, 0.1 in off
    params, rms, perms = fit(obs)
    report(params, rms, perms)
    # The test that matters: do pixels land on the right spot on the table plane?
    worst = 0.0
    for x in (14, 20, 26, 32):
        for y in (-8, 0, 8):
            u, v = project(truth, np.array([[x, y, 3.625]]))[0]
            bx, by = backproject(params, u, v, 3.625)
            worst = max(worst, math.hypot(bx - x, by - y))
    print(f"Worst back-projection error over the approach zone: {worst:.2f} in")
    ok = worst < 0.25 and rms < 1.2
    print("SELFTEST", "PASSED" if ok else "FAILED")
    return ok


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "--selftest":
        sys.exit(0 if selftest() else 1)
    if len(sys.argv) < 2:
        print(__doc__); sys.exit(1)
    report(*fit(load(sys.argv[1])))
