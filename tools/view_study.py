#!/usr/bin/env python3
"""view_study.py -- can the camera see the forearm decal? Re-run after measuring the robot.
Mirrors config.hpp + mount.hpp + color_ref.hpp. Edit the numbers below to match config.hpp.
  python view_study.py            -> decal visibility vs arm angle for the configured mount
  python view_study.py 30 25      -> same for pitch 30 down, yaw 25 inward
Needs numpy."""
import sys, numpy as np
FX, FY, CX, CY = 212.3, 195.8, 160.0, 120.0
TEE_X, TEE_Y, RIM_Z, PIV_ABOVE = 6.375, 6.6875, 18.9, 2.125
LENS_FWD, LENS_UP, ROLL = 1.65, 0.60, 90.0
A2X, A2Z = 6.0, 13.4
FACE_Y = {1: 0.5, -1: 0.0}
ALONG, PERP, PITCH_SQ = 9.0, 0.0, 0.95
pitch = float(sys.argv[1]) if len(sys.argv) > 1 else 45.0
yawin = float(sys.argv[2]) if len(sys.argv) > 2 else 20.0

def lens(side, p, y):
    p, y = np.radians(p), np.radians(y)
    f = LENS_FWD*np.cos(p) + LENS_UP*np.sin(p); z = RIM_Z + PIV_ABOVE - LENS_FWD*np.sin(p) + LENS_UP*np.cos(p)
    return np.array([TEE_X + f*np.cos(y), side*TEE_Y + f*np.sin(y), z])
def axes(p, y, r):
    p, y, r = np.radians([p, y, r])
    f = np.array([np.cos(p)*np.cos(y), np.cos(p)*np.sin(y), -np.sin(p)]); r0 = np.array([np.sin(y), -np.cos(y), 0]); d0 = np.cross(f, r0)
    return f, np.cos(r)*r0 + np.sin(r)*d0, -np.sin(r)*r0 + np.cos(r)*d0
def proj(P, L, A):
    f, R, D = A; d = P - L; zc = d @ f
    return None if zc <= 1e-6 else np.array([CX + FX*(d @ R)/zc, CY + FY*(d @ D)/zc])
def arm_pt(a, s, t, y):
    a = np.radians(a); return np.array([A2X + s*np.cos(a) - t*np.sin(a), y, A2Z + s*np.sin(a) + t*np.cos(a)])
for side in (1, -1):
    yaw = -side*yawin; L = lens(side, pitch, yaw); A = axes(pitch, yaw, ROLL)
    print(f"{'LEFT' if side > 0 else 'RIGHT'} corner: lens X {L[0]:.2f} Y {L[1]:+.2f} Z {L[2]:.2f} | pitch {pitch} down, yaw {yaw:+.0f}")
    for a in range(-50, 31, 5):
        corners = [arm_pt(a, ALONG+ds, PERP+dt, FACE_Y[side]) for ds in (-1, 1) for dt in (-1, 1)]
        q = [proj(P, L, A) for P in corners]
        facing = (L[1] - FACE_Y[side])*side > 0
        ok = facing and all(x is not None and 3 <= x[0] <= 317 and 3 <= x[1] <= 237 for x in q)
        if ok:
            s = [proj(arm_pt(a, ALONG+ds, PERP+dt, FACE_Y[side]), L, A) for ds in (-PITCH_SQ/2, PITCH_SQ/2) for dt in (PITCH_SQ/2, -PITCH_SQ/2)]
            sp = min(np.linalg.norm(s[0]-s[1]), np.linalg.norm(s[0]-s[2]))
            print(f"   arm {a:+4d} deg: decal in view, square spacing {sp:4.1f} px")
        else:
            print(f"   arm {a:+4d} deg: --")
top = RIM_Z + 3.83
print(f"mount top at the worst pitch ~ {top:.2f} in (limit 23.0)")
