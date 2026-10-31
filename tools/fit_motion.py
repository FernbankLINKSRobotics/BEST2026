#!/usr/bin/env python3
"""
fit_motion.py -- fit KV_INPS_PER_MV, DEADBAND_MV and TAU_S (config.hpp) from the
sysid_*.csv files the robot writes when you press DOWN (Exercise E5).

The camera is our only speed sensor: a table tag sits still on the floor, the
robot drives straight at it, and the tag's measured X distance shrinks. The rate
it shrinks IS the robot's speed.

RUN:   python fit_motion.py sysid_*.csv
       python fit_motion.py --selftest
Requires: numpy, scipy
"""
import csv, glob, math, sys
import numpy as np
from scipy.optimize import least_squares

T_ON = 1.2   # seconds the step is held (matches main.cpp)


def travel(t, vss, tau):
    """Distance covered by a first-order-lag robot: step on for T_ON, then coast."""
    t = np.asarray(t, float)
    on = t < T_ON
    p = np.empty_like(t)
    p[on] = vss * (t[on] - tau * (1 - np.exp(-t[on] / tau)))
    pT = vss * (T_ON - tau * (1 - math.exp(-T_ON / tau)))
    vT = vss * (1 - math.exp(-T_ON / tau))
    p[~on] = pT + vT * tau * (1 - np.exp(-(t[~on] - T_ON) / tau))
    return p


def fit_one(t, x):
    def r(q):
        x0, vss, tau = q
        return x0 - travel(t, vss, tau) - x
    q = least_squares(r, [x[0], 5.0, 0.25], bounds=([-np.inf, 0, 0.02], [np.inf, 60, 2.0])).x
    return q


def read(fn):
    t, x, mv = [], [], 0
    for row in csv.DictReader(open(fn)):
        t.append(float(row["t_ms"]) / 1000); x.append(float(row["measX"]))
        mv = max(mv, int(row["cmd_mV"]))
    return np.array(t), np.array(x), mv


def summarize(results):
    mvs = np.array([m for m, v, tau in results]); vss = np.array([v for m, v, tau in results])
    moving = vss > 0.5
    kv, b = np.polyfit(mvs[moving], vss[moving], 1)      # vss = kv*mv + b  ->  db = -b/kv
    db = -b / kv
    tau = float(np.median([tau for (m, v, tau), mv_ in zip(results, moving) if mv_]))
    for m, v, tt in results:
        print(f"  {m:5d} mV -> steady {v:5.2f} in/s, tau {tt:.2f} s")
    print("Paste into config.hpp:")
    print(f"  constexpr double KV_INPS_PER_MV = {kv:.5f};")
    print(f"  constexpr double DEADBAND_MV    = {db:.0f};")
    print(f"  constexpr double TAU_S          = {tau:.2f};")
    return kv, db, tau


def selftest():
    rng = np.random.default_rng(3)
    kv, db, tau = 0.0027, 1300, 0.31
    results = []
    for mv in (2000, 3000, 4000, 5000, 6000, 8000):
        t = np.arange(0, 2.4, 1 / 30)
        x = 40 - travel(t, kv * (mv - db), tau) + rng.normal(0, 0.25, t.size)
        keep = x > 12                                   # tag gets too close / out of view
        results.append((mv, *fit_one(t[keep], x[keep])[1:]))
    k, d, ta = summarize(results)
    ok = abs(k - kv) / kv < 0.05 and abs(d - db) < 150 and abs(ta - tau) < 0.05
    print("SELFTEST", "PASSED" if ok else "FAILED", f"(truth kv={kv}, db={db}, tau={tau})")
    return ok


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "--selftest":
        sys.exit(0 if selftest() else 1)
    files = [f for a in sys.argv[1:] for f in glob.glob(a)]
    if not files:
        print(__doc__); sys.exit(1)
    res = []
    for fn in files:
        t, x, mv = read(fn)
        if len(t) < 8:
            print(f"skip {fn}: only {len(t)} sightings"); continue
        res.append((mv, *fit_one(t, x)[1:]))
    summarize(res)
