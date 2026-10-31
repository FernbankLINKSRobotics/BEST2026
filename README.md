# links_vision -- LINKS BEST Team 701 Dining Room tracker (PROS 4)

See the packet "Seeing the Tables" (LINKS_Noisy_Object_Tracking_Packet.pdf), Sections 7-9.

1. `pros c new links_vision` (PROS 4 kernel), then copy `include/tracker/` into the project's
   `include/` folder and replace `src/main.cpp`.
2. Edit `include/tracker/config.hpp` (ports, camera mount, PLACE_X_IN, USE_IR_ENCODER).
3. `pros mu` to build and upload. Run the Day-1 checklist (packet 7.3) -- MC55 drive test FIRST.

Laptop simulator (no robot needed):
    cd test && g++ -std=c++17 -O2 -I../include host_sim.cpp -o host_sim
    ./host_sim trace 9 0 0.03                         # one run -> trace.csv
    ./host_sim mc 300 [pitchErrDeg] [modelErr] [camH] [pitchDeg] [ir 0/1]   # -> mc.csv

Calibration (pip install numpy scipy):
    python tools/calib_fit.py --selftest ; python tools/calib_fit.py session.csv
    python tools/fit_motion.py --selftest ; python tools/fit_motion.py sysid_*.csv

AI-drafted starter code: disclose and cite per the 2026 Engineering Notebook rules.
