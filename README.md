# links_vision -- LINKS BEST Team 701 match program (PROS 4): controls v2 + Dining Room auto + camera self-check

1. `pros c new links_vision` (PROS 4 kernel), copy `include/tracker/` into the project's `include/`, replace `src/main.cpp`.
2. Edit `include/tracker/config.hpp` (ports, camera mount, presets marked TEACH, claw positions).
3. `pros mu`. Day-1 order: MC55 drive test -> ir_test -> pot_test (LEVEL, signs, presets) -> camera_test.

DRIVER CONTROLS = Best_2026_Controller_Controls_v2.xlsx (controls group). Left stick full-speed arcade (priority),
right stick ~1/2 speed; L1/L2 arm, R1/R2 FlapJack; D-pad alone = claw (LEFT open, RIGHT closed, UP finger up,
DOWN finger down); B+L/R setpoint stepping, B+Dup PLATE CARRY, B+Ddown PLATE CHECK; A+Dup DOOR OPEN, A+Dleft
TOP 3 BREAD, A+Dright MEAT LEVEL 2, A+Ddown LEVEL 1; Y+Dleft / Y+Dright start the auto (left / right search);
A+X+B+Dup+Ddown held 0.5 s = STOW; X held blocks A/B macros. During auto any NEW press after 1 s cancels it.
Controller screen: line 1 mode/macro, line 2 FlapJack angle, line 3 warnings (NO PLATE, CAM CHECK SKIPPED, CAM SIDE WRONG).
Changed from the old code: B no longer abandons (any new press does); X calibration dump moved to camera_test only.

CAMERA (Nov 11 2026): tee-plug mount on a 3/4 in PVC riser in the ARM-side (front) bumper tee, rim <= 18.9 in so the
mount top stays under 23.0 in. Lens ~ X 7.9, Y +/-6.1, Z 20.3 (mount.hpp computes it from config.hpp). v3.2 cheeks,
pitch 45 down = pin on the end of the slot (scale reads "- = DN"), yaw 20 inward (left -20, right +20), portrait (roll 90).
Switch corners = change CAM_SIDE only.

CAMERA SELF-CHECK (forearm decal, LINKS_forearm_decal_letter.pdf): one 2.0 in 4-colour decal on EACH side face of
ForeArm V2, centre 9.0 in from axle 2. On Y+D, the arm goes to CAMCHECK = level (0 deg, the PLATE CARRY pose),
arriving from above; 20 frames -> pitch/yaw/roll change vs the baseline -> applied; then the arm drops to
AUTO_CARRY and the auto runs. Mirror layouts let the handedness catch a wrong CAM_SIDE ("CAM SIDE WRONG" = no start).
Limit: a pot LEVEL error reads as a pitch error (1 deg arm ~ 0.7 deg pitch) -- teach LEVEL with a phone level.
Enable: camera_test baseline (A) -> paste REF_DECAL_* -> USE_DECAL_REF = true.

Laptop (no robot):
    cd test && g++ -std=c++17 -O2 -I../include color_ref_test.cpp -o color_ref_test && ./color_ref_test
    g++ -std=c++17 -O2 -I../include host_sim.cpp -o host_sim
    ./host_sim mc 300 0 0.15 0 45 1                 # nominal mount, both IR encoders
    ./host_sim mc 300 2 0.15 0 45 1 0 0 0 -3        # mount bumped +2 pitch / -3 yaw, NO self-check
    ./host_sim mc 300 2 0.15 0 45 1 0 1 0 -3        # same, self-check ON
    python tools/view_study.py [pitch] [yaw]        # decal visibility vs arm angle (edit numbers to match config)
    python tools/calib_fit.py --selftest ; python tools/calib_fit.py session.csv
Sim (Nov 11, pitch 45): nominal 266/300; +2 pitch / -3 yaw bump 43/300 uncorrected -> 278/300 with the self-check.
Open item: placed runs show a ~0.65 in mean LATERAL bias in the sim (not from the camera check) -- to investigate.

AI-drafted starter code by Claude
