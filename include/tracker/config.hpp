// =====================================================================
//  config.hpp  --  EVERY number you are expected to change lives here.
//  LINKS BEST Team 701 -- Byte to Bite Dining Room tracker
//  Units: inches, seconds, radians (unless the name says _deg / _mV / _ms)
//  Robot frame: X = forward, Y = left, Z = up. Origin = on the floor,
//               directly below the midpoint of the two drive wheels.
// =====================================================================
#pragma once
#include <cstdint>

namespace cfg {

// ---------------- Smart Ports (1-21) ----------------
// Negative port number = reversed motor (PROS 4 convention).
// Turning came out backwards with left = 1 / right = -2, which means the motor on port 1 is on the
// robot's RIGHT side. Swapping here (keeping each port's direction) fixes driver turning AND the
// autonomous steering. (Negating the turn stick would only hide it in driver control.)
constexpr int PORT_LEFT_DRIVE  = -2;   // MC55 on port 2 -> the robot's LEFT wheel (reversed)
constexpr int PORT_RIGHT_DRIVE = 1;    // MC55 on port 1 -> the robot's RIGHT wheel
constexpr int PORT_ARM         = 4;    // MC55 -> main 4-bar arm motor
constexpr int PORT_FLAPJACK    = 5;    // MC55 -> FlapJack end (wrist) motor
constexpr int PORT_AIVISION    = 10;   // AI Vision Sensor

// ---------------- 3-wire ports (A-H) ----------------
constexpr char ADI_ARM_POT     = 'A';  // potentiometer on AXLE 2 (arm angle), geared 1:1 (16T-16T)
constexpr char ADI_FJ_POT      = 'B';  // potentiometer on AXLE 3 (FlapJack angle), geared 1:1 (16T-16T)
constexpr char ADI_CLAW_SERVO  = 'D';  // claw / hook servo at the end of the FlapJack
constexpr char ADI_IR_RX_RIGHT = 'E';  // RIGHT wheel IR receiver (encoder input)
constexpr char ADI_IR_TX_RIGHT = 'F';  // RIGHT wheel IR emitter, driven HIGH = on
constexpr char ADI_IR_RECEIVER = 'G';  // LEFT wheel IR receiver (encoder input)
constexpr char ADI_IR_EMITTER  = 'H';  // LEFT wheel IR emitter, driven HIGH = on
// C is the only free port (plate switch). The whisker / bumper switches would need the
// kit's 3-wire expander on a Smart Port if you ever enable them.

// ---------------- DRIVER CONTROLS: Best_2026_Controller_Controls_v2.xlsx (controls group) ----------------
// Left stick  = full-speed one-stick arcade (axis 3 fwd/back, axis 4 turn). Has priority.
// Right stick = ~1/2-speed arcade (axis 2 / axis 1); ignored while the left stick is outside LEFT_PRIORITY_DB.
// L1/L2 arm up/down, R1/R2 FlapJack tip up/down (hold when released; any L/R press cancels a running preset).
// D-pad ALONE (no face button held at the press) = claw: LEFT open, RIGHT closed, UP finger up, DOWN finger down.
// B = presets: B+L1/L2 next arm setpoint above/below, B+R1/R2 next FlapJack angle, B+Dup PLATE CARRY,
//     B+Ddown PLATE CHECK.   A = pickup heights: A+Dup DOOR OPEN, A+Dleft TOP 3 BREAD, A+Dright MEAT LEVEL 2,
//     A+Ddown LEVEL 1.   Y = auto only: Y+Dleft / Y+Dright start the Dining Room auto (left / right search).
// X = pre-match: while X is held, A+D-pad and B+D-pad macros are blocked. STOW = A+X+B+Dup+Ddown held 0.5 s.
// Auto: any NEW button press from 1 s after the start cancels it (driver takes over).
constexpr int    ARM_MV      = 6000;      // manual fallback (pot unplugged / USE_*_POT false)
constexpr int    FLAPJACK_MV = 5000;
constexpr int    LEFT_PRIORITY_DB  = 15;    // left stick beyond this (of 127) -> right stick ignored
constexpr double RIGHT_STICK_SCALE = 0.5;   // slow drive on the right stick
constexpr double SETPOINT_MIN_STEP_DEG = 3.0;   // B+L/R stepping skips a setpoint closer than this
constexpr int    STOW_HOLD_MS      = 500;   // all five stow buttons held this long
constexpr int    AUTO_CANCEL_GRACE_MS = 1000;   // new presses are ignored for this long after the auto starts
constexpr double FJ_SETPOINTS_DEG[7] = {-90, -45, -15, 0, 15, 45, 90};   // B+R1/R2 (angle to the ground)

// ---------------- ARM + FLAPJACK POTENTIOMETERS (teach these with the pot_test program) ----------------
// Kit pot: 16 kOhm linear, 300 deg -> 0..4095 counts. 1:1 gearing, so pot angle = axle angle.
constexpr bool USE_ARM_POT = true;
constexpr bool USE_FJ_POT  = true;     // the Consumable Kit lists ONE pot -- confirm a second is allowed
constexpr bool FJ_POT_RELATIVE_TO_ARM = true;   // true: axle-3 pot body rides on the arm
constexpr int  ARM_RAW_AT_LEVEL  = 2048; // raw reading with the arm straight forward and level   (TEACH)
constexpr int  ARM_SIGN          = 1;    // +1 if raising the arm INCREASES the raw value, else -1 (TEACH)
constexpr int  FJ_RAW_AT_LEVEL   = 2048; // raw reading with the arm level AND the fork level       (TEACH)
constexpr int  FJ_SIGN           = 1;    // +1 if raising the fork tip INCREASES the raw value      (TEACH)
// Presets (degrees). ARM: 0 = level forward, +90 = up, >90 = folded back into the 24 in start cube.
// Driver macros (controls v2). Arm values marked TEACH come from the pot_test program.
constexpr double ARM_STOW_DEG        = 140.0; // A+X+B+Dup+Ddown: folded inside the 24 in cube, FJ level, claw open (TEACH)
constexpr double ARM_PLATE_CARRY_DEG = 0.0;   // B+Dup: arm parallel to the ground, FJ 0, claw closed (= auto start pose)
constexpr double ARM_PLATE_CHECK_DEG = -50.0; // B+Ddown: arm fully down, FJ +15, claw closed            (TEACH)
constexpr double FJ_PLATE_CHECK_DEG  = 15.0;
constexpr double ARM_DOOR_OPEN_DEG   = 35.0;  // A+Dup: above refrigerator height, FJ -90, claw closed   (TEACH)
constexpr double FJ_DOOR_OPEN_DEG    = -90.0;
constexpr double ARM_TOP3_BREAD_DEG  = 15.0;  // A+Dleft: top 3 grain, FJ 0, claw open                   (TEACH)
constexpr double ARM_MEAT_L2_DEG     = -10.0; // A+Dright: refrigerator level 2, FJ 0, claw open         (TEACH)
constexpr double ARM_LEVEL1_DEG      = -40.0; // A+Ddown: bottom level of grain box / refrigerator       (TEACH)
// Dining Room auto (not driver presets): carry low so the plate does not hide the tables, then place.
constexpr double ARM_AUTO_CARRY_DEG  = -40.0; // plate about 4 in off the floor                          (TEACH)
constexpr double ARM_AUTO_PLACE_DEG  = -46.0; // plate resting on a table top                            (TEACH)
constexpr double ARM_MIN_DEG = -55.0, ARM_MAX_DEG = 150.0;      // soft limits                      (TEACH)
constexpr double FJ_LEVEL_DEG   = 0.0;   // FlapJack ground angle that holds a plate flat
constexpr double FJ_MIN_DEG = -90.0, FJ_MAX_DEG = 90.0;         // soft limits (ground angle)        (TEACH)
// Driver: L1/L2 move the ARM target, R1/R2 move the FLAPJACK target; both HOLD when you let go.
constexpr double ARM_RATE_DPS = 60.0;    // how fast holding L1/L2 moves the arm target
constexpr double FJ_RATE_DPS  = 60.0;
constexpr bool   FJ_AUTO_LEVEL = true;   // keep true (controls v2): matches the virtual 4-bar
// Position-control gains (mV): start low, raise KP until it holds without shaking.
constexpr double ARM_KP = 180.0, ARM_KD = 6.0, ARM_KG = 1200.0; constexpr int ARM_MAX_MV = 8000;
constexpr double FJ_KP  = 150.0, FJ_KD  = 4.0, FJ_KG  = 0.0;    constexpr int FJ_MAX_MV  = 6000;
constexpr double ARM_TOL_DEG = 2.0, FJ_TOL_DEG = 2.0;
// Claw servo absolute positions (-127..127), D-pad alone. TEACH on the robot.
constexpr int CLAW_OPEN        = -127;  // D-pad LEFT: fully open
constexpr int CLAW_CLOSED      = 60;    // D-pad RIGHT: fully closed
constexpr int CLAW_FINGER_UP   = 20;    // D-pad UP: slightly above one grain/protein height (releases the bottom piece)
constexpr int CLAW_FINGER_DOWN = 40;    // D-pad DOWN: one grain/protein height lower (holds the piece)
constexpr bool AUTO_SD_LOG = true;          // start SD-card logging automatically if a card is in the Brain
constexpr bool SYSID_MODE  = false;         // true = Y+Dleft / Y+Dright run the motion-model test (Exercise E5)
                                            //        INSTEAD of starting the autonomous routine

// ---------------- arcade drive ----------------
constexpr int    STICK_DEADBAND   = 8;      // ignore stick values smaller than this (out of 127)
constexpr double TURN_SCALE       = 0.75;   // turn sensitivity relative to forward
constexpr bool   SQUARED_INPUTS   = true;   // finer control near the center, full speed at the edge

// VEXcode limits MC55 voltage commands to +/-8 V. Keep the same cap until
// you have verified on the bench what the MC55 does above 8000 mV.
constexpr int MC55_MAX_MV = 8000;

// ---------------- Camera intrinsics ----------------
// Defaults derived from the VEX spec (74 deg H x 63 deg V, 320x240 px).
// REPLACE with values from the calibration procedure (Section 9).
constexpr double IMG_W = 320.0, IMG_H = 240.0;
constexpr double FX = 212.3;   // = 160 / tan(37.0 deg)
constexpr double FY = 195.8;   // = 120 / tan(31.5 deg)
constexpr double CX = 160.0;
constexpr double CY = 120.0;

// ---------------- Camera mount: tee-plug mount on a PVC riser, ARM side (Nov 11 2026) ----------------
// camera_mount_tee_plug_v3-1_34in.svg on a vertical 3/4 in Sch40 riser that stands in the branch of the
// bumper tee at a FRONT (arm-side) corner. The plug sits in the top socket of a fitting on the riser.
// Stack (3/16 ply): plug discs in the socket -> collar on the rim -> yaw plate (engraved R/L scale) -> two
// pitch cheeks (engraved -20..+10 scale, "- = DN") -> two flanges + face plate carrying the sensor.
// HEIGHT LIMIT: top of the mount <= 23.0 in. Between level and 45 down the mount top is <= 3.83 in above the rim
// (SVG + ~1 in sensor body), so rim <= 18.9 in keeps it under 23.0 with 0.25 in margin -> lens ~20.3 in at 45.
// (Locked at 45 down only: top is 3.0 in above the rim, rim could go to ~19.8 in. Tilting UP raises the top.)
// lensFromMount() in mount.hpp turns these numbers plus pitch/yaw into the lens position, so moving the
// camera to the other corner = change CAM_SIDE only.
constexpr int    CAM_SIDE        = +1;      // +1 = camera on the LEFT front corner, -1 = RIGHT front corner
constexpr double TEE_X_IN        = 6.375;   // riser centre ahead of the drive axle (bumper tee, arm side)  (CAD, MEASURE)
constexpr double TEE_Y_IN        = 6.6875;  // riser centre out from the centreline (13.375/2)               (CAD)
constexpr double PLUG_RIM_Z_IN   = 18.9;    // rim of the socket the plug sits in, above the floor            (MEASURE)
constexpr double PITCH_PIVOT_ABOVE_RIM_IN = 2.125; // collar 0.1875 + yaw plate 0.1875 + cheek pivot 1.75 (SVG)
// Lens from the pitch pivot with the camera looking LEVEL: 0.75 in to the face plate front (SVG) + ~0.9 in
// of sensor body (ASSUMED), and 0.60 in above the pivot (the face plate's "+" mark, SVG).
constexpr double LENS_FWD_OF_PIVOT_IN = 1.65;  // (ASSUMED -- measure the lens-to-face distance)
constexpr double LENS_UP_OF_PIVOT_IN  = 0.60;  // (SVG)
// PITCH: + = tilted DOWN. Engraved cheek scale reads "- = DN", so CAM_PITCH_DEG = -(scale reading).
//   Scale 0 = flange square to the cheek = optical axis LEVEL (from the SVG geometry -- check with a phone
//   level on the face plate). v3.2 cheeks (camera_mount_tee_plug_v3-2): slot allows 45 down .. 15 up.
//   45 down = the pin sits on the END of the slot. The camera's weight is ahead of the pivot, so it rests on
//   that stop: a bump that loosens the clamp drops it back to the same 45, which is what the self-check wants.
//   CHECK: at 45 the sensor body's lower front corner comes ~2.1 in below the pivot, ~0.4 in ahead of it --
//   make sure it clears the yaw plate's front edge (top of the yaw plate is 1.75 in below the pivot).
//   Sim (20.3-21.2 in lens): 25..45 down all score ~85-95 %; steeper is no worse and keeps the decal in view.
constexpr double CAM_PITCH_DEG  = 45.0;
// YAW: + = turned LEFT. Read the yaw plate's engraved scale: toward "L" = +, toward "R" = -.
//   From a corner, aim INWARD toward the robot's path: about 20 deg (left -20, right +20; sim best 20-30).
constexpr double CAM_YAW_LEFT_DEG  = -20.0;
constexpr double CAM_YAW_RIGHT_DEG = +20.0;
constexpr double CAM_ROLL_DEG   = 90.0;  // 90 = sensor sideways (portrait, 74 deg axis vertical); 0 = landscape.
                                         //   If everything comes out mirrored up/down in calibration, use -90.
// Optional: tape-measured lens position overrides the mount model (set true once measured).
constexpr bool   CAM_POS_MEASURED = false;
constexpr double CAM_X_MEAS_IN = 7.87, CAM_Y_MEAS_IN = 6.14, CAM_HEIGHT_MEAS_IN = 20.28;  // lens centre (current side)

// ---------------- Field (from Field.step / Aug 23 2026 drawings) ----------------
constexpr double TABLE_TAG_Z_IN       = 3.625; // table-top tag plane height
constexpr double TAG_QUAD_IN          = 4.25;  // white inner square = detection quad
constexpr double WALL_TAG_CENTER_Z_IN = 9.0;   // wall tag center height
constexpr double PLATE_DIAMETER_IN    = 10.5;  // Appendix E

// ---------------- FOREARM DECAL: camera self-check (replaces the side-link flags) ----------------
// No AprilTags on the robot (hub). Printed paper glued on = decal (hub, Nov 2026), allowed as long as it does
// not distract or confuse other teams; rule 1.3.2.3.B lets decal colour be used functionally. Hub-confirmed
// colours: CYAN #00E5FF, MAGENTA #FF00A0, ORANGE #FF6600, VIOLET #7F00FF.
// One 2.0 x 2.0 in decal on EACH side face of ForeArm V2 (the 2.0 in wide bar): 2x2 squares of 0.85 in with
// 0.10 in black border and gutter. The LEFT-corner camera sees only the LEFT face, the RIGHT camera only
// the RIGHT face. The two faces carry MIRROR-IMAGE layouts, so the decal's handedness in the image tells
// which face (= which corner) the camera is really looking at.
// Self-check: arm to ARM_CAMCHECK_DEG (always arriving from ABOVE so gear backlash takes up the same way),
// average a few frames, solve pitch / yaw / roll change vs the baseline recorded right after calibration.
// LIMIT (tools/view_study.py): from the camera, an arm-angle error looks almost exactly like a camera
// pitch error (1 deg of arm ~ 0.7 deg of pitch). Yaw and roll are clean. Teach the pot LEVEL carefully
// (phone level on the forearm) -- the check is only as good as the pot's LEVEL reading.
constexpr bool    USE_DECAL_REF  = false;             // true after camera_test baseline (A) is pasted below
constexpr int     COL_CYAN = 1, COL_MAGENTA = 2, COL_ORANGE = 3, COL_VIOLET = 4;   // AI Vision colour ids
constexpr uint8_t COLOR_RGB[4][3] = {{0, 229, 255}, {255, 0, 160}, {255, 102, 0}, {127, 0, 255}};  // ids 1..4: TEACH
                                                      //   from the PRINTED decal in the AI Vision Utility
constexpr float   COLOR_HUE_RANGE = 10.0f;            // 1..40; field colours are muted, keep tight
constexpr float   COLOR_SAT_RANGE = 0.35f;            // 0.1..1
constexpr double  ARM_PIVOT_X_IN = 6.0, ARM_PIVOT_Z_IN = 13.4;  // axle 2 (ForeArm V2 STL hole; MEASURE Z)
constexpr double  FOREARM_LEN_IN = 11.75;             // axle 2 -> axle 3 hole centres (ForeArm V2 STL)
constexpr double  FOREARM_FACE_Y_LEFT_IN  = 0.5;      // left face of the 1/2 in plate (+Y)  (STL: plate Y 0..0.5)
constexpr double  FOREARM_FACE_Y_RIGHT_IN = 0.0;      // right face
constexpr double  DECAL_ALONG_IN = 9.0;   // decal centre, along the forearm from axle 2 (keeps 1.2 in clear of axle 3) (MEASURE)
constexpr double  DECAL_PERP_IN  = 0.0;   // decal centre above the axle2-axle3 line (0 = centred on the 2 in bar) (MEASURE)
constexpr double  DECAL_PITCH_IN = 0.95;  // square centre spacing = 0.85 square + 0.10 gutter
// Layout in ARM terms, index k: 0 = axle-2 end / top edge, 1 = axle-3 end / top edge,
//                               2 = axle-2 end / bottom edge, 3 = axle-3 end / bottom edge
// ("top edge" = the edge that is up when the arm points straight forward and level).
constexpr int     DECAL_LEFT[4]  = {COL_CYAN, COL_MAGENTA, COL_ORANGE, COL_VIOLET};
constexpr int     DECAL_RIGHT[4] = {COL_VIOLET, COL_ORANGE, COL_MAGENTA, COL_CYAN};
constexpr double  ARM_CAMCHECK_DEG = 0.0; // camera self-check pose = arm LEVEL (same as the driver's B+Dup PLATE CARRY and the
                                          //   auto start pose). Decal in view at 0 deg for pitch 20-35 / yaw 15-30 (~15 px squares)
constexpr double  ARM_CAMCHECK_FROM_ABOVE_DEG = 6.0;  // go this far above CAMCHECK first, then come down onto it (pot gear backlash)
constexpr double  CAMCHECK_SETTLE_DEG = 1.0;           // arm within this of CHECK ...
constexpr int     CAMCHECK_SETTLE_MS  = 250;           // ... for this long before frames are used
constexpr int     CAMCHECK_FRAMES     = 20;            // good frames to average (~0.7 s at 30 fps)
constexpr bool    DECAL_SOLVE_ROLL = true;          // true: solve pitch + yaw + roll (needs ~20 frames: the decal is
                                                    //   small, so roll is the noisiest). false: pitch + yaw only --
                                                    //   only if camera_test shows roll never moves.
constexpr int     CAMCHECK_TIMEOUT_MS = 2500;          // give up (keep the last correction) after this
constexpr bool    SELFCHECK_ON_START = true;        // Y+Dleft / Y+Dright: self-check first, then start the Dining Room auto
// Baseline (camera_test A, arm at CAMCHECK, right after calibration): the 4 square centres + the arm angle
constexpr double  REF_DECAL_U[4] = {0, 0, 0, 0};
constexpr double  REF_DECAL_V[4] = {0, 0, 0, 0};
constexpr double  REF_DECAL_DEG  = 0;
constexpr double  REF_SEARCH_PX  = 45.0;   // look for each colour within this of where it should be
constexpr double  REF_ARM_WINDOW_DEG = 6.0; // use the decal only with the arm this close to the baseline angle
constexpr double  REF_MAX_DEG    = 10.0;    // ignore a correction bigger than this (bad detection)
constexpr double  REF_MAX_RMS_PX = 2.5;     // ignore a frame whose 4 squares don't fit the solution this well
constexpr bool    COLORS_OFF_IN_APPROACH = true;   // colour detection only during the self-check / early run

// ---------------- Where the plate sits on the robot ----------------
// Plate center (when ready to drop) in robot frame. Measure on YOUR robot.
constexpr double PLACE_X_IN = 19.0;   // ESTIMATE (Main_Robot_261108.stl: 14 in arm lowered ~43 deg to carry at 4 in). MEASURE!
constexpr double PLACE_Y_IN = 0.0;
// Height of the plate's underside while carried on the FINAL approach.
// The plate blocks the camera's view of the table -- the lower you carry it,
// the later the tag disappears (see Section 6). Just clear the table top.
constexpr double PLATE_CARRY_Z_IN = 4.75;   // TOP of the plate rim while carried (bottom ~4.0 in)
constexpr double TAG_OUTER_HALF_IN = 3.83;  // Circle21h7 data bits reach 4.5 modules (0.85 in) from the centre

// ---------------- Drive + motion model ----------------
constexpr double TRACK_WIDTH_IN   = 10.0;   // wheel center to wheel center (CAD: wheel at x=-5.0; assumes right wheel mirrors it)
constexpr double KV_INPS_PER_MV   = 0.0030; // steady wheel speed per mV (FIT THIS)
constexpr double TAU_S            = 0.25;   // first-order lag of MC55+motor+sled (FIT THIS)
constexpr double DEADBAND_MV      = 1200;   // below this the sled does not move (FIT THIS)

// Process noise: how much we distrust the motion model (1-sigma).
constexpr double SIG_V_FRAC  = 0.30;  // fraction of |v|
constexpr double SIG_V_MIN   = 0.8;   // in/s
constexpr double SIG_W_FRAC  = 0.30;  // fraction of |w|
constexpr double SIG_W_MIN   = 0.10;  // rad/s  (left/right motors never match exactly)

// ---------------- Measurement noise model ----------------
constexpr double SIG_PX        = 0.8;   // corner pixel noise (1-sigma), from Exercise E2
constexpr double SIG_PITCH_DEG = 0.6;   // robot rocking on the sled (1-sigma)
constexpr double SIG_FLOOR_IN  = 0.15;  // never claim better than this
constexpr double GATE_CHI2     = 9.21;  // 99% gate for 2 degrees of freedom
constexpr double SIDE_TOL_FRAC = 0.30;  // reject tags whose projected sides are off by >30%
constexpr int    VISION_LATENCY_MS = 50;// measure this in Exercise E3
constexpr int    REINIT_AFTER_REJECTS = 4; // this many gate rejections in a row -> restart the track

// ---------------- Approach controller ----------------
constexpr double KP_TURN        = 2.2;   // rad/s per rad of bearing error
constexpr double KP_DRIVE       = 0.9;   // in/s per inch of along-track error
constexpr double V_MAX_INPS     = 14.0;  // approach speed cap
constexpr double V_MIN_INPS     = 4.0;   // slower than this and the sled stalls
constexpr double W_MAX_RADPS    = 1.6;
constexpr double TURN_IN_PLACE_DEG = 25.0; // bigger bearing error -> pivot first
constexpr double COMMIT_EX_IN   = 6.0;   // go blind no later than this (plate-to-target)
constexpr double COMMIT_MAX_EY_IN = 1.2; // ...and lateral error is below this
constexpr double LOST_TIMEOUT_S = 0.6;   // coast this long without a measurement, then abort
constexpr double MAX_SIGMA_IN   = 3.0;   // abort if the estimate gets this uncertain
constexpr double ENTER_DIST_IN  = 18.0;  // open-loop roll-in past the doorway tape
constexpr double ENTER_V_INPS   = 10.0;
constexpr double SEARCH_W_RADPS = 0.5;   // slow pivot while hunting for a table tag
constexpr double SEARCH_TIMEOUT_S = 3.0;
constexpr double AUTO_TIMEOUT_S = 15.0;
constexpr double BACKOUT_DIST_IN = 8.0;

// ---------------- OPTIONAL: IR wheel encoders (BEST IR Sensor Kit) ----------------
// Plan B / Plan A+: a painted stripe disc on a TEAM-MADE part of each drive
// axle, read by a BEST IR sensor on a 3-wire port. Single channel: direction
// comes from the commanded voltage sign. Set USE_IR_ENCODER = false to run
// on the motion model alone (the default -- works without any IR hardware).
constexpr bool   USE_IR_ENCODER     = true;    // left wheel: receiver on G, emitter on H (decal from make_wheel_decal.py)
constexpr bool   USE_RIGHT_ENCODER  = true;    // right wheel: receiver on E, emitter on F
constexpr char   ADI_ENC_RIGHT_RX   = ADI_IR_RX_RIGHT;
constexpr double WHEEL_DIAM_IN      = 10.0;
constexpr int    ENC_STRIPES        = 24;    // black stripes around the circle (24 black + 24 white bands)
// Where on the wheel face the IR sensor looks. Does NOT change the distance math
// (that only needs WHEEL_DIAM_IN and ENC_STRIPES) -- it only sets how wide each
// painted band is, which must be comfortably wider than the IR sensor's spot.
constexpr double ENC_READ_RADIUS_IN = 4.5;
constexpr double IR_SPOT_IN         = 0.15;  // approx. width the IR sensor "sees" at its mounting gap (measure!)
constexpr double ENC_DISC_PER_WHEEL = 1.0;   // disc revs per wheel rev (1.0 = disc on wheel axle)
constexpr double SIG_ENC_FRAC       = 0.08;  // wheel slip/scrub on the sled (1-sigma, fraction of |v|)
constexpr double SIG_ENC_V_MIN      = 0.4;   // in/s
constexpr double SIG_ENC_W_MIN      = 0.08;
constexpr double ENC_W_BLEND_S      = 0.30;  // both-wheel mode: smoothing of the measured turn rate
constexpr double ENC_W_TRUST        = 0.5;   // 0 = turn rate from model only, 1 = from ticks only  // rad/s
// inches of wheel travel per counted edge (both edges of every stripe counted)
constexpr double ENC_IN_PER_EDGE = 3.14159265358979 * WHEEL_DIAM_IN / (2.0 * ENC_STRIPES * ENC_DISC_PER_WHEEL);
// Width of one painted band at the read radius (on the disc, not the floor).
constexpr double ENC_BAND_IN = 3.14159265358979 * ENC_READ_RADIUS_IN / ENC_STRIPES;
static_assert(ENC_BAND_IN >= 3.0 * IR_SPOT_IN,
              "Encoder bands too narrow for the IR spot: use fewer stripes or read farther out");

// ---------------- OPTIONAL: contact switches (Consumable Kit: 3x Omron SS-01GL13PT) ----------------
// Wire each switch COM -> ground (black) and NO -> signal (white) through a
// Screw Terminal Sensor Interface Cable. The 3-wire port pulls the signal high,
// so a PRESSED switch reads 0.
constexpr int    SWITCH_PRESSED_LEVEL = 0;
// 1) Plate switch on the FlapJack: pressed = plate clamped. Blocks starting auto
//    without a plate, and ends PLACE when the plate is released.
constexpr bool   USE_PLATE_SWITCH = false;
constexpr char   ADI_PLATE_SWITCH = 'C';
// 2) Table-edge whisker: a piano-wire feeler hanging from the arm that brushes the
//    table top's front edge. Gives the blind finish an exact "edge is HERE" fix.
constexpr bool   USE_EDGE_WHISKER = false;
constexpr char   ADI_EDGE_WHISKER = 'C';   // needs a free port: share C with the plate switch OR use the expander
constexpr double WHISKER_X_IN     = 16.0;  // whisker contact point ahead of the drive axle (MEASURE)
constexpr double WHISKER_LAG_IN   = 0.2;   // travel past first touch before the switch clicks (Exercise W1)
constexpr double SIG_WHISKER_IN   = 0.3;   // 1-sigma of that fix
constexpr double WHISKER_WINDOW_IN = 6.0;  // accept a touch if the edge is within this of where we expected it.
                                           // NOT a sigma gate: a real touch beats a drifted estimate.
constexpr double WHISKER_MAX_CREEP_IN = 5.0; // if we 'arrive' but never felt the edge, creep this much more
constexpr double TABLE_HALF_IN    = 6.0;   // half the 12 in table top
// 3) Front bumper on the base: unexpected contact during the run -> ABORT.
constexpr bool   USE_FRONT_BUMPER = false;
constexpr char   ADI_FRONT_BUMPER = 'C';   // needs the 3-wire expander (no free port)

// ---------------- Loop timing ----------------
constexpr int LOOP_MS   = 10;
constexpr int VISION_EVERY_N_LOOPS = 2;  // poll the camera every 20 ms

} // namespace cfg
