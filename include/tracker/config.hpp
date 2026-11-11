// =====================================================================
//  config.hpp  --  EVERY number you are expected to change lives here.
//  LINKS BEST Team 701 -- Byte to Bite Dining Room tracker
//  Units: inches, seconds, radians (unless the name says _deg / _mV / _ms)
//  Robot frame: X = forward, Y = left, Z = up. Origin = on the floor,
//               directly below the midpoint of the two drive wheels.
// =====================================================================
#pragma once

namespace cfg {

// ---------------- Smart Ports (1-21) ----------------
// Negative port number = reversed motor (PROS 4 convention).
constexpr int PORT_LEFT_DRIVE  = -1;    // MC55 -> large motor, left wheel
constexpr int PORT_RIGHT_DRIVE = 2;   // MC55 -> large motor, right wheel (reversed)
constexpr int PORT_AIVISION    = 10;   // AI Vision 
constexpr int PORT_ARM_MOTOR   = 4;    // arm motor
constexpr int PORT_FLAPJACK_MOTOR = 5; // FlapJack motor
constexpr char THREE_WIRE_CLAW  = 'd';    // three-wire claw motor

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

// ---------------- Camera mount ----------------
constexpr double CAM_HEIGHT_IN  = 18.0;  // lens center above floor
constexpr double CAM_PITCH_DEG  = 50.0;  // + = tilted DOWN from level (Section 6 trade study)
constexpr double CAM_X_IN       = 2.0;   // lens ahead of drive axle
constexpr double CAM_Y_IN       = 0.0;   // lens left of centerline

// ---------------- Field (from Field.step / Aug 23 2026 drawings) ----------------
constexpr double TABLE_TAG_Z_IN       = 3.625; // table-top tag plane height
constexpr double TAG_QUAD_IN          = 4.25;  // white inner square = detection quad
constexpr double WALL_TAG_CENTER_Z_IN = 9.0;   // wall tag center height
constexpr double PLATE_DIAMETER_IN    = 10.5;  // Appendix E

// ---------------- Where the plate sits on the robot ----------------
// Plate center (when ready to drop) in robot frame. Measure on YOUR robot.
constexpr double PLACE_X_IN = 19.0;   // ESTIMATE from Main_Robot.stl: arm tip ~16.5 in + half the 5 in FlapJack. MEASURE!
constexpr double PLACE_Y_IN = 0.0;
// Height of the plate's underside while carried on the FINAL approach.
// The plate blocks the camera's view of the table -- the lower you carry it,
// the later the tag disappears (see Section 6). Just clear the table top.
constexpr double PLATE_CARRY_Z_IN = 4.75;

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
constexpr bool   USE_IR_ENCODER     = false;
constexpr char   ADI_ENC_LEFT       = 'A';
constexpr char   ADI_ENC_RIGHT      = 'B';
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
constexpr double SIG_ENC_W_MIN      = 0.08;  // rad/s
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
constexpr char   ADI_EDGE_WHISKER = 'D';
constexpr double WHISKER_X_IN     = 16.0;  // whisker contact point ahead of the drive axle (MEASURE)
constexpr double WHISKER_LAG_IN   = 0.2;   // travel past first touch before the switch clicks (Exercise W1)
constexpr double SIG_WHISKER_IN   = 0.3;   // 1-sigma of that fix
constexpr double WHISKER_WINDOW_IN = 6.0;  // accept a touch if the edge is within this of where we expected it.
                                           // NOT a sigma gate: a real touch beats a drifted estimate.
constexpr double WHISKER_MAX_CREEP_IN = 5.0; // if we 'arrive' but never felt the edge, creep this much more
constexpr double TABLE_HALF_IN    = 6.0;   // half the 12 in table top
// 3) Front bumper on the base: unexpected contact during the run -> ABORT.
constexpr bool   USE_FRONT_BUMPER = false;
constexpr char   ADI_FRONT_BUMPER = 'E';

// ---------------- Loop timing ----------------
constexpr int LOOP_MS   = 10;
constexpr int VISION_EVERY_N_LOOPS = 2;  // poll the camera every 20 ms

} // namespace cfg
