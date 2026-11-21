# ir_test -- LINKS 701 IR wheel-encoder test (PROS 4)

1. `pros c new ir_test`, replace `src/main.cpp` with this one, `pros mu`.
2. Ports: G left receiver, H left emitter, E right receiver, F right emitter (same as the robot).
3. Robot on blocks. Watch the Brain screen (full readout) and the controller (3 lines); `pros terminal` shows CSV.

Controls: A = DIGITAL/ANALOG receiver mode, X = zero counts, B = reset analog min/max,
Y (hold) = both wheels forward at 4000 mV, left stick = slow drive (40 %).

Pass: one hand-turned wheel revolution = 48 counts on that side only; with Y held, left/right edge rates within a few %.
If DIGITAL never changes, switch to ANALOG (A): if the span is several hundred counts, the sensor is analog --
tell the programmers, the robot code's IrWheelCounter then needs the analog threshold version from this test.
