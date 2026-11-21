# pot_test -- LINKS 701 potentiometer test + set-point teaching (PROS 4)

1. `pros c new pot_test`, replace `src/main.cpp`, copy `include/arm_control.hpp` into the project's `include/`, `pros mu`.
2. Ports: A arm pot (axle 2), B FlapJack pot (axle 3), Smart 4 arm motor, Smart 5 FlapJack motor.
3. Open `pros terminal`: every teach step prints a block to paste into the robot's include/tracker/config.hpp.

Kit pot: rotary, linear, 16 kOhm, 300 deg (0..4095 = 0.0733 deg/count), ~6 mm shaft; geared 1:1 with 16T gears.
Wire ends to +5 V/ground, wiper to signal. Set the gear so the pot reads ~2048 mid-travel; never reach 0/4095 ("END!").

Order: MANUAL check (L1/L2 arm, R1/R2 FlapJack) -> A at LEVEL (arm forward+level, fork level) ->
X+LEFT at STOW (sets arm sign) -> Y+UP with fork tipped up (sets FlapJack sign) -> X+UP CARRY, X+DOWN PLACE
(X+RIGHT spare HIGH) -> test closed loop by tapping D-pad LEFT/UP/DOWN/RIGHT. B = stop / clear RUNAWAY.
