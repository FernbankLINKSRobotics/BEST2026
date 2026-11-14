# pot_test -- LINKS 701 potentiometer test + arm preset teaching (PROS 4)
`pros c new pot_test`, replace src/main.cpp, copy include/arm_control.hpp, `pros mu`, `pros terminal`.
A = LEVEL (arm forward + level, fork level, phone level) -- this is also PLATE CARRY and CAMCHECK (0 deg), so do it carefully.
X+LEFT = STOW (sets ARM_SIGN). UP/DOWN choose a preset (STOW, PLATE_CHECK, DOOR_OPEN, TOP3_BREAD, MEAT_L2, LEVEL1,
AUTO_CARRY, AUTO_PLACE), X+RIGHT stores the current arm angle into it, RIGHT goes to it, LEFT goes to STOW.
Y+UP with the fork tipped up = FJ_SIGN. B = stop. Paste the printed block into include/tracker/config.hpp.
