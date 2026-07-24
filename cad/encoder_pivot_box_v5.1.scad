// Standalone rear pivot bearing + encoder bracket (encoder_pivot_box).
// v5: SHORTER (plate follows the 13 mm shaft + AS5047P chain).
// v5.3: sized for the 42 mm AS5047P module; the frame is ASYMMETRIC:
// bottom face stays on the profile top (bearing axis 25 mm above it,
// SAME as the front encoder_pivot_bearing mount - verified 2026-07-24),
// the top bar is raised so the board (+-21) and its SMT connector
// (mounted pointing UP, tip ~axis+27) clear the window.
// v5.4: real-module measurements - 4-corner grid 34 (+-17), M3
// CLEARANCE 3.4 (screws pass the plate and thread into the module's
// rear 8 mm posts), plate set 8 mm back (as_back_post), air gap 1.5
// unchanged. The AS5047P PCB sits on the plate INNER face standoffs,
// chip facing the magnet; the SPI cable exits upward, then loops down
// the rear diagonal path.
// 2026-07-24: the four M3 holes are verified FULLY THROUGH the plate
// (cross-section + ray check), and the cut now overshoots the plate by
// 2 mm on both faces so they stay cleanly open under future edits.

part_mode = "encoder_bracket";
print_export = true;

include <frame_master.scad>
