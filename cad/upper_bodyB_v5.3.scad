// Standalone robot upper body B piece (upper_bodyB v5.3) - A/B-final
// equipment bay, DOOR RETIRED. Changes vs v5.2 (2026-07-25):
//   - FULLY OPEN X- FACE: the return lips (3 mm) are removed, so the
//     service opening spans the FULL interior width - 85 in the 90 mm
//     body zone (vs the 75 board), 93 in the flared head. Reason: the
//     OpenCR connects not only along the board normal but also along
//     the board plane (power lead, USB, pin headers on the board edges)
//     - those cables must bend out through the front, so the face is
//     opened to the maximum. Edge stiffness is carried by the advanced
//     bulkhead (welded across both Y walls), the closed X+ wall, the
//     flare weld below and the head/cap above.
// Carried over from v5.2 (2026-07-24):
//   - +22 mm TALLER (robot_l2 372): 25 mm of open cable room over the
//     board top edge; the board itself does not move (ELL_IMU same).
//   - OpenCR = TAPE PAD + BOTTOM LEDGE (snap rivets retired).
//   - CLOSED TELESCOPING SLEEVE splice into part A, two cross bolts.
//   - POWER TOGGLE front-open slot in the shelf front edge ((*) measure
//     the real switch: tsl_w / tsl_back_x / tsl_y).
// Print UPRIGHT as placed, sleeve down (~221 tall - fits X1C 256);
// the battery shelf bridges ~30 mm across X - allow bridging or a
// little support through the open face.
// Edit shared dimensions in frame_master.scad so the assembly stays linked.

part_mode = "upper_body_b";
part_cutaway = false;     // true removes a corner quarter: bulkhead, tape
                          // pad, seating ledge and battery shelf visible.
                          // Set back to false before exporting the STL!
print_export = true;      // use high quality $fn for STL export

include <frame_master.scad>
