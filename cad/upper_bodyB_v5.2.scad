// Standalone robot upper body B piece (upper_bodyB v5.2) - A/B-final
// equipment bay, DOOR RETIRED. Changes vs v5.1 (2026-07-24):
//   - +22 mm TALLER (robot_l2 350 -> 372): the battery shelf / head is
//     raised so the shelf no longer covers the OpenCR top edge - 25 mm
//     of open cable room over the board top (was 3). The board itself
//     does not move (ELL_IMU unchanged).
//   - OPEN X- FACE with return lips replaces the screwed service door
//     (opening 79 wide vs the 75 board; battery bay opening 87).
//   - OpenCR = TAPE PAD + BOTTOM LEDGE (snap rivets retired): the
//     bulkhead front face is the board back plane, the board rests on a
//     ledge and is taped to the pad.
//   - CLOSED TELESCOPING SLEEVE splice into part A (replaces the two
//     X-side tongues that broke), flare-welded above the split, same
//     two cross bolts.
//   - POWER TOGGLE clicks into a front-open slot in the shelf front
//     edge ((*) measure the real switch: tsl_w / tsl_back_x / tsl_y).
// Print UPRIGHT as placed, sleeve down (~221 tall - fits X1C 256 with
// margin); the battery shelf bridges ~30 mm across X - allow bridging
// or a little support through the open face.
// Edit shared dimensions in frame_master.scad so the assembly stays linked.

part_mode = "upper_body_b";
part_cutaway = false;     // true removes a corner quarter: bulkhead, tape
                          // pad, seating ledge and battery shelf visible.
                          // Set back to false before exporting the STL!
print_export = true;      // use high quality $fn for STL export

include <frame_master.scad>
