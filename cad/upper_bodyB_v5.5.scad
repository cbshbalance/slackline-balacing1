// Standalone robot upper body B piece (upper_bodyB v5.5) - A/B-final
// equipment bay, DOOR RETIRED. Changes vs v5.4 (2026-07-25):
//   - X-ONLY OPEN FACE (CoM request): v5.4's three-side bay (X- wall
//     + both Y-wall front strips removed) pushed the shell CoM to
//     x +7.4 mm behind the swing centerline. Both Y walls are
//     RESTORED to full section; only the X- face is open, at the
//     v5.3 full interior width (no lips): opening 85 in the 90 zone
//     (vs the 75 board), 93 in the head. Shell CoM moves back toward
//     center and the closed box section (Y walls + X+ wall +
//     bulkhead) is fully restored. Accepted trade-off: in-plane edge
//     plugs on the board's Y edges approach at an angle through the
//     wide front opening instead of straight along Y.
//   - SIDE-RAIL SHELF: the full battery shelf plate is replaced by
//     two 15 mm side rails welded into the Y walls; the whole middle
//     (66 wide) is OPEN. That opening is the SWITCH BAY - the power
//     toggle sits there (front-edge toggle slot RETIRED), reached
//     from the front - and a straight wiring drop from the battery
//     to the board top edge (25 mm cable room). The 84 mm pack lies
//     across Y on both rails (Y- end butted on the head wall, Y+ end
//     next to the lead bay); straps move onto the rails, one slot
//     pair per rail (slot length 16 -> 8).
//   - Unchanged: overall height (robot_l2 372, piece ~221 tall),
//     ocr_center_z / ELL_IMU, tape pad + bottom ledge mount, closed
//     telescoping sleeve splice, charge window, T-plug pass.
// Print UPRIGHT as placed, sleeve down (~221 tall - fits X1C 256 with
// margin); the shelf rails bridge ~30 mm across X - allow bridging or
// a little support through the open face.
// Edit shared dimensions in frame_master.scad so the assembly stays linked.

part_mode = "upper_body_b";
part_cutaway = false;     // true removes a corner quarter: bulkhead, tape
                          // pad, seating ledge and shelf rails visible.
                          // Set back to false before exporting the STL!
print_export = true;      // use high quality $fn for STL export

include <frame_master.scad>
