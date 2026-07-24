// Standalone robot upper body B piece (upper_bodyB v5.4) - A/B-final
// equipment bay, DOOR RETIRED. Changes vs v5.3 (2026-07-25):
//   - THREE-SIDE OPEN BAY below the battery shelf (cable face
//     MAXIMIZED, user request): v5.3 removed the return lips for a
//     full-width X- opening (85/93), but the Y walls still sat only
//     5 mm off the 75 board's side edges - the OpenCR's IN-PLANE edge
//     connectors (power cable, micro-USB, switch) had no straight
//     lateral plug path. v5.4 removes EVERYTHING in front of the
//     bulkhead plane (x < bulk_x0) over the whole bay height: the X-
//     wall AND both Y-wall front strips. The board zone is open on
//     X-, Y- and Y+; every edge connector gets a straight,
//     unobstructed plug/cable path. The battery zone above the shelf
//     keeps the v5.3 full-width X- opening (93, no lips).
//   - Structure: the bulkhead (welded into both Y walls) + the rear
//     Y-wall strips + the full X+ wall form a CLOSED rear torsion
//     cell over the bay height - stiffer than the v5.2 lipped
//     C-section (the bulkhead was the main stiffener all along).
//     The seating ledge is the one part still protruding in front of
//     the bulkhead plane.
//   - Unchanged: overall height (robot_l2 372, piece ~221 tall),
//     ocr_center_z / ELL_IMU, tape pad + bottom ledge mount, closed
//     telescoping sleeve splice, shelf-front toggle slot ((*) measure
//     the real switch), 25 mm cable room over the board top.
// Print UPRIGHT as placed, sleeve down (~221 tall - fits X1C 256 with
// margin); the battery shelf bridges/cantilevers over the open bay -
// allow bridging or a little support through the open face.
// Edit shared dimensions in frame_master.scad so the assembly stays linked.

part_mode = "upper_body_b";
part_cutaway = false;     // true removes a corner quarter (bulkhead, tape
                          // pad, seating ledge and battery shelf visible) -
                          // rarely needed now, the bay is open on 3 sides.
                          // Set back to false before exporting the STL!
print_export = true;      // use high quality $fn for STL export

include <frame_master.scad>
