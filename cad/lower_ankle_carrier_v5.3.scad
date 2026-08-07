// Standalone robot ankle bearing carrier (lower_ankle_carrier v5.3).
// v5.3: bridge screw pilots follow the corrected bracket screw row
//   (y -8 -> -3.2, derived from the plate front plane). See CHANGELOG v5.11.
// v5.3-frame OPEN carrier: two 608 bearing housings + top bridge raised
// to z 23..31 (42 mm AS5047P board passes 2 mm under it), bridge end
// walls weld the raised bridge onto the housings, flange z0 = 31.
// v5.2 PART FIXES (2026-07-24):
//   1) LAND BORE THROUGH: the housing land walls (|y| 31..33) are now
//      bored d19 all the way to the bearing inner-ring faces. Before,
//      only the 8.4 shaft bore pierced them, so the 11.4 OD collar nose
//      could not pass - the inner lock collar clamped against the
//      printed end-wall face and RUBBED on it. Now the nose reaches the
//      608 inner ring directly (19 > inner ring OD 12, < bearing OD 22);
//      the printed land touches only the stationary outer race.
//   2) GUSSETS CLIPPED at the housing inner face (|y| = 31): the raw
//      gusset hulls used to bulge inboard past the end-wall plane (down
//      to |y| = 29.6) into the swept volume of the rotating inner collar
//      (max radius 12.8). Nothing protrudes past the end wall anymore.
//   Verified: collar swept volume vs carrier intersection = EMPTY.
// The AS5047P rides on the L-bracket (encoder_ankle_bracket) screwed
// to the underside of the top bridge.
// Edit shared dimensions in frame_master.scad so the assembly stays linked.

part_mode = "ankle_carrier";
part_cutaway = false;     // true cuts the bearing pockets open for inspection
print_export = true;

include <frame_master.scad>
