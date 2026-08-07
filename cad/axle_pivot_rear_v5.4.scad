// Standalone upper socket, REAR/encoder side.
// v5.4: clamp actually closes now -- slit 1.2->2.2 (travel ~0.7 mm diametral)
//   + living-hinge relief groove opposite the slit (wall 1.7 next to bore).
//   Bore stays 8.2. Fix for "bolted tight but pipe still loose".
// v5.3: clamp flipped to the OUTSIDE of the bridge-cup bend (away from
//   the bearing side), unified with the front socket.
// v5.2: pipe-cup clamp fused in (omega lugs + top slit + M3 cross bolt) --
//   the diagonal carbon tube is held mechanically, glue-free.
// v5: shaft SHORTENED (rear_shaft_out 48 -> 13, no coupler) and the tip
// gets a 6.15 x 2.2 magnet pocket (diametral 6x2.5 magnet, 0.3 proud).
// Edit shared dimensions in frame_v5.scad so the assembly stays linked.

part_mode = "upper_socket";
socket_side = "rear";
print_export = true;

include <frame_master.scad>
