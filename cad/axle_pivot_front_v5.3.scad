// Standalone upper rope socket, FRONT side (short shaft, no encoder).
// v5.3: clamp actually closes now -- slit 1.2->2.2 (travel ~0.7 mm diametral)
//   + living-hinge relief groove opposite the slit (wall 1.7 next to bore).
//   Bore stays 8.2. Fix for "bolted tight but pipe still loose".
// v5.2: pipe-cup clamp fused in (omega lugs + top slit + M3 cross bolt) --
//   the diagonal carbon tube is held mechanically, glue-free.
// v5.1: unchanged geometry, include moved to frame_master.scad.

part_mode = "upper_socket";
socket_side = "front";
print_export = true;

include <frame_master.scad>
