// Standalone lower socket part - v5: INTEGRATED ANKLE AXLE version.
// v5.2: pipe-cup clamp fused in (omega lugs + M3 cross bolt + slit 2.2 +
//   living-hinge relief, same recipe as top pivot v5.12) -- the diagonal
//   carbon tube can now be squeezed tighter than the default slip fit.
//   Clamp sits on the OUTSIDE of the trapezoid; mirror-spare property kept.
// The bar cup is retired; the socket prints with a 7.8 mm axle
// (hub stop -> through one carrier 608 -> magnet-pocket tip).
// front = encoder side in the assembly, but both sides print the pocket
// (mirror parts double as spares).
// Edit shared dimensions in frame_v5.scad so the assembly stays linked.

part_mode = "lower_socket";
socket_side = "front";     // "front" or "rear"
part_cutaway = false;
print_export = true;

include <frame_master.scad>
