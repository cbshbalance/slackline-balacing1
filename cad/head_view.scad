part_mode = "upper_body_b";
print_export = false;
intersection() {
    include <frame_master.scad>
    translate([-60, -60, 145]) cube([120, 120, 90]);  // local z 145.. (global 555..)
}
