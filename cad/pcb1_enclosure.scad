/*
  Parametric enclosure for PCB1 derived from Gerber_BoardOutlineLayer.GKO.
  Board outline: 78.867 mm x 70.866 mm with 5.08 mm corner radius.
*/

$fn = 64;

show_mode = "assembled";  // assembled, exploded, base, lid, pcb
export_mode = 0;  // 0=use show_mode, 1=base, 2=lid, 3=exploded, 4=pcb
show_components = true;

// PCB measured from Gerber outline.
board_length = 78.867;
board_width = 70.866;
board_corner_radius = 5.08;
board_thickness = 1.6;

// Mechanical clearances.
fit_gap = 0.35;
board_edge_gap = 0.8;
underside_clearance = 3.0;
topside_clearance = 18.0;
wall_thickness = 2.2;
floor_thickness = 2.4;
lid_top_thickness = 2.2;
lid_clearance = 0.45;
lid_wall_thickness = 2.0;
lid_overlap_height = 6.0;
lid_wall_height = topside_clearance + lid_overlap_height + 1.0;
corner_relief = 0.6;

// Internal support geometry for a hole-free PCB.
support_ring_width = 3.0;
support_ring_height = 1.8;
retainer_height = 2.2;
retainer_width = 7.0;
retainer_inset = 11.0;
retainer_gap = 0.4;

// Optional lid pressure pads that keep the PCB seated.
use_pressure_pads = false;
pressure_pad_height = 1.5;
pressure_pad_diameter = 8.0;
component_clearance_gap = 1.2;

// Edge cutouts in the base, described as [center,width,height].
// Defaults are inferred from the PCB probe and silkscreen data.
north_cutouts = [
    [25.4, 14.0, 8.5],   // ESP32 USB / top-side service access
    [61.9, 12.0, 8.0]    // top-right 4-pin connector
];

south_cutouts = [
    [16.8, 24.0, 8.0],   // lower-left connector cluster
    [47.0, 16.0, 7.0]    // lower-middle/right connector cluster
];

east_cutouts = [];
west_cutouts = [];

exploded_gap = 22.0;

outer_length = board_length + 2 * (board_edge_gap + wall_thickness);
outer_width = board_width + 2 * (board_edge_gap + wall_thickness);
outer_corner_radius = board_corner_radius + board_edge_gap + wall_thickness;

board_origin_x = wall_thickness + board_edge_gap;
board_origin_y = wall_thickness + board_edge_gap;
board_seat_z = floor_thickness + underside_clearance;
base_height = board_seat_z + board_thickness + retainer_height + fit_gap;
lid_total_height = lid_top_thickness + lid_wall_height;
base_inner_length = outer_length - 2 * wall_thickness;
base_inner_width = outer_width - 2 * wall_thickness;
base_inner_radius = max(outer_corner_radius - wall_thickness - corner_relief, 0.8);
lid_inner_length = outer_length + 2 * lid_clearance;
lid_inner_width = outer_width + 2 * lid_clearance;
lid_inner_radius = outer_corner_radius + lid_clearance;
lid_outer_length = lid_inner_length + 2 * lid_wall_thickness;
lid_outer_width = lid_inner_width + 2 * lid_wall_thickness;
lid_outer_radius = lid_inner_radius + lid_wall_thickness;
lid_origin_x = -(lid_wall_thickness + lid_clearance);
lid_origin_y = -(lid_wall_thickness + lid_clearance);
active_mode = export_mode == 1 ? "base"
    : export_mode == 2 ? "lid"
    : export_mode == 3 ? "exploded"
    : export_mode == 4 ? "pcb"
    : show_mode;

module rounded_rect_2d(length, width, radius) {
    hull() {
        for (x = [radius, length - radius])
            for (y = [radius, width - radius])
                translate([x, y]) circle(r = radius);
    }
}

module shell_outer_2d() {
    rounded_rect_2d(outer_length, outer_width, outer_corner_radius);
}

module lid_outer_2d() {
    translate([lid_origin_x, lid_origin_y])
        rounded_rect_2d(lid_outer_length, lid_outer_width, lid_outer_radius);
}

module lid_inner_2d() {
    translate([-lid_clearance, -lid_clearance])
        rounded_rect_2d(lid_inner_length, lid_inner_width, lid_inner_radius);
}

module board_outline_2d(clearance = 0) {
    translate([board_origin_x, board_origin_y])
        offset(r = clearance)
            rounded_rect_2d(board_length, board_width, board_corner_radius);
}

module board_center_relief_2d() {
    translate([board_origin_x + support_ring_width, board_origin_y + support_ring_width])
        rounded_rect_2d(
            board_length - 2 * support_ring_width,
            board_width - 2 * support_ring_width,
            max(board_corner_radius - support_ring_width, 0.8)
        );
}

module edge_cutout(side, center_pos, width, height) {
    if (width > 0 && height > 0) {
        if (side == "north")
            translate([center_pos - width / 2, outer_width - wall_thickness - 0.1, floor_thickness])
                cube([width, wall_thickness + 0.2, height]);
        else if (side == "south")
            translate([center_pos - width / 2, -0.1, floor_thickness])
                cube([width, wall_thickness + 0.2, height]);
        else if (side == "east")
            translate([outer_length - wall_thickness - 0.1, center_pos - width / 2, floor_thickness])
                cube([wall_thickness + 0.2, width, height]);
        else if (side == "west")
            translate([-0.1, center_pos - width / 2, floor_thickness])
                cube([wall_thickness + 0.2, width, height]);
    }
}

module edge_cutouts(side, cutouts) {
    for (cutout = cutouts)
        edge_cutout(side, cutout[0], cutout[1], cutout[2]);
}

module pcb_proxy() {
    color([0.05, 0.45, 0.15])
        translate([board_origin_x, board_origin_y, board_seat_z])
            linear_extrude(board_thickness)
                rounded_rect_2d(board_length, board_width, board_corner_radius);
}

module component_proxy(x, y, length, width, height, color_name) {
    color(color_name)
        translate([board_origin_x + x, board_origin_y + y, board_seat_z + board_thickness])
            cube([length, width, height]);
}

module lid_relief_pocket(x, y, length, width, height) {
    translate([board_origin_x + x - component_clearance_gap,
               board_origin_y + y - component_clearance_gap,
               lid_wall_height - min(height + component_clearance_gap, lid_wall_height - 1.2)])
        cube([
            length + 2 * component_clearance_gap,
            width + 2 * component_clearance_gap,
            min(height + component_clearance_gap, lid_wall_height - 1.2) + 0.2
        ]);
}

module component_proxies() {
    // ESP32 dev board inferred from the large top-left silkscreen rectangle.
    component_proxy(6.35, 36.83, 38.10, 33.29, 15.0, "slategray");

    // Top-right module area, likely the GPS board footprint.
    component_proxy(48.38, 39.68, 27.20, 30.85, 11.0, "goldenrod");

    // Bottom connector regions.
    component_proxy(6.22, 29.46, 20.32, 2.54, 8.0, "indianred");
    component_proxy(40.51, 29.59, 12.70, 2.54, 8.0, "indianred");
    component_proxy(9.40, 24.38, 15.24, 2.54, 9.0, "steelblue");
}

module lid_relief_pockets() {
    lid_relief_pocket(6.35, 36.83, 38.10, 33.29, 15.0);
    lid_relief_pocket(48.38, 39.68, 27.20, 30.85, 11.0);
    lid_relief_pocket(6.22, 29.46, 20.32, 2.54, 8.0);
    lid_relief_pocket(40.51, 29.59, 12.70, 2.54, 8.0);
    lid_relief_pocket(9.40, 24.38, 15.24, 2.54, 9.0);
}

module support_ring() {
    translate([0, 0, board_seat_z - support_ring_height])
        linear_extrude(support_ring_height)
            difference() {
                board_outline_2d(retainer_gap);
                board_center_relief_2d();
            }
}

module board_retainer(px, py) {
    translate([px, py, board_seat_z + board_thickness - 0.2])
        cube([retainer_width, retainer_width, retainer_height]);
}

module retainers() {
    board_retainer(board_origin_x + retainer_inset, board_origin_y - retainer_gap);
    board_retainer(board_origin_x + board_length - retainer_inset - retainer_width, board_origin_y - retainer_gap);
    board_retainer(board_origin_x + retainer_inset, board_origin_y + board_width - retainer_width + retainer_gap);
    board_retainer(board_origin_x + board_length - retainer_inset - retainer_width, board_origin_y + board_width - retainer_width + retainer_gap);
}

module pressure_pad(px, py) {
    translate([px, py, lid_wall_height - pressure_pad_height])
        cylinder(h = pressure_pad_height, d = pressure_pad_diameter);
}

module lid_pressure_pads() {
    if (use_pressure_pads) {
        pressure_pad(board_origin_x + 14, board_origin_y + 14);
        pressure_pad(board_origin_x + board_length - 14, board_origin_y + 14);
        pressure_pad(board_origin_x + 14, board_origin_y + board_width - 14);
        pressure_pad(board_origin_x + board_length - 14, board_origin_y + board_width - 14);
    }
}

module base_shell() {
    difference() {
        union() {
            linear_extrude(base_height)
                shell_outer_2d();
            support_ring();
            retainers();
        }

        translate([0, 0, floor_thickness])
            linear_extrude(base_height - floor_thickness + 0.1)
                translate([wall_thickness, wall_thickness])
                    rounded_rect_2d(
                        base_inner_length,
                        base_inner_width,
                        base_inner_radius
                    );

        translate([0, 0, floor_thickness])
            linear_extrude(board_seat_z - floor_thickness + 0.05)
                offset(r = -support_ring_width)
                    board_outline_2d(retainer_gap);

        edge_cutouts("north", north_cutouts);
        edge_cutouts("south", south_cutouts);
        edge_cutouts("east", east_cutouts);
        edge_cutouts("west", west_cutouts);
    }
}

module lid_shell() {
    difference() {
        union() {
            translate([0, 0, lid_wall_height])
                linear_extrude(lid_top_thickness)
                    lid_outer_2d();

            translate([0, 0, 0])
                linear_extrude(lid_wall_height)
                    difference() {
                        lid_outer_2d();
                        lid_inner_2d();
                    }

            lid_pressure_pads();
        }

        lid_relief_pockets();
    }
}

module assembled_view() {
    color("gainsboro") base_shell();
    pcb_proxy();
    if (show_components)
        component_proxies();
    color("lightsteelblue")
        translate([0, 0, base_height - lid_overlap_height])
            lid_shell();
}

module exploded_view() {
    color("gainsboro") base_shell();
    pcb_proxy();
    if (show_components)
        component_proxies();
    color("lightsteelblue")
        translate([0, 0, base_height - lid_overlap_height + exploded_gap])
            lid_shell();
}

if (active_mode == "assembled")
    assembled_view();
else if (active_mode == "exploded")
    exploded_view();
else if (active_mode == "base")
    color("gainsboro") base_shell();
else if (active_mode == "lid")
    color("lightsteelblue") lid_shell();
else if (active_mode == "pcb")
    pcb_proxy();
