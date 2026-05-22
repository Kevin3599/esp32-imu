# PCB1 Enclosure Model

This folder contains a parametric `OpenSCAD` enclosure for the PCB described by
`Gerber_PCB1_2026-02-03.zip`.

## Source dimensions

- Board outline: `78.867 mm x 70.866 mm`
- Corner radius: `5.08 mm`
- Source file: `Gerber_BoardOutlineLayer.GKO`

## Files

- `pcb1_enclosure.scad`: two-piece printable enclosure with a base, lid, board
  support ring, inferred component keepouts, simple edge retainers, and
  external screw-ear shell fastening.

## Design assumptions

- The PCB does not expose dedicated mounting holes, so the base uses perimeter
  support plus four retainers instead of screw posts.
- Top-side component clearance defaults to `18 mm`.
- Bottom-side component clearance defaults to `3 mm`.
- Edge cutouts default to a best-effort layout inferred from the Gerber probe
  data and should be tuned after checking the exact connector and cable
  locations on the assembled board.
- The lid now includes local relief pockets above the main component regions so
  you can keep the overall shell height reasonable while still clearing modules.
- The upper and lower shells are fastened with four external screw ears so the
  enclosure can still lock together even though the PCB itself has no mounting
  holes.

## Quick use

1. Open `pcb1_enclosure.scad` in OpenSCAD.
2. Change `show_mode` to `base`, `lid`, or `exploded` while tuning.
3. Toggle `show_components` if you want to inspect the inferred keepout boxes.
4. Adjust the clearance and cutout parameters near the top of the file.
5. Edit the cutout arrays as `[center, width, height]` in millimeters.
6. Export each half as STL once the fit looks right.

## Parameters you will likely tune first

- `fit_gap`
- `board_edge_gap`
- `underside_clearance`
- `topside_clearance`
- `north_cutouts`
- `south_cutouts`
- `east_cutouts`
- `west_cutouts`
- `component_clearance_gap`
- `use_fastening_ears`
- `screw_clearance_diameter`
- `screw_pilot_diameter`
- `screw_head_diameter`

## Notes

- The lid is modeled as an outer cap that overlaps the base walls. If your
  printer runs tight, increase `lid_clearance` by `0.1 mm` to `0.2 mm`.
- `show_components = true` displays approximate top-side module volumes inferred
  from silkscreen and flying-probe data. Treat them as starting geometry, not a
  final metrology source.
- `use_pressure_pads` is disabled by default to avoid colliding with tall
  modules until the exact assembled height is measured.
- The default fastening geometry is sized for roughly M3 hardware or similar
  self-tapping screws. Tweak the screw diameters if you plan to use inserts or
  machine screws instead.
- The support ring can be widened if the PCB has heavy modules near its center.
