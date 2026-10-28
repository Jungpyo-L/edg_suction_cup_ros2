# Corner-270 plate

A concave (270°) corner object with built-in calibration bores, cut from one 1/4 in (6.35 mm) acrylic sheet. Replaces
aligning the cup to the vertex by eye (2026-10-27: the eye-aligned corner location was about 2 mm off, which explained
the low 270° readings).

```
 x=0                                                    x=165  x=200
  +------------------------------------------------------+ \
  | o (12.5, +12.5)                                       |   \   <- 270° corner
  |        (A) (40, 0)                 (B) (125, 0)       *  vertex (165, 0)
  | o (12.5, -12.5)                                       |   /     open quadrant ->
  +------------------------------------------------------+ /
  70 mm wide (y = ±35); x is the corner bisector
```

- Two M6 clearance holes (Ø6.6) at the left end, 25 mm apart across the plate: one column of the optical-table grid,
  so the plate bolts down square to the table. No spacers (the lip over the open quadrant does not reach the table).
- Bores A and B (Ø23.4, the AIS calibration-bore size) on the bisector, 85 mm apart. The cup lowered into a bore centres
  itself; A and B give the plate position and the bisector direction in the robot frame.
- Right end: the test corner. Laser cut, so the vertex is sharp; nothing is engraved within 25 mm of it.

## Make

    python3 fixtures/corner_plate/make_corner_plate.py --hole 23.4 --kerf 0.15

- Cut `hole_fit_coupon.svg` first (same sheet): bores 23.0–23.8. Pick the smallest the cup enters without force and sits
  in without visible play; regenerate with that `--hole`. If the bores come out off size, adjust `--kerf` (every cut line
  is offset by kerf/2 so the finished part has the drawn geometry).
- `corner270_plate.svg`: red = cut, blue = engrave. `corner270_plate.json`: the geometry the recorder uses.

## Use

1. Bolt the plate to the table (2 × M6), corner toward where the old 270 object was.
2. `ros2 run suction_cup record_corner_location.py --ch 3 --corner 270 --plate`
   - jog the cup into bore A until it sits, Enter; it lifts 15 mm; same for bore B;
   - the A–B distance is checked against the drawing (85 mm; warning above 0.5 mm);
   - the cup moves above the computed vertex; jog only down/up (f/r) until the lip just touches, Enter.
   Saved to `config/lateral_corner_locations.json`: vertex position, contact height and `axis_deg` (bisector direction).
3. `JP_characterization_lateral_positioning.py --ch 3 --corner 270 ...` then applies the x offsets along the bisector and
   turns the yaw with it, so theta 0 keeps the same chamber direction relative to the corner.

Accuracy (offline check): with 0.1 mm seating error per bore the vertex is within ~0.15 mm typically, 0.5 mm worst
of 500 random plate poses; repeat the bore seating if the A–B distance warning appears.
