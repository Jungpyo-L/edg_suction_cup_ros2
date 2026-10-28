# Corner-270 centring marker

Puts the cup centre on the vertex of the concave (270°) corner object without aligning by eye (2026-10-27: the
eye-aligned corner location was about 2 mm off, which explained the low 270° readings).

One part, cut from one 1/4 in (6.35 mm) acrylic sheet: a 70 mm square plate with the open quadrant of the corner cut
away (same outline as the object top around the vertex) and a Ø23.4 mm bore centred on the vertex. Laid on the object
with its two notch edges flush with the object walls, the bore centre is the vertex. The bore keeps 270° of its wall,
so the cup lowered into it is centred in every direction (to about half the diametral clearance; 0.28 mm toward the
open side for 0.4 mm clearance).

## Make

    python3 fixtures/corner_marker/make_corner_marker.py --hole 23.4 --kerf 0.15

- `hole_fit_coupon.svg` (cut first, same sheet): bores 23.0–23.8 mm. Pick the smallest the cup enters without force and
  sits in without visible play; regenerate the marker with that `--hole`. If the bores come out off size, adjust
  `--kerf` (the lines are offset by kerf/2 so that the finished bore has the drawn diameter).
- `corner270_marker.svg`: red = cut, blue = engrave (−X object-side line, 30° ticks round the bore, labels).
- Check before use: the cup body must be about the bore diameter over its lowest ~6 mm (the plate thickness), and the
  object top must be flat for ~35 mm around the vertex.

## Use

1. Lay the marker on the object, notch toward the open quadrant. Push it so both notch edges are flush with the two walls
   (feel with a fingernail or hold a straight edge against wall and plate together).
2. `ros2 run suction_cup record_corner_location.py --ch 3 --corner 270 --marker`
   - jog the cup down into the bore until it sits, Enter (xy is recorded);
   - the cup lifts 15 mm; remove the marker, Enter;
   - jog only down/up (f/r) until the lip just touches, Enter (z is recorded with the marker xy).
3. Run the pre-checks before the sweep (direction / y centre, seal, x position).
