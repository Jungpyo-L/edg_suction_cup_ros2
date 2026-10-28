#!/usr/bin/env python3
"""Laser-cut concave (270 deg) corner object with built-in calibration bores, from one 1/4 in (6.35 mm) acrylic sheet.

  python3 fixtures/corner_plate/make_corner_plate.py [--hole 23.4] [--kerf 0.15]

Writes next to this script (mm; red = cut, blue = engrave):
  corner270_plate.svg   the plate
  corner270_plate.json  geometry in the plate frame, read by src/record_corner_location.py --plate
  hole_fit_coupon.svg   bores hole-0.4 .. hole+0.4 (cut first, same sheet)

Plate frame: x along the plate (= the corner bisector), origin at the left end, y across (centre line y = 0).
- Two M6 clearance holes at the left end, one above the other, 25 mm apart (one column of the 25 mm optical-table grid),
  so the plate bolts down square to the grid without spacers.
- Two calibration bores A and B on the bisector, 85 mm apart. The cup lowered into a bore centres itself; recording the
  TCP in both gives the plate position and the bisector direction in the robot frame.
- The right end is the test corner: the open quadrant |y| < x - x_vertex is cut away, so the object occupies 270 deg
  around the vertex (x_vertex, 0). Laser cut, so the concave vertex is sharp; nothing is engraved within 25 mm of it.

Kerf: every cut line is offset by kerf/2 toward the waste, so the finished part has the drawn geometry.
"""
import argparse
import json
import math
from pathlib import Path

CUT, ENGRAVE, HAIR = '#ff0000', '#0000ff', 0.01
HERE = Path(__file__).resolve().parent

LENGTH, WIDTH = 200.0, 70.0          # plate outline (the horns of the corner end at x = LENGTH)
X_VERTEX = LENGTH - WIDTH / 2        # 165: the 45 deg notch edges meet the plate sides at the right end
M6_HOLES = [(12.5, 12.5), (12.5, -12.5)]
M6_CLEAR = 6.6
BORE_A, BORE_B = (40.0, 0.0), (125.0, 0.0)
QUIET_MM = 25.0                      # no engraving this close to the vertex


def svg(width, height, body):
    return (f'<svg xmlns="http://www.w3.org/2000/svg" width="{width:.3f}mm" height="{height:.3f}mm" '
            f'viewBox="0 0 {width:.3f} {height:.3f}">\n' + '\n'.join(body) + '\n</svg>\n')


def plate(a):
    k2 = a.kerf / 2
    m = 10.0                                             # sheet margin
    P = lambda x, y: f'{m + x:.4f},{m + WIDTH / 2 - y:.4f}'   # plate frame -> SVG (y down)
    # outline (waste outside): left, top, right horn, notch edge, vertex, notch edge, bottom horn, bottom
    # the notch edges y = +-(x - X_VERTEX) move into the open quadrant by kerf/2: the vertex moves +x by kerf/2 * sqrt 2
    xv = X_VERTEX + k2 * math.sqrt(2)
    top, bot, left, right = WIDTH / 2 + k2, -WIDTH / 2 - k2, -k2, LENGTH + k2
    horn = xv + top                                     # where the notch edge meets y = top
    pts = [(left, bot), (left, top), (min(horn, right), top), (xv, 0.0), (min(horn, right), bot)]
    outline = 'M ' + ' L '.join(P(*p) for p in pts) + ' Z'
    body = [f'<path d="{outline}" fill="none" stroke="{CUT}" stroke-width="{HAIR}"/>']
    for (x, y) in M6_HOLES:
        body.append(f'<circle cx="{m + x:.4f}" cy="{m + WIDTH / 2 - y:.4f}" r="{(M6_CLEAR - a.kerf) / 2:.4f}" '
                    f'fill="none" stroke="{CUT}" stroke-width="{HAIR}"/>')
    for (x, y) in (BORE_A, BORE_B):
        body.append(f'<circle cx="{m + x:.4f}" cy="{m + WIDTH / 2 - y:.4f}" r="{(a.hole - a.kerf) / 2:.4f}" '
                    f'fill="none" stroke="{CUT}" stroke-width="{HAIR}"/>')
    txt = lambda x, y, s, size=3.0: (f'<text x="{m + x:.3f}" y="{m + WIDTH / 2 - y:.3f}" font-family="sans-serif" '
                                     f'font-size="{size}" fill="{ENGRAVE}">{s}</text>')
    body += [txt(BORE_A[0] - 2, -a.hole / 2 - 5, 'A'), txt(BORE_B[0] - 2, -a.hole / 2 - 5, 'B'),
             txt(25, WIDTH / 2 - 6, f'CORNER 270  bores {a.hole:.1f}  kerf {a.kerf:.2f}', 3.0),
             txt(60, -WIDTH / 2 + 4, 'bisector +x  (open quadrant to the right)', 2.5)]
    # engraved bisector between the bores only (everything engraved stays > QUIET_MM from the vertex)
    body.append(f'<line x1="{m + BORE_A[0] + a.hole / 2 + 2:.3f}" y1="{m + WIDTH / 2:.3f}" '
                f'x2="{m + BORE_B[0] - a.hole / 2 - 2:.3f}" y2="{m + WIDTH / 2:.3f}" stroke="{ENGRAVE}" stroke-width="{HAIR}"/>')
    return svg(LENGTH + 2 * m, WIDTH + 2 * m, body)


def coupon(a):
    sizes = [a.hole - 0.4, a.hole - 0.2, a.hole, a.hole + 0.2, a.hole + 0.4]
    pitch, margin = a.hole + 6, 6
    W, H = margin * 2 + pitch * len(sizes), a.hole + 18
    body = [f'<rect x="{margin / 2:.3f}" y="{margin / 2:.3f}" width="{W - margin:.3f}" height="{H - margin:.3f}" '
            f'fill="none" stroke="{CUT}" stroke-width="{HAIR}"/>']
    for i, d in enumerate(sizes):
        cx, cy = margin + pitch * (i + 0.5), H / 2 - 2
        body.append(f'<circle cx="{cx:.3f}" cy="{cy:.3f}" r="{(d - a.kerf) / 2:.4f}" fill="none" stroke="{CUT}" '
                    f'stroke-width="{HAIR}"/>')
        body.append(f'<text x="{cx - 5:.3f}" y="{H - 4:.3f}" font-family="sans-serif" font-size="3" '
                    f'fill="{ENGRAVE}">{d:.1f}</text>')
    body.append(f'<text x="{margin:.3f}" y="{margin + 1:.3f}" font-family="sans-serif" font-size="2.5" '
                f'fill="{ENGRAVE}">bore fit, kerf {a.kerf:.2f} applied</text>')
    return svg(W, H, body)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--hole', type=float, default=23.4, help='finished calibration-bore diameter (mm)')
    ap.add_argument('--kerf', type=float, default=0.15, help='laser kerf width (mm); cut lines move by kerf/2')
    a = ap.parse_args()
    (HERE / 'corner270_plate.svg').write_text(plate(a))
    (HERE / 'hole_fit_coupon.svg').write_text(coupon(a))
    geometry = dict(units='mm', frame='plate: x along the bisector from the left end, y across; vertex open toward +x',
                    vertex=[X_VERTEX, 0.0], bore_a=list(BORE_A), bore_b=list(BORE_B), bore_diameter=a.hole,
                    m6_holes=M6_HOLES, length=LENGTH, width=WIDTH, thickness=6.35, kerf=a.kerf)
    (HERE / 'corner270_plate.json').write_text(json.dumps(geometry, indent=1))
    print('wrote corner270_plate.svg, corner270_plate.json, hole_fit_coupon.svg in', HERE)


if __name__ == '__main__':
    main()
