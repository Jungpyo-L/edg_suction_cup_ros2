#!/usr/bin/env python3
"""Laser-cut centring marker for the concave (270 deg) corner object, from one 1/4 in (6.35 mm) acrylic sheet.

  python3 fixtures/corner_marker/make_corner_marker.py [--hole 23.4] [--kerf 0.15] [--size 70]

Writes corner270_marker.svg and hole_fit_coupon.svg next to this script (mm; red = cut, blue = engrave).

The marker is a square plate with the open quadrant of the corner removed, so it has the same outline as the object top
around the vertex. Laid on the object with its two notch edges flush with the object's two walls, its Ø`hole` bore is
centred on the vertex. The bore keeps 270 deg of its wall (the open quadrant is cut away), which is more than 180 deg,
so the cup lowered into it is centred in every direction: the cup sits in the bore and its centre is the vertex to
within half the diametral clearance (and 1/cos 45 of that toward the open side).

Kerf: cut lines are offset by kerf/2 so the finished part has the drawn size (bore = hole, notch edges on the vertex).
Cut hole_fit_coupon.svg first from the same sheet and pick the smallest finished bore the cup enters without force.

Frames: marker x points from the vertex into the open quadrant (= the corner bisector, robot +X in the lateral
characterization), y to the left of it seen from above. The object occupies everything except |angle| < 45 deg.
"""
import argparse
import math
from pathlib import Path

CUT, ENGRAVE, HAIR = '#ff0000', '#0000ff', 0.01
HERE = Path(__file__).resolve().parent


def svg(width, height, body):
    return (f'<svg xmlns="http://www.w3.org/2000/svg" width="{width:.3f}mm" height="{height:.3f}mm" '
            f'viewBox="0 0 {width:.3f} {height:.3f}">\n' + '\n'.join(body) + '\n</svg>\n')


def line_intersection(p, d, q, e):
    """Intersection of p + t d and q + s e."""
    den = d[0] * e[1] - d[1] * e[0]
    t = ((q[0] - p[0]) * e[1] - (q[1] - p[1]) * e[0]) / den
    return (p[0] + t * d[0], p[1] + t * d[1])


def marker(a):
    h = a.size / 2
    k2 = a.kerf / 2
    r = (a.hole - a.kerf) / 2                      # cut radius: the kerf widens the bore to `hole`
    c = math.sqrt(0.5)
    # notch edges (rays at +-45 deg from the vertex), moved into the open quadrant by kerf/2 so the finished plate
    # edges pass through the vertex; the square outline moves out by kerf/2
    up_dir, up_pt = (c, c), (k2 * c, -k2 * c)      # +45 deg ray, normal into the open side (c, -c)
    dn_dir, dn_pt = (c, -c), (k2 * c, k2 * c)      # -45 deg ray, normal (c, c)
    H = h + k2
    up_end = line_intersection(up_pt, up_dir, (H, 0), (0, 1))      # ray meets the right side x = H
    dn_end = line_intersection(dn_pt, dn_dir, (H, 0), (0, 1))
    if up_end[1] > H:                                              # ray leaves through the top side instead
        up_end = line_intersection(up_pt, up_dir, (0, H), (1, 0))
        dn_end = (up_end[0], -up_end[1])

    def ray_circle(p, d):                          # point at distance r from the bore centre along the ray
        b = p[0] * d[0] + p[1] * d[1]
        cc = p[0] ** 2 + p[1] ** 2 - r * r
        t = -b + math.sqrt(b * b - cc)
        return (p[0] + t * d[0], p[1] + t * d[1])
    up_c, dn_c = ray_circle(up_pt, up_dir), ray_circle(dn_pt, dn_dir)

    W = 2 * H + 20                                 # sheet window with 10 mm margin
    ox, oy = W / 2, W / 2                          # vertex in SVG coordinates (y down)
    X = lambda p: f'{ox + p[0]:.4f},{oy - p[1]:.4f}'
    # one closed cut: bore arc (long way round, through 180 deg) -> +45 ray -> square -> -45 ray
    path = (f'M {X(dn_c)} A {r:.4f},{r:.4f}  0 1 1 {X(up_c)} '   # sweep 1: clockwise on the y-down page = the long way through 180 deg
            f'L {X(up_end)} '
            + ('' if up_end[0] < H - 1e-9 else f'L {X((H, H))} ')
            + f'L {X((-H, H))} L {X((-H, -H))} '
            + ('' if dn_end[0] < H - 1e-9 else f'L {X((H, -H))} ')
            + f'L {X(dn_end)} L {X(dn_c)} Z')
    body = [f'<path d="{path}" fill="none" stroke="{CUT}" stroke-width="{HAIR}"/>']
    # engraving: object-side bisector (-X) from the bore to the edge, 30 deg ticks round the bore, labels
    body.append(f'<line x1="{ox - a.hole / 2 - 1:.3f}" y1="{oy:.3f}" x2="{ox - H + 3:.3f}" y2="{oy:.3f}" '
                f'stroke="{ENGRAVE}" stroke-width="{HAIR}"/>')
    for deg in range(60, 301, 30):
        t = math.radians(deg)
        p1 = (math.cos(t) * (a.hole / 2 + 1), math.sin(t) * (a.hole / 2 + 1))
        p2 = (math.cos(t) * (a.hole / 2 + (4 if deg % 90 == 0 else 2.5)), math.sin(t) * (a.hole / 2 + (4 if deg % 90 == 0 else 2.5)))
        body.append(f'<line x1="{ox + p1[0]:.3f}" y1="{oy - p1[1]:.3f}" x2="{ox + p2[0]:.3f}" y2="{oy - p2[1]:.3f}" '
                    f'stroke="{ENGRAVE}" stroke-width="{HAIR}"/>')
    txt = lambda x, y, s, size=3.0: (f'<text x="{ox + x:.3f}" y="{oy - y:.3f}" font-family="sans-serif" font-size="{size}" '
                                     f'fill="{ENGRAVE}">{s}</text>')
    body += [txt(-H + 3, -H + 5, f'CORNER 270  bore {a.hole:.1f}  kerf {a.kerf:.2f}', 3.0),
             txt(-H + 3, 4, '-X (object side)', 2.5),
             txt(-H + 3, H - 6, 'notch edges flush with the object walls', 2.5),
             txt(4, H - 6, '+X: open quadrant', 2.5)]
    return svg(W, W, body)


def coupon(a):
    sizes = [a.hole - 0.4, a.hole - 0.2, a.hole, a.hole + 0.2, a.hole + 0.4]
    pitch, margin = a.hole + 6, 6
    W, Hh = margin * 2 + pitch * len(sizes), a.hole + 18
    body = [f'<rect x="{margin / 2:.3f}" y="{margin / 2:.3f}" width="{W - margin:.3f}" height="{Hh - margin:.3f}" '
            f'fill="none" stroke="{CUT}" stroke-width="{HAIR}"/>']
    for i, d in enumerate(sizes):
        cx, cy = margin + pitch * (i + 0.5), Hh / 2 - 2
        body.append(f'<circle cx="{cx:.3f}" cy="{cy:.3f}" r="{(d - a.kerf) / 2:.4f}" fill="none" stroke="{CUT}" '
                    f'stroke-width="{HAIR}"/>')
        body.append(f'<text x="{cx - 5:.3f}" y="{Hh - 4:.3f}" font-family="sans-serif" font-size="3" '
                    f'fill="{ENGRAVE}">{d:.1f}</text>')
    body.append(f'<text x="{margin:.3f}" y="{margin + 1:.3f}" font-family="sans-serif" font-size="2.5" '
                f'fill="{ENGRAVE}">bore fit, kerf {a.kerf:.2f} applied</text>')
    return svg(W, Hh, body)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--hole', type=float, default=23.4, help='finished bore diameter for the cup (mm)')
    ap.add_argument('--kerf', type=float, default=0.15, help='laser kerf width (mm); cut lines move by kerf/2')
    ap.add_argument('--size', type=float, default=70.0, help='marker square side (mm)')
    a = ap.parse_args()
    (HERE / 'corner270_marker.svg').write_text(marker(a))
    (HERE / 'hole_fit_coupon.svg').write_text(coupon(a))
    print('wrote', HERE / 'corner270_marker.svg', 'and', HERE / 'hole_fit_coupon.svg')


if __name__ == '__main__':
    main()
