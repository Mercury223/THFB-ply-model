"""Half-circle blade test — curves only, no solids.

Simplified model: half-circle blade (180°, R=33), no straight segments.
Generate ply reference curves: stations, blade_equal_arc, expanded_equal_arc, side_a, side_b.
"""
import math
import cadquery as cq
from pathlib import Path

# ── Parameters ──
BLADE_HEIGHT = 80.0
OUTER_RADIUS = 33.0
BLADE_THICKNESS = 4.0
FILLET_RADIUS = 2.0
PLY_EXPAND_OFFSET = 60.0
PLY_LAYER_THICKNESS = 1.0
ANGLE_START = -90.0
ANGLE_END = 90.0
N_BANDS = 3
PLY_FILLET_SAMPLES = 12

R = OUTER_RADIUS
r = FILLET_RADIUS
off = PLY_EXPAND_OFFSET
t = PLY_LAYER_THICKNESS
ARC_LEN = R * math.radians(ANGLE_END - ANGLE_START)
BLADE_SEG_LEN = ARC_LEN / N_BANDS
ARC_ANGLE_RAD = math.radians(ANGLE_END - ANGLE_START)

print(f"Half-circle curves test")
print(f"  arc: {ANGLE_START}° → {ANGLE_END}°")
print(f"  blade arc length: {ARC_LEN:.3f} mm")
print(f"  expanded arc length: {(R+off) * ARC_ANGLE_RAD:.3f} mm")
print(f"  bands: {N_BANDS}, seg_len: {BLADE_SEG_LEN:.3f} mm")
print(f"  blade angle per band: {ANGLE_END-ANGLE_START:.0f}°/{N_BANDS} = {math.degrees(BLADE_SEG_LEN/R):.1f}°")
print(f"  expanded angle per band: {math.degrees(BLADE_SEG_LEN/(R+off)):.1f}°")


def v3(p, z):
    return cq.Vector(float(p.x), float(p.y), float(z))


def make_arc_edge(cx, cy, radius, angle0_deg, angle1_deg):
    a0 = math.radians(angle0_deg)
    a1 = math.radians(angle1_deg)
    p0 = cq.Vector(cx + radius * math.cos(a0), cy + radius * math.sin(a0), 0)
    p1 = cq.Vector(cx + radius * math.cos(a1), cy + radius * math.sin(a1), 0)
    mid = 0.5 * (a0 + a1)
    pm = cq.Vector(cx + radius * math.cos(mid), cy + radius * math.sin(mid), 0)
    return cq.Edge.makeThreePointArc(p0, pm, p1)


def make_polyline_edge(pts_3d):
    edges = []
    for i in range(len(pts_3d) - 1):
        edges.append(cq.Edge.makeLine(pts_3d[i], pts_3d[i + 1]))
    return cq.Wire.assembleEdges(edges)


def connector_wire(root_xy, end_xy, normal_xy):
    """Build connector wire from blade top → blade-fillet → fillet surface → expanded → endwall bottom."""
    pts = [
        v3(root_xy, BLADE_HEIGHT),   # blade top
        v3(root_xy, r),              # blade-fillet junction
    ]
    # Follow fillet quarter-circle along normal direction
    n = max(3, PLY_FILLET_SAMPLES)
    # Skip last sample (i=n) to avoid duplicating endwall_start
    for i in range(1, n):
        theta = 0.5 * math.pi * i / n
        rad = r * (1.0 - math.cos(theta))
        z = r * (1.0 - math.sin(theta))
        q = cq.Vector(
            root_xy.x + normal_xy.x * rad,
            root_xy.y + normal_xy.y * rad, 0.0)
        pts.append(v3(q, z))
    # Fillet bottom = endwall top (z=0, R+r along normal)
    ew_start = cq.Vector(
        root_xy.x + normal_xy.x * r,
        root_xy.y + normal_xy.y * r, 0.0)
    pts.append(v3(ew_start, 0.0))
    # Expanded curve point at z=0
    pts.append(v3(end_xy, 0.0))
    # Endwall bottom
    pts.append(v3(end_xy, -t))
    return make_polyline_edge(pts)


# ═══════════════════════════════════════════════════════════════
# Station points on blade profile
# ═══════════════════════════════════════════════════════════════

stations_s = [i * BLADE_SEG_LEN for i in range(N_BANDS + 1)]
stations = []
for s in stations_s:
    angle_deg = ANGLE_START + math.degrees(s / R)
    angle_rad = math.radians(angle_deg)
    pt = cq.Vector(R * math.cos(angle_rad), R * math.sin(angle_rad), 0)
    normal = cq.Vector(math.cos(angle_rad), math.sin(angle_rad), 0)
    stations.append((s, angle_deg, pt, normal))

# ═══════════════════════════════════════════════════════════════
# Expanded curve points
# ═══════════════════════════════════════════════════════════════

def map_to_expanded(s):
    """Map blade arc position to expanded curve position (arc-only)."""
    return s * (R + off) / R

expanded_pts = []
for s in stations_s:
    exp_s = map_to_expanded(s)
    angle_deg = ANGLE_START + math.degrees(exp_s / (R + off))
    angle_rad = math.radians(angle_deg)
    pt = cq.Vector((R + off) * math.cos(angle_rad),
                   (R + off) * math.sin(angle_rad), 0)
    expanded_pts.append((exp_s, angle_deg, pt))

# ═══════════════════════════════════════════════════════════════
# Build curves
# ═══════════════════════════════════════════════════════════════

curves = {}

# 1) Station connector wires
for idx, (s, angle_deg, pt, normal) in enumerate(stations):
    exp_s, exp_angle, exp_pt = expanded_pts[idx]
    curves[f"ply_station_{idx:04d}"] = connector_wire(pt, exp_pt, normal)

# 2) Band curves: blade_equal_arc, expanded_equal_arc, side_a, side_b
for idx in range(N_BANDS):
    s0_s, s0_angle, s0_pt, s0_n = stations[idx]
    s1_s, s1_angle, s1_pt, s1_n = stations[idx + 1]
    exp0_s, exp0_angle, exp0_pt = expanded_pts[idx]
    exp1_s, exp1_angle, exp1_pt = expanded_pts[idx + 1]

    # Blade equal-arc segment
    blade_arc = make_arc_edge(0, 0, R, s0_angle, s1_angle)
    curves[f"ply_band_{idx:04d}_blade_equal_arc"] = blade_arc

    # Expanded equal-arc segment
    exp_center_s = map_to_expanded(0.5 * (s0_s + s1_s))
    q0_s = exp_center_s - BLADE_SEG_LEN / 2
    q1_s = exp_center_s + BLADE_SEG_LEN / 2
    q0_angle = ANGLE_START + math.degrees(q0_s / (R + off))
    q1_angle = ANGLE_START + math.degrees(q1_s / (R + off))
    expanded_arc = make_arc_edge(0, 0, R + off, q0_angle, q1_angle)
    curves[f"ply_band_{idx:04d}_expanded_equal_arc"] = expanded_arc

    # Side A and Side B connector wires
    q0_pt = cq.Vector((R + off) * math.cos(math.radians(q0_angle)),
                      (R + off) * math.sin(math.radians(q0_angle)), 0)
    q1_pt = cq.Vector((R + off) * math.cos(math.radians(q1_angle)),
                      (R + off) * math.sin(math.radians(q1_angle)), 0)

    curves[f"ply_band_{idx:04d}_side_a"] = connector_wire(s0_pt, q0_pt, s0_n)
    curves[f"ply_band_{idx:04d}_side_b"] = connector_wire(s1_pt, q1_pt, s1_n)
    print(f"  band {idx}: blade [{s0_angle:.1f}°→{s1_angle:.1f}°] "
          f"expanded [{q0_angle:.1f}°→{q1_angle:.1f}°]")

# ═══════════════════════════════════════════════════════════════
# Export STEP
# ═══════════════════════════════════════════════════════════════

current_dir = Path(__file__).resolve().parent
step_path = current_dir / "halfcircle_curves.step"

assy = cq.Assembly(name="HALFCIRCLE_CURVES")

# Color by type
for name, wire in curves.items():
    if "station" in name:
        color = cq.Color(1.0, 1.0, 0.0)  # yellow
    elif "side_a" in name:
        color = cq.Color(1.0, 0.0, 0.0)  # red
    elif "side_b" in name:
        color = cq.Color(0.0, 0.0, 1.0)  # blue
    elif "blade_equal_arc" in name:
        color = cq.Color(0.0, 1.0, 0.0)  # green
    elif "expanded_equal_arc" in name:
        color = cq.Color(1.0, 0.5, 0.0)  # orange
    else:
        color = cq.Color(1.0, 1.0, 1.0)
    assy.add(wire, name=name, color=color)

if step_path.exists():
    step_path.unlink()
assy.export(str(step_path))
print(f"\n[STEP exported] {step_path}")
print(f"  curves: {len(curves)}")

# Print key coordinates for verification
print(f"\n[Verification points]")
for idx in range(N_BANDS + 1):
    s, angle, pt, _ = stations[idx]
    exp_s, exp_angle, exp_pt = expanded_pts[idx]
    print(f"  station {idx}: s={s:.1f}mm, blade({pt.x:.2f},{pt.y:.2f}) "
          f"→ expanded({exp_pt.x:.2f},{exp_pt.y:.2f})")
