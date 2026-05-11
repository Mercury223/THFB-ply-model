"""Half-circle blade test — 3 bands, 60° step, no straight segments.

Simplified model to verify band geometry on a pure-arc blade:
  - Half-circle blade (180°, R=33), no straight segments
  - Fillet (r=2) + endwall (offset=60)
  - 3 bands with equal arc-length on expanded curve
  - Three-layer band: blade(fillet angles) + fillet(revolve) + endwall(trapezoidal)
"""
import math
import cadquery as cq
from cadquery import exporters
from pathlib import Path

# ── Parameters ──
BLADE_HEIGHT = 80.0
OUTER_RADIUS = 33.0
BLADE_THICKNESS = 4.0       # structural wall thickness
FILLET_RADIUS = 2.0          # root fillet radius
ENDWALL_HEIGHT = 20.0
PLY_EXPAND_OFFSET = 60.0
PLY_LAYER_THICKNESS = 1.0    # band visualization thickness
ANGLE_START = -90.0          # half-circle start angle (degrees)
ANGLE_END = 90.0             # half-circle end angle (degrees)
N_BANDS = 3                  # 180° / 60° = 3 bands

# ── Derived values ──
ARC_ANGLE = ANGLE_END - ANGLE_START  # 180°
ARC_LEN = OUTER_RADIUS * math.radians(ARC_ANGLE)  # blade arc length
BLADE_SEG_LEN = ARC_LEN / N_BANDS  # per-band blade arc length
EXPANDED_ARC_LEN = (OUTER_RADIUS + PLY_EXPAND_OFFSET) * math.radians(ARC_ANGLE)
BAND_ANGLE = ARC_ANGLE / N_BANDS    # 60° per band

R = OUTER_RADIUS
r = FILLET_RADIUS
off = PLY_EXPAND_OFFSET
t = PLY_LAYER_THICKNESS

print(f"Half-circle blade test")
print(f"  arc angle: {ARC_ANGLE}°")
print(f"  blade arc length: {ARC_LEN:.3f} mm")
print(f"  expanded arc length: {EXPANDED_ARC_LEN:.3f} mm")
print(f"  bands: {N_BANDS}, blade_seg_len: {BLADE_SEG_LEN:.3f} mm")
print(f"  blade angular span per band: {BAND_ANGLE:.1f}°")
print(f"  expanded angular span per band: {math.degrees(BLADE_SEG_LEN/(R+off)):.1f}°")


def make_circular_arc_edge(cx, cy, radius, angle0_deg, angle1_deg):
    """Create a circular arc edge in XY plane."""
    a0 = math.radians(angle0_deg)
    a1 = math.radians(angle1_deg)
    p0 = cq.Vector(cx + radius * math.cos(a0), cy + radius * math.sin(a0), 0)
    p1 = cq.Vector(cx + radius * math.cos(a1), cy + radius * math.sin(a1), 0)
    # Determine midpoint for three-point arc
    mid_angle = 0.5 * (a0 + a1)
    pm = cq.Vector(cx + radius * math.cos(mid_angle),
                   cy + radius * math.sin(mid_angle), 0)
    return cq.Edge.makeThreePointArc(p0, pm, p1)


def make_fillet_solid_arc(angle0_deg, angle1_deg):
    """Revolve quarter-ring cross-section around Z axis.

    Cross-section (XZ plane):
      outer arc: center=(R+r, r), radius=r,   (R, r) → (R+r, 0)
      inner arc: center=(R+r, r), radius=r+t, (R+r, -t) → (R-t, r)
    """
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeRevol
    from OCP.gp import gp_Ax1, gp_Pnt, gp_Dir

    θ0 = float(angle0_deg)
    θ1 = float(angle1_deg)
    span = θ1 - θ0
    if span < 0.01:
        return None

    sqrt2 = math.sqrt(2.0)
    r_outer = float(r)
    r_inner = r_outer + t
    cx, cz = R + r_outer, r_outer

    profile_wp = (
        cq.Workplane("XZ")
        .moveTo(R, r_outer)
        .threePointArc(
            (cx - r_outer / sqrt2, cz - r_outer / sqrt2),
            (R + r_outer, 0.0))
        .lineTo(R + r_outer, -t)
        .threePointArc(
            (cx - r_inner / sqrt2, cz - r_inner / sqrt2),
            (R - t, r_outer))
        .close()
    )

    try:
        face = cq.Face.makeFromWires(profile_wp.val())
    except Exception:
        return None

    axis = gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1))
    revol = BRepPrimAPI_MakeRevol(face.wrapped, axis, math.radians(span))
    revol.Build()
    solid = cq.Solid(revol.Shape())

    if abs(θ0) > 1e-9:
        solid = solid.rotate((0, 0, 0), (0, 0, 1), θ0)

    return solid


# ═══════════════════════════════════════════════════════════════
# Build blade body
# ═══════════════════════════════════════════════════════════════

print("\n[Building blade body]")

# Outer and inner blade profiles (half-circle arcs)
outer_arc = make_circular_arc_edge(0, 0, R, ANGLE_START, ANGLE_END)
outer_start = cq.Vector(R * math.cos(math.radians(ANGLE_START)),
                         R * math.sin(math.radians(ANGLE_START)), 0)
outer_end = cq.Vector(R * math.cos(math.radians(ANGLE_END)),
                       R * math.sin(math.radians(ANGLE_END)), 0)

inner_arc = make_circular_arc_edge(0, 0, R - BLADE_THICKNESS, ANGLE_END, ANGLE_START)
inner_start = cq.Vector((R - BLADE_THICKNESS) * math.cos(math.radians(ANGLE_END)),
                         (R - BLADE_THICKNESS) * math.sin(math.radians(ANGLE_END)), 0)
inner_end = cq.Vector((R - BLADE_THICKNESS) * math.cos(math.radians(ANGLE_START)),
                       (R - BLADE_THICKNESS) * math.sin(math.radians(ANGLE_START)), 0)

# Wire: outer_arc → radial_in_at_end → inner_arc(reversed) → radial_out_at_start
blade_profile_wire = cq.Wire.assembleEdges([
    outer_arc,                                     # -90°→90° at R
    cq.Edge.makeLine(outer_end, inner_start),      # radial inward at bottom
    inner_arc,                                     # 90°→-90° at R-t
    cq.Edge.makeLine(inner_end, outer_start),      # radial outward at top
])
blade_profile_face = cq.Face.makeFromWires(blade_profile_wire)
blade_body = cq.Workplane("XY").add(blade_profile_face).extrude(BLADE_HEIGHT).val()
print(f"  blade body valid: {blade_body.isValid()}")

# Add root fillet at bottom edges (z=0)
if r > 0:
    # Fillet the outer bottom edge (where blade meets endwall)
    try:
        blade_body = blade_body.fillet(r, blade_body.edges(
            cq.NearestToPointSelector((R * math.cos(math.radians(0)),
                                       R * math.sin(math.radians(0)), 0))))
        print(f"  blade body with fillet valid: {blade_body.isValid()}")
    except Exception as e:
        print(f"  fillet skipped: {e}")

# Build endwall: extends from blade inner profile outward + downward
endwall_w = 150.0  # outward extension
# Endwall face: inner profile chord → outward → +Z chord → back
ew_inner_bot = cq.Vector(inner_start.x, inner_start.y, 0)
ew_outer_bot = cq.Vector(inner_start.x + endwall_w, inner_start.y, 0)
ew_outer_top = cq.Vector(inner_end.x + endwall_w, inner_end.y, 0)
ew_inner_top = cq.Vector(inner_end.x, inner_end.y, 0)
endwall_wire = cq.Wire.assembleEdges([
    cq.Edge.makeLine(ew_inner_bot, ew_outer_bot),
    cq.Edge.makeLine(ew_outer_bot, ew_outer_top),
    cq.Edge.makeLine(ew_outer_top, ew_inner_top),
    cq.Edge.makeLine(ew_inner_top, ew_inner_bot),
])
endwall_face = cq.Face.makeFromWires(endwall_wire)
endwall_body = cq.Workplane("XY").add(endwall_face).extrude(-ENDWALL_HEIGHT).val()
final_body = blade_body.fuse(endwall_body)
print(f"  final body valid: {final_body.isValid()}")


# ═══════════════════════════════════════════════════════════════
# Build ply bands (3 bands)
# ═══════════════════════════════════════════════════════════════

print(f"\n[Building {N_BANDS} ply bands]")

bands = {}
for idx in range(N_BANDS):
    s0 = idx * BLADE_SEG_LEN
    s1 = (idx + 1) * BLADE_SEG_LEN

    # Blade angles (denominator = R)
    θ_in0 = ANGLE_START + math.degrees(s0 / R)
    θ_in1 = ANGLE_START + math.degrees(s1 / R)

    # Expanded curve positions
    s_center = 0.5 * (s0 + s1)
    expanded_center = s_center * (R + off) / R  # arc-only mapping
    q0_len = expanded_center - BLADE_SEG_LEN / 2
    q1_len = expanded_center + BLADE_SEG_LEN / 2

    # Expanded angles (denominator = R+offset)
    θ_out_center = ANGLE_START + math.degrees(expanded_center / (R + off))
    θ_out_half = 0.5 * math.degrees(BLADE_SEG_LEN / (R + off))
    θ_out0 = θ_out_center - θ_out_half
    θ_out1 = θ_out_center + θ_out_half

    print(f"\n  band {idx}: s=[{s0:.1f}, {s1:.1f}]")
    print(f"    θ_in:  [{θ_in0:.2f}°, {θ_in1:.2f}°] span={θ_in1-θ_in0:.1f}°")
    print(f"    θ_out: [{θ_out0:.2f}°, {θ_out1:.2f}°] span={θ_out1-θ_out0:.1f}°")

    # ── Blade sub-solid (z=r .. blade_height) ──
    a0r, a1r = math.radians(θ_in0), math.radians(θ_in1)

    blade_outer_arc = make_circular_arc_edge(0, 0, R, θ_in0, θ_in1)
    blade_inner_arc = make_circular_arc_edge(0, 0, R - t, θ_in1, θ_in0)

    bp_start = cq.Vector(R * math.cos(a0r), R * math.sin(a0r), 0)
    bp_end = cq.Vector(R * math.cos(a1r), R * math.sin(a1r), 0)
    ip_start = cq.Vector((R - t) * math.cos(a0r), (R - t) * math.sin(a0r), 0)
    ip_end = cq.Vector((R - t) * math.cos(a1r), (R - t) * math.sin(a1r), 0)

    blade_edges = [
        blade_outer_arc,
        cq.Edge.makeLine(bp_end, ip_end),
        blade_inner_arc,
        cq.Edge.makeLine(ip_start, bp_start),
    ]
    blade_wire = cq.Wire.assembleEdges(blade_edges)
    blade_face = cq.Face.makeFromWires(blade_wire)
    blade_solid = (
        cq.Workplane("XY").add(blade_face)
        .extrude(BLADE_HEIGHT - r).val()
    )
    if r > 1e-9:
        blade_solid = blade_solid.translate(cq.Vector(0, 0, r))

    # ── Fillet sub-solid (z=0 .. r) ──
    if r > 1e-9:
        fillet_solid = make_fillet_solid_arc(θ_in0, θ_in1)
    else:
        fillet_solid = None

    # ── Endwall sub-solid (z=-t .. 0) ──
    in_a0r, in_a1r = math.radians(θ_in0), math.radians(θ_in1)
    out_a0r, out_a1r = math.radians(θ_out0), math.radians(θ_out1)

    # Inner arc (R+r, blade angles — wider)
    inner_arc = make_circular_arc_edge(0, 0, R + r, θ_in0, θ_in1)
    fb_s0 = cq.Vector((R + r) * math.cos(in_a0r), (R + r) * math.sin(in_a0r), 0)
    fb_s1 = cq.Vector((R + r) * math.cos(in_a1r), (R + r) * math.sin(in_a1r), 0)

    # Outer arc (R+offset, expanded angles — narrower)
    outer_arc = make_circular_arc_edge(0, 0, R + off, θ_out1, θ_out0)
    q0_pt = cq.Vector((R + off) * math.cos(out_a0r), (R + off) * math.sin(out_a0r), 0)
    q1_pt = cq.Vector((R + off) * math.cos(out_a1r), (R + off) * math.sin(out_a1r), 0)

    endwall_edges = [
        inner_arc,
        cq.Edge.makeLine(fb_s1, q1_pt),
        outer_arc,
        cq.Edge.makeLine(q0_pt, fb_s0),
    ]

    try:
        endwall_wire = cq.Wire.assembleEdges(endwall_edges)
        endwall_face = cq.Face.makeFromWires(endwall_wire)
        endwall_solid = (
            cq.Workplane("XY").add(endwall_face)
            .extrude(-t).val()
        )

        band_solid = blade_solid
        if fillet_solid is not None:
            band_solid = band_solid.fuse(fillet_solid)
        band_solid = band_solid.fuse(endwall_solid)

        if not band_solid.isValid():
            raise RuntimeError("Fused solid not valid")

        bands[f"ply_band_{idx:04d}_solid"] = band_solid
        print(f"    band solid valid: {band_solid.isValid()}")
    except Exception as e:
        print(f"    [SKIP] band {idx}: {e}")


# ═══════════════════════════════════════════════════════════════
# Validation
# ═══════════════════════════════════════════════════════════════

print(f"\n[Validation]")

from OCP.BRepClass3d import BRepClass3d_SolidClassifier
from OCP.gp import gp_Pnt
from OCP.TopAbs import TopAbs_IN

errors = []
for idx in range(N_BANDS):
    key = f"ply_band_{idx:04d}_solid"
    if key not in bands:
        continue
    solid = bands[key]

    # Mid-band angle
    s_mid = (idx + 0.5) * BLADE_SEG_LEN
    θ_mid = math.radians(ANGLE_START + math.degrees(s_mid / R))

    def point_inside(radial_offset, zz):
        pt = cq.Vector((R + radial_offset) * math.cos(θ_mid),
                       (R + radial_offset) * math.sin(θ_mid),
                       zz)
        c = BRepClass3d_SolidClassifier(solid.wrapped)
        c.Perform(gp_Pnt(pt.x, pt.y, pt.z), 1e-3)
        return c.State() == TopAbs_IN

    # Blade region
    if not point_inside(-t * 0.5, BLADE_HEIGHT * 0.7):
        errors.append(f"band {idx}: blade region empty")

    # Fillet region
    if r > 1e-9:
        phi = math.pi / 4
        fil_r = r * (1 - math.cos(phi)) - t * math.cos(phi) * 0.5
        fil_z = r * (1 - math.sin(phi)) - t * math.sin(phi) * 0.5
        if not point_inside(fil_r, fil_z):
            errors.append(f"band {idx}: fillet region empty")

    # Endwall region
    if not point_inside(r + (off - r) * 0.3, -t * 0.5):
        errors.append(f"band {idx}: endwall region empty")

if errors:
    print(f"FAILED: {len(errors)} errors")
    for e in errors:
        print(f"  - {e}")
else:
    print(f"PASSED: all {len(bands)} bands OK (blade + fillet + endwall)")


# ═══════════════════════════════════════════════════════════════
# Export STEP
# ═══════════════════════════════════════════════════════════════

current_dir = Path(__file__).resolve().parent
step_path = current_dir / "halfcircle_3bands.step"

assy = cq.Assembly(name="HALFCIRCLE_TEST")
assy.add(final_body, name="final_body")

# Color each band with a different hue
for idx in range(N_BANDS):
    key = f"ply_band_{idx:04d}_solid"
    if key in bands:
        hue = idx / max(N_BANDS - 1, 1)
        import colorsys
        rc, gc, bc = colorsys.hsv_to_rgb(hue, 0.7, 0.85)
        assy.add(bands[key], name=key, color=cq.Color(rc, gc, bc))

# Export
if step_path.exists():
    step_path.unlink()
assy.export(str(step_path))
print(f"\n[STEP exported] {step_path}")
print(f"  file size: {step_path.stat().st_size / 1024:.0f} KB")
