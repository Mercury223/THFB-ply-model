"""Half-circle blade test — curves only, symmetric hyperbolic spirals.

Both side_a and side_b follow hyperbolic spirals symmetric about the band center:
  θ_center = θ_start + C/(2r)
  θ_a(ρ) = θ_center - C/(2ρ)   (left endpoint)
  θ_b(ρ) = θ_center + C/(2ρ)   (right endpoint)

z(ρ) = R - √(R² - (r+R-ρ)²)  for ρ ∈ [r, r+R]
"""
import math
import cadquery as cq
from pathlib import Path

# ── Parameters ──
BLADE_HEIGHT = 80.0
OUTER_RADIUS = 33.0     # r — blade outer radius
FILLET_RADIUS = 2.0     # R — fillet quarter-circle radius
PLY_EXPAND_OFFSET = 60.0
PLY_LAYER_THICKNESS = 1.0
ANGLE_START = -90.0
ANGLE_END = 90.0
N_BANDS = 3
N_SAMPLES = 40

r_blade = OUTER_RADIUS
R_fillet = FILLET_RADIUS
off = PLY_EXPAND_OFFSET
t = PLY_LAYER_THICKNESS
C = r_blade * math.radians(ANGLE_END - ANGLE_START) / N_BANDS

print(f"Hyperbolic spiral curves test")
print(f"  r (blade radius) = {r_blade}")
print(f"  R (fillet radius) = {R_fillet}")
print(f"  offset = {off}")
print(f"  C (arc length per band) = {C:.4f} mm")
print(f"  bands: {N_BANDS}")
print(f"  blade angle per band: {math.degrees(C/r_blade):.1f}°")
print(f"  angle at ρ=r+R={r_blade+R_fillet}: {math.degrees(C/(r_blade+R_fillet)):.1f}°")
print(f"  angle at ρ=r+R+off={r_blade+R_fillet+off}: {math.degrees(C/(r_blade+R_fillet+off)):.1f}°")


def v3(x, y, z):
    return cq.Vector(float(x), float(y), float(z))


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
        d = pts_3d[i].sub(pts_3d[i + 1]).Length
        if d < 1e-9:
            continue
        edges.append(cq.Edge.makeLine(pts_3d[i], pts_3d[i + 1]))
    return cq.Wire.assembleEdges(edges)


def z_fillet(rho):
    """Quarter-circle height: z = R - √(R² - (r+R-ρ)²) for ρ ∈ [r, r+R]."""
    d = r_blade + R_fillet - rho
    if d < 0:
        d = 0
    if d > R_fillet:
        d = R_fillet
    return R_fillet - math.sqrt(max(0, R_fillet**2 - d**2))


def connector_wire(theta_center_deg, sign):
    """Build spiral connector wire for one side of a band.

    sign = -1 for side_a (left), +1 for side_b (right).
    θ(ρ) = θ_center + sign × C/(2ρ)

    Wire: blade top(z=blade_height) → blade-fillet(z=R) → fillet spiral
          → endwall top(z=0). No endwall bottom drop.
    """
    θ_c = math.radians(theta_center_deg)
    half_C = C / 2.0
    pts = []

    # Blade junction angle
    θ_junction = θ_c + sign * half_C / r_blade

    # 1) Blade top → blade-fillet junction (vertical at ρ=r_blade)
    pts.append(v3(r_blade * math.cos(θ_junction),
                  r_blade * math.sin(θ_junction), BLADE_HEIGHT))
    pts.append(v3(r_blade * math.cos(θ_junction),
                  r_blade * math.sin(θ_junction), R_fillet))

    # 2) Fillet spiral: ρ ∈ [r_blade, r_blade+R_fillet]
    n_fillet = N_SAMPLES // 2
    for i in range(1, n_fillet + 1):
        rho = r_blade + R_fillet * i / n_fillet
        theta = θ_c + sign * half_C / rho
        z = z_fillet(rho)
        pts.append(v3(rho * math.cos(theta), rho * math.sin(theta), z))

    # 3) Endwall top: ρ ∈ [r_blade+R_fillet, r_blade+R_fillet+off], z=0
    n_endwall = N_SAMPLES // 2
    for i in range(1, n_endwall + 1):
        rho = (r_blade + R_fillet) + off * i / n_endwall
        theta = θ_c + sign * half_C / rho
        pts.append(v3(rho * math.cos(theta), rho * math.sin(theta), 0.0))

    return make_polyline_edge(pts)


def make_fillet_revolve(angle0_deg, angle1_deg):
    """Revolve quarter-ring around Z — smooth fillet solid (QYModel approach)."""
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeRevol
    from OCP.gp import gp_Ax1, gp_Pnt, gp_Dir

    θ0, θ1 = float(angle0_deg), float(angle1_deg)
    span = θ1 - θ0
    if span < 0.01:
        return None

    sqrt2 = math.sqrt(2.0)
    r_outer = float(R_fillet)
    r_inner = r_outer + t
    cx, cz = r_blade + r_outer, r_outer

    profile_wp = (
        cq.Workplane("XZ")
        .moveTo(r_blade, r_outer)
        .threePointArc((cx - r_outer / sqrt2, cz - r_outer / sqrt2),
                       (r_blade + r_outer, 0.0))
        .lineTo(r_blade + r_outer, -t)
        .threePointArc((cx - r_inner / sqrt2, cz - r_inner / sqrt2),
                       (r_blade - t, r_outer))
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


def band_solid(θ_c_deg, θ_in0_deg, θ_in1_deg, θ_out0_deg, θ_out1_deg):
    """Three-layer band solid with split at z=R_fillet.

    - Blade: extrude +Z, blade angles (wide), radial sides
    - Fillet: BRepPrimAPI_MakeRevol, spiral angles at fillet-bottom (narrow)
      → split starts at z=R_fillet (blade wide vs fillet narrow)
    - Endwall: extrude -Z, trapezoid (inner=spiral@ρ=r+R, outer=spiral@ρ=r+R+off)
      → continuous with fillet at z=0 (same spiral angles)
    """
    half_C = C / 2.0
    θ_c = math.radians(float(θ_c_deg))
    θ_in0 = float(θ_in0_deg)
    θ_in1 = float(θ_in1_deg)
    θ_out0 = float(θ_out0_deg)
    θ_out1 = float(θ_out1_deg)

    # Spiral angles at fillet bottom (ρ=r+R=35) — used for fillet span & endwall inner
    θ_fb0 = math.degrees(θ_c - half_C / (r_blade + R_fillet))
    θ_fb1 = math.degrees(θ_c + half_C / (r_blade + R_fillet))

    # ── Blade sub-solid (z=R_fillet .. blade_height) ──
    a0r, a1r = math.radians(θ_in0), math.radians(θ_in1)
    blade_outer = make_arc_edge_z(r_blade, θ_in0, θ_in1, 0)
    blade_inner = make_arc_edge_z(r_blade - t, θ_in1, θ_in0, 0)
    bp_s = cq.Vector(r_blade * math.cos(a0r), r_blade * math.sin(a0r), 0)
    bp_e = cq.Vector(r_blade * math.cos(a1r), r_blade * math.sin(a1r), 0)
    ip_s = cq.Vector((r_blade - t) * math.cos(a0r), (r_blade - t) * math.sin(a0r), 0)
    ip_e = cq.Vector((r_blade - t) * math.cos(a1r), (r_blade - t) * math.sin(a1r), 0)
    blade_edges = [blade_outer, cq.Edge.makeLine(bp_e, ip_e),
                   blade_inner, cq.Edge.makeLine(ip_s, bp_s)]
    blade_wire = cq.Wire.assembleEdges(blade_edges)
    blade_face = cq.Face.makeFromWires(blade_wire)
    blade_solid = cq.Workplane("XY").add(blade_face).extrude(
        BLADE_HEIGHT - R_fillet).val()
    blade_solid = blade_solid.translate(cq.Vector(0, 0, R_fillet))

    # ── Fillet sub-solid: BRepPrimAPI_MakeRevol (smooth, no facets) ──
    if R_fillet > 1e-9:
        fillet_solid = make_fillet_revolve(θ_fb0, θ_fb1)
    else:
        fillet_solid = None

    # ── Endwall sub-solid: extrude trapezoid downward ──
    Rinner = r_blade + R_fillet
    Router = r_blade + R_fillet + off

    in_a0r, in_a1r = math.radians(θ_fb0), math.radians(θ_fb1)
    out_a0r, out_a1r = math.radians(θ_out0), math.radians(θ_out1)

    inner_arc = make_arc_edge_z(Rinner, θ_fb0, θ_fb1, 0)
    outer_arc = make_arc_edge_z(Router, θ_out1, θ_out0, 0)
    fb_s0 = cq.Vector(Rinner * math.cos(in_a0r), Rinner * math.sin(in_a0r), 0)
    fb_s1 = cq.Vector(Rinner * math.cos(in_a1r), Rinner * math.sin(in_a1r), 0)
    q0 = cq.Vector(Router * math.cos(out_a0r), Router * math.sin(out_a0r), 0)
    q1 = cq.Vector(Router * math.cos(out_a1r), Router * math.sin(out_a1r), 0)

    endwall_edges = [inner_arc, cq.Edge.makeLine(fb_s1, q1),
                     outer_arc, cq.Edge.makeLine(q0, fb_s0)]
    endwall_wire = cq.Wire.assembleEdges(endwall_edges)
    endwall_face = cq.Face.makeFromWires(endwall_wire)
    endwall_solid = cq.Workplane("XY").add(endwall_face).extrude(-t).val()

    # ── Fuse: blade + fillet + endwall ──
    band = blade_solid
    if fillet_solid is not None and fillet_solid.isValid():
        band = band.fuse(fillet_solid)
    band = band.fuse(endwall_solid)
    return band if band.isValid() else None


def make_arc_edge_z(radius, angle0_deg, angle1_deg, z):
    """Create circular arc edge at given z height (for curve display)."""
    a0 = math.radians(angle0_deg)
    a1 = math.radians(angle1_deg)
    p0 = cq.Vector(radius * math.cos(a0), radius * math.sin(a0), z)
    p1 = cq.Vector(radius * math.cos(a1), radius * math.sin(a1), z)
    mid = 0.5 * (a0 + a1)
    pm = cq.Vector(radius * math.cos(mid), radius * math.sin(mid), z)
    return cq.Edge.makeThreePointArc(p0, pm, p1)


# ═══════════════════════════════════════════════════════════════
# Build curves
# ═══════════════════════════════════════════════════════════════

curves = {}

# Station positions on blade (equal arc-length)
station_angles_deg = [
    ANGLE_START + math.degrees(i * C / r_blade) for i in range(N_BANDS + 1)
]

# Band center angles
band_centers_deg = [
    0.5 * (station_angles_deg[i] + station_angles_deg[i + 1])
    for i in range(N_BANDS)
]

# 1) Station wires (radial: θ fixed, from blade-fillet z=R to expanded z=0)
for idx, ang in enumerate(station_angles_deg):
    curves[f"ply_station_{idx:04d}"] = connector_wire(ang, 0)
    # Also save blade-fillet junction points for top arc
    print(f"  station {idx}: blade θ={ang:.1f}° (radial)")

# 2) Band curves
half_C_deg = math.degrees(C / (2.0 * r_blade))  # half angular span on blade
ρ_end = r_blade + R_fillet + off
half_C_at_end_deg = math.degrees(C / (2.0 * ρ_end))

for idx in range(N_BANDS):
    θ_c = band_centers_deg[idx]   # band center angle
    θ0 = station_angles_deg[idx]  # band start on blade
    θ1 = station_angles_deg[idx + 1]  # band end on blade

    # Blade equal-arc at z=blade_height (blade top, "向上平移和叶身高度一致")
    blade_arc_top = make_arc_edge_z(r_blade, θ0, θ1, BLADE_HEIGHT)
    curves[f"ply_band_{idx:04d}_blade_equal_arc"] = blade_arc_top

    # Expanded equal-arc at z=0 (endwall top)
    θ_a_exp = θ_c - half_C_at_end_deg
    θ_b_exp = θ_c + half_C_at_end_deg
    expanded_arc_z0 = make_arc_edge_z(ρ_end, θ_a_exp, θ_b_exp, 0.0)
    curves[f"ply_band_{idx:04d}_expanded_equal_arc"] = expanded_arc_z0

    # Side A: θ(ρ) = θ_c - C/(2ρ)  (sign=-1)
    curves[f"ply_band_{idx:04d}_side_a"] = connector_wire(θ_c, -1)

    # Side B: θ(ρ) = θ_c + C/(2ρ)  (sign=+1)
    curves[f"ply_band_{idx:04d}_side_b"] = connector_wire(θ_c, +1)

    # Band solid: three-layer with spiral side faces
    solid = band_solid(θ_c, θ0, θ1, θ_a_exp, θ_b_exp)
    if solid is not None and solid.isValid():
        curves[f"ply_band_{idx:04d}_solid"] = solid

    print(f"  band {idx}: center={θ_c:.1f}° "
          f"blade[{θ0:.1f}°→{θ1:.1f}°] span={θ1-θ0:.1f}° "
          f"expanded[{θ_a_exp:.1f}°→{θ_b_exp:.1f}°] span={θ_b_exp-θ_a_exp:.1f}° "
          f"solid={'OK' if solid is not None and solid.isValid() else 'FAIL'}")

# ═══════════════════════════════════════════════════════════════
# Export STEP
# ═══════════════════════════════════════════════════════════════

current_dir = Path(__file__).resolve().parent
step_path = current_dir / "halfcircle_curves.step"

assy = cq.Assembly(name="HALFCIRCLE_CURVES")

for name, item in curves.items():
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
    elif "solid" in name:
        hue = int(name.split("_")[2]) / max(N_BANDS - 1, 1)
        import colorsys
        rc, gc, bc = colorsys.hsv_to_rgb(hue, 0.8, 0.85)
        color = cq.Color(rc, gc, bc)
    else:
        color = cq.Color(1.0, 1.0, 1.0)
    assy.add(item, name=name, color=color)

if step_path.exists():
    step_path.unlink()
assy.export(str(step_path))
print(f"\n[STEP exported] {step_path}")
print(f"  curves: {len(curves)}")

# Verification
print(f"\n[Verification — symmetric spirals]")
for idx in range(N_BANDS):
    θ_c = band_centers_deg[idx]
    half_C = C / 2.0
    for label, rho in [("r (blade)", r_blade),
                        ("r+R (fillet bottom)", r_blade + R_fillet),
                        ("r+R+off (expanded)", r_blade + R_fillet + off)]:
        θ_a = θ_c - math.degrees(half_C / rho)
        θ_b = θ_c + math.degrees(half_C / rho)
        print(f"  band {idx} at ρ={rho:.0f}: side_a={θ_a:.1f}° side_b={θ_b:.1f}° span={θ_b-θ_a:.1f}°")
