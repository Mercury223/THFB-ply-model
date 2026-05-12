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


def thickened_wire(theta_center_deg, sign, inner=False):
    """Generate thickened side wire: inner (blade+fillet+endwall surface) or outer (offset by t).

    sign=-1 for side_a, +1 for side_b.
    inner=True: on blade+fillet+endwall outer surface (band outer face).
    inner=False: offset inward/downward by t (band inner face).
    """
    θ_c = math.radians(theta_center_deg)
    half_C = C / 2.0
    pts = []

    # Blade junction angle (on band outer surface at ρ=r_blade)
    θ_junction_outer = θ_c + sign * half_C / r_blade

    # Blade portion: ρ=r_blade, z from blade_height down to R_fillet
    if inner:
        # Offset inward: ρ = r_blade - t
        ρ_blade = r_blade - t
        θ_junction = θ_c + sign * half_C / ρ_blade
    else:
        ρ_blade = r_blade
        θ_junction = θ_junction_outer

    pts.append(v3(ρ_blade * math.cos(θ_junction),
                  ρ_blade * math.sin(θ_junction), BLADE_HEIGHT))
    pts.append(v3(ρ_blade * math.cos(θ_junction),
                  ρ_blade * math.sin(θ_junction), R_fillet))

    # Fillet portion: ρ from r_blade to r_blade+R_fillet
    n_f = N_SAMPLES // 2
    for i in range(1, n_f):
        if inner:
            # Inner quarter-circle radius
            z = R_fillet * (1.0 - i / n_f)
            rr = R_fillet + t
            sin_phi = (R_fillet - z) / rr
            if sin_phi > 1.0:
                sin_phi = 1.0
            cos_phi = math.sqrt(max(0, 1.0 - sin_phi ** 2))
            ρ = (r_blade + R_fillet) - rr * cos_phi
        else:
            ρ = r_blade + R_fillet * i / n_f
            z = z_fillet(ρ) if i < n_f else 0.0

        θ = θ_c + sign * half_C / ρ
        pts.append(v3(ρ * math.cos(θ), ρ * math.sin(θ), z))

    # Fillet bottom / endwall top
    if inner:
        rr = R_fillet + t
        sin_phi = R_fillet / rr
        cos_phi = math.sqrt(max(0, 1.0 - sin_phi ** 2))
        ρ_bottom = (r_blade + R_fillet) - rr * cos_phi
    else:
        ρ_bottom = r_blade + R_fillet
    θ_bottom = θ_c + sign * half_C / ρ_bottom
    pts.append(v3(ρ_bottom * math.cos(θ_bottom),
                  ρ_bottom * math.sin(θ_bottom), 0.0))

    # Endwall top: ρ from r+R to r+R+off, z=0 (inner) or z=-t (outer)
    if inner:
        z_ew = -t
    else:
        z_ew = 0.0

    n_e = N_SAMPLES // 2
    for i in range(1, n_e + 1):
        ρ = (r_blade + R_fillet) + off * i / n_e
        θ = θ_c + sign * half_C / ρ
        pts.append(v3(ρ * math.cos(θ), ρ * math.sin(θ), z_ew))

    return make_polyline_edge(pts)


def band_boundary_curves(θ_c_deg):
    """Return the 4 boundary curves of a band: top arc, bottom arc, left wire, right wire.

    Each curve has an inner (offset) and outer (on surface) version.
    """
    θ_c = float(θ_c_deg)
    half_C = C / 2.0
    ρ_end = r_blade + R_fillet + off

    curves = {}

    # ── Side A (left) ──
    curves["side_a_outer"] = connector_wire(θ_c, -1)
    curves["side_a_inner"] = thickened_wire(θ_c, -1, inner=True)

    # ── Side B (right) ──
    curves["side_b_outer"] = connector_wire(θ_c, +1)
    curves["side_b_inner"] = thickened_wire(θ_c, +1, inner=True)

    # ── Top arc (at z=blade_height) ──
    θ_a0 = math.degrees(math.radians(θ_c) - half_C / r_blade)
    θ_a1 = math.degrees(math.radians(θ_c) + half_C / r_blade)
    curves["top_arc_outer"] = make_arc_edge_z(r_blade, θ_a0, θ_a1, BLADE_HEIGHT)
    curves["top_arc_inner"] = make_arc_edge_z(r_blade - t, θ_a0, θ_a1, BLADE_HEIGHT)

    # ── Bottom arc (at z=-t or z=0) ──
    θ_b0 = math.degrees(math.radians(θ_c) - half_C / ρ_end)
    θ_b1 = math.degrees(math.radians(θ_c) + half_C / ρ_end)
    curves["bottom_arc_outer"] = make_arc_edge_z(ρ_end, θ_b0, θ_b1, 0.0)
    curves["bottom_arc_inner"] = make_arc_edge_z(ρ_end, θ_b0, θ_b1, -t)

    return curves


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

    # Band offset curves (inner/outer boundaries)
    bw = band_boundary_curves(θ_c)
    for k, v in bw.items():
        curves[f"ply_band_{idx:04d}_{k}"] = v

    print(f"  band {idx}: center={θ_c:.1f}° "
          f"blade[{θ0:.1f}°→{θ1:.1f}°] span={θ1-θ0:.1f}° "
          f"expanded[{θ_a_exp:.1f}°→{θ_b_exp:.1f}°] span={θ_b_exp-θ_a_exp:.1f}°")

# ═══════════════════════════════════════════════════════════════
# Export STEP
# ═══════════════════════════════════════════════════════════════

current_dir = Path(__file__).resolve().parent
step_path = current_dir / "halfcircle_curves.step"

assy = cq.Assembly(name="HALFCIRCLE_CURVES")

for name, item in curves.items():
    if "station" in name:
        color = cq.Color(1.0, 1.0, 0.0)  # yellow
    elif "side_a_outer" in name:
        color = cq.Color(1.0, 0.0, 0.0)  # red
    elif "side_b_outer" in name:
        color = cq.Color(0.0, 0.0, 1.0)  # blue
    elif "side_a_inner" in name:
        color = cq.Color(1.0, 0.3, 0.3)  # light red
    elif "side_b_inner" in name:
        color = cq.Color(0.3, 0.3, 1.0)  # light blue
    elif "top_arc" in name:
        color = cq.Color(0.0, 1.0, 0.0)  # green
    elif "bottom_arc" in name:
        color = cq.Color(1.0, 0.5, 0.0)  # orange
    elif "blade_equal_arc" in name:
        color = cq.Color(0.0, 1.0, 0.0)  # green
    elif "expanded_equal_arc" in name:
        color = cq.Color(1.0, 0.5, 0.0)  # orange
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
