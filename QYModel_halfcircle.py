"""Half-circle blade test — curves only, hyperbolic spiral formulation.

User's notation:
  r = blade outer radius (= OUTER_RADIUS = 33)
  R = fillet radius in xz profile (= FILLET_RADIUS = 2)

Hyperbolic spiral (双曲螺线) in fillet:
  ρ ∈ [r, r+R]:   ρ × Δθ = C  (equal arc-length),  z = R - √(R² - (r+R-ρ)²)
  ρ ∈ [r+R, r+R+off]:  z = 0, θ follows same spiral to expanded endpoint
  ρ = r+R+off:  drop to z = -t
"""
import math
import cadquery as cq
from pathlib import Path

# ── Parameters ──
BLADE_HEIGHT = 80.0
OUTER_RADIUS = 33.0     # user's r — blade outer radius
FILLET_RADIUS = 2.0     # user's R — fillet quarter-circle radius
PLY_EXPAND_OFFSET = 60.0
PLY_LAYER_THICKNESS = 1.0
ANGLE_START = -90.0
ANGLE_END = 90.0
N_BANDS = 3
N_SAMPLES = 30  # sample count for spiral curves

# User's notation (match the derivation)
r_blade = OUTER_RADIUS   # inner circle radius = 33
R_fillet = FILLET_RADIUS  # xz profile quarter-circle radius = 2
off = PLY_EXPAND_OFFSET
t = PLY_LAYER_THICKNESS
C = r_blade * math.radians(ANGLE_END - ANGLE_START) / N_BANDS  # equal arc-length per band

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


def connector_wire(theta_ref_deg, is_side_b=False):
    """Build 3D connector wire.

    theta_ref = band START angle θ₀.

    side_a (is_side_b=False): radial line, θ = θ_ref for all ρ.
      Blade junction at (r_blade, θ_ref).

    side_b (is_side_b=True): hyperbolic spiral, θ(ρ) = θ_ref + C/ρ.
      At ρ=r_blade: θ = θ_ref + C/r_blade = band END angle.  ← wire starts here
      Blade junction at (r_blade, θ_ref + C/r_blade).

    Path: blade top → blade-fillet junction → fillet spiral (ρ: r→r+R)
          → endwall top (ρ: r+R→r+R+off, z=0) → drop to z=-t
    """
    θ_ref = math.radians(theta_ref_deg)

    # Blade junction angle
    if is_side_b:
        θ_junction = θ_ref + C / r_blade  # band end angle on inner circle
    else:
        θ_junction = θ_ref               # band start angle (radial)

    pts = []

    # 1) Blade top → blade-fillet junction
    pts.append(v3(r_blade * math.cos(θ_junction),
                  r_blade * math.sin(θ_junction), BLADE_HEIGHT))
    pts.append(v3(r_blade * math.cos(θ_junction),
                  r_blade * math.sin(θ_junction), R_fillet))

    # 2) Fillet: ρ ∈ [r_blade, r_blade + R_fillet]
    n_fillet = N_SAMPLES // 2
    for i in range(1, n_fillet + 1):
        rho = r_blade + R_fillet * i / n_fillet
        if is_side_b:
            theta = θ_ref + C / rho
        else:
            theta = θ_ref
        z = z_fillet(rho)
        pts.append(v3(rho * math.cos(theta), rho * math.sin(theta), z))

    # 3) Endwall top: ρ ∈ [r_blade+R_fillet, r_blade+R_fillet+off], z=0
    n_endwall = N_SAMPLES // 2
    for i in range(1, n_endwall + 1):
        rho = (r_blade + R_fillet) + off * i / n_endwall
        if is_side_b:
            theta = θ_ref + C / rho
        else:
            theta = θ_ref
        pts.append(v3(rho * math.cos(theta), rho * math.sin(theta), 0.0))

    # 4) Drop to endwall bottom
    rho_end = r_blade + R_fillet + off
    if is_side_b:
        theta_end = θ_ref + C / rho_end
    else:
        theta_end = θ_ref
    pts.append(v3(rho_end * math.cos(theta_end), rho_end * math.sin(theta_end), -t))

    return make_polyline_edge(pts)


# ═══════════════════════════════════════════════════════════════
# Build curves
# ═══════════════════════════════════════════════════════════════

curves = {}

# Station positions on blade (equal arc-length)
station_angles_deg = [
    ANGLE_START + math.degrees(i * C / r_blade) for i in range(N_BANDS + 1)
]

# 1) Station wires (θ fixed = radial lines)
for idx, ang in enumerate(station_angles_deg):
    curves[f"ply_station_{idx:04d}"] = connector_wire(ang, is_side_b=False)
    print(f"  station {idx}: blade θ={ang:.1f}° (radial)")

# 2) Band curves
for idx in range(N_BANDS):
    θ0 = station_angles_deg[idx]      # band start on blade
    θ1 = station_angles_deg[idx + 1]  # band end on blade

    # Blade equal-arc segment
    blade_arc = make_arc_edge(0, 0, r_blade, θ0, θ1)
    curves[f"ply_band_{idx:04d}_blade_equal_arc"] = blade_arc

    # Expanded equal-arc segment: hyperbolic spiral endpoints at ρ_end
    ρ_end = r_blade + R_fillet + off
    θ0_exp_deg = math.degrees(math.radians(θ0) + C / ρ_end)
    θ1_exp_deg = math.degrees(math.radians(θ0) + C / ρ_end)
    # For side_b at band end: θ_end = θ_start + C/ρ_end
    # θ_start of band = θ0. Band covers C/r_blade on blade.
    # At ρ_end, side_a is at θ0_exp (θ0 + C/ρ_end)
    # side_b is at θ0 + C/ρ_end  (same formula since θ0 already includes offset)

    # The expanded band endpoints are:
    # side_a of band i: θ = θ0 + C/ρ_end  (if side_a follows spiral too? No, side_a is radial)
    # Hmm, but the user says side_a is radial (θ constant)
    # So at ρ_end: side_a at θ0, side_b at θ0 + C/ρ_end

    # For the expanded equal-arc curve, we show the band at ρ = r+R+off:
    θ_a_end = θ0  # side_a at expanded radius (radial)
    θ_b_end = math.degrees(math.radians(θ0) + C / ρ_end)  # side_b at expanded radius (spiral)

    expanded_arc = make_arc_edge(0, 0, ρ_end, θ_a_end, θ_b_end)
    curves[f"ply_band_{idx:04d}_expanded_equal_arc"] = expanded_arc

    # Side A: θ = θ0 (radial line, fixed angle)
    curves[f"ply_band_{idx:04d}_side_a"] = connector_wire(θ0, is_side_b=False)

    # Side B: hyperbolic spiral from θ = θ1 at ρ=r to θ = θ0 + C/ρ at each ρ
    # Wait — at ρ=r, side_b should be at θ1 = θ0 + C/r (band end on blade)
    # The spiral formula: θ(ρ) = θ0 + C/ρ
    # At ρ=r: θ = θ0 + C/r = θ1 ✓
    # So side_b spiral starts from θ0 (the band START angle), not θ1!
    curves[f"ply_band_{idx:04d}_side_b"] = connector_wire(θ0, is_side_b=True)

    print(f"  band {idx}: blade[{θ0:.1f}°→{θ1:.1f}°] "
          f"expanded[{θ_a_end:.1f}°→{θ_b_end:.1f}°] "
          f"span={θ_b_end-θ_a_end:.1f}°")

# ═══════════════════════════════════════════════════════════════
# Export STEP
# ═══════════════════════════════════════════════════════════════

current_dir = Path(__file__).resolve().parent
step_path = current_dir / "halfcircle_curves.step"

assy = cq.Assembly(name="HALFCIRCLE_CURVES")

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

# Verification
print(f"\n[Verification]")
for idx in range(N_BANDS):
    θ0 = station_angles_deg[idx]
    ρ_end = r_blade + R_fillet + off
    print(f"  band {idx}: side_a spiral? No (θ={θ0:.1f}° fixed)")
    print(f"           side_b at ρ={r_blade}: θ={θ0 + math.degrees(C/r_blade):.1f}°")
    print(f"           side_b at ρ={r_blade+R_fillet}: θ={θ0 + math.degrees(C/(r_blade+R_fillet)):.1f}°")
    print(f"           side_b at ρ={ρ_end}: θ={θ0 + math.degrees(C/ρ_end):.1f}°")
