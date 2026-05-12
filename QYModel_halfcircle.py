"""Single band offset curves — inner surface = outer surface shifted by ply thickness.

Outer: blade ρ=33, fillet r=2, endwall z=0
Inner: blade ρ=32, fillet r=3, endwall z=-1
Both use same spiral formula θ(ρ)=θ_c ± C/(2ρ).
"""
import math
import cadquery as cq
from pathlib import Path

BLADE_HEIGHT = 80.0
r_blade = 33.0
R_fillet = 2.0
off = 60.0
t = 1.0
ANGLE_START = -90.0
ANGLE_END = 90.0
N_BANDS = 3
N_SAMPLES = 40

C = r_blade * math.radians(ANGLE_END - ANGLE_START) / N_BANDS
θ_center = -60.0  # band 0 center

ρ_end = r_blade + R_fillet + off  # 95


def v3(x, y, z):
    return cq.Vector(float(x), float(y), float(z))


def polyline_edge(pts):
    edges = []
    for i in range(len(pts) - 1):
        d = pts[i].sub(pts[i + 1]).Length
        if d > 1e-9:
            edges.append(cq.Edge.makeLine(pts[i], pts[i + 1]))
    return cq.Wire.assembleEdges(edges)


def arc_edge_z(radius, a0_deg, a1_deg, z):
    a0, a1 = math.radians(a0_deg), math.radians(a1_deg)
    p0 = cq.Vector(radius * math.cos(a0), radius * math.sin(a0), z)
    p1 = cq.Vector(radius * math.cos(a1), radius * math.sin(a1), z)
    mid = 0.5 * (a0 + a1)
    pm = cq.Vector(radius * math.cos(mid), radius * math.sin(mid), z)
    return cq.Edge.makeThreePointArc(p0, pm, p1)


def z_fillet_outer(rho):
    """Quarter-circle z for outer surface (radius R_fillet)."""
    d = r_blade + R_fillet - rho
    if d <= 0:
        return 0.0
    if d >= R_fillet:
        return R_fillet
    return R_fillet - math.sqrt(R_fillet**2 - d**2)


def z_fillet_inner(rho):
    """Quarter-circle z for inner surface (radius R_fillet+t)."""
    rr = R_fillet + t
    center_rho = r_blade + R_fillet  # 35
    d = center_rho - rho
    if d <= 0:
        return -t
    if d >= rr:
        return R_fillet
    return R_fillet - math.sqrt(rr**2 - d**2)


def side_wire(theta_center_deg, sign, inner=False):
    """Generate one side wire (outer or inner surface)."""
    θ_c = math.radians(theta_center_deg)
    half_C = C / 2.0
    pts = []

    if inner:
        ρ_blade = r_blade - t
        R_f = R_fillet + t
        # Blade portion: radial sides (same angle as outer at blade)
        θ_blade_outer = θ_c + sign * half_C / r_blade
        θ_j = θ_blade_outer
    else:
        ρ_blade = r_blade
        R_f = R_fillet
        θ_j = θ_c + sign * half_C / ρ_blade

    pts.append(v3(ρ_blade * math.cos(θ_j), ρ_blade * math.sin(θ_j), BLADE_HEIGHT))
    pts.append(v3(ρ_blade * math.cos(θ_j), ρ_blade * math.sin(θ_j), R_fillet))

    # Fillet: ρ from ρ_blade to r_blade+R_fillet
    n_f = N_SAMPLES // 2
    for i in range(1, n_f):
        frac = i / n_f
        if inner:
            # ρ along inner quarter-circle
            phi = frac * 0.5 * math.pi
            ρ = (r_blade + R_fillet) - R_f * math.cos(phi)
            z = R_fillet - R_f * math.sin(phi)
            if z < -t:
                z = -t
        else:
            ρ = r_blade + R_fillet * frac
            z = z_fillet_outer(ρ)
        θ = θ_c + sign * half_C / ρ
        pts.append(v3(ρ * math.cos(θ), ρ * math.sin(θ), z))

    # Fillet bottom
    if inner:
        ρ_bot = r_blade + R_fillet  # 35, inner arc reaches here at z=-t
        z_bot = -t
    else:
        ρ_bot = r_blade + R_fillet  # 35
        z_bot = 0.0
    θ_bot = θ_c + sign * half_C / ρ_bot
    pts.append(v3(ρ_bot * math.cos(θ_bot), ρ_bot * math.sin(θ_bot), z_bot))

    # Endwall: ρ from 35 to 95
    n_e = N_SAMPLES // 2
    for i in range(1, n_e + 1):
        ρ = (r_blade + R_fillet) + off * i / n_e
        θ = θ_c + sign * half_C / ρ
        z = -t if inner else 0.0
        pts.append(v3(ρ * math.cos(θ), ρ * math.sin(θ), z))

    return polyline_edge(pts)


def top_arc(inner=False):
    """Top arc: same angles for inner/outer (perpendicular side edges)."""
    half_C = C / 2.0
    θ_a = math.degrees(math.radians(θ_center) - half_C / r_blade)
    θ_b = math.degrees(math.radians(θ_center) + half_C / r_blade)
    ρ = r_blade - t if inner else r_blade
    return arc_edge_z(ρ, θ_a, θ_b, BLADE_HEIGHT)


def bottom_arc(inner=False):
    half_C = C / 2.0
    θ_a = math.degrees(math.radians(θ_center) - half_C / ρ_end)
    θ_b = math.degrees(math.radians(θ_center) + half_C / ρ_end)
    z = -t if inner else 0.0
    return arc_edge_z(ρ_end, θ_a, θ_b, z)


# ═══════════════════════════════════════════════════════════════
# Generate curves
# ═══════════════════════════════════════════════════════════════
curves = {}

# Outer surface
curves["side_a_outer"] = side_wire(θ_center, -1, inner=False)
curves["side_b_outer"] = side_wire(θ_center, +1, inner=False)
curves["top_arc_outer"] = top_arc(inner=False)
curves["bottom_arc_outer"] = bottom_arc(inner=False)

# Inner surface (offset by t)
curves["side_a_inner"] = side_wire(θ_center, -1, inner=True)
curves["side_b_inner"] = side_wire(θ_center, +1, inner=True)
curves["top_arc_inner"] = top_arc(inner=True)
curves["bottom_arc_inner"] = bottom_arc(inner=True)

# Print key angles for verification
half_C = C / 2.0
print(f"Band center θ_c = {θ_center}°")
print(f"C = {C:.3f} mm")
print()
print("Outer surface angles:")
for label, ρ in [("blade top", r_blade),
                  ("fillet bottom", r_blade + R_fillet),
                  ("endwall bottom", r_blade + R_fillet + off)]:
    θ_a = θ_center - math.degrees(half_C / ρ)
    θ_b = θ_center + math.degrees(half_C / ρ)
    print(f"  ρ={ρ:3.0f}: side_a={θ_a:.1f}° side_b={θ_b:.1f}° span={θ_b-θ_a:.1f}°")

print(f"\nInner surface:")
print(f"  blade: same angles as outer ({θ_center - math.degrees(half_C/r_blade):.1f}° → {θ_center + math.degrees(half_C/r_blade):.1f}°), ρ={r_blade-t}")
print(f"  fillet+endwall: spiral at inner radii")

# ═══════════════════════════════════════════════════════════════
# Export
# ═══════════════════════════════════════════════════════════════
step_path = Path(__file__).resolve().parent / "halfcircle_curves.step"
if step_path.exists():
    step_path.unlink()

assy = cq.Assembly(name="SINGLE_BAND_OFFSET")
for name, wire in curves.items():
    if "side_a_outer" in name:
        c = cq.Color(1, 0, 0)       # red
    elif "side_b_outer" in name:
        c = cq.Color(0, 0, 1)       # blue
    elif "side_a_inner" in name:
        c = cq.Color(1, 0.4, 0.4)   # light red
    elif "side_b_inner" in name:
        c = cq.Color(0.4, 0.4, 1)   # light blue
    elif "top_arc_outer" in name:
        c = cq.Color(0, 0.8, 0)     # green
    elif "top_arc_inner" in name:
        c = cq.Color(0.4, 0.6, 0.4) # light green
    elif "bottom_arc_outer" in name:
        c = cq.Color(1, 0.6, 0)     # orange
    elif "bottom_arc_inner" in name:
        c = cq.Color(1, 0.8, 0.4)   # light orange
    else:
        c = cq.Color(1, 1, 1)
    assy.add(wire, name=name, color=c)

assy.export(str(step_path))
print(f"\n[STEP] {step_path} ({len(curves)} curves)")
