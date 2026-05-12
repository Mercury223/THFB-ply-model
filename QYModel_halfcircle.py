"""Single band offset curves — inner = fully independent, shifted inward by t.

Outer: blade ρ=33, fillet center(35,2) r=2, endwall ρ=35→95, C=34.56
Inner: blade ρ=32, fillet center(34,2) r=2, endwall ρ=34→94, C=33.51
Each surface has its own geometry, its own C, its own spiral.
"""
import math
import cadquery as cq
from pathlib import Path

BLADE_HEIGHT = 80.0
r_blade_o, r_blade_i = 33.0, 32.0  # outer/inner blade radius
R_fillet = 2.0                      # fillet radius (same for both)
ctr_o = r_blade_o + R_fillet       # outer fillet center ρ = 35
ctr_i = r_blade_i + R_fillet       # inner fillet center ρ = 34
off = 60.0
t = 1.0
N_SAMPLES = 40
Δθ_deg = 60.0
θ_center = -60.0

ρ_ew_start_o = ctr_o  # 35
ρ_ew_start_i = ctr_i  # 34
ρ_ew_end_o = ctr_o + off      # 95
ρ_ew_end_i = ctr_i + off      # 94

C_o = r_blade_o * math.radians(Δθ_deg)  # 34.56
C_i = r_blade_i * math.radians(Δθ_deg)  # 33.51

print(f"Outer: r={r_blade_o} ctr={ctr_o} C={C_o:.4f} ew={ρ_ew_start_o}→{ρ_ew_end_o}")
print(f"Inner: r={r_blade_i} ctr={ctr_i} C={C_i:.4f} ew={ρ_ew_start_i}→{ρ_ew_end_i}")


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
    pm = cq.Vector(radius * math.cos(0.5*(a0+a1)), radius * math.sin(0.5*(a0+a1)), z)
    return cq.Edge.makeThreePointArc(p0, pm, p1)


def z_fillet(rho, center_rho):
    """Quarter-circle z: center at (center_rho, R_fillet)."""
    d = center_rho - rho
    if d <= 0:
        return 0.0
    if d >= R_fillet:
        return R_fillet
    return R_fillet - math.sqrt(R_fillet**2 - d**2)


def side_wire(sign, inner=False):
    """sign=-1 side_a, +1 side_b."""
    if inner:
        CC = C_i; r_blade = r_blade_i; ctr = ctr_i
        ρ_ew_start = ρ_ew_start_i; ρ_ew_end = ρ_ew_end_i
        z_ew = -t
    else:
        CC = C_o; r_blade = r_blade_o; ctr = ctr_o
        ρ_ew_start = ρ_ew_start_o; ρ_ew_end = ρ_ew_end_o
        z_ew = 0.0
    hC = CC / 2.0
    θ_c = math.radians(θ_center)
    pts = []

    # Blade junction angle (radial sides: use plane-perpendicular angle)
    if inner:
        θ_j = θ_c + sign * (C_o / 2.0) / r_blade_o  # same as outer
    else:
        θ_j = θ_c + sign * hC / r_blade

    pts.append(v3(r_blade * math.cos(θ_j), r_blade * math.sin(θ_j), BLADE_HEIGHT))
    pts.append(v3(r_blade * math.cos(θ_j), r_blade * math.sin(θ_j), R_fillet))

    # Fillet: ρ from r_blade to ctr
    n_f = N_SAMPLES // 2
    for i in range(1, n_f):
        frac = i / n_f
        ρ = r_blade + (ctr - r_blade) * frac
        z = z_fillet(ρ, ctr)
        θ = θ_c + sign * hC / ρ
        pts.append(v3(ρ * math.cos(θ), ρ * math.sin(θ), z))

    # Fillet bottom = endwall start
    θ_bot = θ_c + sign * hC / ctr
    pts.append(v3(ctr * math.cos(θ_bot), ctr * math.sin(θ_bot), z_ew))

    # Endwall
    n_e = N_SAMPLES // 2
    for i in range(1, n_e + 1):
        ρ = ctr + off * i / n_e
        θ = θ_c + sign * hC / ρ
        pts.append(v3(ρ * math.cos(θ), ρ * math.sin(θ), z_ew))

    return polyline_edge(pts)


def top_arc(inner=False):
    r_blade = r_blade_i if inner else r_blade_o
    hC = C_o / 2.0  # same angles for both (radial sides)
    θ_a = math.degrees(math.radians(θ_center) - hC / r_blade_o)
    θ_b = math.degrees(math.radians(θ_center) + hC / r_blade_o)
    return arc_edge_z(r_blade, θ_a, θ_b, BLADE_HEIGHT)


def bottom_arc(inner=False):
    if inner:
        CC = C_i; ρ = ρ_ew_end_i; z = -t
    else:
        CC = C_o; ρ = ρ_ew_end_o; z = 0.0
    hC = CC / 2.0
    θ_a = math.degrees(math.radians(θ_center) - hC / ρ)
    θ_b = math.degrees(math.radians(θ_center) + hC / ρ)
    return arc_edge_z(ρ, θ_a, θ_b, z)


# ═══════════════════════════════
curves = {}
for name, fn, args in [
    ("side_a_outer", side_wire, (-1, False)),
    ("side_b_outer", side_wire, (+1, False)),
    ("top_arc_outer", top_arc, (False,)),
    ("bottom_arc_outer", bottom_arc, (False,)),
    ("side_a_inner", side_wire, (-1, True)),
    ("side_b_inner", side_wire, (+1, True)),
    ("top_arc_inner", top_arc, (True,)),
    ("bottom_arc_inner", bottom_arc, (True,)),
]:
    curves[name] = fn(*args)

# Print
print("\nOuter (C=%.4f):" % C_o)
for ρ in [r_blade_o, ctr_o, ρ_ew_end_o]:
    θ_a = θ_center - math.degrees(C_o/(2*ρ))
    θ_b = θ_center + math.degrees(C_o/(2*ρ))
    print(f"  ρ={ρ:3.0f}: {θ_a:.1f}°→{θ_b:.1f}°  span={θ_b-θ_a:.1f}°")

print(f"\nInner (C={C_i:.4f}):")
print(f"  blade: same angles as outer, ρ={r_blade_i}")
for ρ in [ctr_i, ρ_ew_end_i]:
    θ_a = θ_center - math.degrees(C_i/(2*ρ))
    θ_b = θ_center + math.degrees(C_i/(2*ρ))
    print(f"  ρ={ρ:3.0f}: {θ_a:.1f}°→{θ_b:.1f}°  span={θ_b-θ_a:.1f}°")

# Export
step_path = Path(__file__).resolve().parent / "halfcircle_curves.step"
if step_path.exists():
    step_path.unlink()
assy = cq.Assembly(name="SINGLE_BAND_OFFSET")
colors = {
    "side_a_outer": cq.Color(1, 0, 0), "side_b_outer": cq.Color(0, 0, 1),
    "side_a_inner": cq.Color(1, 0.4, 0.4), "side_b_inner": cq.Color(0.4, 0.4, 1),
    "top_arc_outer": cq.Color(0, 0.8, 0), "top_arc_inner": cq.Color(0.4, 0.6, 0.4),
    "bottom_arc_outer": cq.Color(1, 0.6, 0), "bottom_arc_inner": cq.Color(1, 0.8, 0.4),
}
for name, wire in curves.items():
    assy.add(wire, name=name, color=colors.get(name, cq.Color(1, 1, 1)))
assy.export(str(step_path))
print(f"\n[STEP] {step_path} ({len(curves)} curves)")
