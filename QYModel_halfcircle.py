"""Single band offset curves — inner surface = scaled-down outer surface.

Outer: blade ρ=33, fillet r=2, C=34.56, endwall ρ_end=95
Inner: blade ρ=32, fillet r=3, C=33.51, endwall ρ_end=94
Each uses its own C and its own spiral θ(ρ)=θ_c ± C/(2ρ).
"""
import math
import cadquery as cq
from pathlib import Path

BLADE_HEIGHT = 80.0
r_blade = 33.0
R_fillet = 2.0
off = 60.0
t = 1.0
N_SAMPLES = 40
Δθ_deg = 60.0  # angular span per band

θ_center = -60.0  # band 0 center

C_outer = r_blade * math.radians(Δθ_deg)
C_inner = (r_blade - t) * math.radians(Δθ_deg)
ρ_end_outer = r_blade + R_fillet + off  # 95
ρ_end_inner = (r_blade - t) + (R_fillet + t) + off - t  # 32+3+60-1=94

print(f"C_outer = {C_outer:.4f}, C_inner = {C_inner:.4f}")


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


def z_fillet_outer(rho):
    d = r_blade + R_fillet - rho
    if d <= 0:
        return 0.0
    if d >= R_fillet:
        return R_fillet
    return R_fillet - math.sqrt(R_fillet**2 - d**2)


def side_wire(sign, inner=False):
    """sign=-1 side_a, +1 side_b. inner=True for offset surface."""
    θ_c = math.radians(θ_center)
    if inner:
        CC = C_inner
        ρ_blade = r_blade - t
        R_f = R_fillet + t
        ρ_ew_end = ρ_end_inner
    else:
        CC = C_outer
        ρ_blade = r_blade
        R_f = R_fillet
        ρ_ew_end = ρ_end_outer
    hC = CC / 2.0
    pts = []

    # Blade (radial sides: inner uses same angle as outer)
    if inner:
        θ_j = θ_c + sign * (C_outer / 2.0) / r_blade
    else:
        θ_j = θ_c + sign * hC / ρ_blade

    pts.append(v3(ρ_blade * math.cos(θ_j), ρ_blade * math.sin(θ_j), BLADE_HEIGHT))
    pts.append(v3(ρ_blade * math.cos(θ_j), ρ_blade * math.sin(θ_j), R_fillet))

    # Fillet
    n_f = N_SAMPLES // 2
    ρ_bot = r_blade + R_fillet  # 35 for both
    for i in range(1, n_f):
        frac = i / n_f
        if inner:
            phi = frac * 0.5 * math.pi
            ρ = ρ_bot - R_f * math.cos(phi)
            z = R_fillet - R_f * math.sin(phi)
        else:
            ρ = r_blade + R_fillet * frac
            z = z_fillet_outer(ρ)
        θ = θ_c + sign * hC / ρ
        pts.append(v3(ρ * math.cos(θ), ρ * math.sin(θ), z))

    # Fillet bottom
    z_bot = -t if inner else 0.0
    θ_bot = θ_c + sign * hC / ρ_bot
    pts.append(v3(ρ_bot * math.cos(θ_bot), ρ_bot * math.sin(θ_bot), z_bot))

    # Endwall
    n_e = N_SAMPLES // 2
    for i in range(1, n_e + 1):
        ρ = ρ_bot + (ρ_ew_end - ρ_bot) * i / n_e
        θ = θ_c + sign * hC / ρ
        z = -t if inner else 0.0
        pts.append(v3(ρ * math.cos(θ), ρ * math.sin(θ), z))

    return polyline_edge(pts)


def top_arc(inner=False):
    hC = (C_outer / 2.0)  # same angles for both (radial sides)
    θ_a = math.degrees(math.radians(θ_center) - hC / r_blade)
    θ_b = math.degrees(math.radians(θ_center) + hC / r_blade)
    ρ = (r_blade - t) if inner else r_blade
    return arc_edge_z(ρ, θ_a, θ_b, BLADE_HEIGHT)


def bottom_arc(inner=False):
    if inner:
        CC = C_inner
        ρ = ρ_end_inner
        z = -t
    else:
        CC = C_outer
        ρ = ρ_end_outer
        z = 0.0
    hC = CC / 2.0
    θ_a = math.degrees(math.radians(θ_center) - hC / ρ)
    θ_b = math.degrees(math.radians(θ_center) + hC / ρ)
    return arc_edge_z(ρ, θ_a, θ_b, z)


# ═══════════════════════════════════════════
curves = {}
curves["side_a_outer"] = side_wire(-1, False)
curves["side_b_outer"] = side_wire(+1, False)
curves["top_arc_outer"] = top_arc(False)
curves["bottom_arc_outer"] = bottom_arc(False)
curves["side_a_inner"] = side_wire(-1, True)
curves["side_b_inner"] = side_wire(+1, True)
curves["top_arc_inner"] = top_arc(True)
curves["bottom_arc_inner"] = bottom_arc(True)

# Print angles
print("\nOuter (C=%.4f):" % C_outer)
for label, ρ in [("blade", r_blade), ("fillet_bot", r_blade+R_fillet), ("endwall", ρ_end_outer)]:
    θ_a = θ_center - math.degrees(C_outer/(2*ρ))
    θ_b = θ_center + math.degrees(C_outer/(2*ρ))
    print(f"  ρ={ρ:3.0f}: {θ_a:.1f}° → {θ_b:.1f}°  span={θ_b-θ_a:.1f}°")

print(f"\nInner (C={C_inner:.4f}):")
print(f"  blade: same angles as outer (radial sides), ρ={r_blade-t}")
for label, ρ in [("fillet_bot", r_blade+R_fillet), ("endwall", ρ_end_inner)]:
    θ_a = θ_center - math.degrees(C_inner/(2*ρ))
    θ_b = θ_center + math.degrees(C_inner/(2*ρ))
    print(f"  ρ={ρ:3.0f}: {θ_a:.1f}° → {θ_b:.1f}°  span={θ_b-θ_a:.1f}°")

# Export
step_path = Path(__file__).resolve().parent / "halfcircle_curves.step"
if step_path.exists():
    step_path.unlink()

assy = cq.Assembly(name="SINGLE_BAND_OFFSET")
colors = {
    "side_a_outer": cq.Color(1, 0, 0),
    "side_b_outer": cq.Color(0, 0, 1),
    "side_a_inner": cq.Color(1, 0.4, 0.4),
    "side_b_inner": cq.Color(0.4, 0.4, 1),
    "top_arc_outer": cq.Color(0, 0.8, 0),
    "top_arc_inner": cq.Color(0.4, 0.6, 0.4),
    "bottom_arc_outer": cq.Color(1, 0.6, 0),
    "bottom_arc_inner": cq.Color(1, 0.8, 0.4),
}
for name, wire in curves.items():
    assy.add(wire, name=name, color=colors.get(name, cq.Color(1, 1, 1)))
assy.export(str(step_path))
print(f"\n[STEP] {step_path} ({len(curves)} curves)")
