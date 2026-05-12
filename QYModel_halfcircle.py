"""Single band outer surface curves only — hyperbolic spiral formulation.

θ(ρ) = θ_c ± C/(2ρ),  z(ρ) = R - √(R² - (r+R-ρ)²) for ρ ∈ [r, r+R]
"""
import math, cadquery as cq
from pathlib import Path

BLADE_HEIGHT = 80.0
r_blade = 33.0
R_fillet = 2.0
off = 60.0
t = 1.0
N_SAMPLES = 40
Δθ_deg = 60.0
θ_center = -60.0

C = r_blade * math.radians(Δθ_deg)
ρ_fbot = r_blade + R_fillet       # 35
ρ_end = r_blade + R_fillet + off   # 95

print(f"Outer surface: r={r_blade} R={R_fillet} C={C:.4f} ρ=[{r_blade}→{ρ_fbot}→{ρ_end}]")


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


def side_wire(sign):
    θ_c = math.radians(θ_center)
    hC = C / 2.0
    θ_j = θ_c + sign * hC / r_blade
    pts = [v3(r_blade * math.cos(θ_j), r_blade * math.sin(θ_j), BLADE_HEIGHT),
           v3(r_blade * math.cos(θ_j), r_blade * math.sin(θ_j), R_fillet)]
    n_f = N_SAMPLES // 2
    for i in range(1, n_f):
        ρ = r_blade + R_fillet * i / n_f
        d = ρ_fbot - ρ
        z = R_fillet - math.sqrt(max(0, R_fillet**2 - d**2)) if d <= R_fillet else 0
        pts.append(v3(ρ * math.cos(θ_c + sign * hC / ρ),
                      ρ * math.sin(θ_c + sign * hC / ρ), z))
    pts.append(v3(ρ_fbot * math.cos(θ_c + sign * hC / ρ_fbot),
                  ρ_fbot * math.sin(θ_c + sign * hC / ρ_fbot), 0.0))
    n_e = N_SAMPLES // 2
    for i in range(1, n_e + 1):
        ρ = ρ_fbot + off * i / n_e
        pts.append(v3(ρ * math.cos(θ_c + sign * hC / ρ),
                      ρ * math.sin(θ_c + sign * hC / ρ), 0.0))
    return polyline_edge(pts)


def top_arc():
    hC = C / 2.0
    θ_a = math.degrees(math.radians(θ_center) - hC / r_blade)
    θ_b = math.degrees(math.radians(θ_center) + hC / r_blade)
    return arc_edge_z(r_blade, θ_a, θ_b, BLADE_HEIGHT)


def bottom_arc():
    hC = C / 2.0
    θ_a = math.degrees(math.radians(θ_center) - hC / ρ_end)
    θ_b = math.degrees(math.radians(θ_center) + hC / ρ_end)
    return arc_edge_z(ρ_end, θ_a, θ_b, 0.0)


curves = {
    "side_a": side_wire(-1), "side_b": side_wire(+1),
    "blade_equal_arc": top_arc(), "expanded_equal_arc": bottom_arc(),
}

print(f"Angles: blade={θ_center - math.degrees(C/(2*r_blade)):.1f}→{θ_center + math.degrees(C/(2*r_blade)):.1f}")
print(f"        fillet_bot={θ_center - math.degrees(C/(2*ρ_fbot)):.1f}→{θ_center + math.degrees(C/(2*ρ_fbot)):.1f}")
print(f"        endwall={θ_center - math.degrees(C/(2*ρ_end)):.1f}→{θ_center + math.degrees(C/(2*ρ_end)):.1f}")

step_path = Path(__file__).resolve().parent / "halfcircle_curves.step"
if step_path.exists(): step_path.unlink()
assy = cq.Assembly(name="OUTER_SURFACE")
for name, wire in curves.items():
    c = {"side_a": cq.Color(1,0,0), "side_b": cq.Color(0,0,1),
         "blade_equal_arc": cq.Color(0,1,0), "expanded_equal_arc": cq.Color(1,0.5,0)}.get(name, cq.Color(1,1,1))
    assy.add(wire, name=name, color=c)
assy.export(str(step_path))
print(f"[STEP] {step_path}")
