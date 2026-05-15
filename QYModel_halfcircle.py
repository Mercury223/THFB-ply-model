import math
import cadquery as cq
from pathlib import Path
from cadquery.occ_impl.shapes import loft as cq_loft

from OCP.BRepBuilderAPI import BRepBuilderAPI_Sewing, BRepBuilderAPI_MakeSolid
from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.TopAbs import TopAbs_SHELL
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS


# ============================================================
# 参数
# ============================================================

BLADE_HEIGHT = 80.0

r_blade = 33.0
R_fillet = 2.0
off = 60.0

d_inset = 1.0

N_BLADE_SECTIONS = 8
N_FILLET_SECTIONS = 24
N_ENDWALL_SECTIONS = 32

N_INSET_SECTIONS = 12

N_SIDE_FILLET_POINTS = 40
N_SIDE_ENDWALL_POINTS = 60

delta_theta_deg = 60.0
theta_center_deg = -60.0

theta_c = math.radians(theta_center_deg)
delta_theta = math.radians(delta_theta_deg)


# ============================================================
# 基础函数
# ============================================================

def v3(x, y, z):
    return cq.Vector(float(x), float(y), float(z))


def delta_params(delta):
    """
    连续缩进层参数。

        r_delta = r - delta
        R_delta = R_fillet + delta
        z_endwall_delta = -delta
        C_delta = r_delta * delta_theta
    """
    r0 = r_blade - delta
    R0 = R_fillet + delta
    z_endwall = -delta
    C = r0 * delta_theta

    return r0, R0, z_endwall, C


def theta_side(rho, sign, C):
    return theta_c + sign * C / (2.0 * rho)


def point_polar(rho, theta, z):
    return v3(
        rho * math.cos(theta),
        rho * math.sin(theta),
        z
    )


def fillet_z(rho, r0, R0, z_endwall):
    d_rho = r0 + R0 - rho

    return z_endwall + R0 - math.sqrt(
        max(0.0, R0**2 - d_rho**2)
    )


def arc_section(rho, z, C):
    """
    固定 rho、z 的横向圆弧截面。
    """
    theta_a = theta_side(rho, -1, C)
    theta_b = theta_side(rho, +1, C)
    theta_m = 0.5 * (theta_a + theta_b)

    p0 = point_polar(rho, theta_a, z)
    pm = point_polar(rho, theta_m, z)
    p1 = point_polar(rho, theta_b, z)

    return cq.Edge.makeThreePointArc(p0, pm, p1)


def make_spline_edge(points):
    if len(points) < 2:
        raise ValueError("至少需要两个点。")

    if len(points) == 2:
        return cq.Edge.makeLine(points[0], points[1])

    return cq.Edge.makeSpline(points)


def loft_open_sections(sections, name="surface"):
    if len(sections) < 2:
        raise ValueError(f"{name}: loft 至少需要两个截面。")

    return cq_loft(
        sections,
        cap=False,
        ruled=False,
        continuity="C2",
        parametrization="chordal",
        degree=3,
        compat=True,
        smoothing=False
    )


# ============================================================
# 主曲面：outer / inner — 统一连续 loft，消除 blade-fillet-endwall 间内部边界
# ============================================================

def unified_surface_smooth(r0, R0, z_endwall, C):
    """单一 loft 曲面，覆盖 blade + fillet + endwall 三个区域。"""
    sections = []
    z_bottom = z_endwall + R0  # = R_fillet

    # Blade — 从 z=BLADE_HEIGHT 向下到 z=R_fillet，定 ρ=r0
    for i in range(N_BLADE_SECTIONS + 1):
        u = i / N_BLADE_SECTIONS
        z = BLADE_HEIGHT - (BLADE_HEIGHT - z_bottom) * u
        sections.append(arc_section(r0, z, C))

    # Fillet — ρ 从 r0 到 r0+R0（跳过第一节，避免重复）
    for i in range(1, N_FILLET_SECTIONS + 1):
        u = i / N_FILLET_SECTIONS
        rho = r0 + R0 * u
        z = fillet_z(rho, r0, R0, z_endwall)
        sections.append(arc_section(rho, z, C))

    # Endwall — ρ 从 r0+R0 到 r0+R0+off（跳过第一节）
    rho_start = r0 + R0
    rho_end = r0 + R0 + off
    for i in range(1, N_ENDWALL_SECTIONS + 1):
        u = i / N_ENDWALL_SECTIONS
        rho = rho_start + (rho_end - rho_start) * u
        sections.append(arc_section(rho, z_endwall, C))

    return loft_open_sections(sections, "unified_surface")


def make_unified_surface(prefix, delta):
    r0, R0, z_endwall, C = delta_params(delta)
    return {f"{prefix}_surface": unified_surface_smooth(r0, R0, z_endwall, C)}


# ============================================================
# 侧壁曲线（保留用于边界曲线诊断，不再用于曲面生成）
# ============================================================

def blade_side_curve_at_delta(sign, delta):
    r0, R0, z_endwall, C = delta_params(delta)

    rho = r0
    theta = theta_side(rho, sign, C)

    p_bottom = point_polar(rho, theta, R_fillet)
    p_top = point_polar(rho, theta, BLADE_HEIGHT)

    return cq.Edge.makeLine(p_bottom, p_top)


def fillet_side_curve_at_delta(sign, delta):
    r0, R0, z_endwall, C = delta_params(delta)

    pts = []

    for i in range(N_SIDE_FILLET_POINTS + 1):
        t = i / N_SIDE_FILLET_POINTS

        rho = r0 + R0 * t
        z = fillet_z(rho, r0, R0, z_endwall)
        theta = theta_side(rho, sign, C)

        pts.append(point_polar(rho, theta, z))

    return make_spline_edge(pts)


def endwall_side_curve_at_delta(sign, delta):
    r0, R0, z_endwall, C = delta_params(delta)

    rho_start = r_blade + R_fillet
    rho_end = r_blade + R_fillet + off

    pts = []

    for i in range(N_SIDE_ENDWALL_POINTS + 1):
        u = i / N_SIDE_ENDWALL_POINTS

        rho = rho_start + (rho_end - rho_start) * u
        z = z_endwall
        theta = theta_side(rho, sign, C)

        pts.append(point_polar(rho, theta, z))

    return make_spline_edge(pts)


# ============================================================
# 边提取：从主 loft 曲面中提取边界边，保证边共享
# ============================================================

def _edge_z(edge):
    return edge.Center().z


def _edge_theta(edge):
    c = edge.Center()
    return math.atan2(c.y, c.x)


def _extract_unified_edges(face):
    """从统一连续曲面中提取 top / bottom / left / right 四边。"""
    edges = list(face.outerWire().Edges())

    indexed = [(_edge_z(e), i, e) for i, e in enumerate(edges)]
    indexed.sort(key=lambda x: x[0])

    top = indexed[-1][2]
    bottom = indexed[0][2]

    excl = {indexed[0][1], indexed[-1][1]}
    remaining = [e for i, e in enumerate(edges) if i not in excl]
    if len(remaining) == 0:
        remaining = [bottom, top]

    remaining.sort(key=_edge_theta)
    left = remaining[0]
    right = remaining[-1]

    return {"left": left, "right": right, "bottom": bottom, "top": top}


def _reverse_edge(e):
    """OCP 级别反转边的方向。"""
    rev = e.wrapped.Reversed()
    return cq.Shape.cast(rev)


def _reverse_face(f):
    """OCP 级别反转面的方向。"""
    rev = f.wrapped.Reversed()
    return cq.Shape.cast(rev)


def _make_ruled_surface(edge_a, edge_b, name="ruled"):
    """在两条边之间生成直纹面，尝试所有方向组合。"""
    combos = [
        (edge_a, edge_b),
        (_reverse_edge(edge_a), edge_b),
        (edge_a, _reverse_edge(edge_b)),
        (_reverse_edge(edge_a), _reverse_edge(edge_b)),
    ]
    for ea, eb in combos:
        try:
            face = cq.Face.makeRuledSurface(ea, eb)
            if face.isValid():
                return face
        except Exception:
            continue
    raise RuntimeError(f"makeRuledSurface 失败: {name}")


# ============================================================
# 侧壁曲面 — 从主曲面提取边界，用直纹面保证边共享
# ============================================================

def _build_side_surfaces(outer_edges, inner_edges):
    """用直纹面构建左右侧壁（各一张，覆盖全高）。"""
    sides = {}
    for sign, side_key in [(-1, "side_a"), (+1, "side_b")]:
        key = "left" if sign == -1 else "right"
        name = f"{side_key}_surface"
        sides[name] = _make_ruled_surface(
            outer_edges[key], inner_edges[key], name
        )
    return sides


# ============================================================
# 顶盖和外端盖 — 用直纹面保证边共享
# ============================================================

def _build_cap_surfaces(outer_edges, inner_edges):
    """用直纹面构建顶盖和外端盖。"""
    return {
        "top_cap_surface": _make_ruled_surface(
            outer_edges["top"], inner_edges["top"], "top_cap"
        ),
        "end_cap_surface": _make_ruled_surface(
            outer_edges["bottom"], inner_edges["bottom"], "end_cap"
        ),
    }


# ============================================================
# 边界曲线，仅用于检查
# ============================================================

def make_polyline_wire(pts):
    edges = []

    for i in range(len(pts) - 1):
        if pts[i].sub(pts[i + 1]).Length > 1e-9:
            edges.append(cq.Edge.makeLine(pts[i], pts[i + 1]))

    return cq.Wire.assembleEdges(edges)


def full_side_boundary_wire(sign, delta):
    r0, R0, z_endwall, C = delta_params(delta)

    pts = []

    theta0 = theta_side(r0, sign, C)

    pts.append(point_polar(r0, theta0, BLADE_HEIGHT))
    pts.append(point_polar(r0, theta0, R_fillet))

    for i in range(1, N_SIDE_FILLET_POINTS + 1):
        t = i / N_SIDE_FILLET_POINTS

        rho = r0 + R0 * t
        z = fillet_z(rho, r0, R0, z_endwall)
        theta = theta_side(rho, sign, C)

        pts.append(point_polar(rho, theta, z))

    rho_start = r_blade + R_fillet

    for i in range(1, N_SIDE_ENDWALL_POINTS + 1):
        u = i / N_SIDE_ENDWALL_POINTS

        rho = rho_start + off * u
        z = z_endwall
        theta = theta_side(rho, sign, C)

        pts.append(point_polar(rho, theta, z))

    return make_polyline_wire(pts)


# ============================================================
# 曲面 sewing 成实体（含方向修复与诊断）
# ============================================================

def _compute_interior_point():
    """返回固体内一点（用于判断面法向）。"""
    r_mid = r_blade - d_inset * 0.5
    z_mid = BLADE_HEIGHT * 0.5
    return point_polar(r_mid, theta_c, z_mid)


def _fix_face_orientations(faces):
    """反转法向指向实体内部的面。"""
    interior = _compute_interior_point()
    fixed = []
    for i, face in enumerate(faces):
        try:
            fc = face.Center()
            normal = face.normalAt(fc)
            to_interior = interior - fc
            if normal.dot(to_interior) > 0:
                face = _reverse_face(face)
        except Exception:
            pass
        fixed.append(face)
    return fixed


def _try_sew_and_make_solid(faces, tol):
    """尝试用给定容差 sewing 并生成实体。返回 (solid, None) 或 (None, error_msg)。"""
    sewing = BRepBuilderAPI_Sewing(tol)

    for face in faces:
        sewing.Add(face.wrapped)

    sewing.Perform()

    free_edges = sewing.NbFreeEdges()
    print(f"  [SEW tol={tol:.0e}] free_edges={free_edges}, "
          f"contig_edges={sewing.NbContigousEdges()}")

    if free_edges > 0:
        for i in range(1, free_edges + 1):
            fe = sewing.FreeEdge(i)
            # fe is a TopoDS_Edge
            try:
                edge = cq.Shape.cast(fe)
                c = edge.Center()
                print(f"    free edge {i}: center=({c.x:.3f}, {c.y:.3f}, {c.z:.3f})")
            except Exception:
                print(f"    free edge {i}: <could not inspect>")

    sewed_shape = sewing.SewedShape()

    shell = None
    if sewed_shape.ShapeType() == TopAbs_SHELL:
        shell = TopoDS.Shell_s(sewed_shape)
    else:
        explorer = TopExp_Explorer(sewed_shape, TopAbs_SHELL)
        if explorer.More():
            shell = TopoDS.Shell_s(explorer.Current())

    if shell is None:
        return None, f"sewing 未生成 Shell (free_edges={free_edges})"

    solid_maker = BRepBuilderAPI_MakeSolid()
    solid_maker.Add(shell)
    solid_occ = solid_maker.Solid()
    solid = cq.Shape.cast(solid_occ)

    analyzer = BRepCheck_Analyzer(solid.wrapped)
    if not analyzer.IsValid():
        return None, f"BRepCheck_Analyzer 判定实体无效 (free_edges={free_edges})"

    return solid, None


def make_solid_from_surfaces(surface_shapes, sewing_tolerance=1e-5):
    faces = []
    for shape in surface_shapes:
        shape_faces = shape.Faces()
        if len(shape_faces) == 0:
            raise ValueError("某个曲面没有可用 Face，无法生成实体。")
        faces.extend(shape_faces)

    print(f"\n[Sewing] 共 {len(faces)} 个面")

    # 修复方向
    faces = _fix_face_orientations(faces)

    tolerances = [sewing_tolerance, 1e-4, 5e-4, 1e-3]
    last_err = None

    for tol in tolerances:
        solid, err = _try_sew_and_make_solid(faces, tol)
        if solid is not None:
            print(f"  [OK] 实体生成成功 (tol={tol:.0e})")
            return solid
        last_err = err
        print(f"  [FAIL] {err}")

    raise RuntimeError(
        f"所有容差均失败。最后一次错误: {last_err}\n"
        "请检查曲面边界是否闭合、法向是否一致。"
    )


# ============================================================
# 参数检查
# ============================================================

if d_inset <= 0:
    raise ValueError("d_inset 必须大于 0。")

if r_blade - d_inset <= 0:
    raise ValueError("d_inset 太大，导致 r_blade - d_inset <= 0。")


def print_layer_info(name, delta):
    r0, R0, z_endwall, C = delta_params(delta)

    rho_fillet_end = r0 + R0
    rho_end = r0 + R0 + off

    print(f"\n[{name}]")
    print(f"  delta          = {delta:.6f}")
    print(f"  r0             = {r0:.6f}")
    print(f"  R0             = {R0:.6f}")
    print(f"  z_endwall      = {z_endwall:.6f}")
    print(f"  C              = {C:.6f}")
    print(f"  blade arc len  = {C:.6f}")
    print(f"  rho range      = {r0:.6f} -> {rho_fillet_end:.6f} -> {rho_end:.6f}")

    print(
        f"  blade angles   = "
        f"{theta_center_deg - delta_theta_deg / 2.0:.6f}°"
        f" -> "
        f"{theta_center_deg + delta_theta_deg / 2.0:.6f}°"
    )

    print(
        f"  fillet angles  = "
        f"{theta_center_deg - math.degrees(C / (2.0 * rho_fillet_end)):.6f}°"
        f" -> "
        f"{theta_center_deg + math.degrees(C / (2.0 * rho_fillet_end)):.6f}°"
    )

    print(
        f"  endwall angles = "
        f"{theta_center_deg - math.degrees(C / (2.0 * rho_end)):.6f}°"
        f" -> "
        f"{theta_center_deg + math.degrees(C / (2.0 * rho_end)):.6f}°"
    )


print("Closed smooth hyperbolic spiral solid")
print(f"theta_center = {theta_center_deg:.6f}°")
print(f"delta_theta  = {delta_theta_deg:.6f}°")
print(f"d_inset      = {d_inset:.6f}")

print_layer_info("OUTER delta=0", 0.0)
print_layer_info("INNER delta=d", d_inset)


# ============================================================
# 生成曲面 — 统一连续主曲面 + 直纹面侧壁/端盖
# ============================================================

# 1. 外层 / 内层统一主曲面 (loft，无内部边界)
outer_dict = make_unified_surface("outer", 0.0)
inner_dict = make_unified_surface("inner", d_inset)
outer_surface = outer_dict["outer_surface"]
inner_surface = inner_dict["inner_surface"]

# 2. 从统一曲面提取边界边
outer_edges = _extract_unified_edges(outer_surface)
inner_edges = _extract_unified_edges(inner_surface)

# 3. 侧壁 — 左右各一张直纹面，覆盖全高
side_surfaces = _build_side_surfaces(outer_edges, inner_edges)

# 4. 顶盖 + 外端盖
cap_surfaces = _build_cap_surfaces(outer_edges, inner_edges)

print(f"\n[Edge extraction] outer {len(outer_edges)} edges, inner {len(inner_edges)} edges")


# ============================================================
# 由曲面生成实体
# ============================================================

all_surface_shapes = (
    [outer_surface]
    + [inner_surface]
    + list(side_surfaces.values())
    + list(cap_surfaces.values())
)

closed_solid = make_solid_from_surfaces(
    all_surface_shapes,
    sewing_tolerance=1e-5
)

print("\n[SOLID] closed_solid generated successfully.")


# ============================================================
# 保留边界曲线
# ============================================================

boundary_curves = {
    "outer_side_a_boundary": full_side_boundary_wire(-1, 0.0),
    "outer_side_b_boundary": full_side_boundary_wire(+1, 0.0),
    "inner_side_a_boundary": full_side_boundary_wire(-1, d_inset),
    "inner_side_b_boundary": full_side_boundary_wire(+1, d_inset),
}


# ============================================================
# 导出 STEP
# ============================================================

step_path = Path(__file__).resolve().parent / "closed_smooth_hyperbolic_solid.step"

if step_path.exists():
    step_path.unlink()

assy = cq.Assembly(name="CLOSED_SMOOTH_HYPERBOLIC_SOLID")


surface_colors = {
    # main
    "outer_surface": cq.Color(0.8, 0.8, 0.9, 0.35),
    "inner_surface": cq.Color(0.45, 0.45, 0.65, 0.35),

    # sides
    "side_a_surface": cq.Color(0.9, 0.2, 0.2, 0.35),
    "side_b_surface": cq.Color(0.2, 0.2, 0.9, 0.35),

    # caps
    "top_cap_surface": cq.Color(0.2, 0.9, 0.2, 0.35),
    "end_cap_surface": cq.Color(0.9, 0.5, 0.1, 0.35),
}

curve_colors = {
    "outer_side_a_boundary": cq.Color(1.0, 0.0, 0.0),
    "outer_side_b_boundary": cq.Color(0.0, 0.0, 1.0),
    "inner_side_a_boundary": cq.Color(0.45, 0.0, 0.0),
    "inner_side_b_boundary": cq.Color(0.0, 0.0, 0.45),
}


# 1. 添加实体
assy.add(
    closed_solid,
    name="closed_solid",
    color=cq.Color(0.7, 0.7, 0.7, 0.85)
)


# 2. 保留曲面
surface_groups = {
    "outer_surface": outer_surface,
    "inner_surface": inner_surface,
    **side_surfaces,
    **cap_surfaces,
}
for name, shape in surface_groups.items():
    assy.add(
        shape,
        name=name,
        color=surface_colors.get(name, cq.Color(0.8, 0.8, 0.8, 0.35))
    )


# 3. 保留边界曲线
for name, wire in boundary_curves.items():
    assy.add(
        wire,
        name=name,
        color=curve_colors.get(name, cq.Color(1.0, 1.0, 1.0))
    )


assy.export(str(step_path))

print(f"\n[STEP] {step_path}")