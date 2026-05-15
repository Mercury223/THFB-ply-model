import math
import cadquery as cq
from pathlib import Path
from cadquery.occ_impl.shapes import loft as cq_loft

from OCP.BRepBuilderAPI import BRepBuilderAPI_Sewing, BRepBuilderAPI_MakeSolid
from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.TopAbs import TopAbs_SHELL, TopAbs_SOLID
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS

try:
    from OCP.ShapeFix import ShapeFix_Solid
    HAS_SHAPEFIX_SOLID = True
except Exception:
    ShapeFix_Solid = None
    HAS_SHAPEFIX_SOLID = False

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
    连续缩进层参数：

        r_delta = r - delta
        R_delta = R_fillet + delta
        z_endwall_delta = -delta
        C_delta = r_delta * delta_theta

    注意：
        r_delta + R_delta = r_blade + R_fillet，
        因此倒圆末端与缘板起点在所有 delta 层上严格共径向位置。
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
    inside = max(0.0, R0**2 - d_rho**2)
    return z_endwall + R0 - math.sqrt(inside)


def arc_section(rho, z, C):
    """
    固定 rho、z 的横向圆弧截面。
    截面方向统一为 sign=-1 -> sign=+1。
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

    # CadQuery 的底层 loft 支持 continuity、parametrization、degree、compat 等参数。
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
# 主曲面：outer / inner
# ============================================================

def blade_surface_smooth(r0, R0, z_endwall, C):
    sections = []

    z0 = z_endwall + R0
    z1 = BLADE_HEIGHT

    for i in range(N_BLADE_SECTIONS + 1):
        u = i / N_BLADE_SECTIONS
        z = z0 + (z1 - z0) * u
        sections.append(arc_section(r0, z, C))

    return loft_open_sections(sections, "blade_surface")


def fillet_surface_smooth(r0, R0, z_endwall, C):
    sections = []

    for i in range(N_FILLET_SECTIONS + 1):
        u = i / N_FILLET_SECTIONS

        rho = r0 + R0 * u
        z = fillet_z(rho, r0, R0, z_endwall)

        sections.append(arc_section(rho, z, C))

    return loft_open_sections(sections, "fillet_surface")


def endwall_surface_smooth(r0, R0, off0, z_endwall, C):
    sections = []

    rho_start = r0 + R0
    rho_end = r0 + R0 + off0

    for i in range(N_ENDWALL_SECTIONS + 1):
        u = i / N_ENDWALL_SECTIONS

        rho = rho_start + (rho_end - rho_start) * u
        z = z_endwall

        sections.append(arc_section(rho, z, C))

    return loft_open_sections(sections, "endwall_surface")


def make_main_surface_group(prefix, delta):
    r0, R0, z_endwall, C = delta_params(delta)

    return {
        f"{prefix}_blade_surface": blade_surface_smooth(
            r0, R0, z_endwall, C
        ),
        f"{prefix}_fillet_surface": fillet_surface_smooth(
            r0, R0, z_endwall, C
        ),
        f"{prefix}_endwall_surface": endwall_surface_smooth(
            r0, R0, off, z_endwall, C
        ),
    }


# ============================================================
# 侧壁曲线
# ============================================================

def blade_side_curve_at_delta(sign, delta):
    """
    叶身侧壁在某一 delta 层上的纵向边界曲线。

    这里底部 z 使用 z_endwall + R0，而不是直接写 R_fillet。
    数值上二者相等，但用同一参数公式可以减少边界漂移风险。
    """
    r0, R0, z_endwall, C = delta_params(delta)

    rho = r0
    theta = theta_side(rho, sign, C)

    z_bottom = z_endwall + R0
    z_top = BLADE_HEIGHT

    p_bottom = point_polar(rho, theta, z_bottom)
    p_top = point_polar(rho, theta, z_top)

    return cq.Edge.makeLine(p_bottom, p_top)


def fillet_side_curve_at_delta(sign, delta):
    """
    倒圆侧壁在某一 delta 层上的边界曲线。
    参数形式等价于：

        rho(t,delta) = r + R_fillet * t - delta * (1 - t)
        z(t,delta)   = R_fillet - (R_fillet + delta) * sqrt(2t - t^2)

    但这里统一复用 r0、R0、z_endwall、fillet_z。
    """
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
    """
    缘板侧壁在某一 delta 层上的边界曲线。

    rho_start 使用 r0 + R0。
    由于 r0 + R0 = r_blade + R_fillet，该写法与主曲面严格同源。
    """
    r0, R0, z_endwall, C = delta_params(delta)

    rho_start = r0 + R0
    rho_end = r0 + R0 + off

    pts = []

    for i in range(N_SIDE_ENDWALL_POINTS + 1):
        u = i / N_SIDE_ENDWALL_POINTS

        rho = rho_start + (rho_end - rho_start) * u
        z = z_endwall
        theta = theta_side(rho, sign, C)

        pts.append(point_polar(rho, theta, z))

    return make_spline_edge(pts)


# ============================================================
# 侧壁曲面
# ============================================================

def blade_side_surface(sign):
    sections = []

    for k in range(N_INSET_SECTIONS + 1):
        delta = d_inset * k / N_INSET_SECTIONS
        sections.append(blade_side_curve_at_delta(sign, delta))

    return loft_open_sections(sections, f"blade_side_{sign}")


def fillet_side_surface(sign):
    sections = []

    for k in range(N_INSET_SECTIONS + 1):
        delta = d_inset * k / N_INSET_SECTIONS
        sections.append(fillet_side_curve_at_delta(sign, delta))

    return loft_open_sections(sections, f"fillet_side_{sign}")


def endwall_side_surface(sign):
    sections = []

    for k in range(N_INSET_SECTIONS + 1):
        delta = d_inset * k / N_INSET_SECTIONS
        sections.append(endwall_side_curve_at_delta(sign, delta))

    return loft_open_sections(sections, f"endwall_side_{sign}")


# ============================================================
# 顶盖和外端盖
# ============================================================

def top_cap_surface():
    """
    顶部封闭面：
        z = BLADE_HEIGHT
        delta: 0 -> d_inset
        rho = r_blade - delta

    截面方向仍然统一为 sign=-1 -> sign=+1。
    """
    sections = []

    for k in range(N_INSET_SECTIONS + 1):
        delta = d_inset * k / N_INSET_SECTIONS
        r0, R0, z_endwall, C = delta_params(delta)

        sections.append(
            arc_section(r0, BLADE_HEIGHT, C)
        )

    return loft_open_sections(sections, "top_cap_surface")


def end_cap_surface():
    """
    缘板外端封闭面：
        rho = r0 + R0 + off = r_blade + R_fillet + off
        z = z_endwall = -delta
        delta: 0 -> d_inset
    """
    sections = []

    for k in range(N_INSET_SECTIONS + 1):
        delta = d_inset * k / N_INSET_SECTIONS
        r0, R0, z_endwall, C = delta_params(delta)

        rho_end = r0 + R0 + off

        sections.append(
            arc_section(rho_end, z_endwall, C)
        )

    return loft_open_sections(sections, "end_cap_surface")


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
    pts.append(point_polar(r0, theta0, z_endwall + R0))

    for i in range(1, N_SIDE_FILLET_POINTS + 1):
        t = i / N_SIDE_FILLET_POINTS

        rho = r0 + R0 * t
        z = fillet_z(rho, r0, R0, z_endwall)
        theta = theta_side(rho, sign, C)

        pts.append(point_polar(rho, theta, z))

    rho_start = r0 + R0
    rho_end = r0 + R0 + off

    for i in range(1, N_SIDE_ENDWALL_POINTS + 1):
        u = i / N_SIDE_ENDWALL_POINTS

        rho = rho_start + (rho_end - rho_start) * u
        z = z_endwall
        theta = theta_side(rho, sign, C)

        pts.append(point_polar(rho, theta, z))

    return make_polyline_wire(pts)


# ============================================================
# Sewing / Solid 工具函数
# ============================================================

def collect_faces_from_shapes(surface_shapes):
    """
    从 CadQuery Shape 中收集所有 Face。
    """
    faces = []

    for shape in surface_shapes:
        shape_faces = shape.Faces()

        if len(shape_faces) == 0:
            raise ValueError("某个曲面没有可用 Face，无法生成实体。")

        faces.extend(shape_faces)

    return faces


def collect_shells_from_shape(shape):
    """
    从 sewing 结果中提取 shell。
    sewing 结果可能是 Shell、Solid 或 Compound。
    """
    shells = []

    if shape.ShapeType() == TopAbs_SHELL:
        shells.append(TopoDS.Shell_s(shape))
        return shells

    if shape.ShapeType() == TopAbs_SOLID:
        explorer = TopExp_Explorer(shape, TopAbs_SHELL)
        while explorer.More():
            shells.append(TopoDS.Shell_s(explorer.Current()))
            explorer.Next()
        return shells

    explorer = TopExp_Explorer(shape, TopAbs_SHELL)

    while explorer.More():
        shells.append(TopoDS.Shell_s(explorer.Current()))
        explorer.Next()

    return shells


def make_solid_from_shell_with_fix(shell):
    """
    优先使用 ShapeFix_Solid.SolidFromShell。
    若当前 OCP 环境没有 ShapeFix_Solid，则退回 BRepBuilderAPI_MakeSolid。
    """
    if HAS_SHAPEFIX_SOLID:
        fixer = ShapeFix_Solid()
        solid_occ = fixer.SolidFromShell(shell)
        return cq.Shape.cast(solid_occ)

    solid_maker = BRepBuilderAPI_MakeSolid()
    solid_maker.Add(shell)
    solid_occ = solid_maker.Solid()
    return cq.Shape.cast(solid_occ)


def try_make_solid_once(surface_shapes, sewing_tolerance):
    faces = collect_faces_from_shapes(surface_shapes)

    # 参数含义：
    #   option1=True: connexity sewing
    #   option2=True: face analysis
    #   option3=True: cutting of free edges
    #   option4=False: non-manifold processing off
    sewing = BRepBuilderAPI_Sewing(
        sewing_tolerance,
        True,
        True,
        True,
        False
    )

    for face in faces:
        sewing.Add(face.wrapped)

    sewing.Perform()

    sewed_shape = sewing.SewedShape()

    nb_free = sewing.NbFreeEdges()
    nb_multi = sewing.NbMultipleEdges()
    nb_contig = sewing.NbContigousEdges()

    shells = collect_shells_from_shape(sewed_shape)

    if len(shells) == 0:
        return None, {
            "ok": False,
            "reason": "Sewing 后没有得到 Shell。",
            "tolerance": sewing_tolerance,
            "nb_free_edges": nb_free,
            "nb_multiple_edges": nb_multi,
            "nb_contiguous_edges": nb_contig,
            "nb_shells": 0,
        }

    if len(shells) != 1:
        return None, {
            "ok": False,
            "reason": f"Sewing 后得到 {len(shells)} 个 Shell，不是单一闭合 Shell。",
            "tolerance": sewing_tolerance,
            "nb_free_edges": nb_free,
            "nb_multiple_edges": nb_multi,
            "nb_contiguous_edges": nb_contig,
            "nb_shells": len(shells),
        }

    shell = shells[0]

    solid = make_solid_from_shell_with_fix(shell)

    analyzer = BRepCheck_Analyzer(solid.wrapped)
    is_valid = analyzer.IsValid()

    if not is_valid:
        return None, {
            "ok": False,
            "reason": "已经生成 Solid，但 BRepCheck_Analyzer 判断实体无效。",
            "tolerance": sewing_tolerance,
            "nb_free_edges": nb_free,
            "nb_multiple_edges": nb_multi,
            "nb_contiguous_edges": nb_contig,
            "nb_shells": len(shells),
        }

    return solid, {
        "ok": True,
        "reason": "Solid valid.",
        "tolerance": sewing_tolerance,
        "nb_free_edges": nb_free,
        "nb_multiple_edges": nb_multi,
        "nb_contiguous_edges": nb_contig,
        "nb_shells": len(shells),
    }


def make_solid_from_surfaces(surface_shapes, sewing_tolerances=None):
    """
    将一组封闭曲面 sewing 成 shell，再转为 solid。

    重要修改：
        1. 支持多 sewing tolerance 回退；
        2. 输出 sewing free edge / multiple edge 诊断；
        3. 优先使用 ShapeFix_Solid.SolidFromShell 修正 shell 方向；
        4. 只有 BRepCheck_Analyzer 判断有效时才返回实体。
    """
    if sewing_tolerances is None:
        sewing_tolerances = [1e-5, 3e-5, 1e-4, 3e-4, 1e-3]

    diagnostics = []

    for tol in sewing_tolerances:
        solid, info = try_make_solid_once(surface_shapes, tol)
        diagnostics.append(info)

        print(
            "[SEWING] "
            f"tol={info['tolerance']:.1e}, "
            f"ok={info['ok']}, "
            f"shells={info['nb_shells']}, "
            f"free={info['nb_free_edges']}, "
            f"multi={info['nb_multiple_edges']}, "
            f"contig={info['nb_contiguous_edges']}, "
            f"reason={info['reason']}"
        )

        if solid is not None:
            return solid

    msg_lines = [
        "无法生成有效 Solid。所有 sewing_tolerance 均失败。",
        "诊断信息如下："
    ]

    for info in diagnostics:
        msg_lines.append(
            f"  tol={info['tolerance']:.1e}, "
            f"ok={info['ok']}, "
            f"shells={info['nb_shells']}, "
            f"free={info['nb_free_edges']}, "
            f"multi={info['nb_multiple_edges']}, "
            f"contig={info['nb_contiguous_edges']}, "
            f"reason={info['reason']}"
        )

    msg_lines.append(
        "建议检查：1）free edge 是否非零；2）side/cap 与 outer/inner 是否严格共边；"
        "3）loft 曲线方向是否一致；4）是否存在自交或局部退化。"
    )

    raise RuntimeError("\n".join(msg_lines))


# ============================================================
# 参数检查
# ============================================================

if d_inset <= 0:
    raise ValueError("d_inset 必须大于 0。")

if r_blade - d_inset <= 0:
    raise ValueError("d_inset 太大，导致 r_blade - d_inset <= 0。")

if BLADE_HEIGHT <= R_fillet:
    raise ValueError("BLADE_HEIGHT 必须大于 R_fillet。")

if off <= 0:
    raise ValueError("off 必须大于 0。")

if delta_theta <= 0:
    raise ValueError("delta_theta 必须大于 0。")


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
print(f"ShapeFix_Solid available = {HAS_SHAPEFIX_SOLID}")

print_layer_info("OUTER delta=0", 0.0)
print_layer_info("INNER delta=d", d_inset)


# ============================================================
# 生成曲面
# ============================================================

outer_surfaces = make_main_surface_group("outer", 0.0)
inner_surfaces = make_main_surface_group("inner", d_inset)

side_surfaces = {
    "side_a_blade_surface": blade_side_surface(-1),
    "side_a_fillet_surface": fillet_side_surface(-1),
    "side_a_endwall_surface": endwall_side_surface(-1),

    "side_b_blade_surface": blade_side_surface(+1),
    "side_b_fillet_surface": fillet_side_surface(+1),
    "side_b_endwall_surface": endwall_side_surface(+1),
}

cap_surfaces = {
    "top_cap_surface": top_cap_surface(),
    "end_cap_surface": end_cap_surface(),
}


# ============================================================
# 由曲面生成实体
# ============================================================

all_surface_shapes = (
    list(outer_surfaces.values())
    + list(inner_surfaces.values())
    + list(side_surfaces.values())
    + list(cap_surfaces.values())
)

closed_solid = make_solid_from_surfaces(
    all_surface_shapes,
    sewing_tolerances=[1e-5, 3e-5, 1e-4, 3e-4, 1e-3]
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
    # outer
    "outer_blade_surface": cq.Color(0.8, 0.8, 0.9, 0.35),
    "outer_fillet_surface": cq.Color(1.0, 0.4, 0.2, 0.35),
    "outer_endwall_surface": cq.Color(0.9, 0.9, 0.6, 0.35),

    # inner
    "inner_blade_surface": cq.Color(0.45, 0.45, 0.65, 0.35),
    "inner_fillet_surface": cq.Color(0.65, 0.2, 0.1, 0.35),
    "inner_endwall_surface": cq.Color(0.55, 0.55, 0.25, 0.35),

    # side a
    "side_a_blade_surface": cq.Color(0.9, 0.2, 0.2, 0.35),
    "side_a_fillet_surface": cq.Color(0.9, 0.1, 0.1, 0.35),
    "side_a_endwall_surface": cq.Color(0.75, 0.15, 0.15, 0.35),

    # side b
    "side_b_blade_surface": cq.Color(0.2, 0.2, 0.9, 0.35),
    "side_b_fillet_surface": cq.Color(0.1, 0.1, 0.9, 0.35),
    "side_b_endwall_surface": cq.Color(0.15, 0.15, 0.75, 0.35),

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
for group in [outer_surfaces, inner_surfaces, side_surfaces, cap_surfaces]:
    for name, shape in group.items():
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