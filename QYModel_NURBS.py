# ============================================================
# 本文件作用：
# 1. 使用完整 3 次 4 控制点 Bezier 曲线生成一组曲线、曲面和实体；
# 2. 使用原始曲线弧长 25%~50% 的曲线段生成一组曲线、曲面和实体；
# 3. 使用原始曲线弧长 50%~75% 的曲线段生成一组曲线、曲面和实体；
# 4. 三组结构同时导出到 STEP，用于比较完整体与两个局部体。
# ============================================================

import math
from pathlib import Path

import cadquery as cq

from cadquery.occ_impl.shapes import loft as cq_loft

from OCP.BRepAdaptor import BRepAdaptor_Curve
from OCP.BRepBuilderAPI import (
    BRepBuilderAPI_MakeEdge,
    BRepBuilderAPI_Sewing,
    BRepBuilderAPI_MakeSolid,
)
from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.Geom import Geom_BezierCurve
from OCP.ShapeFix import ShapeFix_Solid
from OCP.TColgp import TColgp_Array1OfPnt
from OCP.TopAbs import TopAbs_SHELL
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS
from OCP.gp import gp_Pnt, gp_Vec


# ============================================================
# 参数
# ============================================================

R_FILLET = 6.0
BLADE_HEIGHT = 50.0
ENDWALL_OFF = 18.0
D_INSET = 3.0

N_CURVE_SAMPLE = 400
N_ARCLEN_SAMPLE = 2500
N_SECTION_SAMPLE = 180

N_BLADE_SECTIONS = 6
N_FILLET_SECTIONS = 18
N_ENDWALL_SECTIONS = 16
N_INSET_SIDE_SECTIONS = 10

OUTER_SIDE = "left"
INNER_SIDE = "right"

EXPORT_STEP = True


# ============================================================
# 基础向量函数
# ============================================================

def v3(x, y, z=0.0):
    return cq.Vector(float(x), float(y), float(z))


def vec_sub(a, b):
    return cq.Vector(a.x - b.x, a.y - b.y, a.z - b.z)


def vec_len(a):
    return math.sqrt(a.x * a.x + a.y * a.y + a.z * a.z)


def cqvec_from_gpnt(p):
    return cq.Vector(p.X(), p.Y(), p.Z())


def cqvec_from_gpvec(v):
    return cq.Vector(v.X(), v.Y(), v.Z())


def left_normal_xy(t):
    return cq.Vector(-t.y, t.x, 0.0)


def right_normal_xy(t):
    return cq.Vector(t.y, -t.x, 0.0)


def get_normal_xy(tangent, side):
    if side == "left":
        return left_normal_xy(tangent)

    if side == "right":
        return right_normal_xy(tangent)

    raise ValueError("side 必须为 'left' 或 'right'。")


# ============================================================
# Edge 参数域与取点
# ============================================================

def edge_adaptor(edge):
    return BRepAdaptor_Curve(edge.wrapped)


def edge_param_range(edge):
    c = edge_adaptor(edge)
    return c.FirstParameter(), c.LastParameter()


def normalized_to_real_param(edge, t):
    u0, u1 = edge_param_range(edge)
    return u0 + (u1 - u0) * float(t)


def edge_point(edge, t):
    c = edge_adaptor(edge)
    u = normalized_to_real_param(edge, t)
    p = c.Value(u)
    return cqvec_from_gpnt(p)


def edge_tangent(edge, t, eps=1e-12):
    c = edge_adaptor(edge)
    u = normalized_to_real_param(edge, t)

    p = gp_Pnt()
    v = gp_Vec()

    c.D1(u, p, v)

    tv = cqvec_from_gpvec(v)
    l = vec_len(tv)

    if l < eps:
        raise ValueError(f"曲线在 t={t:.6f} 处导数接近零，无法计算切向。")

    return cq.Vector(tv.x / l, tv.y / l, tv.z / l)


# ============================================================
# 曲线构造
# ============================================================

def make_spline_edge(points):
    if len(points) < 2:
        raise ValueError("至少需要两个点。")

    if len(points) == 2:
        return cq.Edge.makeLine(points[0], points[1])

    return cq.Edge.makeSpline(points)


def make_cubic_bezier_edge(control_points):
    if len(control_points) != 4:
        raise ValueError("3 次 Bezier 曲线必须使用 4 个控制点。")

    arr = TColgp_Array1OfPnt(1, 4)

    for i, p in enumerate(control_points, start=1):
        arr.SetValue(i, gp_Pnt(p.x, p.y, p.z))

    bezier_curve = Geom_BezierCurve(arr)
    edge = BRepBuilderAPI_MakeEdge(bezier_curve).Edge()

    return cq.Edge(edge)


def resample_edge_to_spline(edge, n_piece=N_SECTION_SAMPLE):
    pts = []

    for i in range(n_piece + 1):
        t = i / n_piece
        pts.append(edge_point(edge, t))

    return make_spline_edge(pts), pts


# ============================================================
# 原始 Bezier 曲线
# ============================================================

def make_demo_bezier_base_curve_at_z(z_value):
    p0 = v3(-55.0, 0.0, z_value)
    p1 = v3(-25.0, 12.0, z_value)
    p2 = v3(25.0, 14.0, z_value)
    p3 = v3(58.0, 1.5, z_value)

    return make_cubic_bezier_edge([p0, p1, p2, p3])


# ============================================================
# 弧长计算与弧长比例截取
# ============================================================

def sampled_arclength_table(edge, n_sample=N_ARCLEN_SAMPLE):
    ts = []
    ss = []

    total = 0.0
    prev = None

    for i in range(n_sample + 1):
        t = i / n_sample
        p = edge_point(edge, t)

        if prev is not None:
            total += vec_len(vec_sub(p, prev))

        ts.append(t)
        ss.append(total)

        prev = p

    return ts, ss


def curve_length(edge, n_sample=N_ARCLEN_SAMPLE):
    _, ss = sampled_arclength_table(edge, n_sample)
    return ss[-1]


def parameter_at_arclength(ts, ss, target_s):
    if target_s <= ss[0]:
        return ts[0]

    if target_s >= ss[-1]:
        return ts[-1]

    lo = 0
    hi = len(ss) - 1

    while hi - lo > 1:
        mid = (lo + hi) // 2

        if ss[mid] < target_s:
            lo = mid
        else:
            hi = mid

    s0 = ss[lo]
    s1 = ss[hi]

    t0 = ts[lo]
    t1 = ts[hi]

    if abs(s1 - s0) < 1e-12:
        return t0

    alpha = (target_s - s0) / (s1 - s0)

    return t0 + alpha * (t1 - t0)


def subedge_by_arclength(edge, s0, s1, n_piece=N_SECTION_SAMPLE):
    if s1 <= s0:
        raise ValueError("s1 必须大于 s0。")

    ts, ss = sampled_arclength_table(edge, N_ARCLEN_SAMPLE)

    t0 = parameter_at_arclength(ts, ss, s0)
    t1 = parameter_at_arclength(ts, ss, s1)

    pts = []

    for i in range(n_piece + 1):
        a = i / n_piece
        t = t0 + (t1 - t0) * a
        pts.append(edge_point(edge, t))

    new_edge = make_spline_edge(pts)

    return new_edge, pts, t0, t1


def make_curve_by_arclength_fraction(
    edge,
    start_fraction,
    end_fraction,
    n_piece=N_SECTION_SAMPLE
):
    """
    按弧长比例截取曲线。

    例如：
        start_fraction = 0.25
        end_fraction   = 0.50

    表示截取原始曲线弧长 25% 到 50% 的部分。
    """
    if start_fraction < 0.0:
        raise ValueError("start_fraction 不能小于 0。")

    if end_fraction > 1.0:
        raise ValueError("end_fraction 不能大于 1。")

    if end_fraction <= start_fraction:
        raise ValueError("end_fraction 必须大于 start_fraction。")

    L = curve_length(edge)

    s0 = L * start_fraction
    s1 = L * end_fraction

    sub_edge, sub_pts, t0, t1 = subedge_by_arclength(
        edge,
        s0,
        s1,
        n_piece=n_piece
    )

    return {
        "edge": sub_edge,
        "points": sub_pts,
        "origin_length": L,
        "length": s1 - s0,
        "s0": s0,
        "s1": s1,
        "t0": t0,
        "t1": t1,
        "start_fraction": start_fraction,
        "end_fraction": end_fraction,
    }


def centered_equal_length_segment(
    target_edge,
    target_length,
    n_piece=N_SECTION_SAMPLE,
    name="curve"
):
    L = curve_length(target_edge)

    if L + 1e-8 < target_length:
        raise ValueError(
            f"{name}: 当前曲线长度 {L:.6f} 小于目标等弧长 {target_length:.6f}，"
            f"无法截取。请检查 OUTER_SIDE 是否选反，或减小偏置距离。"
        )

    s_mid = 0.5 * L
    s0 = s_mid - 0.5 * target_length
    s1 = s_mid + 0.5 * target_length

    marked_edge, marked_pts, t0, t1 = subedge_by_arclength(
        target_edge,
        s0,
        s1,
        n_piece=n_piece
    )

    return {
        "edge": marked_edge,
        "points": marked_pts,
        "curve_length": L,
        "target_length": target_length,
        "s0": s0,
        "s1": s1,
        "t0": t0,
        "t1": t1,
    }


# ============================================================
# 正常法向偏置
# ============================================================

def make_normal_offset_section_from_base(
    base_edge,
    offset_dist,
    z_value,
    side=OUTER_SIDE,
    n_sample=N_CURVE_SAMPLE
):
    pts = []

    for i in range(n_sample + 1):
        t = i / n_sample

        p = edge_point(base_edge, t)
        tangent = edge_tangent(base_edge, t)
        n = get_normal_xy(tangent, side)

        q = cq.Vector(
            p.x + offset_dist * n.x,
            p.y + offset_dist * n.y,
            z_value
        )

        pts.append(q)

    edge = make_spline_edge(pts)

    return edge, pts


def make_inset_base_curve_from_origin(
    origin_edge,
    inset_dist,
    z_value,
    side=INNER_SIDE
):
    if abs(inset_dist) < 1e-12:
        return resample_edge_to_spline(origin_edge, n_piece=N_SECTION_SAMPLE)

    return make_normal_offset_section_from_base(
        origin_edge,
        offset_dist=inset_dist,
        z_value=z_value,
        side=side,
        n_sample=N_CURVE_SAMPLE
    )


# ============================================================
# Loft 工具
# ============================================================

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
# blade 截面与曲面
# ============================================================

def build_blade_sections(
    base_edge,
    start_z,
    end_z,
    side=OUTER_SIDE
):
    sections = []

    for i in range(N_BLADE_SECTIONS + 1):
        q = i / N_BLADE_SECTIONS
        z = start_z + (end_z - start_z) * q

        sec, pts = make_normal_offset_section_from_base(
            base_edge,
            offset_dist=0.0,
            z_value=z,
            side=side,
            n_sample=N_SECTION_SAMPLE
        )

        sections.append({
            "index": i,
            "z": z,
            "edge": sec,
            "points": pts,
        })

    return sections


def build_blade_surface_from_sections(blade_sections, name="blade_surface"):
    edges = [item["edge"] for item in blade_sections]
    return loft_open_sections(edges, name)


# ============================================================
# fillet 截面与曲面
# ============================================================

def build_fillet_section_infos(
    base_edge,
    target_length,
    start_z,
    fillet_radius,
    side=OUTER_SIDE,
    group_name="group"
):
    marked_sections = []
    section_infos = []

    for i in range(N_FILLET_SECTIONS + 1):
        q = i / N_FILLET_SECTIONS
        alpha = 0.5 * math.pi * q

        offset_dist = fillet_radius * (1.0 - math.cos(alpha))
        z = start_z - fillet_radius * math.sin(alpha)

        section_name = (
            f"{group_name}_fillet_{i:02d}_"
            f"offset_{offset_dist:.6f}_z_{z:.6f}"
        )

        if i == 0:
            full_edge, full_pts = resample_edge_to_spline(
                base_edge,
                n_piece=N_SECTION_SAMPLE
            )

            marked_edge = full_edge
            marked_pts = full_pts

            info = {
                "kind": "fillet",
                "group": group_name,
                "index": i,
                "alpha": alpha,
                "offset_dist": offset_dist,
                "z": z,
                "full_edge": full_edge,
                "full_points": full_pts,
                "marked_edge": marked_edge,
                "marked_points": marked_pts,
                "curve_length": target_length,
                "target_length": target_length,
                "s0": 0.0,
                "s1": target_length,
                "t0": 0.0,
                "t1": 1.0,
            }

        else:
            full_edge, full_pts = make_normal_offset_section_from_base(
                base_edge,
                offset_dist=offset_dist,
                z_value=z,
                side=side,
                n_sample=N_CURVE_SAMPLE
            )

            marked = centered_equal_length_segment(
                full_edge,
                target_length=target_length,
                n_piece=N_SECTION_SAMPLE,
                name=section_name
            )

            info = {
                "kind": "fillet",
                "group": group_name,
                "index": i,
                "alpha": alpha,
                "offset_dist": offset_dist,
                "z": z,
                "full_edge": full_edge,
                "full_points": full_pts,
                "marked_edge": marked["edge"],
                "marked_points": marked["points"],
                "curve_length": marked["curve_length"],
                "target_length": marked["target_length"],
                "s0": marked["s0"],
                "s1": marked["s1"],
                "t0": marked["t0"],
                "t1": marked["t1"],
            }

        marked_sections.append(info["marked_edge"])
        section_infos.append(info)

    return marked_sections, section_infos


def build_fillet_surface_from_sections(
    fillet_marked_sections,
    group_name="group"
):
    return loft_open_sections(
        fillet_marked_sections,
        f"{group_name}_fillet_equal_length_surface"
    )


# ============================================================
# endwall 截面与曲面
# ============================================================

def build_endwall_section_infos(
    base_edge,
    target_length,
    start_offset,
    endwall_z,
    fillet_end_info,
    side=OUTER_SIDE,
    group_name="group"
):
    marked_sections = []
    section_infos = []

    for i in range(N_ENDWALL_SECTIONS + 1):
        q = i / N_ENDWALL_SECTIONS

        offset_dist = start_offset + ENDWALL_OFF * q
        z = endwall_z

        section_name = (
            f"{group_name}_endwall_{i:02d}_"
            f"offset_{offset_dist:.6f}_z_{z:.6f}"
        )

        full_edge, full_pts = make_normal_offset_section_from_base(
            base_edge,
            offset_dist=offset_dist,
            z_value=z,
            side=side,
            n_sample=N_CURVE_SAMPLE
        )

        if i == 0:
            info = {
                "kind": "endwall",
                "group": group_name,
                "index": i,
                "q": q,
                "offset_dist": offset_dist,
                "z": z,
                "full_edge": full_edge,
                "full_points": full_pts,
                "marked_edge": fillet_end_info["marked_edge"],
                "marked_points": fillet_end_info["marked_points"],
                "curve_length": fillet_end_info["curve_length"],
                "target_length": fillet_end_info["target_length"],
                "s0": fillet_end_info["s0"],
                "s1": fillet_end_info["s1"],
                "t0": fillet_end_info["t0"],
                "t1": fillet_end_info["t1"],
            }

        else:
            marked = centered_equal_length_segment(
                full_edge,
                target_length=target_length,
                n_piece=N_SECTION_SAMPLE,
                name=section_name
            )

            info = {
                "kind": "endwall",
                "group": group_name,
                "index": i,
                "q": q,
                "offset_dist": offset_dist,
                "z": z,
                "full_edge": full_edge,
                "full_points": full_pts,
                "marked_edge": marked["edge"],
                "marked_points": marked["points"],
                "curve_length": marked["curve_length"],
                "target_length": marked["target_length"],
                "s0": marked["s0"],
                "s1": marked["s1"],
                "t0": marked["t0"],
                "t1": marked["t1"],
            }

        marked_sections.append(info["marked_edge"])
        section_infos.append(info)

    return marked_sections, section_infos


def build_endwall_surface_from_sections(
    endwall_marked_sections,
    group_name="group"
):
    return loft_open_sections(
        endwall_marked_sections,
        f"{group_name}_endwall_equal_length_surface"
    )


# ============================================================
# 构造一组 delta 曲面
# ============================================================

def build_surface_group(
    base_edge,
    target_length,
    start_z,
    blade_top_z,
    fillet_radius,
    endwall_z,
    side=OUTER_SIDE,
    group_name="group",
    build_surfaces=True
):
    blade_sections = build_blade_sections(
        base_edge=base_edge,
        start_z=start_z,
        end_z=blade_top_z,
        side=side
    )

    fillet_marked_sections, fillet_infos = build_fillet_section_infos(
        base_edge=base_edge,
        target_length=target_length,
        start_z=start_z,
        fillet_radius=fillet_radius,
        side=side,
        group_name=group_name
    )

    endwall_marked_sections, endwall_infos = build_endwall_section_infos(
        base_edge=base_edge,
        target_length=target_length,
        start_offset=fillet_radius,
        endwall_z=endwall_z,
        fillet_end_info=fillet_infos[-1],
        side=side,
        group_name=group_name
    )

    blade_surface = None
    fillet_surface = None
    endwall_surface = None

    if build_surfaces:
        blade_surface = build_blade_surface_from_sections(
            blade_sections,
            name=f"{group_name}_blade_vertical_surface"
        )

        fillet_surface = build_fillet_surface_from_sections(
            fillet_marked_sections,
            group_name=group_name
        )

        endwall_surface = build_endwall_surface_from_sections(
            endwall_marked_sections,
            group_name=group_name
        )

    return {
        "group_name": group_name,
        "base_edge": base_edge,
        "target_length": target_length,
        "start_z": start_z,
        "blade_top_z": blade_top_z,
        "fillet_radius": fillet_radius,
        "endwall_z": endwall_z,
        "blade_surface": blade_surface,
        "blade_sections": blade_sections,
        "fillet_surface": fillet_surface,
        "fillet_marked_sections": fillet_marked_sections,
        "fillet_infos": fillet_infos,
        "endwall_surface": endwall_surface,
        "endwall_marked_sections": endwall_marked_sections,
        "endwall_infos": endwall_infos,
    }


def build_surface_group_from_continuous_delta(
    origin_edge,
    delta_value,
    group_name,
    build_surfaces=True
):
    base_delta_edge, base_delta_pts = make_inset_base_curve_from_origin(
        origin_edge=origin_edge,
        inset_dist=delta_value,
        z_value=R_FILLET,
        side=INNER_SIDE
    )

    target_length = curve_length(base_delta_edge)
    fillet_radius = R_FILLET + delta_value
    endwall_z = -delta_value

    group_data = build_surface_group(
        base_edge=base_delta_edge,
        target_length=target_length,
        start_z=R_FILLET,
        blade_top_z=BLADE_HEIGHT,
        fillet_radius=fillet_radius,
        endwall_z=endwall_z,
        side=OUTER_SIDE,
        group_name=group_name,
        build_surfaces=build_surfaces
    )

    group_data["delta_value"] = delta_value
    group_data["base_delta_points"] = base_delta_pts

    return group_data


# ============================================================
# 显示用完整偏置曲线
# ============================================================

def get_group_display_curves(group_data):
    fillet_infos = group_data["fillet_infos"]
    endwall_infos = group_data["endwall_infos"]

    return {
        "fillet_start_full": fillet_infos[0]["full_edge"],
        "fillet_end_full": fillet_infos[-1]["full_edge"],
        "endwall_start_full": endwall_infos[0]["full_edge"],
        "endwall_end_full": endwall_infos[-1]["full_edge"],
    }


# ============================================================
# 侧边边界曲线提取
# ============================================================

def make_blade_side_boundary_curve(group_data, side_index):
    pts = []

    for sec in group_data["blade_sections"]:
        pts.append(sec["points"][side_index])

    return make_spline_edge(pts), pts


def make_info_side_boundary_curve(section_infos, side_index):
    pts = []

    for info in section_infos:
        pts.append(info["marked_points"][side_index])

    return make_spline_edge(pts), pts


# ============================================================
# 连续 d 方向侧面与封口面
# ============================================================

def build_continuous_inset_side_and_cap_surfaces(origin_edge, model_name):
    blade_side_0_curves = []
    blade_side_1_curves = []

    fillet_side_0_curves = []
    fillet_side_1_curves = []

    endwall_side_0_curves = []
    endwall_side_1_curves = []

    blade_top_cap_curves = []
    endwall_outer_cap_curves = []

    sampled_groups = []

    for i in range(N_INSET_SIDE_SECTIONS + 1):
        delta_value = D_INSET * i / N_INSET_SIDE_SECTIONS
        group_name = f"{model_name}_side_delta_{i:02d}"

        group_data = build_surface_group_from_continuous_delta(
            origin_edge=origin_edge,
            delta_value=delta_value,
            group_name=group_name,
            build_surfaces=False
        )

        sampled_groups.append(group_data)

        blade_side_0_edge, _ = make_blade_side_boundary_curve(
            group_data,
            side_index=0
        )

        blade_side_1_edge, _ = make_blade_side_boundary_curve(
            group_data,
            side_index=-1
        )

        fillet_side_0_edge, _ = make_info_side_boundary_curve(
            group_data["fillet_infos"],
            side_index=0
        )

        fillet_side_1_edge, _ = make_info_side_boundary_curve(
            group_data["fillet_infos"],
            side_index=-1
        )

        endwall_side_0_edge, _ = make_info_side_boundary_curve(
            group_data["endwall_infos"],
            side_index=0
        )

        endwall_side_1_edge, _ = make_info_side_boundary_curve(
            group_data["endwall_infos"],
            side_index=-1
        )

        blade_side_0_curves.append(blade_side_0_edge)
        blade_side_1_curves.append(blade_side_1_edge)

        fillet_side_0_curves.append(fillet_side_0_edge)
        fillet_side_1_curves.append(fillet_side_1_edge)

        endwall_side_0_curves.append(endwall_side_0_edge)
        endwall_side_1_curves.append(endwall_side_1_edge)

        blade_top_cap_curves.append(
            group_data["blade_sections"][-1]["edge"]
        )

        endwall_outer_cap_curves.append(
            group_data["endwall_infos"][-1]["marked_edge"]
        )

    side_and_cap_surfaces = {
        "blade_side_0": loft_open_sections(
            blade_side_0_curves,
            f"{model_name}_continuous_d_blade_side_0"
        ),
        "blade_side_1": loft_open_sections(
            blade_side_1_curves,
            f"{model_name}_continuous_d_blade_side_1"
        ),
        "fillet_side_0": loft_open_sections(
            fillet_side_0_curves,
            f"{model_name}_continuous_d_fillet_side_0"
        ),
        "fillet_side_1": loft_open_sections(
            fillet_side_1_curves,
            f"{model_name}_continuous_d_fillet_side_1"
        ),
        "endwall_side_0": loft_open_sections(
            endwall_side_0_curves,
            f"{model_name}_continuous_d_endwall_side_0"
        ),
        "endwall_side_1": loft_open_sections(
            endwall_side_1_curves,
            f"{model_name}_continuous_d_endwall_side_1"
        ),
        "blade_top_cap": loft_open_sections(
            blade_top_cap_curves,
            f"{model_name}_continuous_d_blade_top_cap"
        ),
        "endwall_outer_cap": loft_open_sections(
            endwall_outer_cap_curves,
            f"{model_name}_continuous_d_endwall_outer_cap"
        ),
    }

    return side_and_cap_surfaces, sampled_groups


# ============================================================
# Solid 构造
# ============================================================

def extract_first_shell(shape):
    explorer = TopExp_Explorer(shape, TopAbs_SHELL)

    if not explorer.More():
        raise RuntimeError("Sewing 后没有找到 shell，无法生成 solid。")

    shell = TopoDS.Shell_s(explorer.Current())

    return shell


def make_solid_from_faces(face_shapes, sewing_tolerance=1e-5):
    sewing = BRepBuilderAPI_Sewing(sewing_tolerance)

    for shp in face_shapes:
        sewing.Add(shp.wrapped)

    sewing.Perform()

    sewed_shape = sewing.SewedShape()

    shell = extract_first_shell(sewed_shape)

    shell_check = BRepCheck_Analyzer(shell)

    if not shell_check.IsValid():
        print("\n[WARNING]")
        print("  Sewed shell is not valid before solid creation.")
        print("  Still trying MakeSolid / ShapeFix_Solid.")

    solid_builder = BRepBuilderAPI_MakeSolid(shell)

    if not solid_builder.IsDone():
        raise RuntimeError("BRepBuilderAPI_MakeSolid failed.")

    raw_solid = solid_builder.Solid()

    fixer = ShapeFix_Solid(raw_solid)
    fixer.Perform()

    fixed_solid = fixer.Solid()

    solid_check = BRepCheck_Analyzer(fixed_solid)

    if not solid_check.IsValid():
        print("\n[WARNING]")
        print("  Generated solid is not valid after ShapeFix_Solid.")
        print("  可能存在面方向、缝隙或自交问题。")

    return cq.Solid(fixed_solid), cq.Shape(sewed_shape)


def collect_closed_volume_faces(group_1, group_2, side_and_cap_surfaces):
    faces = [
        group_1["blade_surface"],
        group_1["fillet_surface"],
        group_1["endwall_surface"],

        group_2["blade_surface"],
        group_2["fillet_surface"],
        group_2["endwall_surface"],

        side_and_cap_surfaces["blade_side_0"],
        side_and_cap_surfaces["blade_side_1"],
        side_and_cap_surfaces["fillet_side_0"],
        side_and_cap_surfaces["fillet_side_1"],
        side_and_cap_surfaces["endwall_side_0"],
        side_and_cap_surfaces["endwall_side_1"],

        side_and_cap_surfaces["blade_top_cap"],
        side_and_cap_surfaces["endwall_outer_cap"],
    ]

    return faces


# ============================================================
# 构造一个完整模型包
# ============================================================

def build_closed_model_from_origin_curve(origin_edge, model_name):
    model_length = curve_length(origin_edge)

    print(f"\n[{model_name.upper()} ORIGIN CURVE]")
    print(f"  length                          = {model_length:.6f}")

    group_outer = build_surface_group_from_continuous_delta(
        origin_edge=origin_edge,
        delta_value=0.0,
        group_name=f"{model_name}_outer_group",
        build_surfaces=True
    )

    group_inner = build_surface_group_from_continuous_delta(
        origin_edge=origin_edge,
        delta_value=D_INSET,
        group_name=f"{model_name}_inner_group",
        build_surfaces=True
    )

    side_and_cap_surfaces, sampled_side_groups = (
        build_continuous_inset_side_and_cap_surfaces(
            origin_edge=origin_edge,
            model_name=model_name
        )
    )

    closed_faces = collect_closed_volume_faces(
        group_outer,
        group_inner,
        side_and_cap_surfaces
    )

    solid_shape, sewed_shell_shape = make_solid_from_faces(
        closed_faces,
        sewing_tolerance=1e-5
    )

    solid_valid = BRepCheck_Analyzer(solid_shape.wrapped).IsValid()

    print(f"\n[{model_name.upper()} SOLID]")
    print(f"  face count before sewing        = {len(closed_faces)}")
    print(f"  solid valid                     = {solid_valid}")

    try:
        print(f"  solid volume                    = {solid_shape.Volume():.6f}")
    except Exception:
        print("  solid volume                    = unavailable")

    return {
        "model_name": model_name,
        "origin_edge": origin_edge,
        "origin_length": model_length,
        "group_outer": group_outer,
        "group_inner": group_inner,
        "side_and_cap_surfaces": side_and_cap_surfaces,
        "sampled_side_groups": sampled_side_groups,
        "closed_faces": closed_faces,
        "solid_shape": solid_shape,
        "sewed_shell_shape": sewed_shell_shape,
        "outer_display_curves": get_group_display_curves(group_outer),
        "inner_display_curves": get_group_display_curves(group_inner),
    }


# ============================================================
# Assembly 添加工具
# ============================================================

def model_color_set(model_name):
    if model_name == "full":
        return {
            "solid": cq.Color(0.80, 0.86, 1.00, 0.60),
            "shell": cq.Color(1.00, 1.00, 1.00, 0.08),
            "origin": cq.Color(1.00, 1.00, 1.00),
        }

    if model_name == "quarter_25_50":
        return {
            "solid": cq.Color(1.00, 0.62, 0.25, 0.72),
            "shell": cq.Color(1.00, 0.80, 0.55, 0.10),
            "origin": cq.Color(1.00, 0.85, 0.00),
        }

    if model_name == "quarter_50_75":
        return {
            "solid": cq.Color(0.35, 1.00, 0.55, 0.72),
            "shell": cq.Color(0.65, 1.00, 0.75, 0.10),
            "origin": cq.Color(0.20, 1.00, 0.20),
        }

    return {
        "solid": cq.Color(0.90, 0.90, 0.90, 0.60),
        "shell": cq.Color(1.00, 1.00, 1.00, 0.08),
        "origin": cq.Color(1.00, 1.00, 1.00),
    }


def add_model_to_assembly(assy, model_data):
    name = model_data["model_name"]
    colors = model_color_set(name)

    group_outer = model_data["group_outer"]
    group_inner = model_data["group_inner"]
    side_and_cap = model_data["side_and_cap_surfaces"]

    outer_display = model_data["outer_display_curves"]
    inner_display = model_data["inner_display_curves"]

    assy.add(
        model_data["solid_shape"],
        name=f"{name}_closed_solid",
        color=colors["solid"]
    )

    assy.add(
        model_data["sewed_shell_shape"],
        name=f"{name}_sewed_shell_debug",
        color=colors["shell"]
    )

    assy.add(
        group_outer["blade_surface"],
        name=f"{name}_outer_group_blade_vertical_surface",
        color=cq.Color(0.75, 0.75, 0.90, 0.14)
    )

    assy.add(
        group_outer["fillet_surface"],
        name=f"{name}_outer_group_fillet_surface_equal_length",
        color=cq.Color(1.00, 0.45, 0.15, 0.14)
    )

    assy.add(
        group_outer["endwall_surface"],
        name=f"{name}_outer_group_endwall_surface_equal_length",
        color=cq.Color(0.90, 0.90, 0.55, 0.14)
    )

    assy.add(
        group_inner["blade_surface"],
        name=f"{name}_inner_group_blade_vertical_surface",
        color=cq.Color(0.55, 0.75, 1.00, 0.14)
    )

    assy.add(
        group_inner["fillet_surface"],
        name=f"{name}_inner_group_fillet_surface_equal_length",
        color=cq.Color(0.20, 0.90, 0.55, 0.14)
    )

    assy.add(
        group_inner["endwall_surface"],
        name=f"{name}_inner_group_endwall_surface_equal_length",
        color=cq.Color(0.55, 0.90, 0.90, 0.14)
    )

    assy.add(
        side_and_cap["blade_side_0"],
        name=f"{name}_continuous_d_blade_side_0",
        color=cq.Color(0.95, 0.35, 0.85, 0.16)
    )

    assy.add(
        side_and_cap["blade_side_1"],
        name=f"{name}_continuous_d_blade_side_1",
        color=cq.Color(0.95, 0.35, 0.85, 0.16)
    )

    assy.add(
        side_and_cap["fillet_side_0"],
        name=f"{name}_continuous_d_fillet_side_0",
        color=cq.Color(0.95, 0.65, 0.25, 0.16)
    )

    assy.add(
        side_and_cap["fillet_side_1"],
        name=f"{name}_continuous_d_fillet_side_1",
        color=cq.Color(0.95, 0.65, 0.25, 0.16)
    )

    assy.add(
        side_and_cap["endwall_side_0"],
        name=f"{name}_continuous_d_endwall_side_0",
        color=cq.Color(0.35, 0.95, 0.95, 0.16)
    )

    assy.add(
        side_and_cap["endwall_side_1"],
        name=f"{name}_continuous_d_endwall_side_1",
        color=cq.Color(0.35, 0.95, 0.95, 0.16)
    )

    assy.add(
        side_and_cap["blade_top_cap"],
        name=f"{name}_continuous_d_blade_top_cap",
        color=cq.Color(0.70, 0.35, 1.00, 0.20)
    )

    assy.add(
        side_and_cap["endwall_outer_cap"],
        name=f"{name}_continuous_d_endwall_outer_cap",
        color=cq.Color(0.35, 1.00, 0.45, 0.20)
    )

    assy.add(
        model_data["origin_edge"],
        name=f"{name}_origin_curve",
        color=colors["origin"]
    )

    assy.add(
        group_inner["base_edge"],
        name=f"{name}_inner_offset_curve_as_second_origin",
        color=cq.Color(1.00, 0.00, 0.00)
    )

    assy.add(
        outer_display["fillet_start_full"],
        name=f"{name}_outer_group_fillet_start_full_offset_curve",
        color=cq.Color(0.00, 1.00, 0.00)
    )

    assy.add(
        outer_display["fillet_end_full"],
        name=f"{name}_outer_group_fillet_end_full_offset_curve",
        color=cq.Color(0.00, 1.00, 0.00)
    )

    assy.add(
        outer_display["endwall_start_full"],
        name=f"{name}_outer_group_endwall_start_full_offset_curve",
        color=cq.Color(0.00, 0.80, 1.00)
    )

    assy.add(
        outer_display["endwall_end_full"],
        name=f"{name}_outer_group_endwall_end_full_offset_curve",
        color=cq.Color(0.00, 0.80, 1.00)
    )

    assy.add(
        inner_display["fillet_start_full"],
        name=f"{name}_inner_group_fillet_start_full_offset_curve",
        color=cq.Color(1.00, 0.20, 0.20)
    )

    assy.add(
        inner_display["fillet_end_full"],
        name=f"{name}_inner_group_fillet_end_full_offset_curve",
        color=cq.Color(1.00, 0.20, 0.20)
    )

    assy.add(
        inner_display["endwall_start_full"],
        name=f"{name}_inner_group_endwall_start_full_offset_curve",
        color=cq.Color(1.00, 0.45, 0.00)
    )

    assy.add(
        inner_display["endwall_end_full"],
        name=f"{name}_inner_group_endwall_end_full_offset_curve",
        color=cq.Color(1.00, 0.45, 0.00)
    )


# ============================================================
# 主流程
# ============================================================

if __name__ == "__main__":

    print("\n[FULL BODY + 25-50 BODY + 50-75 BODY COMPARISON]")

    if R_FILLET <= 0:
        raise ValueError("R_FILLET 必须大于 0。")

    if BLADE_HEIGHT <= R_FILLET:
        raise ValueError("BLADE_HEIGHT 必须大于 R_FILLET。")

    if ENDWALL_OFF <= 0:
        raise ValueError("ENDWALL_OFF 必须大于 0。")

    if D_INSET <= 0:
        raise ValueError("D_INSET 必须大于 0。")

    if N_INSET_SIDE_SECTIONS < 1:
        raise ValueError("N_INSET_SIDE_SECTIONS 必须至少为 1。")

    # --------------------------------------------------------
    # 1. 完整原始 Bezier 曲线
    # --------------------------------------------------------

    original_bezier_curve = make_demo_bezier_base_curve_at_z(R_FILLET)

    u0, u1 = edge_param_range(original_bezier_curve)
    original_length = curve_length(original_bezier_curve)

    print("\n[ORIGINAL FULL BEZIER CURVE]")
    print(f"  real parameter range            = {u0:.6f} -> {u1:.6f}")
    print(f"  parameter span                  = {u1 - u0:.6f}")
    print(f"  z                               = {R_FILLET:.6f}")
    print(f"  length                          = {original_length:.6f}")

    # --------------------------------------------------------
    # 2. 按弧长截取 25%-50% 曲线
    # --------------------------------------------------------

    curve_25_50_info = make_curve_by_arclength_fraction(
        original_bezier_curve,
        start_fraction=0.25,
        end_fraction=0.50,
        n_piece=N_SECTION_SAMPLE
    )

    curve_25_50 = curve_25_50_info["edge"]

    print("\n[CURVE 25%-50% BY ARC LENGTH]")
    print(f"  origin length                   = {curve_25_50_info['origin_length']:.6f}")
    print(f"  segment length                  = {curve_25_50_info['length']:.6f}")
    print(f"  s0                              = {curve_25_50_info['s0']:.6f}")
    print(f"  s1                              = {curve_25_50_info['s1']:.6f}")
    print(f"  t0 approx                       = {curve_25_50_info['t0']:.6f}")
    print(f"  t1 approx                       = {curve_25_50_info['t1']:.6f}")

    # --------------------------------------------------------
    # 3. 按弧长截取 50%-75% 曲线
    # --------------------------------------------------------

    curve_50_75_info = make_curve_by_arclength_fraction(
        original_bezier_curve,
        start_fraction=0.50,
        end_fraction=0.75,
        n_piece=N_SECTION_SAMPLE
    )

    curve_50_75 = curve_50_75_info["edge"]

    print("\n[CURVE 50%-75% BY ARC LENGTH]")
    print(f"  origin length                   = {curve_50_75_info['origin_length']:.6f}")
    print(f"  segment length                  = {curve_50_75_info['length']:.6f}")
    print(f"  s0                              = {curve_50_75_info['s0']:.6f}")
    print(f"  s1                              = {curve_50_75_info['s1']:.6f}")
    print(f"  t0 approx                       = {curve_50_75_info['t0']:.6f}")
    print(f"  t1 approx                       = {curve_50_75_info['t1']:.6f}")

    # --------------------------------------------------------
    # 4. 使用完整曲线生成完整体
    # --------------------------------------------------------

    full_model = build_closed_model_from_origin_curve(
        origin_edge=original_bezier_curve,
        model_name="full"
    )

    # --------------------------------------------------------
    # 5. 使用 25%-50% 曲线生成局部体
    # --------------------------------------------------------

    quarter_25_50_model = build_closed_model_from_origin_curve(
        origin_edge=curve_25_50,
        model_name="quarter_25_50"
    )

    # --------------------------------------------------------
    # 6. 使用 50%-75% 曲线生成局部体
    # --------------------------------------------------------

    quarter_50_75_model = build_closed_model_from_origin_curve(
        origin_edge=curve_50_75,
        model_name="quarter_50_75"
    )

    # --------------------------------------------------------
    # 7. 输出体积对比
    # --------------------------------------------------------

    print("\n[VOLUME COMPARISON]")

    try:
        full_volume = full_model["solid_shape"].Volume()
        volume_25_50 = quarter_25_50_model["solid_shape"].Volume()
        volume_50_75 = quarter_50_75_model["solid_shape"].Volume()

        print(f"  full volume                     = {full_volume:.6f}")
        print(f"  25%-50% volume                  = {volume_25_50:.6f}")
        print(f"  50%-75% volume                  = {volume_50_75:.6f}")

        if abs(full_volume) > 1e-12:
            print(f"  25%-50% / full volume ratio     = {volume_25_50 / full_volume:.6f}")
            print(f"  50%-75% / full volume ratio     = {volume_50_75 / full_volume:.6f}")
            print(f"  sum local / full volume ratio   = {(volume_25_50 + volume_50_75) / full_volume:.6f}")

    except Exception:
        print("  volume comparison unavailable")

    # --------------------------------------------------------
    # 8. 导出 STEP
    # --------------------------------------------------------

    if EXPORT_STEP:
        step_path = (
            Path(__file__).resolve().parent
            / "full_body_25_50_body_50_75_body_comparison.step"
        )

        if step_path.exists():
            step_path.unlink()

        assy = cq.Assembly(
            name="FULL_BODY_25_50_BODY_50_75_BODY_COMPARISON"
        )

        add_model_to_assembly(assy, full_model)
        add_model_to_assembly(assy, quarter_25_50_model)
        add_model_to_assembly(assy, quarter_50_75_model)

        assy.add(
            original_bezier_curve,
            name="reference_original_full_bezier_curve",
            color=cq.Color(1.00, 1.00, 1.00)
        )

        assy.add(
            curve_25_50,
            name="reference_curve_25_50_arc_length",
            color=cq.Color(1.00, 0.85, 0.00)
        )

        assy.add(
            curve_50_75,
            name="reference_curve_50_75_arc_length",
            color=cq.Color(0.20, 1.00, 0.20)
        )

        assy.export(str(step_path))

        print(f"\n[STEP] {step_path}")