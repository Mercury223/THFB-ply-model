import math
import shutil
from dataclasses import dataclass
from pathlib import Path

import cadquery as cq
from cadquery import exporters


@dataclass
class Params:
    # =========================================================
    # 叶身截面参数
    # =========================================================
    blade_height: float = 80.0          # 叶身沿 +Z 拉伸高度
    outer_radius: float = 33.0          # 外侧前缘圆半径 R
    blade_thickness: float = 4.0        # 叶身壁厚

    # 上切点角度，位于第二象限 (-x, +y)
    upper_tangent_angle_deg: float = 130.0

    # 下切点固定为 (0, -R)
    lower_tangent_angle_deg: float = -90.0

    # 两条切线分别截断，不再延伸到自然尾缘交点
    blade_upper_length: float = 32.0
    blade_lower_length: float = 36.0

    # =========================================================
    # 缘板截面参数
    # =========================================================
    endwall_height: float = 20.0        # 缘板沿 -Z 拉伸厚度

    lower_drop: float = 20.0
    lower_run: float = 150.0

    upper_drop: float = 25.0
    upper_run: float = 160.0

    # =========================================================
    # 根部倒角参数
    # =========================================================
    root_fillet_radius: float = 2.0     # 叶身 / 缘板根部倒角半径；<=0 表示不倒角

    # 倒角曲线：
    #   "outer" = z=0 平面上的叶身外部曲线
    #   "inner" = z=0 平面上的叶身内部曲线
    #   "both"  = 内外曲线都尝试倒角
    root_fillet_curve: str = "outer"

    # =========================================================
    # 铺层划分 / 展平参考线参数
    # =========================================================
    make_ply_curves: bool = True

    # 当相邻剪口点的切线方向累计偏差达到该角度时，认为需要剪口/分段
    ply_tangent_angle_step_deg: float = 20.0

    # 扩展曲线相对叶身外侧曲线的外法向偏置距离。
    # 应大于缘板局部覆盖范围。
    ply_expand_offset: float = 60.0

    # 用于构造扩展曲线的离散采样数量
    ply_expand_samples: int = 1200

    # 每条铺层路径在倒圆段上的离散数量
    ply_fillet_samples: int = 12

    # 每个铺层分段边界曲线的颜色，仅 Assembly STEP 中有效
    ply_curve_color: tuple = (1.0, 0.0, 0.0)

    # 调试时导出分体；最终只要 final_fillet_body 时可设 False
    export_debug_parts: bool = True

    # 铺层带厚度，用于生成可视化铺层实体（0 表示不生成）
    ply_layer_thickness: float = 1.0

    # 输出
    out_dir: str = "."
    step_name: str = "vane_blade_three_parts_negative_endwall.step"


class VaneBladeAndEndwallBuilder:
    KEEP_FILES = {
        "QYModel.py",
        "QYread.prt",
        "QYTest.py",
        "PCTest.m",
        "visualize_ply.py",
        "history.md",
        ".git",
        ".gitignore",
        ".claude",
    }

    def __init__(self, p: Params):
        self.p = p
        self._validate()
        self._init_dirs()

    # =========================================================
    # 基础向量工具
    # =========================================================
    @staticmethod
    def vec(x, y):
        return cq.Vector(float(x), float(y), 0.0)

    @staticmethod
    def add(a, b):
        return cq.Vector(a.x + b.x, a.y + b.y, 0.0)

    @staticmethod
    def sub(a, b):
        return cq.Vector(a.x - b.x, a.y - b.y, 0.0)

    @staticmethod
    def mul(a, k):
        return cq.Vector(a.x * k, a.y * k, 0.0)

    @staticmethod
    def norm(a):
        return math.hypot(a.x, a.y)

    def unit(self, a):
        n = self.norm(a)
        if n <= 1e-12:
            raise ValueError("零向量不能单位化")
        return self.vec(a.x / n, a.y / n)

    @staticmethod
    def dot(a, b):
        return a.x * b.x + a.y * b.y

    @staticmethod
    def cross2(a, b):
        return a.x * b.y - a.y * b.x

    @staticmethod
    def polar(r, angle_deg):
        a = math.radians(angle_deg)
        return cq.Vector(r * math.cos(a), r * math.sin(a), 0.0)

    @staticmethod
    def tangent_dir_for_circle_point(p):
        """
        圆上点 P 的切线方向。
        对第二象限上切点，(-y, x) 指向左下。
        """
        return cq.Vector(-p.y, p.x, 0.0)

    @staticmethod
    def rot90_ccw(v):
        return cq.Vector(-v.y, v.x, 0.0)

    @staticmethod
    def rot90_cw(v):
        return cq.Vector(v.y, -v.x, 0.0)

    @staticmethod
    def line_intersection(p1, d1, p2, d2):
        """
        求二维直线交点：
            p1 + t d1 = p2 + u d2
        """
        cross = d1.x * d2.y - d1.y * d2.x
        if abs(cross) <= 1e-12:
            raise ValueError("两条直线平行或几乎平行，无法求交点")

        q = cq.Vector(p2.x - p1.x, p2.y - p1.y, 0.0)
        t = (q.x * d2.y - q.y * d2.x) / cross

        return cq.Vector(
            p1.x + t * d1.x,
            p1.y + t * d1.y,
            0.0,
        )

    @staticmethod
    def make_wire(edges):
        return cq.Wire.assembleEdges(edges)

    @staticmethod
    def make_face(edges):
        return cq.Face.makeFromWires(cq.Wire.assembleEdges(edges))

    def solid_from_face(self, face, height):
        return (
            cq.Workplane("XY")
            .add(face)
            .extrude(height)
            .val()
        )

    # =========================================================
    # 参数检查
    # =========================================================
    def _validate(self):
        p = self.p

        if p.blade_height <= 0:
            raise ValueError("blade_height 必须为正数")
        if p.endwall_height <= 0:
            raise ValueError("endwall_height 必须为正数")
        if p.outer_radius <= 0:
            raise ValueError("outer_radius 必须为正数")
        if p.blade_thickness <= 0:
            raise ValueError("blade_thickness 必须为正数")
        if p.outer_radius <= p.blade_thickness:
            raise ValueError("outer_radius 必须大于 blade_thickness")
        if not (90.0 < p.upper_tangent_angle_deg < 180.0):
            raise ValueError("upper_tangent_angle_deg 必须在 90 到 180 度之间")
        if abs(p.lower_tangent_angle_deg + 90.0) > 1e-9:
            raise ValueError("lower_tangent_angle_deg 固定为 -90.0")
        if p.blade_upper_length <= 0 or p.blade_lower_length <= 0:
            raise ValueError("blade_upper_length 和 blade_lower_length 必须为正数")
        if p.lower_drop <= 0 or p.lower_run <= 0:
            raise ValueError("lower_drop 和 lower_run 必须为正数")
        if p.upper_drop <= 0 or p.upper_run <= 0:
            raise ValueError("upper_drop 和 upper_run 必须为正数")

        if p.root_fillet_radius < 0:
            raise ValueError("root_fillet_radius 必须大于等于 0")

        if p.root_fillet_radius > 0:
            limit = min(p.blade_thickness, p.endwall_height)
            if p.root_fillet_radius >= limit:
                raise ValueError(
                    "root_fillet_radius 必须小于 blade_thickness 和 endwall_height 中的较小值，"
                    "否则倒角可能切穿几何。"
                )

        if p.root_fillet_curve not in {"outer", "inner", "both"}:
            raise ValueError('root_fillet_curve 只能是 "outer"、"inner" 或 "both"')

        if p.make_ply_curves:
            if p.ply_tangent_angle_step_deg <= 0:
                raise ValueError("ply_tangent_angle_step_deg 必须为正数")
            if p.ply_expand_offset <= 0:
                raise ValueError("ply_expand_offset 必须为正数")
            if p.ply_expand_samples < 20:
                raise ValueError("ply_expand_samples 建议至少为 20")
            if p.ply_fillet_samples < 3:
                raise ValueError("ply_fillet_samples 建议至少为 3")

    # =========================================================
    # 初始化方向
    # =========================================================
    def _init_dirs(self):
        p = self.p

        outer_upper_tangent = self.polar(
            p.outer_radius,
            p.upper_tangent_angle_deg,
        )

        # 上边叶身方向：
        # tangent -> cut 是左下
        self.upper_dir_tangent_to_cut = self.unit(
            self.tangent_dir_for_circle_point(outer_upper_tangent)
        )

        # cut -> tangent 是右上
        self.upper_dir_cut_to_tangent = self.mul(
            self.upper_dir_tangent_to_cut,
            -1.0,
        )

        # 下边叶身方向：
        # tangent -> cut 是向左
        self.lower_dir_tangent_to_cut = self.vec(-1.0, 0.0)

        # cut -> tangent 是向右
        self.lower_dir_cut_to_tangent = self.vec(1.0, 0.0)

        # 下边缘板第一条垂线：向下
        self.lower_normal_out = self.vec(0.0, -1.0)

        # 下边缘板第三条垂线：向上
        self.lower_normal_back = self.vec(0.0, 1.0)

        # 上边缘板第一条垂线：相对上边直线向外，约为左上
        # 对 cut -> tangent 方向做逆时针 90 度
        self.upper_normal_out = self.unit(
            self.rot90_ccw(self.upper_dir_cut_to_tangent)
        )

        # 上边缘板第三条垂线：反方向，约为右下
        self.upper_normal_back = self.mul(self.upper_normal_out, -1.0)

    # =========================================================
    # 叶片截面关键点
    # =========================================================
    def make_blade_profile(self, radius):
        p = self.p

        upper_tangent = self.polar(radius, p.upper_tangent_angle_deg)
        lower_tangent = self.polar(radius, p.lower_tangent_angle_deg)
        nose = self.polar(radius, 0.0)

        upper_cut = self.add(
            upper_tangent,
            self.mul(self.upper_dir_tangent_to_cut, p.blade_upper_length),
        )

        lower_cut = self.add(
            lower_tangent,
            self.mul(self.lower_dir_tangent_to_cut, p.blade_lower_length),
        )

        return {
            "radius": radius,
            "upper_cut": upper_cut,
            "upper_tangent": upper_tangent,
            "nose": nose,
            "lower_tangent": lower_tangent,
            "lower_cut": lower_cut,
        }

    # =========================================================
    # 叶身三段截面
    # =========================================================
    def make_lower_blade_face(self, outer, inner):
        edges = [
            cq.Edge.makeLine(outer["lower_cut"], outer["lower_tangent"]),
            cq.Edge.makeLine(outer["lower_tangent"], inner["lower_tangent"]),
            cq.Edge.makeLine(inner["lower_tangent"], inner["lower_cut"]),
            cq.Edge.makeLine(inner["lower_cut"], outer["lower_cut"]),
        ]
        return self.make_face(edges)

    def make_upper_blade_face(self, outer, inner):
        edges = [
            cq.Edge.makeLine(outer["upper_cut"], outer["upper_tangent"]),
            cq.Edge.makeLine(outer["upper_tangent"], inner["upper_tangent"]),
            cq.Edge.makeLine(inner["upper_tangent"], inner["upper_cut"]),
            cq.Edge.makeLine(inner["upper_cut"], outer["upper_cut"]),
        ]
        return self.make_face(edges)

    def make_arc_blade_face(self, outer, inner):
        edges = [
            cq.Edge.makeThreePointArc(
                outer["upper_tangent"],
                outer["nose"],
                outer["lower_tangent"],
            ),
            cq.Edge.makeLine(outer["lower_tangent"], inner["lower_tangent"]),
            cq.Edge.makeThreePointArc(
                inner["lower_tangent"],
                inner["nose"],
                inner["upper_tangent"],
            ),
            cq.Edge.makeLine(inner["upper_tangent"], outer["upper_tangent"]),
        ]
        return self.make_face(edges)

    def build_blade_body(self):
        p = self.p

        outer = self.make_blade_profile(p.outer_radius)
        inner = self.make_blade_profile(p.outer_radius - p.blade_thickness)

        lower_face = self.make_lower_blade_face(outer, inner)
        upper_face = self.make_upper_blade_face(outer, inner)
        arc_face = self.make_arc_blade_face(outer, inner)

        lower_body = self.solid_from_face(lower_face, p.blade_height)
        upper_body = self.solid_from_face(upper_face, p.blade_height)
        arc_body = self.solid_from_face(arc_face, p.blade_height)

        blade_body = arc_body.fuse(lower_body).fuse(upper_body)

        if not blade_body.isValid():
            raise RuntimeError("blade_body 无效，请检查叶片三段壁厚轮廓")

        print("[叶身：三段各自闭合]")
        self.print_point("outer upper_cut", outer["upper_cut"])
        self.print_point("outer upper_tangent", outer["upper_tangent"])
        self.print_point("outer lower_tangent", outer["lower_tangent"])
        self.print_point("outer lower_cut", outer["lower_cut"])
        self.print_point("inner upper_cut", inner["upper_cut"])
        self.print_point("inner upper_tangent", inner["upper_tangent"])
        self.print_point("inner lower_tangent", inner["lower_tangent"])
        self.print_point("inner lower_cut", inner["lower_cut"])

        return {
            "blade_body": blade_body,
            "blade_lower_body": lower_body,
            "blade_arc_body": arc_body,
            "blade_upper_body": upper_body,
            "outer_profile": outer,
            "inner_profile": inner,
        }

    # =========================================================
    # 缘板：保持原始 raw_endwall_body
    # =========================================================
    def compute_endwall_points(self, inner):
        p = self.p

        L0 = inner["lower_cut"]
        U0 = inner["upper_cut"]

        L1 = self.add(
            L0,
            self.mul(self.lower_normal_out, p.lower_drop),
        )

        L2 = self.add(
            L1,
            self.mul(self.lower_dir_cut_to_tangent, p.lower_run),
        )

        lower_final_point = L2
        lower_final_dir = self.lower_normal_back

        U1 = self.add(
            U0,
            self.mul(self.upper_normal_out, p.upper_drop),
        )

        U2 = self.add(
            U1,
            self.mul(self.upper_dir_cut_to_tangent, p.upper_run),
        )

        upper_final_point = U2
        upper_final_dir = self.upper_normal_back

        I = self.line_intersection(
            lower_final_point,
            lower_final_dir,
            upper_final_point,
            upper_final_dir,
        )

        if I.y <= L2.y:
            raise ValueError(
                "缘板交点 I 不在下边最后垂线的上方。"
                "请增大 upper_run / lower_run，或调整 upper_drop、upper_tangent_angle_deg。"
            )

        if self.dot(self.sub(I, U2), upper_final_dir) < -1e-9:
            raise ValueError(
                "缘板交点 I 位于上边最后垂线的反方向。"
                "请增大 lower_run 或调整 upper_run。"
            )

        print("[缘板关键点]")
        self.print_point("L0 lower inner cut", L0)
        self.print_point("L1 lower drop", L1)
        self.print_point("L2 lower run end", L2)
        self.print_point("I final intersection", I)
        self.print_point("U2 upper run end", U2)
        self.print_point("U1 upper drop", U1)
        self.print_point("U0 upper inner cut", U0)

        return {
            "L0": L0,
            "L1": L1,
            "L2": L2,
            "I": I,
            "U2": U2,
            "U1": U1,
            "U0": U0,
        }

    def make_raw_endwall_wire(self, inner):
        pts = self.compute_endwall_points(inner)

        edges = [
            cq.Edge.makeLine(inner["lower_cut"], inner["lower_tangent"]),
            cq.Edge.makeThreePointArc(
                inner["lower_tangent"],
                inner["nose"],
                inner["upper_tangent"],
            ),
            cq.Edge.makeLine(inner["upper_tangent"], inner["upper_cut"]),

            cq.Edge.makeLine(inner["upper_cut"], pts["U1"]),
            cq.Edge.makeLine(pts["U1"], pts["U2"]),
            cq.Edge.makeLine(pts["U2"], pts["I"]),
            cq.Edge.makeLine(pts["I"], pts["L2"]),
            cq.Edge.makeLine(pts["L2"], pts["L1"]),
            cq.Edge.makeLine(pts["L1"], inner["lower_cut"]),
        ]

        return self.make_wire(edges)

    def build_raw_endwall_body(self, inner_profile):
        p = self.p

        raw_wire = self.make_raw_endwall_wire(inner_profile)
        raw_face = cq.Face.makeFromWires(raw_wire)

        raw_body = self.solid_from_face(
            raw_face,
            -p.endwall_height,
        )

        if not raw_body.isValid():
            raise RuntimeError("raw_endwall_body 无效，请检查缘板边界是否自交")

        return raw_body

    # =========================================================
    # 根部倒角选择器
    # =========================================================
    @staticmethod
    def _dist_point_to_segment_xy(p, a, b):
        ab = cq.Vector(b.x - a.x, b.y - a.y, 0.0)
        ap = cq.Vector(p.x - a.x, p.y - a.y, 0.0)

        ab2 = ab.x * ab.x + ab.y * ab.y
        if ab2 <= 1e-18:
            return math.hypot(p.x - a.x, p.y - a.y)

        t = (ap.x * ab.x + ap.y * ab.y) / ab2
        t = max(0.0, min(1.0, t))

        qx = a.x + t * ab.x
        qy = a.y + t * ab.y

        return math.hypot(p.x - qx, p.y - qy)

    @staticmethod
    def _angle_deg_0_360(v):
        a = math.degrees(math.atan2(v.y, v.x))
        if a < 0:
            a += 360.0
        return a

    def _is_point_on_open_profile(self, point, profile, tol=1e-3):
        """
        判断点是否位于开口轮廓：
            lower_cut -> lower_tangent
            lower_tangent -> arc -> upper_tangent
            upper_tangent -> upper_cut
        """
        p = self.p

        # 下直线段
        d_lower = self._dist_point_to_segment_xy(
            point,
            profile["lower_cut"],
            profile["lower_tangent"],
        )
        if d_lower <= tol:
            return True

        # 上直线段
        d_upper = self._dist_point_to_segment_xy(
            point,
            profile["upper_tangent"],
            profile["upper_cut"],
        )
        if d_upper <= tol:
            return True

        # 圆弧段，圆心为原点
        r = profile["radius"]
        rp = math.hypot(point.x, point.y)

        if abs(rp - r) <= tol:
            a = self._angle_deg_0_360(point)

            lower_a = p.lower_tangent_angle_deg
            upper_a = p.upper_tangent_angle_deg

            if lower_a < 0:
                lower_a += 360.0

            # 当前圆弧从 -90 度经过 0 度到 upper_tangent_angle_deg
            # 即 270 -> 360 -> upper_a
            if a >= lower_a or a <= upper_a:
                return True

        return False

    class RootFilletEdgeSelector(cq.Selector):
        """
        选择 z=0 平面上的叶身根部曲线。

        curve:
            "outer" = 只选 outer_profile
            "inner" = 只选 inner_profile
            "both"  = inner_profile + outer_profile
        """
        def __init__(
            self,
            builder,
            inner_profile,
            outer_profile,
            curve="outer",
            z=0.0,
            tol=1e-3,
        ):
            self.builder = builder
            self.inner_profile = inner_profile
            self.outer_profile = outer_profile
            self.curve = curve
            self.z = z
            self.tol = tol

        def filter(self, objectList):
            result = []

            for edge in objectList:
                try:
                    bb = edge.BoundingBox()

                    # 目标倒角边应整体位于 z = 0 平面
                    if abs(bb.zmin - self.z) > self.tol:
                        continue
                    if abs(bb.zmax - self.z) > self.tol:
                        continue

                    c = edge.Center()

                    on_outer = self.builder._is_point_on_open_profile(
                        c,
                        self.outer_profile,
                        self.tol,
                    )

                    on_inner = self.builder._is_point_on_open_profile(
                        c,
                        self.inner_profile,
                        self.tol,
                    )

                    if self.curve == "outer":
                        if on_outer:
                            result.append(edge)

                    elif self.curve == "inner":
                        if on_inner:
                            result.append(edge)

                    elif self.curve == "both":
                        if on_outer or on_inner:
                            result.append(edge)

                except Exception:
                    continue

            return result

    def apply_root_fillet(self, blade_body, raw_endwall_body, outer_profile, inner_profile):
        """
        真倒角版本。

        final_body = blade_body + raw_endwall_body
        倒角边 = z=0 平面上的叶身外部曲线 outer_profile
        """
        p = self.p

        fused_body = blade_body.fuse(raw_endwall_body)

        if not fused_body.isValid():
            raise RuntimeError("blade_body 与 raw_endwall_body fuse 后无效")

        if p.root_fillet_radius <= 0:
            return fused_body

        selector = self.RootFilletEdgeSelector(
            builder=self,
            inner_profile=inner_profile,
            outer_profile=outer_profile,
            curve=p.root_fillet_curve,
            z=0.0,
            tol=1e-3,
        )

        wp = cq.Workplane("XY").add(fused_body).edges(selector)
        selected_edges = wp.vals()

        print("[根部倒角]")
        print(f"  root_fillet_radius      = {p.root_fillet_radius}")
        print(f"  root_fillet_curve       = {p.root_fillet_curve}")
        print(f"  selected root edges     = {len(selected_edges)}")

        if len(selected_edges) == 0:
            raise RuntimeError(
                "未选中任何根部倒角边。"
                "请检查 root_fillet_curve，或适当增大 RootFilletEdgeSelector 的 tol。"
            )

        try:
            filleted_body = wp.fillet(p.root_fillet_radius).val()
        except Exception as e:
            raise RuntimeError(
                "根部倒角失败。建议：\n"
                "1) 先减小 root_fillet_radius，例如 2.0 改成 1.0；\n"
                "2) 确认当前想倒的是 outer_profile；\n"
                "3) 检查 blade_body 与 raw_endwall_body 融合后的 z=0 外部边是否连续。\n"
                f"原始错误: {e}"
            )

        if not filleted_body.isValid():
            raise RuntimeError(
                "根部倒角后实体无效。请减小 root_fillet_radius，"
                "或检查叶身厚度、缘板厚度和根部轮廓。"
            )

        return filleted_body

    # =========================================================
    # 铺层划分参考曲线：保持实体不变，只生成外表面 wire
    # =========================================================
    @staticmethod
    def _angle_between_deg(a, b):
        na = math.hypot(a.x, a.y)
        nb = math.hypot(b.x, b.y)
        if na <= 1e-12 or nb <= 1e-12:
            return 0.0

        c = (a.x * b.x + a.y * b.y) / (na * nb)
        c = max(-1.0, min(1.0, c))
        return math.degrees(math.acos(c))

    @staticmethod
    def _v3(p, z):
        return cq.Vector(float(p.x), float(p.y), float(z))

    def _outer_profile_total_length(self, profile):
        p = self.p
        r = profile["radius"]

        lower_len = p.blade_lower_length
        arc_len = r * math.radians(
            p.upper_tangent_angle_deg - p.lower_tangent_angle_deg
        )
        upper_len = p.blade_upper_length

        return lower_len + arc_len + upper_len

    def _outer_profile_point_tangent_normal_at_s(self, profile, s):
        """
        外侧叶型开曲线参数化，方向为：
            lower_cut -> lower_tangent -> 圆弧 -> upper_tangent -> upper_cut

        返回：
            point   : XY 点
            tangent : 沿曲线前进方向的单位切线
            normal  : 指向叶身外侧 / 缘板外扩方向的单位法向

        对该开曲线而言，外法向 = tangent 顺时针旋转 90°。
        在圆弧段，这正好等于半径外法向。
        """
        p = self.p
        r = profile["radius"]

        lower_len = p.blade_lower_length
        arc_angle_span_deg = p.upper_tangent_angle_deg - p.lower_tangent_angle_deg
        arc_len = r * math.radians(arc_angle_span_deg)
        total_len = self._outer_profile_total_length(profile)

        s = max(0.0, min(float(s), total_len))

        # 1) 下直线：lower_cut -> lower_tangent
        if s <= lower_len:
            t = s / max(lower_len, 1e-12)
            a = profile["lower_cut"]
            b = profile["lower_tangent"]

            point = cq.Vector(
                a.x + (b.x - a.x) * t,
                a.y + (b.y - a.y) * t,
                0.0,
            )
            tangent = self.lower_dir_cut_to_tangent
            normal = self.rot90_cw(tangent)
            return point, self.unit(tangent), self.unit(normal)

        # 2) 圆弧：lower_tangent -> upper_tangent
        s_arc = s - lower_len
        if s_arc <= arc_len:
            angle_deg = p.lower_tangent_angle_deg + math.degrees(s_arc / r)
            angle_rad = math.radians(angle_deg)

            point = cq.Vector(
                r * math.cos(angle_rad),
                r * math.sin(angle_rad),
                0.0,
            )

            # 圆弧按角度递增方向走，切线为 (-sin, cos)
            tangent = cq.Vector(
                -math.sin(angle_rad),
                math.cos(angle_rad),
                0.0,
            )

            # 外法向为半径方向
            normal = cq.Vector(
                math.cos(angle_rad),
                math.sin(angle_rad),
                0.0,
            )

            return point, self.unit(tangent), self.unit(normal)

        # 3) 上直线：upper_tangent -> upper_cut
        s_up = s - lower_len - arc_len
        upper_len = p.blade_upper_length
        t = s_up / max(upper_len, 1e-12)

        a = profile["upper_tangent"]
        b = profile["upper_cut"]

        point = cq.Vector(
            a.x + (b.x - a.x) * t,
            a.y + (b.y - a.y) * t,
            0.0,
        )

        tangent = self.upper_dir_tangent_to_cut
        normal = self.rot90_cw(tangent)

        return point, self.unit(tangent), self.unit(normal)

    def _make_ply_station_s_values(self, outer_profile):
        """
        按“当前点切线与上一个剪口点切线偏差达到 5°”生成分段点。
        这里的 5° 来自 Params.ply_tangent_angle_step_deg。
        """
        p = self.p

        total_len = self._outer_profile_total_length(outer_profile)
        max_angle = p.ply_tangent_angle_step_deg

        if max_angle <= 0:
            raise ValueError("ply_tangent_angle_step_deg 必须为正数")

        dense_n = max(4000, int(total_len * 20))
        ds = total_len / dense_n

        stations = [0.0]
        _, last_tangent, _ = self._outer_profile_point_tangent_normal_at_s(
            outer_profile,
            0.0,
        )

        prev_s = 0.0
        for i in range(1, dense_n + 1):
            s = min(total_len, i * ds)
            _, tangent, _ = self._outer_profile_point_tangent_normal_at_s(
                outer_profile,
                s,
            )

            angle = self._angle_between_deg(last_tangent, tangent)
            if angle + 1e-9 >= max_angle:
                lo = prev_s
                hi = s

                # 二分找到更接近 5° 的位置
                for _ in range(32):
                    mid = 0.5 * (lo + hi)
                    _, tm, _ = self._outer_profile_point_tangent_normal_at_s(
                        outer_profile,
                        mid,
                    )
                    am = self._angle_between_deg(last_tangent, tm)
                    if am < max_angle:
                        lo = mid
                    else:
                        hi = mid

                cut_s = hi
                stations.append(cut_s)

                _, last_tangent, _ = self._outer_profile_point_tangent_normal_at_s(
                    outer_profile,
                    cut_s,
                )
                prev_s = cut_s
            else:
                prev_s = s

        if total_len - stations[-1] > 1e-6:
            stations.append(total_len)

        return stations

    def _make_expanded_polyline(self, outer_profile):
        """
        基于叶身外侧曲线，沿外法向生成 z=0 平面上的扩展曲线。
        """
        p = self.p

        total_len = self._outer_profile_total_length(outer_profile)
        n = max(20, int(p.ply_expand_samples))
        offset = float(p.ply_expand_offset)

        pts = []
        cum = [0.0]

        for i in range(n):
            s = total_len * i / (n - 1)
            point, _, normal = self._outer_profile_point_tangent_normal_at_s(
                outer_profile,
                s,
            )

            q = cq.Vector(
                point.x + normal.x * offset,
                point.y + normal.y * offset,
                0.0,
            )
            pts.append(q)

            if i > 0:
                d = self.norm(self.sub(pts[i], pts[i - 1]))
                cum.append(cum[-1] + d)

        return pts, cum

    def _map_blade_s_to_expanded_length(self, outer_profile, s):
        """
        将叶身外侧曲线上的弧长参数 s 映射到扩展曲线上的弧长。

        关键：直线段在偏移后长度不变，而圆弧段半径从 R 变为 R+offset，
        弧长按比例增长。简单的百分比映射忽略了这一差异，导致直线段上
        的站点被映射到扩展曲线的圆弧段上，使连接线不再垂直于直线段。
        """
        p = self.p
        r = float(outer_profile["radius"])
        offset = float(p.ply_expand_offset)

        lower_len = p.blade_lower_length
        arc_angle_rad = math.radians(
            p.upper_tangent_angle_deg - p.lower_tangent_angle_deg
        )
        arc_len = r * arc_angle_rad
        upper_len = p.blade_upper_length

        expanded_arc_len = (r + offset) * arc_angle_rad

        s = max(0.0, min(float(s), lower_len + arc_len + upper_len))

        if s <= lower_len:
            return s
        elif s <= lower_len + arc_len:
            frac = (s - lower_len) / arc_len
            return lower_len + frac * expanded_arc_len
        else:
            return lower_len + expanded_arc_len + (s - lower_len - arc_len)

    @staticmethod
    def _interp_polyline_at_length(pts, cum, target_len):
        if not pts:
            raise ValueError("空折线无法插值")

        total = cum[-1]
        target_len = max(0.0, min(float(target_len), total))

        if target_len <= 0:
            return pts[0]
        if target_len >= total:
            return pts[-1]

        lo = 0
        hi = len(cum) - 1

        while hi - lo > 1:
            mid = (lo + hi) // 2
            if cum[mid] < target_len:
                lo = mid
            else:
                hi = mid

        seg_len = cum[hi] - cum[lo]
        if seg_len <= 1e-12:
            return pts[lo]

        t = (target_len - cum[lo]) / seg_len
        a = pts[lo]
        b = pts[hi]

        return cq.Vector(
            a.x + (b.x - a.x) * t,
            a.y + (b.y - a.y) * t,
            0.0,
        )

    @staticmethod
    def _clamped_interval(center, length, total):
        """在 [0, total] 内以 center 为中心取 length 长度，越界则平移。"""
        length = min(float(length), float(total))
        a = center - 0.5 * length
        b = center + 0.5 * length
        if a < 0.0:
            b -= a; a = 0.0
        if b > total:
            a -= b - total; b = total
        a = max(0.0, a)
        b = min(total, b)
        return a, b

    def _make_edge_polyline(self, pts):
        """
        用多段直线生成 wire。比 makeSpline 更稳，STEP 下游软件也更容易识别。
        """
        clean = []

        for pt in pts:
            if not clean:
                clean.append(pt)
                continue

            if (
                abs(pt.x - clean[-1].x) > 1e-9
                or abs(pt.y - clean[-1].y) > 1e-9
                or abs(pt.z - clean[-1].z) > 1e-9
            ):
                clean.append(pt)

        if len(clean) < 2:
            raise ValueError("至少需要两个不同点才能生成线")

        edges = [
            cq.Edge.makeLine(clean[i], clean[i + 1])
            for i in range(len(clean) - 1)
        ]

        return self.make_wire(edges)

    @staticmethod
    def _make_circular_arc_edge(cx, cy, radius, angle_start_deg, angle_end_deg):
        """在 XY 平面上创建真实圆弧边（非折线逼近）。"""
        a0 = math.radians(angle_start_deg)
        a1 = math.radians(angle_end_deg)
        p0 = cq.Vector(cx + radius * math.cos(a0), cy + radius * math.sin(a0), 0.0)
        p1 = cq.Vector(cx + radius * math.cos(a1), cy + radius * math.sin(a1), 0.0)
        am = 0.5 * (a0 + a1)
        pm = cq.Vector(cx + radius * math.cos(am), cy + radius * math.sin(am), 0.0)
        return cq.Edge.makeThreePointArc(p0, pm, p1)

    def _make_profile_segment_wire(self, outer_profile, s0, s1, z, sample_count=8):
        """
        叶身外侧曲线上的一小段，用于显示每条铺层带在叶身侧的等弧长段。
        """
        if s1 < s0:
            s0, s1 = s1, s0

        n = max(2, int(sample_count))
        pts = []

        for i in range(n):
            s = s0 + (s1 - s0) * i / (n - 1)
            q, _, _ = self._outer_profile_point_tangent_normal_at_s(
                outer_profile,
                s,
            )
            pts.append(self._v3(q, z))

        return self._make_edge_polyline(pts)

    def _make_expanded_segment_wire(self, q0, q1, z=0.0):
        """
        扩展曲线上的等弧长截取段。
        这里用直线连接端点。
        """
        return self._make_edge_polyline([
            self._v3(q0, z),
            self._v3(q1, z),
        ])

    def _connector_polyline_pts(self, root_xy, end_xy, normal_xy):
        """返回翻折路径的有序点列表 (top → bottom):
        blade 顶面 → 竖向段 → 倒圆段 → 缘板顶面 → 扩展曲线端点。
        """
        p = self.p
        r = max(0.0, float(p.root_fillet_radius))
        blade_z_top = float(p.blade_height)

        pts = []
        pts.append(self._v3(root_xy, blade_z_top))

        if r > 1e-9:
            pts.append(self._v3(root_xy, r))
            n = max(3, int(p.ply_fillet_samples))
            for i in range(1, n + 1):
                theta = 0.5 * math.pi * i / n
                radial = r * (1.0 - math.cos(theta))
                z = r * (1.0 - math.sin(theta))
                q = cq.Vector(
                    root_xy.x + normal_xy.x * radial,
                    root_xy.y + normal_xy.y * radial,
                    0.0,
                )
                pts.append(self._v3(q, z))
            endwall_start = cq.Vector(
                root_xy.x + normal_xy.x * r,
                root_xy.y + normal_xy.y * r,
                0.0,
            )
        else:
            pts.append(self._v3(root_xy, 0.0))
            endwall_start = root_xy

        pts.append(self._v3(endwall_start, 0.0))
        pts.append(self._v3(end_xy, 0.0))
        return pts

    def _make_surface_connector_wire(self, root_xy, end_xy, normal_xy):
        """从叶身外表面翻折到缘板 z=0 平面的参考线 (Wire)。"""
        pts = self._connector_polyline_pts(root_xy, end_xy, normal_xy)
        return self._make_edge_polyline(pts)

    def build_ply_reference_curves(self, outer_profile):
        """
        生成铺层划分参考曲线。

        输出内容：
            - ply_station_xxxx：每个剪口/分段点对应的翻折路径线
            - ply_band_xxxx_blade_equal_arc：叶身外侧等弧长分段
            - ply_band_xxxx_expanded_equal_arc：扩展曲线上居中截取的等弧长段
            - ply_band_xxxx_side_a / side_b：每个铺层带的左右翻折边界

        这些曲线只作为 STEP 中的 wire，不修改 final_fillet_body。
        """
        p = self.p

        if not p.make_ply_curves:
            return {}

        stations = self._make_ply_station_s_values(outer_profile)

        expanded_pts, expanded_cum = self._make_expanded_polyline(outer_profile)

        blade_total = self._outer_profile_total_length(outer_profile)
        expanded_total = expanded_cum[-1]

        curves = {}

        print("[铺层划分参考曲线]")
        print(f"  tangent angle step      = {p.ply_tangent_angle_step_deg} deg")
        print(f"  blade outer length      = {blade_total:.6f}")
        print(f"  expanded length         = {expanded_total:.6f}")
        print(f"  station count           = {len(stations)}")
        print(f"  band count              = {max(0, len(stations) - 1)}")

        # 1) 先生成所有分段点对应的”叶身 -> 倒圆 -> 缘板扩展曲线”路径
        for idx, s in enumerate(stations):
            exp_len = self._map_blade_s_to_expanded_length(outer_profile, s)

            root_xy, _, normal_xy = self._outer_profile_point_tangent_normal_at_s(
                outer_profile,
                s,
            )
            end_xy = self._interp_polyline_at_length(
                expanded_pts,
                expanded_cum,
                exp_len,
            )

            curves[f"ply_station_{idx:04d}"] = self._make_surface_connector_wire(
                root_xy=root_xy,
                end_xy=end_xy,
                normal_xy=normal_xy,
            )

        # 2) 对每个分段，在扩展曲线上截取等弧长段（与叶身段等长）
        arc_start_blade = p.blade_lower_length
        arc_angle_rad = math.radians(
            p.upper_tangent_angle_deg - p.lower_tangent_angle_deg)
        arc_len_blade = float(outer_profile["radius"]) * arc_angle_rad
        arc_end_blade = arc_start_blade + arc_len_blade
        expanded_arc_len = (float(outer_profile["radius"]) + p.ply_expand_offset) * arc_angle_rad

        for idx in range(len(stations) - 1):
            s0 = stations[idx]
            s1 = stations[idx + 1]
            blade_seg_len = s1 - s0

            # 等弧长：中心映射，长度为叶身段弧长
            expanded_center_len = self._map_blade_s_to_expanded_length(
                outer_profile, 0.5 * (s0 + s1))
            q0_len, q1_len = self._clamped_interval(
                expanded_center_len, blade_seg_len, expanded_total)
            q0 = self._interp_polyline_at_length(expanded_pts, expanded_cum, q0_len)
            q1 = self._interp_polyline_at_length(expanded_pts, expanded_cum, q1_len)

            curves[f"ply_band_{idx:04d}_blade_equal_arc"] = (
                self._make_profile_segment_wire(
                    outer_profile=outer_profile,
                    s0=s0,
                    s1=s1,
                    z=p.blade_height,
                    sample_count=8,
                )
            )

            # 扩展段：圆弧段用真实圆弧，直线段用直线
            on_expanded_arc = (q0_len >= arc_start_blade - 1e-9
                               and q1_len <= arc_start_blade + expanded_arc_len + 1e-9)
            if on_expanded_arc:
                offset = float(p.ply_expand_offset)
                Rblade = float(outer_profile["radius"])
                q0_ang = p.lower_tangent_angle_deg + math.degrees(
                    (q0_len - arc_start_blade) / (Rblade + offset))
                q1_ang = p.lower_tangent_angle_deg + math.degrees(
                    (q1_len - arc_start_blade) / (Rblade + offset))
                exp_arc_edge = self._make_circular_arc_edge(
                    0, 0, Rblade + offset, q0_ang, q1_ang)
                curves[f"ply_band_{idx:04d}_expanded_equal_arc"] = (
                    cq.Wire.assembleEdges([exp_arc_edge]))
            else:
                curves[f"ply_band_{idx:04d}_expanded_equal_arc"] = (
                    self._make_expanded_segment_wire(q0=q0, q1=q1, z=0.0))

            root0, _, n0 = self._outer_profile_point_tangent_normal_at_s(
                outer_profile, s0)
            root1, _, n1 = self._outer_profile_point_tangent_normal_at_s(
                outer_profile, s1)

            curves[f"ply_band_{idx:04d}_side_a"] = self._make_surface_connector_wire(
                root_xy=root0, end_xy=q0, normal_xy=n0)
            curves[f"ply_band_{idx:04d}_side_b"] = self._make_surface_connector_wire(
                root_xy=root1, end_xy=q1, normal_xy=n1)

        return curves

    def build_ply_band_solids(self, outer_profile):
        """为每个铺层带生成有厚度的可视化实体。

        每个 band 实体 = blade 段 + fillet 段 + endwall 段，三段融合：
          - blade  层 (z=r .. blade_height)：叶身外表面向内偏置(叶身角度)，沿 Z 拉伸
          - fillet 层 (z=0 .. r)：倒圆四分之一圆环截面绕 Z 轴回转/沿切线拉伸 (叶身角度)
          - endwall 层 (z=-thickness .. 0)：梯形面，内边界叶身角度(宽)→外边界扩展角度(窄)

        三层曲线（叶身外廓/倒圆底边/扩展曲线）上取等弧长段，侧面连接。
        """
        p = self.p
        thickness = float(p.ply_layer_thickness)

        if thickness <= 0:
            return {}

        stations = self._make_ply_station_s_values(outer_profile)
        expanded_pts, expanded_cum = self._make_expanded_polyline(outer_profile)
        expanded_total = expanded_cum[-1]
        r = max(0.0, float(p.root_fillet_radius))

        # 分段信息
        arc_start = p.blade_lower_length
        arc_angle_rad = math.radians(
            p.upper_tangent_angle_deg - p.lower_tangent_angle_deg)
        arc_len = float(outer_profile["radius"]) * arc_angle_rad
        arc_end = arc_start + arc_len

        solids = {}
        success = 0
        failed = 0

        for idx in range(len(stations) - 1):
            s0 = stations[idx]
            s1 = stations[idx + 1]

            root0, _, n0 = self._outer_profile_point_tangent_normal_at_s(
                outer_profile, s0)
            root1, _, n1 = self._outer_profile_point_tangent_normal_at_s(
                outer_profile, s1)

            # 扩展曲线上等弧长段端点（与叶身段等长，居中放置）
            blade_seg_len = s1 - s0
            expanded_center_len = self._map_blade_s_to_expanded_length(
                outer_profile, 0.5 * (s0 + s1))
            q0_len, q1_len = self._clamped_interval(
                expanded_center_len, blade_seg_len, expanded_total)
            q0 = self._interp_polyline_at_length(expanded_pts, expanded_cum, q0_len)
            q1 = self._interp_polyline_at_length(expanded_pts, expanded_cum, q1_len)

            # endwall_start 点
            if r > 1e-9:
                ew_start0 = cq.Vector(
                    root0.x + n0.x * r, root0.y + n0.y * r, 0.0)
                ew_start1 = cq.Vector(
                    root1.x + n1.x * r, root1.y + n1.y * r, 0.0)
            else:
                ew_start0 = root0
                ew_start1 = root1

            # ── 判断 band 所在分段类型 ──
            on_arc = (s0 >= arc_start - 1e-9 and s1 <= arc_end + 1e-9)
            on_lower_straight = (s1 <= arc_start + 1e-9)
            on_upper_straight = (s0 >= arc_end - 1e-9)
            is_lower_trans = (s0 < arc_start - 1e-9 and s1 > arc_start + 1e-9)
            is_upper_trans = (s0 < arc_end - 1e-9 and s1 > arc_end + 1e-9)

            # ── 角度（内边界=叶身角/宽，外边界=扩展角/窄）──
            _offset = float(p.ply_expand_offset)
            _Rblade = float(outer_profile["radius"])
            _θ_low = p.lower_tangent_angle_deg
            _θ_up = p.upper_tangent_angle_deg
            _deg = math.degrees

            if on_arc:
                # 内边界（叶身）角度：分母=R
                θ_in0, θ_in1 = self._band_arc_angles_deg(s0, s1)
                # 外边界（扩展曲线）角度：分母=R+offset
                _out_center = _θ_low + _deg(
                    (expanded_center_len - arc_start) / (_Rblade + _offset))
                _out_half = 0.5 * _deg(
                    blade_seg_len / (_Rblade + _offset))
                θ_out0 = _out_center - _out_half
                θ_out1 = _out_center + _out_half
            elif is_lower_trans:
                θ_in0 = _θ_low
                _, θ_in1 = self._band_arc_angles_deg(arc_start, s1)
                # outer angles from expanded curve positions
                if q0_len >= arc_start:
                    θ_out0 = _θ_low + _deg(
                        (q0_len - arc_start) / (_Rblade + _offset))
                else:
                    θ_out0 = _θ_low
                θ_out1 = _θ_low + _deg(
                    (q1_len - arc_start) / (_Rblade + _offset))
            elif is_upper_trans:
                θ_in0, _ = self._band_arc_angles_deg(s0, arc_end)
                θ_in1 = _θ_up
                # outer angles from expanded curve positions
                _exp_arc_end = arc_start + (
                    float(outer_profile["radius"]) + _offset) * arc_angle_rad
                θ_out0 = _θ_low + _deg(
                    (q0_len - arc_start) / (_Rblade + _offset))
                if q1_len <= _exp_arc_end:
                    θ_out1 = _θ_low + _deg(
                        (q1_len - arc_start) / (_Rblade + _offset))
                else:
                    θ_out1 = _θ_up

            # ── Build fillet sub-solid (z=0 .. r) ──
            if r > 1e-9:
                if on_arc:
                    fillet_solid = self._make_fillet_solid_arc(
                        float(outer_profile["radius"]), r, thickness,
                        θ_in0, θ_in1)
                elif on_lower_straight or on_upper_straight:
                    fillet_solid = self._make_fillet_solid_straight(
                        float(outer_profile["radius"]), r, thickness,
                        s0, s1, outer_profile)
                else:
                    if is_lower_trans:
                        _f_arc_θ0, _f_arc_θ1 = θ_in0, θ_in1
                    else:
                        _f_arc_θ0, _f_arc_θ1 = θ_in0, θ_in1
                    fillet_solid = self._make_fillet_solid_transition(
                        float(outer_profile["radius"]), r, thickness,
                        s0, s1, outer_profile,
                        arc_start, arc_end, _f_arc_θ0, _f_arc_θ1)
            else:
                fillet_solid = None

            # ── Build blade sub-solid (z=r .. blade_height) ──
            blade_height = float(p.blade_height)
            if on_arc:
                Rblade = float(outer_profile["radius"])
                a0r, a1r = math.radians(θ_in0), math.radians(θ_in1)

                blade_outer_arc = self._make_circular_arc_edge(
                    0, 0, Rblade, θ_in0, θ_in1)
                blade_inner_arc = self._make_circular_arc_edge(
                    0, 0, Rblade - thickness, θ_in1, θ_in0)

                bp_start = cq.Vector(
                    Rblade * math.cos(a0r), Rblade * math.sin(a0r), 0.0)
                bp_end = cq.Vector(
                    Rblade * math.cos(a1r), Rblade * math.sin(a1r), 0.0)
                ip_start = cq.Vector(
                    (Rblade - thickness) * math.cos(a0r),
                    (Rblade - thickness) * math.sin(a0r), 0.0)
                ip_end = cq.Vector(
                    (Rblade - thickness) * math.cos(a1r),
                    (Rblade - thickness) * math.sin(a1r), 0.0)

                blade_edges = [
                    blade_outer_arc,
                    cq.Edge.makeLine(bp_end, ip_end),
                    blade_inner_arc,
                    cq.Edge.makeLine(ip_start, bp_start),
                ]
            elif is_lower_trans or is_upper_trans:
                Rblade = float(outer_profile["radius"])
                if is_lower_trans:
                    s_line = (s0, arc_start)
                    θ_arc0 = p.lower_tangent_angle_deg
                    _, θ_arc1 = self._band_arc_angles_deg(arc_start, s1)
                else:
                    s_line = (arc_end, s1)
                    θ_arc0, _ = self._band_arc_angles_deg(s0, arc_end)
                    θ_arc1 = p.upper_tangent_angle_deg

                a0r, a1r = math.radians(θ_arc0), math.radians(θ_arc1)
                arc_start_pt = cq.Vector(
                    Rblade * math.cos(a0r), Rblade * math.sin(a0r), 0.0)
                arc_end_pt = cq.Vector(
                    Rblade * math.cos(a1r), Rblade * math.sin(a1r), 0.0)
                ias = cq.Vector(
                    (Rblade - thickness) * math.cos(a0r),
                    (Rblade - thickness) * math.sin(a0r), 0.0)
                iae = cq.Vector(
                    (Rblade - thickness) * math.cos(a1r),
                    (Rblade - thickness) * math.sin(a1r), 0.0)

                pt_l0, _, n_l0 = self._outer_profile_point_tangent_normal_at_s(
                    outer_profile, s_line[0])
                pt_l1, _, n_l1 = self._outer_profile_point_tangent_normal_at_s(
                    outer_profile, s_line[1])
                il0 = cq.Vector(pt_l0.x - n_l0.x * thickness,
                                pt_l0.y - n_l0.y * thickness, 0.0)
                il1 = cq.Vector(pt_l1.x - n_l1.x * thickness,
                                pt_l1.y - n_l1.y * thickness, 0.0)

                outer_arc_edge = self._make_circular_arc_edge(
                    0, 0, Rblade, θ_arc0, θ_arc1)
                inner_arc_edge = self._make_circular_arc_edge(
                    0, 0, Rblade - thickness, θ_arc1, θ_arc0)

                if is_lower_trans:
                    bp_start, bp_end = pt_l0, arc_end_pt
                    ip_start, ip_end = il0, iae
                    blade_edges = [
                        cq.Edge.makeLine(pt_l0, arc_start_pt),
                        outer_arc_edge,
                        cq.Edge.makeLine(arc_end_pt, iae),
                        inner_arc_edge,
                        cq.Edge.makeLine(ias, il0),
                        cq.Edge.makeLine(il0, pt_l0),
                    ]
                else:
                    bp_start, bp_end = arc_start_pt, pt_l1
                    ip_start, ip_end = ias, il1
                    blade_edges = [
                        outer_arc_edge,
                        cq.Edge.makeLine(arc_end_pt, pt_l1),
                        cq.Edge.makeLine(pt_l1, il1),
                        cq.Edge.makeLine(il1, iae),
                        inner_arc_edge,
                        cq.Edge.makeLine(ias, arc_start_pt),
                    ]
            else:
                n_pts = max(4, int((s1 - s0) * 4))
                outer_xy = []
                inner_xy = []
                for i in range(n_pts + 1):
                    s = s0 + (s1 - s0) * i / n_pts
                    pt, _, normal = self._outer_profile_point_tangent_normal_at_s(
                        outer_profile, s)
                    outer_xy.append(pt)
                    inner_xy.append(cq.Vector(
                        pt.x - normal.x * thickness,
                        pt.y - normal.y * thickness,
                        0.0,
                    ))
                bp_start = outer_xy[0]
                bp_end = outer_xy[-1]
                ip_start = inner_xy[0]
                ip_end = inner_xy[-1]

                blade_edges = []
                for i in range(len(outer_xy) - 1):
                    blade_edges.append(
                        cq.Edge.makeLine(outer_xy[i], outer_xy[i + 1]))
                for i in range(len(inner_xy) - 1, 0, -1):
                    blade_edges.append(
                        cq.Edge.makeLine(inner_xy[i], inner_xy[i - 1]))
                blade_edges.append(cq.Edge.makeLine(bp_start, ip_start))
                blade_edges.append(cq.Edge.makeLine(ip_end, bp_end))

            blade_wire = cq.Wire.assembleEdges(blade_edges)
            blade_face = cq.Face.makeFromWires(blade_wire)
            blade_solid = (
                cq.Workplane("XY")
                .add(blade_face)
                .extrude(blade_height - r)
                .val()
            )
            if r > 1e-9:
                blade_solid = blade_solid.translate(
                    cq.Vector(0.0, 0.0, r))

            # ── Build endwall sub-solid (z=-thickness .. 0) ──
            # ── 梯形端壁面：内边界(叶身角度/宽)→外边界(扩展角度/窄) ──
            if on_arc:
                offset = float(p.ply_expand_offset)
                Rblade = float(outer_profile["radius"])

                in_a0r, in_a1r = math.radians(θ_in0), math.radians(θ_in1)
                out_a0r, out_a1r = math.radians(θ_out0), math.radians(θ_out1)

                # 外边界点位（扩展角度）
                q0 = cq.Vector((Rblade + offset) * math.cos(out_a0r),
                               (Rblade + offset) * math.sin(out_a0r), 0.0)
                q1 = cq.Vector((Rblade + offset) * math.cos(out_a1r),
                               (Rblade + offset) * math.sin(out_a1r), 0.0)

                # 内边界：叶身角度 (宽)
                inner_arc = self._make_circular_arc_edge(
                    0, 0, Rblade + r, θ_in0, θ_in1)
                # 外边界：扩展角度 (窄)
                outer_arc = self._make_circular_arc_edge(
                    0, 0, Rblade + offset, θ_out1, θ_out0)

                fb_s0 = cq.Vector((Rblade + r) * math.cos(in_a0r),
                                  (Rblade + r) * math.sin(in_a0r), 0.0)
                fb_s1 = cq.Vector((Rblade + r) * math.cos(in_a1r),
                                  (Rblade + r) * math.sin(in_a1r), 0.0)

                endwall_edges = [
                    inner_arc,
                    cq.Edge.makeLine(fb_s1, q1),
                    outer_arc,
                    cq.Edge.makeLine(q0, fb_s0),
                ]
            elif is_lower_trans or is_upper_trans:
                offset = float(p.ply_expand_offset)
                Rblade = float(outer_profile["radius"])
                in_a0r, in_a1r = math.radians(θ_in0), math.radians(θ_in1)
                out_a0r, out_a1r = math.radians(θ_out0), math.radians(θ_out1)

                if is_lower_trans:
                    # Inner: straight(s0→as) + arc(as→s1, blade angles)
                    pt_s0, _, n_s0 = self._outer_profile_point_tangent_normal_at_s(
                        outer_profile, s0)
                    pt_as, _, n_as = self._outer_profile_point_tangent_normal_at_s(
                        outer_profile, arc_start)
                    fb_s0 = cq.Vector(pt_s0.x + n_s0.x * r,
                                      pt_s0.y + n_s0.y * r, 0.0)
                    fb_as = cq.Vector(pt_as.x + n_as.x * r,
                                      pt_as.y + n_as.y * r, 0.0)

                    inner_arc = self._make_circular_arc_edge(
                        0, 0, Rblade + r, θ_in0, θ_in1)

                    fb_s1 = cq.Vector((Rblade + r) * math.cos(in_a1r),
                                      (Rblade + r) * math.sin(in_a1r), 0.0)
                    eq_s1 = cq.Vector((Rblade + offset) * math.cos(out_a1r),
                                      (Rblade + offset) * math.sin(out_a1r), 0.0)

                    # Outer: may be all-arc or polyline+arc
                    if q0_len >= arc_start:
                        # all arc: q0→q1
                        eq_s0 = cq.Vector((Rblade + offset) * math.cos(out_a0r),
                                          (Rblade + offset) * math.sin(out_a0r), 0.0)
                        outer_arc = self._make_circular_arc_edge(
                            0, 0, Rblade + offset, θ_out1, θ_out0)
                        endwall_edges = [
                            cq.Edge.makeLine(fb_s0, fb_as),
                            inner_arc,
                            cq.Edge.makeLine(fb_s1, eq_s1),
                            outer_arc,
                            cq.Edge.makeLine(eq_s0, fb_s0),
                        ]
                    else:
                        # straight(q0→exp_as) + arc(exp_as→q1)
                        eq_as = cq.Vector((Rblade + offset) * math.cos(out_a0r),
                                          (Rblade + offset) * math.sin(out_a0r), 0.0)
                        outer_arc = self._make_circular_arc_edge(
                            0, 0, Rblade + offset, θ_out1, θ_out0)
                        endwall_edges = [
                            cq.Edge.makeLine(fb_s0, fb_as),
                            inner_arc,
                            cq.Edge.makeLine(fb_s1, eq_s1),
                            outer_arc,
                            cq.Edge.makeLine(eq_as, q0),
                            cq.Edge.makeLine(q0, fb_s0),
                        ]
                else:
                    # Upper transition
                    # Inner: arc(s0→ae, blade angles) + straight(ae→s1)
                    inner_arc = self._make_circular_arc_edge(
                        0, 0, Rblade + r, θ_in0, θ_in1)

                    pt_ae, _, n_ae = self._outer_profile_point_tangent_normal_at_s(
                        outer_profile, arc_end)
                    pt_s1, _, n_s1 = self._outer_profile_point_tangent_normal_at_s(
                        outer_profile, s1)
                    fb_ae = cq.Vector(pt_ae.x + n_ae.x * r,
                                      pt_ae.y + n_ae.y * r, 0.0)
                    fb_s1 = cq.Vector(pt_s1.x + n_s1.x * r,
                                      pt_s1.y + n_s1.y * r, 0.0)

                    fb_s0 = cq.Vector((Rblade + r) * math.cos(in_a0r),
                                      (Rblade + r) * math.sin(in_a0r), 0.0)
                    eq_s0 = cq.Vector((Rblade + offset) * math.cos(out_a0r),
                                      (Rblade + offset) * math.sin(out_a0r), 0.0)

                    _exp_arc_end = arc_start + (
                        float(outer_profile["radius"]) + offset) * arc_angle_rad

                    # Outer: may be all-arc or arc+polyline
                    if q1_len <= _exp_arc_end:
                        # all arc: q0→q1
                        eq_s1 = cq.Vector((Rblade + offset) * math.cos(out_a1r),
                                          (Rblade + offset) * math.sin(out_a1r), 0.0)
                        outer_arc = self._make_circular_arc_edge(
                            0, 0, Rblade + offset, θ_out1, θ_out0)
                        endwall_edges = [
                            inner_arc,
                            cq.Edge.makeLine(fb_ae, fb_s1),
                            cq.Edge.makeLine(fb_s1, eq_s1),
                            outer_arc,
                            cq.Edge.makeLine(eq_s0, fb_s0),
                        ]
                    else:
                        # arc(q0→exp_ae) + polyline(exp_ae→q1)
                        eq_ae = cq.Vector((Rblade + offset) * math.cos(out_a1r),
                                          (Rblade + offset) * math.sin(out_a1r), 0.0)
                        outer_arc = self._make_circular_arc_edge(
                            0, 0, Rblade + offset, θ_out1, θ_out0)
                        endwall_edges = [
                            inner_arc,
                            cq.Edge.makeLine(fb_ae, fb_s1),
                            cq.Edge.makeLine(fb_s1, q1),
                            cq.Edge.makeLine(q1, eq_ae),
                            outer_arc,
                            cq.Edge.makeLine(eq_s0, fb_s0),
                        ]
            else:
                # 纯直线段：折线采样
                exp_bottom_n = max(4, int(abs(q1_len - q0_len) * 0.3))
                exp_seg_xy = []
                for i in range(exp_bottom_n + 1):
                    t_val = q0_len if exp_bottom_n == 0 else (
                        q0_len + (q1_len - q0_len) * i / exp_bottom_n)
                    pt = self._interp_polyline_at_length(expanded_pts, expanded_cum, t_val)
                    exp_seg_xy.append(pt)

                endwall_edges = [
                    cq.Edge.makeLine(ew_start0, ew_start1),
                    cq.Edge.makeLine(ew_start1, exp_seg_xy[-1]),
                ]
                for i in range(len(exp_seg_xy) - 1, 0, -1):
                    endwall_edges.append(
                        cq.Edge.makeLine(exp_seg_xy[i], exp_seg_xy[i - 1]))
                endwall_edges.append(
                    cq.Edge.makeLine(exp_seg_xy[0], ew_start0))

            try:
                endwall_wire = cq.Wire.assembleEdges(endwall_edges)
                endwall_face = cq.Face.makeFromWires(endwall_wire)
                endwall_solid = (
                    cq.Workplane("XY")
                    .add(endwall_face)
                    .extrude(-thickness)
                    .val()
                )
                # 融合：blade + fillet + endwall
                band_solid = blade_solid
                if fillet_solid is not None:
                    band_solid = band_solid.fuse(fillet_solid)
                band_solid = band_solid.fuse(endwall_solid)

                if not band_solid.isValid():
                    raise RuntimeError("Fused solid not valid")
            except Exception as e:
                failed += 1
                print(f"  [SKIP] band {idx}: {e}")
                continue

            solids[f"ply_band_{idx:04d}_solid"] = band_solid
            success += 1

        print(f"[铺层带实体] {success}/{len(stations) - 1} bands generated"
              + (f", {failed} failed" if failed else ""))

        # 自检
        if solids:
            self._validate_band_solids(solids, outer_profile)
        return solids

    def _validate_band_solids(self, solids, outer_profile):
        """自检: 验证每个 band solid 在 blade / fillet / endwall 区域都有材料.

        测试点选取:
          - blade:  mid-band 处, r=-t/2, z=blade_h*0.7
          - fillet: mid-band 处, 内外弧中点高度
          - endwall: mid-band 处, 端壁中间深度
        方向向量使用 band 中点处的实际外法向 (pt_m/normal).
        """
        from OCP.BRepClass3d import BRepClass3d_SolidClassifier
        from OCP.gp import gp_Pnt
        from OCP.TopAbs import TopAbs_IN

        p = self.p
        R = float(p.outer_radius)
        r = float(p.root_fillet_radius)
        t = float(p.ply_layer_thickness)
        blade_h = float(p.blade_height)
        stations = self._make_ply_station_s_values(outer_profile)

        errors = []

        for idx in range(len(stations) - 1):
            key = f"ply_band_{idx:04d}_solid"
            if key not in solids:
                continue
            solid = solids[key]

            sm = 0.5 * (stations[idx] + stations[idx + 1])
            pt_m, tangent, normal = self._outer_profile_point_tangent_normal_at_s(
                outer_profile, sm)

            def point_inside(pt_along_normal, zz):
                """pt_along_normal: 沿外法向距离 blade 表面的偏移; zz: Z 坐标."""
                pt = cq.Vector(
                    pt_m.x + normal.x * pt_along_normal,
                    pt_m.y + normal.y * pt_along_normal,
                    zz)
                c = BRepClass3d_SolidClassifier(solid.wrapped)
                c.Perform(gp_Pnt(pt.x, pt.y, pt.z), 1e-3)
                return c.State() == TopAbs_IN

            # Blade: 向内 (opposite normal) t/2, 在 blade 中上部
            if not point_inside(-t * 0.5, blade_h * 0.7):
                errors.append(f"band {idx}: blade region empty")

            # Fillet: XZ 轮廓中点 (dist=2.5 from center(35,2), angle=225°)
            # 径向偏置 ≈ r*(1-cos45°)≈0.586, z≈r*(1-sin45°)≈0.586
            if r > 1e-9:
                phi = math.pi / 4.0
                fil_r = r * (1.0 - math.cos(phi))
                fil_z = r * (1.0 - math.sin(phi))
                # 厚度中间：径向向内偏 t*cos(phi)/2 ≈ t*0.35
                fil_r -= t * math.cos(phi) * 0.5
                fil_z -= t * math.sin(phi) * 0.5
                if not point_inside(fil_r, fil_z):
                    errors.append(f"band {idx}: fillet region empty")

            # Endwall: 在端壁中间深度 (R+r + offset*0.3 处)
            if not point_inside(r + (p.ply_expand_offset - r) * 0.3, -t * 0.5):
                errors.append(f"band {idx}: endwall region empty")

        if errors:
            print(f"[VALIDATION FAILED] {len(errors)} errors:")
            for e in errors:
                print(f"  - {e}")
        else:
            print(f"[VALIDATION PASSED] all {len(solids)} bands OK"
                  f" (blade + fillet + endwall)")

    # ── Fillet solid helpers ──

    def _band_arc_angles_deg(self, s0, s1):
        """返回 band [s0, s1] 在圆弧段上对应的起止角度 (度)."""
        p = self.p
        lower_len = p.blade_lower_length
        arc_angle_span = p.upper_tangent_angle_deg - p.lower_tangent_angle_deg
        arc_len = p.outer_radius * math.radians(arc_angle_span)

        s_arc0 = s0 - lower_len
        s_arc1 = s1 - lower_len
        θ0 = p.lower_tangent_angle_deg + (s_arc0 / arc_len) * arc_angle_span
        θ1 = p.lower_tangent_angle_deg + (s_arc1 / arc_len) * arc_angle_span
        return θ0, θ1

    def _make_fillet_solid_arc(self, R, r, thickness, angle0_deg, angle1_deg):
        """Arc bands 倒圆实体: 四分之一圆环截面绕 Z 轴回转.

        截面构建 (XZ 面):
          外弧: center=(R+r, r), radius=r,   (R, r) → (R+r, 0)
          内弧: center=(R+r, r), radius=r+t, (R+r, -t) → (R-t, r)
        外弧和内弧同心，内弧半径 r+t > r，往 blade 内部/endwall 下方偏移。
        """
        import math as _math
        from OCP.BRepPrimAPI import BRepPrimAPI_MakeRevol
        from OCP.gp import gp_Ax1, gp_Pnt, gp_Dir

        sqrt2 = _math.sqrt(2.0)
        t = float(thickness)

        θ0, θ1 = float(angle0_deg), float(angle1_deg)
        span = θ1 - θ0
        if span < 0.01:
            return None

        r_outer = float(r)
        r_inner = r_outer + t          # ← 关键: 内弧半径 > 外弧半径，往内/下偏
        cx, cz = R + r_outer, r_outer  # 圆心

        profile_wp = (
            cq.Workplane("XZ")
            .moveTo(R, r_outer)                          # 外弧起点 (blade 接点)
            .threePointArc(
                (cx - r_outer / sqrt2, cz - r_outer / sqrt2),
                (R + r_outer, 0.0))                       # 外弧终点 (endwall 接点)
            .lineTo(R + r_outer, -t)                      # 直下到内弧起点
            .threePointArc(
                (cx - r_inner / sqrt2, cz - r_inner / sqrt2),
                (R - t, r_outer))                          # 内弧终点 (blade 内侧)
            .close()
        )

        try:
            face = cq.Face.makeFromWires(profile_wp.val())
        except Exception:
            return None

        axis = gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1))
        revol = BRepPrimAPI_MakeRevol(face.wrapped, axis, _math.radians(span))
        revol.Build()
        solid = cq.Solid(revol.Shape())

        if _math.fabs(θ0) > 1e-9:
            solid = solid.rotate((0, 0, 0), (0, 0, 1), θ0)

        return solid

    def _make_fillet_solid_straight(self, R, r, thickness, s0, s1, outer_profile):
        """Straight bands 倒圆实体: 四分之一圆环截面沿切线方向拉伸.

        截面放置在 band 起点 s0, 向 s1 方向 (tangent) 拉伸 seg_len.
        """
        import math as _math
        from OCP.BRepPrimAPI import BRepPrimAPI_MakePrism
        from OCP.gp import gp_Vec

        sqrt2 = _math.sqrt(2.0)
        t = float(thickness)
        seg_len = s1 - s0

        # 使用 band 起点 s0 处的位置和方向
        pt_start, tangent, normal = self._outer_profile_point_tangent_normal_at_s(
            outer_profile, s0)

        r_outer = float(r)
        r_inner = r_outer + t
        cx, cz = R + r_outer, r_outer

        # 1) 创建截面
        profile_wp = (
            cq.Workplane("XZ")
            .moveTo(R, r_outer)
            .threePointArc(
                (cx - r_outer / sqrt2, cz - r_outer / sqrt2),
                (R + r_outer, 0.0))
            .lineTo(R + r_outer, -t)
            .threePointArc(
                (cx - r_inner / sqrt2, cz - r_inner / sqrt2),
                (R - t, r_outer))
            .close()
        )

        try:
            face = cq.Face.makeFromWires(profile_wp.val())
        except Exception:
            return None

        # 2) 绕 Z 旋转使 X 对齐 normal
        angle = _math.degrees(_math.atan2(normal.y, normal.x))
        if abs(angle) > 1e-9:
            face = face.rotate((0, 0, 0), (0, 0, 1), angle)

        # 3) 平移到 s0 位置
        face = face.translate(cq.Vector(
            pt_start.x - normal.x * R,
            pt_start.y - normal.y * R,
            0.0))

        # 4) 沿 tangent 拉伸
        try:
            prism_vec = gp_Vec(
                tangent.x * seg_len,
                tangent.y * seg_len,
                0.0)
            prism = BRepPrimAPI_MakePrism(face.wrapped, prism_vec)
            prism.Build()
            return cq.Solid(prism.Shape())
        except Exception:
            return None

    def _make_fillet_solid_transition(self, R, r, thickness, s0, s1,
                                       outer_profile, arc_s, arc_e,
                                       arc_angle0, arc_angle1):
        """Transition band: split into straight + arc sub-bands."""
        t = float(thickness)

        if s0 < arc_s < s1:
            s_straight = (s0, arc_s)
            s_arc = (arc_s, s1)
        elif s0 < arc_e < s1:
            s_straight = (arc_e, s1)
            s_arc = (s0, arc_e)
        else:
            return None

        sub_solids = []
        for (sa, sb), is_arc in [
            (s_straight, False),
            (s_arc, True),
        ]:
            if sb - sa < 0.01:
                continue
            if is_arc:
                sub = self._make_fillet_solid_arc(
                    R, r, t, arc_angle0, arc_angle1)
            else:
                sub = self._make_fillet_solid_straight(
                    R, r, t, sa, sb, outer_profile)
            if sub is not None:
                sub_solids.append(sub)

        if not sub_solids:
            return None
        result = sub_solids[0]
        for s in sub_solids[1:]:
            result = result.fuse(s)
        return result

    def _thicken_face_to_solid(self, face, thickness):
        """将曲面沿法向加厚为实体。

        使用 OCCT BRepOffset_MakeOffset (Skin 模式) 对面进行等距偏移，
        自动生成侧面并与原始面缝合为封闭壳，再转为实体。
        """
        from OCP.BRepOffsetAPI import BRepOffsetAPI_MakeOffsetShape
        from OCP.BRepOffset import BRepOffset_Mode
        from OCP.BRepBuilderAPI import (
            BRepBuilderAPI_Sewing,
            BRepBuilderAPI_MakeSolid,
        )
        from OCP.TopAbs import TopAbs_SOLID, TopAbs_SHELL
        from OCP.TopExp import TopExp_Explorer
        from OCP.TopoDS import TopoDS

        # 用 PerformByJoin 偏置面 → 得到偏移面 + 侧面
        maker = BRepOffsetAPI_MakeOffsetShape()
        maker.PerformByJoin(
            face.wrapped,
            float(thickness),
            1e-3,
            BRepOffset_Mode.BRepOffset_Skin,
            False,
            False,
        )
        offset_shell = maker.Shape()

        # 将原始面也加入，缝合成封闭壳
        sewer = BRepBuilderAPI_Sewing(1e-3)
        sewer.Add(offset_shell)
        sewer.Add(face.wrapped)
        sewer.Perform()
        sewed = sewer.SewedShape()

        # 从缝合结果中找实体
        explorer = TopExp_Explorer(sewed, TopAbs_SOLID)
        if explorer.More():
            return cq.Solid(explorer.Current())

        # 找壳并转为实体
        explorer = TopExp_Explorer(sewed, TopAbs_SHELL)
        if explorer.More():
            solid_maker = BRepBuilderAPI_MakeSolid()
            solid_maker.Add(explorer.Current())
            return cq.Solid(solid_maker.Solid())

        if sewed.ShapeType() == TopAbs_SHELL:
            solid_maker = BRepBuilderAPI_MakeSolid()
            solid_maker.Add(sewed)
            return cq.Solid(solid_maker.Solid())
        if sewed.ShapeType() == TopAbs_SOLID:
            return cq.Solid(sewed)

        raise RuntimeError(
            f"Offset+sew did not produce solid (got {sewed.ShapeType()})")

    # =========================================================
    # 构造所有实体
    # =========================================================
    def build_solids(self):
        blade_data = self.build_blade_body()
        outer = blade_data["outer_profile"]
        inner = blade_data["inner_profile"]

        raw_endwall_body = self.build_raw_endwall_body(inner)

        final_body = self.apply_root_fillet(
            blade_body=blade_data["blade_body"],
            raw_endwall_body=raw_endwall_body,
            outer_profile=outer,
            inner_profile=inner,
        )

        ply_curves = self.build_ply_reference_curves(outer)
        ply_band_solids = self.build_ply_band_solids(outer)

        solids = {
            "blade_body": blade_data["blade_body"],
            "blade_lower_body": blade_data["blade_lower_body"],
            "blade_arc_body": blade_data["blade_arc_body"],
            "blade_upper_body": blade_data["blade_upper_body"],
            "raw_endwall_body": raw_endwall_body,
            "final_fillet_body": final_body,
            "ply_reference_curves": ply_curves,
            "ply_band_solids": ply_band_solids,
        }

        return solids

    # =========================================================
    # 文件处理与导出
    # =========================================================
    def cleanup_output_dir(self):
        out_dir = Path(self.p.out_dir).resolve()
        out_dir.mkdir(parents=True, exist_ok=True)

        print(f"[清理目录] {out_dir}")

        for item in out_dir.iterdir():
            if item.name in self.KEEP_FILES:
                print(f"  保留: {item.name}")
                continue

            if item.is_file() or item.is_symlink():
                item.unlink()
                print(f"  删除文件: {item.name}")
            elif item.is_dir():
                shutil.rmtree(item)
                print(f"  删除目录: {item.name}")

        return out_dir

    @staticmethod
    def remove_if_exists(path):
        if path.exists():
            if path.is_file() or path.is_symlink():
                path.unlink()
            else:
                raise RuntimeError(f"{path} 已存在且不是普通文件，无法覆盖")

    def export_step(self):
        out_dir = self.cleanup_output_dir()
        step_path = out_dir / self.p.step_name
        self.remove_if_exists(step_path)

        solids = self.build_solids()

        assy = cq.Assembly(name="VANE_BLADE_ENDWALL_WITH_PLY_REFERENCE_CURVES")

        # 最终实体：blade_body + raw_endwall_body + z=0 外部曲线倒角
        assy.add(solids["final_fillet_body"], name="final_fillet_body")

        # 铺层参考线：不参与实体建模，只作为 STEP 中的 wire 输出
        ply_curves = solids.get("ply_reference_curves", {})
        if ply_curves:
            cr, cg, cb = self.p.ply_curve_color
            ply_color = cq.Color(cr, cg, cb)

            for name, wire in ply_curves.items():
                assy.add(wire, name=name, color=ply_color)

        # 铺层带实体：每个 band 单独着色
        ply_band_solids = solids.get("ply_band_solids", {})
        if ply_band_solids:
            band_count = len(ply_band_solids)
            for name, solid in ply_band_solids.items():
                idx = int(name.split("_")[2])  # ply_band_XXXX_solid
                hue = idx / max(band_count - 1, 1) if band_count > 1 else 0.5
                # HSV → RGB 色相环，V=0.85 保持半透明感
                r, g, b = self._hsv_to_rgb(hue, 0.70, 0.85)
                assy.add(solid, name=name, color=cq.Color(r, g, b))

        # 调试分体
        if self.p.export_debug_parts:
            assy.add(solids["blade_lower_body"], name="debug_blade_lower_body")
            assy.add(solids["blade_arc_body"], name="debug_blade_arc_body")
            assy.add(solids["blade_upper_body"], name="debug_blade_upper_body")
            assy.add(solids["blade_body"], name="debug_blade_body")
            assy.add(solids["raw_endwall_body"], name="debug_raw_endwall_body")

        try:
            assy.export(str(step_path))
            print(f"[STEP 导出完成] {step_path}")
        except Exception as e:
            print(f"[Assembly 导出失败，回退导出 final_fillet_body] {e}")
            self.remove_if_exists(step_path)

            exporters.export(solids["final_fillet_body"], str(step_path))
            print(f"[STEP 导出完成] {step_path}")

        return step_path

    @staticmethod
    def _hsv_to_rgb(h, s, v):
        """HSV → RGB，h/s/v 均在 0..1 范围。"""
        import colorsys
        r, g, b = colorsys.hsv_to_rgb(h, s, v)
        return float(r), float(g), float(b)

    @staticmethod
    def print_point(name, p):
        print(f"  {name:28s} = ({p.x:10.4f}, {p.y:10.4f})")

    def run(self):
        print("=" * 80)
        print("三段闭合叶身 + 原始负向缘板 + z=0 外部曲线根部倒角 + 铺层参考线")
        print("=" * 80)
        print("[参数]")
        print(f"  blade_height             = {self.p.blade_height}")
        print(f"  endwall_height           = {self.p.endwall_height}")
        print(f"  outer_radius             = {self.p.outer_radius}")
        print(f"  blade_thickness          = {self.p.blade_thickness}")
        print(f"  upper_tangent_angle_deg  = {self.p.upper_tangent_angle_deg}")
        print(f"  blade_upper_length       = {self.p.blade_upper_length}")
        print(f"  blade_lower_length       = {self.p.blade_lower_length}")
        print(f"  lower_drop               = {self.p.lower_drop}")
        print(f"  lower_run                = {self.p.lower_run}")
        print(f"  upper_drop               = {self.p.upper_drop}")
        print(f"  upper_run                = {self.p.upper_run}")
        print(f"  root_fillet_radius       = {self.p.root_fillet_radius}")
        print(f"  root_fillet_curve        = {self.p.root_fillet_curve}")
        print(f"  make_ply_curves          = {self.p.make_ply_curves}")
        print(f"  ply_tangent_angle_step   = {self.p.ply_tangent_angle_step_deg}")
        print(f"  ply_expand_offset        = {self.p.ply_expand_offset}")
        print(f"  ply_expand_samples       = {self.p.ply_expand_samples}")
        print(f"  ply_fillet_samples       = {self.p.ply_fillet_samples}")
        print(f"  step_name                = {self.p.step_name}")
        print("=" * 80)

        return self.export_step()


def main():
    current_dir = Path(__file__).resolve().parent

    params = Params(
        # 叶身
        blade_height=80.0,
        outer_radius=33.0,
        blade_thickness=4.0,
        upper_tangent_angle_deg=130.0,
        lower_tangent_angle_deg=-90.0,

        # 叶片上下两侧分别截断
        blade_upper_length=32.0,
        blade_lower_length=36.0,

        # 缘板沿 -Z 拉伸
        endwall_height=20.0,

        # 下边：向下，再向右，再向上到交点
        lower_drop=20.0,
        lower_run=150.0,

        # 上边：向外，再沿上边方向，再作垂线到交点
        upper_drop=25.0,
        upper_run=160.0,

        # 根部真倒角
        root_fillet_radius=2.0,

        # 按你的要求：倒 z=0 平面上的叶身外部曲线
        root_fillet_curve="outer",

        # 铺层划分参考线
        make_ply_curves=True,
        ply_tangent_angle_step_deg=20.0,
        ply_expand_offset=60.0,
        ply_expand_samples=1200,
        ply_fillet_samples=12,
        ply_curve_color=(1.0, 0.0, 0.0),

        # 调试时建议 True；最终只想导出 final_fillet_body 时可设 False
        export_debug_parts=True,

        # 铺层带可视化厚度
        ply_layer_thickness=1.0,

        # 输出
        out_dir=str(current_dir),
        step_name="vane_blade_three_parts_negative_endwall_with_ply_curves.step",
    )

    builder = VaneBladeAndEndwallBuilder(params)
    builder.run()


if __name__ == "__main__":
    main()