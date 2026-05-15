我们正在用 CadQuery / OCCT 构造一个由双曲螺线截取的叶身—倒圆—缘板结构，并逐步从三角面片曲面迭代到光顺曲面，再迭代到封闭实体。最初版本的曲面是离散三角网格生成的：先在参数曲面上采样点，再把相邻四个点组成的小四边形拆成两个三角形，因此导出的 STEP 里可以看到大量三角面片。这种方法稳定、直观，但本质上不是 CAD 解析曲面或 NURBS/BSpline 曲面，而是离散面片近似，不利于后续布尔、缝合、实体化。后来改为光顺 CAD 曲面生成方法：沿 
𝑧
z 或 
𝜌
ρ 方向生成一系列横向圆弧截面，再使用 cadquery.occ_impl.shapes.loft(..., cap=False, ruled=False, continuity="C2", parametrization="chordal", degree=3, compat=True) 对开放截面进行放样，得到光顺 BREP 曲面。几何参数为：叶身半径 
𝑟
=
𝑟
blade
r=r
blade
	​

，倒圆半径 
𝑅
𝑓
=
𝑅
fillet
R
f
	​

=R
fillet
	​

，缘板径向长度 
𝑜
𝑓
𝑓
off，叶高 
𝐻
=
𝐵
𝐿
𝐴
𝐷
𝐸
_
𝐻
𝐸
𝐼
𝐺
𝐻
𝑇
H=BLADE_HEIGHT，中心角 
𝜃
𝑐
θ
c
	​

，叶身角宽 
Δ
𝜃
Δθ，缩进量 
𝑑
=
𝑑
inset
d=d
inset
	​

。外层曲面对应 
𝛿
=
0
δ=0，内层缩进曲面对应 
𝛿
=
𝑑
δ=d，并引入连续缩进参数 
𝛿
∈
[
0
,
𝑑
]
δ∈[0,d]。每一层缩进曲面的参数定义为：
𝑟
𝛿
=
𝑟
−
𝛿
r
δ
	​

=r−δ，
𝑅
𝛿
=
𝑅
𝑓
+
𝛿
R
δ
	​

=R
f
	​

+δ，
𝑧
endwall
,
𝛿
=
−
𝛿
z
endwall,δ
	​

=−δ，
𝐶
𝛿
=
𝑟
𝛿
Δ
𝜃
C
δ
	​

=r
δ
	​

Δθ。这样可以保证内外两层叶身的角度范围一致，因为 
𝐶
𝛿
/
(
2
𝑟
𝛿
)
=
Δ
𝜃
/
2
C
δ
	​

/(2r
δ
	​

)=Δθ/2。主曲面分为三段：叶身段、倒圆段、缘板段。叶身段类似圆柱面，
𝜌
=
𝑟
𝛿
ρ=r
δ
	​

，
𝑧
∈
[
𝑅
𝑓
,
𝐻
]
z∈[R
f
	​

,H]，角度范围为 
𝜃
∈
[
𝜃
𝑐
−
Δ
𝜃
/
2
,
𝜃
𝑐
+
Δ
𝜃
/
2
]
θ∈[θ
c
	​

−Δθ/2,θ
c
	​

+Δθ/2]。倒圆段满足 
𝜌
∈
[
𝑟
𝛿
,
𝑟
𝛿
+
𝑅
𝛿
]
ρ∈[r
δ
	​

,r
δ
	​

+R
δ
	​

]，高度为 
𝑧
=
𝑧
endwall
,
𝛿
+
𝑅
𝛿
−
𝑅
𝛿
2
−
(
𝑟
𝛿
+
𝑅
𝛿
−
𝜌
)
2
z=z
endwall,δ
	​

+R
δ
	​

−
R
δ
2
	​

−(r
δ
	​

+R
δ
	​

−ρ)
2
	​

，左右双曲螺线边界角为 
𝜃
±
(
𝜌
,
𝛿
)
=
𝜃
𝑐
±
𝐶
𝛿
/
(
2
𝜌
)
θ
±
	​

(ρ,δ)=θ
c
	​

±C
δ
	​

/(2ρ)。缘板段满足 
𝜌
∈
[
𝑟
𝛿
+
𝑅
𝛿
,
𝑟
𝛿
+
𝑅
𝛿
+
𝑜
𝑓
𝑓
]
ρ∈[r
δ
	​

+R
δ
	​

,r
δ
	​

+R
δ
	​

+off]，由于 
𝑟
𝛿
+
𝑅
𝛿
=
(
𝑟
−
𝛿
)
+
(
𝑅
𝑓
+
𝛿
)
=
𝑟
+
𝑅
𝑓
r
δ
	​

+R
δ
	​

=(r−δ)+(R
f
	​

+δ)=r+R
f
	​

，所以缘板起始半径对所有缩进层一致；缘板高度为 
𝑧
=
−
𝛿
z=−δ，边界角仍为 
𝜃
±
(
𝜌
,
𝛿
)
=
𝜃
𝑐
±
𝐶
𝛿
/
(
2
𝜌
)
θ
±
	​

(ρ,δ)=θ
c
	​

±C
δ
	​

/(2ρ)。代码中外层和内层主曲面分别由 blade_surface_smooth()、fillet_surface_smooth()、endwall_surface_smooth() 生成，每个函数都先生成一组横向圆弧截面，再 loft 成曲面。

为了把 
𝛿
=
0
δ=0 和 
𝛿
=
𝑑
δ=d 两组孤立曲面从侧面连起来，我们把缩进量 
𝛿
δ 当成连续参数。离散角度看，是在 
𝛿
=
0
δ=0 到 
𝛿
=
𝑑
δ=d 之间插入无数层很小缩进量的中间曲线；连续角度看，这些中间边界曲线族扫掠出侧面。左右侧面各由三张 patch 组成：叶身侧面、倒圆侧面、缘板侧面。叶身侧面为

𝑆
±
𝑏
𝑙
𝑎
𝑑
𝑒
(
𝑧
,
𝛿
)
=
(
(
𝑟
−
𝛿
)
cos
⁡
(
𝜃
𝑐
±
Δ
𝜃
/
2
)
,
(
𝑟
−
𝛿
)
sin
⁡
(
𝜃
𝑐
±
Δ
𝜃
/
2
)
,
𝑧
)
S
±
blade
	​

(z,δ)=((r−δ)cos(θ
c
	​

±Δθ/2),(r−δ)sin(θ
c
	​

±Δθ/2),z)

其中 
𝑧
∈
[
𝑅
𝑓
,
𝐻
]
z∈[R
f
	​

,H]，
𝛿
∈
[
0
,
𝑑
]
δ∈[0,d]。倒圆侧面引入 
𝑡
∈
[
0
,
1
]
t∈[0,1]，有

𝜌
(
𝑡
,
𝛿
)
=
𝑟
+
𝑅
𝑓
𝑡
−
𝛿
(
1
−
𝑡
)
ρ(t,δ)=r+R
f
	​

t−δ(1−t)
𝑧
(
𝑡
,
𝛿
)
=
𝑅
𝑓
−
(
𝑅
𝑓
+
𝛿
)
2
𝑡
−
𝑡
2
z(t,δ)=R
f
	​

−(R
f
	​

+δ)
2t−t
2
	​

𝜃
±
(
𝑡
,
𝛿
)
=
𝜃
𝑐
±
(
𝑟
−
𝛿
)
Δ
𝜃
2
𝜌
(
𝑡
,
𝛿
)
θ
±
	​

(t,δ)=θ
c
	​

±
2ρ(t,δ)
(r−δ)Δθ
	​


于是

𝑆
±
𝑓
𝑖
𝑙
𝑙
𝑒
𝑡
(
𝑡
,
𝛿
)
=
(
𝜌
(
𝑡
,
𝛿
)
cos
⁡
𝜃
±
(
𝑡
,
𝛿
)
,
𝜌
(
𝑡
,
𝛿
)
sin
⁡
𝜃
±
(
𝑡
,
𝛿
)
,
𝑧
(
𝑡
,
𝛿
)
)
S
±
fillet
	​

(t,δ)=(ρ(t,δ)cosθ
±
	​

(t,δ),ρ(t,δ)sinθ
±
	​

(t,δ),z(t,δ))

缘板侧面引入 
𝑢
∈
[
0
,
1
]
u∈[0,1]，有

𝜌
(
𝑢
)
=
𝑟
+
𝑅
𝑓
+
𝑜
𝑓
𝑓
⋅
𝑢
ρ(u)=r+R
f
	​

+off⋅u
𝑧
=
−
𝛿
z=−δ
𝜃
±
(
𝑢
,
𝛿
)
=
𝜃
𝑐
±
(
𝑟
−
𝛿
)
Δ
𝜃
2
𝜌
(
𝑢
)
θ
±
	​

(u,δ)=θ
c
	​

±
2ρ(u)
(r−δ)Δθ
	​


于是

𝑆
±
𝑒
𝑛
𝑑
𝑤
𝑎
𝑙
𝑙
(
𝑢
,
𝛿
)
=
(
𝜌
(
𝑢
)
cos
⁡
𝜃
±
(
𝑢
,
𝛿
)
,
𝜌
(
𝑢
)
sin
⁡
𝜃
±
(
𝑢
,
𝛿
)
,
−
𝛿
)
S
±
endwall
	​

(u,δ)=(ρ(u)cosθ
±
	​

(u,δ),ρ(u)sinθ
±
	​

(u,δ),−δ)

这个侧面不是原始曲面的严格法向偏置面，而是由我们定义的“设计缩进族”连续扫掠生成的连接面。代码实现中，用 blade_side_curve_at_delta(sign, delta)、fillet_side_curve_at_delta(sign, delta)、endwall_side_curve_at_delta(sign, delta) 在不同 
𝛿
δ 层生成侧边界曲线，再分别用 blade_side_surface(sign)、fillet_side_surface(sign)、endwall_side_surface(sign) loft 成侧壁面。这里 sign=-1 表示左侧，sign=+1 表示右侧。

为了形成封闭空间，仅有外层曲面、内层曲面和左右侧壁还不够，还必须补两个端盖：叶身顶部封闭面和缘板最外端封闭面。顶部封闭面 top_cap_surface() 用 
𝛿
∈
[
0
,
𝑑
]
δ∈[0,d] 的一族顶部圆弧 loft 得到，满足 
𝑧
=
𝐻
z=H，连接外层 blade 顶部圆弧和内层 blade 顶部圆弧。缘板外端封闭面 end_cap_surface() 用 
𝛿
∈
[
0
,
𝑑
]
δ∈[0,d] 的一族最外端圆弧 loft 得到，满足 
𝜌
=
𝑟
𝛿
+
𝑅
𝛿
+
𝑜
𝑓
𝑓
ρ=r
δ
	​

+R
δ
	​

+off，
𝑧
=
−
𝛿
z=−δ，连接 outer endwall 最外端和 inner endwall 最外端。这样完整边界由：outer 三段主曲面、inner 三段主曲面、左右两侧各三段侧壁、top cap、end cap 共同组成。几何上这是一组闭合曲面，但要成为真正 CAD 实体，还需要进行 sewing 和 solid 构造。

最初实体化尝试是把所有曲面的 Face 收集起来，用 BRepBuilderAPI_Sewing 缝合成 shell，再用 BRepBuilderAPI_MakeSolid 从 shell 生成 solid，并用 BRepCheck_Analyzer 检查有效性。早期版本大致流程为：Face -> Sewing -> Shell -> BRepBuilderAPI_MakeSolid -> BRepCheck_Analyzer。代码中曾写过 make_solid_from_surfaces(surface_shapes, sewing_tolerance=1e-5)，其内部收集所有 face，执行 sewing，提取 shell，然后用 BRepBuilderAPI_MakeSolid().Add(shell) 生成 solid。如果 BRepCheck_Analyzer(solid.wrapped).IsValid() 为 False，就抛出错误。实际运行时报错：已经生成 Solid，但 BRepCheck_Analyzer 判断实体无效。这说明 sewing 至少产生了 shell，MakeSolid 也返回了 solid，但拓扑或方向不满足有效实体要求。可能原因包括：曲面方向不一致、inner 三张主曲面按 outer 同向参与成壳、相邻曲面边界不完全共边、局部缝隙、重复边、多重边或局部自交。

后来做了一个重要判断：这个问题更像是几何边界“不共边”或 shell 面方向不一致导致 sewing 后拓扑壳无效，而不是单纯 tolerance 不够。特别是 inner 三张主曲面仍按 outer 同向参与成壳，BRepBuilderAPI_MakeSolid 并不会自动保证所有壳面法向构成有限体积。OCCT 的思路中，BRepBuilderAPI_MakeSolid 只是从 shell 构造 solid，本身不负责完整检查和修复拓扑一致性；ShapeFix_Solid 才更适合用于从 shell 构造并调整为有限有效实体；ShapeFix_Shell / ShapeFix_Solid 负责修正 shell 或 solid 的方向和有效性。因此最终修改重点不再是盲目把 sewing_tolerance 从 1e-5 放大到 1e-3，而是改造 sewing 到 solid 的流程：输出 free edge / multiple edge 诊断；如果 sewing 后有多个 shell，不再只取第一个 shell；使用 ShapeFix_Solid.SolidFromShell(shell) 修复壳面方向并生成有限体积 solid；自动尝试 1e-5, 1e-4, 1e-3 三档 tolerance；保留失败原因，便于判断是边界没有缝上还是方向/自交问题。

最终实体化相关 import 修改为：

from OCP.BRepBuilderAPI import BRepBuilderAPI_Sewing, BRepBuilderAPI_MakeSolid
from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.ShapeFix import ShapeFix_Solid
from OCP.TopAbs import TopAbs_SHELL
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS

新增/替换的实体生成函数如下：

def extract_shells(shape):
    """
    从 sewing 结果中提取所有 Shell。

    注意：
        Sewing 后的结果不一定直接就是 TopAbs_SHELL，
        也可能是 Compound，里面包含一个或多个 Shell。
    """
    shells = []

    if shape.ShapeType() == TopAbs_SHELL:
        shells.append(TopoDS.Shell_s(shape))
        return shells

    explorer = TopExp_Explorer(shape, TopAbs_SHELL)

    while explorer.More():
        shells.append(TopoDS.Shell_s(explorer.Current()))
        explorer.Next()

    return shells


def make_solid_from_one_shell(shell):
    """
    从单个 shell 生成 solid。

    优先使用 ShapeFix_Solid.SolidFromShell：
        - 能调用 MakeSolid；
        - 能尝试调整 shell / face 方向；
        - 目标是得到有限体积 solid。
    """
    fixer = ShapeFix_Solid()
    solid_occ = fixer.SolidFromShell(shell)

    solid = cq.Shape.cast(solid_occ)
    analyzer = BRepCheck_Analyzer(solid.wrapped)

    if analyzer.IsValid():
        return solid

    # 兜底：直接 MakeSolid，再检查。
    solid_maker = BRepBuilderAPI_MakeSolid(shell)
    solid_occ = solid_maker.Solid()
    solid = cq.Shape.cast(solid_occ)

    analyzer = BRepCheck_Analyzer(solid.wrapped)

    if analyzer.IsValid():
        return solid

    return None


def make_solid_from_surfaces_once(surface_shapes, sewing_tolerance=1e-5):
    """
    单次 sewing + solid 构造。
    """
    faces = collect_faces_from_shapes(surface_shapes)

    sewing = BRepBuilderAPI_Sewing(sewing_tolerance)

    # 这些模式对由 loft / spline 面组成的边界更稳。
    sewing.SetFaceMode(True)
    sewing.SetSameParameterMode(True)
    sewing.SetLocalTolerancesMode(True)

    # 避免局部边容差无限放大。
    sewing.SetMinTolerance(sewing_tolerance * 0.1)
    sewing.SetMaxTolerance(sewing_tolerance * 10.0)

    # 这里不建议开启 NonManifoldMode。
    # 目标是封闭实体，理论上每条边应最多被两个面共享。
    sewing.SetNonManifoldMode(False)

    for face in faces:
        sewing.Add(face.wrapped)

    sewing.Perform()

    sewed_shape = sewing.SewedShape()
    shells = extract_shells(sewed_shape)

    print(
        f"\n[SEWING] tol={sewing_tolerance:g}, "
        f"faces={len(faces)}, "
        f"shells={len(shells)}, "
        f"free_edges={sewing.NbFreeEdges()}, "
        f"multiple_edges={sewing.NbMultipleEdges()}, "
        f"contiguous_edges={sewing.NbContigousEdges()}"
    )

    if len(shells) == 0:
        raise RuntimeError(
            f"Sewing 后没有得到 Shell。tol={sewing_tolerance:g}, "
            f"free_edges={sewing.NbFreeEdges()}, "
            f"multiple_edges={sewing.NbMultipleEdges()}"
        )

    if len(shells) > 1:
        raise RuntimeError(
            f"Sewing 后得到多个 Shell：{len(shells)}。这通常说明曲面没有缝成一个连通闭壳。"
            f"tol={sewing_tolerance:g}, "
            f"free_edges={sewing.NbFreeEdges()}, "
            f"multiple_edges={sewing.NbMultipleEdges()}"
        )

    shell = shells[0]
    solid = make_solid_from_one_shell(shell)

    if solid is not None:
        print(f"[SOLID] valid solid at sewing_tolerance={sewing_tolerance:g}")
        return solid

    raise RuntimeError(
        f"已经得到单个 Shell，但无法生成有效 Solid。tol={sewing_tolerance:g}, "
        f"free_edges={sewing.NbFreeEdges()}, "
        f"multiple_edges={sewing.NbMultipleEdges()}。"
        "如果 free_edges > 0，优先检查边界不共边；"
        "如果 free_edges = 0 但 solid 无效，优先检查面方向或自交。"
    )


def make_solid_from_surfaces(surface_shapes, sewing_tolerances=(1e-5, 1e-4, 1e-3)):
    """
    多容差尝试生成 solid。

    不建议一开始就用 1e-3。
    先从 1e-5 开始，失败后逐步放大。
    """
    last_error = None

    for tol in sewing_tolerances:
        try:
            return make_solid_from_surfaces_once(
                surface_shapes,
                sewing_tolerance=tol
            )
        except Exception as exc:
            last_error = exc
            print(f"[SOLID WARNING] tol={tol:g} failed: {exc}")

    raise RuntimeError(
        "所有 sewing_tolerance 都失败，未能得到有效 Solid。\n"
        f"最后一次错误：{last_error}"
    )

调用方式从原来的：

closed_solid = make_solid_from_surfaces(
    all_surface_shapes,
    sewing_tolerance=1e-5
)

改为：

closed_solid = make_solid_from_surfaces(
    all_surface_shapes,
    sewing_tolerances=(1e-5, 1e-4, 1e-3)
)

此外，还同步修改了一个潜在不一致点。原来的 endwall_side_curve_at_delta() 写成：

rho_start = r_blade + R_fillet
rho_end = r_blade + R_fillet + off

在当前参数定义下数值上没错，因为：

r0 + R0 = (r_blade - delta) + (R_fillet + delta)
        = r_blade + R_fillet

但为了让代码逻辑完全依赖当前缩进层参数，并避免后续改参数时出错，建议改为：

def endwall_side_curve_at_delta(sign, delta):
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

同理，end_cap_surface() 也建议从固定 rho_end = r_blade + R_fillet + off 改成每层显式计算：

def end_cap_surface():
    sections = []

    for k in range(N_INSET_SECTIONS + 1):
        delta = d_inset * k / N_INSET_SECTIONS
        r0, R0, z_endwall, C = delta_params(delta)

        rho_end = r0 + R0 + off

        sections.append(
            arc_section(rho_end, z_endwall, C)
        )

    return loft_open_sections(sections, "end_cap_surface")

这不会改变当前几何结果，但能保证代码逻辑和连续缩进参数定义一致。

后续调试时，重点看输出中的 sewing 诊断行：

[SEWING] tol=..., faces=..., shells=..., free_edges=..., multiple_edges=..., contiguous_edges=...

判断规则如下：如果 free_edges > 0，说明有边没缝上，此时主要问题不是实体方向，而是相邻曲面边界几何不够重合，应优先检查 blade/fillet 交界、fillet/endwall 交界、top_cap/blade 交界、end_cap/endwall 交界，以及 side_a、side_b 与 outer/inner 主曲面边界是否真正共边。如果 free_edges = 0，但 BRepCheck_Analyzer 仍然判断 solid 无效，则更可能是面方向、局部自交、退化边或多重边问题，此时 ShapeFix_Solid.SolidFromShell(shell) 通常比直接 BRepBuilderAPI_MakeSolid().Add(shell) 更合适。如果 multiple_edges > 0，说明某些边被超过两个面共享，通常来自重复面、端盖和侧壁重复覆盖，或者某些 patch 在边界上重叠。

这一轮修改解决的核心问题是：原先直接使用

solid_maker = BRepBuilderAPI_MakeSolid()
solid_maker.Add(shell)
solid_occ = solid_maker.Solid()

只是在拓扑上把 shell 放进 solid，并不保证 shell 的 face 方向一致，也不保证最终是有限体积的有效实体。更稳妥的流程应当是：

收集所有 Face
→ BRepBuilderAPI_Sewing
→ 输出 free edge / multiple edge 诊断
→ 提取 exactly one Shell
→ ShapeFix_Solid.SolidFromShell(shell)
→ BRepCheck_Analyzer 检查
→ 若失败，再尝试更大 sewing_tolerance

当前最终目标是：Assembly 中保留 closed_solid、outer/inner 主曲面、side/cap 曲面和边界检查曲线；实体本身由所有封闭曲面 sewing 后通过 ShapeFix_Solid.SolidFromShell(shell) 生成。这样既可以检查构造过程中的曲线和曲面，也可以得到可用于后续 CAD 操作的实体。