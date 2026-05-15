Bézier 曲线分段生成曲线、曲面和实体的代码迭代说明
1. 当前任务背景

我正在使用 Python + CadQuery + OCP 构造一类由 Bézier 曲线驱动生成的几何结构。

原始代码的核心逻辑是：

定义一条三次 Bézier 曲线作为基础曲线；

沿该曲线生成 blade 竖直曲面；

在曲线法向方向生成 fillet 曲面；

继续向外生成 endwall 曲面；

通过内外两组偏置曲面以及侧边封口面，sewing 成 shell；

再尝试生成 solid；

最后导出 STEP 文件，用于 CAD 中查看曲线、曲面和实体。

原本代码只使用整条 Bézier 曲线生成一套完整结构。

后续需求是：不仅要生成完整结构，还要使用 Bézier 曲线的局部弧长段生成对应的局部结构，以便比较完整体和局部体之间的差异。

2. 第一轮修改：使用中间一半 Bézier 曲线生成结构

第一轮需求是：

当前结构是使用整条 Bézier 曲线生成的，现在想用中间一半的 Bézier 曲线再生成一组相关的曲线、曲面和实体，这样可以看到中间的体和完整的体的差别。

这里的“中间一半”没有直接使用参数区间：

t = 0.25 ~ 0.75

而是按弧长比例截取：

s = 0.25L ~ 0.75L

其中：

L = 原始 Bézier 曲线总弧长

这样做的原因是 Bézier 曲线参数 t 通常不是等弧长分布的。
如果直接使用 t=0.25~0.75，得到的曲线段不一定是真正几何意义上的中间一半。

因此第一轮新增了这些功能：

sampled_arclength_table(edge)
curve_length(edge)
parameter_at_arclength(ts, ss, target_s)
subedge_by_arclength(edge, s0, s1)
make_center_ratio_curve(edge, ratio)

其中：

make_center_ratio_curve(original_bezier_curve, ratio=0.5)

表示按弧长截取原始曲线中间 50%。

即：

s0 = 0.5 * L * (1 - 0.5) = 0.25L
s1 = 0.5 * L * (1 + 0.5) = 0.75L

生成了两套结构：

full
middle_half

分别对应：

full_closed_solid
middle_half_closed_solid

导出的 STEP 文件为：

full_body_and_middle_half_body_comparison.step
3. 第二轮修改：不再使用 25%–75%，改为 25%–50% 和 50%–75%

第二轮需求是：

修改代码，改成使用 25%–50%，和 50%–75% 的曲线生成结构，以及使用完整曲线生成结构。不用 25%–75% 了。

因此删除了原来 middle_half 的逻辑，不再生成：

25%–75%

而是改为生成三套结构：

full
quarter_25_50
quarter_50_75

分别表示：

full
使用完整 Bézier 曲线：

s = 0 ~ L

quarter_25_50
使用原始 Bézier 曲线弧长 25%–50%：

s = 0.25L ~ 0.50L

quarter_50_75
使用原始 Bézier 曲线弧长 50%–75%：

s = 0.50L ~ 0.75L

对应新增/替换的函数是：

make_curve_by_arclength_fraction(
    edge,
    start_fraction,
    end_fraction,
    n_piece=N_SECTION_SAMPLE
)

该函数的作用是按照弧长比例截取任意曲线段。

例如：

curve_25_50_info = make_curve_by_arclength_fraction(
    original_bezier_curve,
    start_fraction=0.25,
    end_fraction=0.50,
    n_piece=N_SECTION_SAMPLE
)

以及：

curve_50_75_info = make_curve_by_arclength_fraction(
    original_bezier_curve,
    start_fraction=0.50,
    end_fraction=0.75,
    n_piece=N_SECTION_SAMPLE
)
4. 当前代码的主要几何逻辑
4.1 原始 Bézier 曲线

当前示例 Bézier 曲线由 4 个控制点定义：

p0 = v3(-55.0, 0.0, z_value)
p1 = v3(-25.0, 12.0, z_value)
p2 = v3(25.0, 14.0, z_value)
p3 = v3(58.0, 1.5, z_value)

通过 OCP 的：

Geom_BezierCurve
BRepBuilderAPI_MakeEdge

生成 CadQuery Edge。

4.2 弧长表

因为 Bézier 参数不是等弧长分布，所以通过离散采样建立弧长表：

sampled_arclength_table(edge, n_sample=N_ARCLEN_SAMPLE)

该函数返回：

ts
ss

其中：

ts[i] = 归一化参数 t
ss[i] = 从曲线起点到 t 的累计弧长

总弧长为：

ss[-1]
4.3 弧长反求参数

给定目标弧长：

target_s

用二分法在弧长表中找到对应的参数：

parameter_at_arclength(ts, ss, target_s)

这一步的作用是把：

s = 0.25L
s = 0.50L
s = 0.75L

转换成近似参数：

t0
t1
4.4 截取局部曲线

截取方式是：

subedge_by_arclength(edge, s0, s1)

内部流程：

根据 s0、s1 反求参数 t0、t1；

在 t0~t1 之间重新采样点；

用采样点生成 spline edge；

返回局部曲线 edge 和点列。

注意：这里生成的局部曲线不是严格的 Bézier 子曲线，而是通过采样点拟合得到的 spline。
这样更方便后续使用 CadQuery 继续做 offset、loft 和 solid。

5. 当前生成的三套模型

当前主流程中生成三条输入曲线：

original_bezier_curve
curve_25_50
curve_50_75

然后分别调用：

full_model = build_closed_model_from_origin_curve(
    origin_edge=original_bezier_curve,
    model_name="full"
)

quarter_25_50_model = build_closed_model_from_origin_curve(
    origin_edge=curve_25_50,
    model_name="quarter_25_50"
)

quarter_50_75_model = build_closed_model_from_origin_curve(
    origin_edge=curve_50_75,
    model_name="quarter_50_75"
)

也就是说，三套结构使用完全相同的曲面和实体生成流程，只是输入的 origin curve 不同。

6. 每套模型内部包含的结构

每套模型内部都会生成：

6.1 外组曲面

对应：

delta_value = 0.0

包括：

outer_group_blade_vertical_surface
outer_group_fillet_surface_equal_length
outer_group_endwall_surface_equal_length
6.2 内组曲面

对应：

delta_value = D_INSET

包括：

inner_group_blade_vertical_surface
inner_group_fillet_surface_equal_length
inner_group_endwall_surface_equal_length
6.3 连续 d 方向侧面

用于连接外组和内组。

包括：

continuous_d_blade_side_0
continuous_d_blade_side_1
continuous_d_fillet_side_0
continuous_d_fillet_side_1
continuous_d_endwall_side_0
continuous_d_endwall_side_1
6.4 封口面

包括：

continuous_d_blade_top_cap
continuous_d_endwall_outer_cap
6.5 最终实体

通过 sewing 后构造 solid：

closed_solid
7. STEP 文件导出结果

当前 STEP 文件名是：

full_body_25_50_body_50_75_body_comparison.step

导出的主要实体名称包括：

full_closed_solid
quarter_25_50_closed_solid
quarter_50_75_closed_solid

参考曲线包括：

reference_original_full_bezier_curve
reference_curve_25_50_arc_length
reference_curve_50_75_arc_length
8. 当前代码中最重要的新增函数
8.1 按弧长比例截取曲线
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
9. 当前主流程摘要

当前主流程可以概括为：

if __name__ == "__main__":

    original_bezier_curve = make_demo_bezier_base_curve_at_z(R_FILLET)

    curve_25_50_info = make_curve_by_arclength_fraction(
        original_bezier_curve,
        start_fraction=0.25,
        end_fraction=0.50,
        n_piece=N_SECTION_SAMPLE
    )

    curve_50_75_info = make_curve_by_arclength_fraction(
        original_bezier_curve,
        start_fraction=0.50,
        end_fraction=0.75,
        n_piece=N_SECTION_SAMPLE
    )

    curve_25_50 = curve_25_50_info["edge"]
    curve_50_75 = curve_50_75_info["edge"]

    full_model = build_closed_model_from_origin_curve(
        origin_edge=original_bezier_curve,
        model_name="full"
    )

    quarter_25_50_model = build_closed_model_from_origin_curve(
        origin_edge=curve_25_50,
        model_name="quarter_25_50"
    )

    quarter_50_75_model = build_closed_model_from_origin_curve(
        origin_edge=curve_50_75,
        model_name="quarter_50_75"
    )

    assy = cq.Assembly(
        name="FULL_BODY_25_50_BODY_50_75_BODY_COMPARISON"
    )

    add_model_to_assembly(assy, full_model)
    add_model_to_assembly(assy, quarter_25_50_model)
    add_model_to_assembly(assy, quarter_50_75_model)

    assy.export(
        "full_body_25_50_body_50_75_body_comparison.step"
    )
10. 需要注意的问题
10.1 当前截取是按弧长比例，不是按 Bézier 参数

也就是说：

25%–50%

指的是曲线几何弧长上的 25%–50%，不是参数 t=0.25~0.50。

这是有意设计的，因为弧长比例更符合几何直觉。

10.2 局部曲线是 spline，不是精确 Bézier 子曲线

目前局部曲线通过采样点重新拟合为 spline：

make_spline_edge(pts)

这对于建模和可视化通常足够稳定，但如果后续需要严格数学意义上的 Bézier 子曲线，需要使用 de Casteljau 算法进一步拆分原始 Bézier。

10.3 局部体之间不一定能简单拼成完整体

quarter_25_50 和 quarter_50_75 是分别独立生成的实体。

它们的体积之和不一定严格等于完整体某一段体积，原因包括：

每段局部曲线独立生成 offset；

fillet 和 endwall 的等弧长截取是针对每段局部曲线重新居中处理；

局部段两端会各自形成独立侧边封口；

spline 重采样会带来微小几何误差。

因此目前它们主要用于对比局部几何结构，而不是严格做体积分割积分。

11. 后续可能继续修改的方向

接下来可能继续做这些修改：

让 25%–50% 和 50%–75% 两段共享边界，减少中间断面误差；

增加每段的起止截面封口，使局部体更明确；

输出每个局部段的起止截面曲线；

用 de Casteljau 算法生成严格 Bézier 子曲线；

把曲线分段参数改成列表，例如：

SEGMENTS = [
    ("quarter_25_50", 0.25, 0.50),
    ("quarter_50_75", 0.50, 0.75),
]

这样以后可以很方便地扩展成：

0%–25%
25%–50%
50%–75%
75%–100%

让 STEP 中不同模型自动分层、自动命名、自动配色。

12. 给新对话的请求可以这样写

请基于下面这个背景继续修改代码：

我有一段 Python + CadQuery + OCP 代码，用三次 Bézier 曲线生成 blade、fillet、endwall 曲面，并通过内外偏置曲面、侧面和封口面 sewing 成 solid。

目前代码已经从原始完整 Bézier 曲线中按弧长比例截取了两段曲线：25%–50% 和 50%–75%，并且分别生成了 quarter_25_50 和 quarter_50_75 两个局部体，同时也保留了 full 完整体。

注意这里的 25%、50%、75% 都是按曲线弧长比例，不是按 Bézier 参数 t。

当前导出 STEP 文件名是 full_body_25_50_body_50_75_body_comparison.step，主要实体名称是：

full_closed_solid

quarter_25_50_closed_solid

quarter_50_75_closed_solid

请在这个基础上继续帮我修改代码。

后续我可能要做的修改包括：让两个局部体共享 50% 截面、增加端面封口、输出分段截面曲线、或者把分段逻辑改成可配置列表。