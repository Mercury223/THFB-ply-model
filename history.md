# 工作记录 — THFB 铺层划分修复与可视化

**日期**：2026-05-07 (初版), 2026-05-09 (更新)  
**Python 环境**：`D:\anaconda3\envs\env_cad` (conda env)  
**工作目录**：`G:\study\project\THFB\20260507QY`

---

## 1. 问题背景

`QYModel.py` 实现了简化叶身 + 缘板建模，以及外表面铺层划分。铺层划分存在两个问题：

### 1.1 铺层带不垂直于直线段

**根因**：`build_ply_reference_curves` 中用 `percent = s / blade_total` 将叶身弧长参数线性映射到扩展曲线。但直线段偏移后长度不变，圆弧段从半径 R 变为 R+offset（弧长大 2.88 倍），百分比映射导致直线段站点被映射到扩展曲线圆弧段上。

**修复**：新增 `_map_blade_s_to_expanded_length` 方法，分段感知映射：
- 下/上直线段：长度不变，直接映射
- 圆弧段：按角度比例映射，弧长缩放 = `(R+offset)/R`

### 1.2 倒圆段方向错误

**根因**：`_make_surface_connector_wire` 中倒圆四分之一圆使用 `radial = r*sin(θ), z = r*cos(θ)`，形成外凸曲线。

**修复**：改为 `radial = r*(1-cos(θ)), z = r*(1-sin(θ))`，形成内凹曲线。

### 1.3 铺层带实体加厚边界错误 (2026-05-09)

**根因 (第一次修复)**：`build_ply_band_solids` 中 endwall 面的左右边界使用 blade 曲线端点直接连到 expanded 曲线端点（单一直线段），未经过倒圆末端点 `endwall_start`。

**修复**：endwall 面的左右边界改为 blade_curve → endwall_start → expanded_curve 两段式路径。

**根因 (第二次修复 — 倒圆段缺失)**：仅 blade + endwall 两个直片融合，缺少倒圆 (fillet) 区域的过渡段。每个 band 实体应有 blade + fillet + endwall 三段结构。

**修复**：新增 fillet 段实体：
- Arc bands：四分之一圆环截面绕 Z 轴回转 (`_make_fillet_solid_arc`)
- Straight bands：沿直线方向拉伸 (`_make_fillet_solid_straight`)
- Blade 段 Z 范围从 [0, blade_height] 改为 [r, blade_height]，腾出空间给 fillet 段

---

## 2. 代码修改清单

### QYModel.py 修改

| 位置 | 变更 | 说明 |
|------|------|------|
| Params 类 | 新增 `ply_layer_thickness: float = 1.0` | 铺层带可视化厚度参数 |
| KEEP_FILES | 新增 `"visualize_ply.py"`, `"history.md"` | 防止清理时误删 |
| `_make_surface_connector_wire` | 修改倒圆公式 | 外凸 → 内凹 |
| `_connector_polyline_pts` | **新增方法** | 返回翻折路径有序点列 |
| `_make_surface_connector_wire` | 重构为调用新方法 | 代码复用 |
| `_map_blade_s_to_expanded_length` | **新增方法** | 分段感知弧长映射 |
| `build_ply_reference_curves` | 2 处替换 | `percent → segment-aware mapping` |
| `build_ply_band_solids` | 重写 | 三段式 (blade+fillet+endwall) 融合 |
| `_band_arc_angles_deg` | **新增方法** | band 弧长→角度转换 |
| `_make_fillet_solid_arc` | **新增方法** | 四分之一圆环回转生成倒圆实体 |
| `_make_fillet_solid_straight` | **新增方法** | 直线段倒圆实体 |
| `build_solids` | 新增 `ply_band_solids` 调用 | 集成铺层带实体 |
| `export_step` | 新增 band solids 导出 | 每个 band 不同色相着色 |
| `_hsv_to_rgb` | **新增方法** | 色相环颜色生成 |
| `main()` | 新增 `ply_layer_thickness=1.0` | 默认参数 |

### 可视化脚本

| 文件 | 说明 |
|------|------|
| `visualize_ply.py` | 2D 顶视图（含铺层带填充 + 三个局部放大）+ 3D 视图（含 tessellated 实体） |

---

## 3. 关键参数

```python
blade_height=80.0          # 叶身高度 (Z+)
endwall_height=20.0        # 缘板厚度 (Z-)
outer_radius=33.0          # 外侧前缘圆半径 R
blade_thickness=4.0        # 叶身壁厚
upper_tangent_angle_deg=130.0
blade_upper_length=32.0    # 上直线段长
blade_lower_length=36.0    # 下直线段长
lower_drop=20.0, lower_run=150.0
upper_drop=25.0, upper_run=160.0
root_fillet_radius=2.0     # 根部倒角半径
root_fillet_curve="outer"

ply_tangent_angle_step_deg=5.0   # 铺层分段角度步长
ply_expand_offset=60.0           # 扩展偏置距离
ply_expand_samples=1200
ply_fillet_samples=12
ply_layer_thickness=1.0          # 铺层带可视化厚度
```

## 4. 几何概要

```
叶身外侧曲线总长: 194.71 mm
  - 下直线段: 36.0 mm (s: 0 → 36)
  - 圆弧段:   126.7 mm (s: 36 → 162.7, R=33)
  - 上直线段: 32.0 mm (s: 162.7 → 194.7)

扩展曲线总长: 425.09 mm
  - 下直线段: 36.0 mm (同长)
  - 圆弧段:   357.1 mm (R=93, 同角度)
  - 上直线段: 32.0 mm (同长)

站点数: 46
铺层带数: 45
```

## 5. Python 曲面创建与加厚原理

### 曲线围成曲面的方法

在 CadQuery (基于 OCCT) 中，从曲线创建曲面有几种方式：

1. **MakeRuledSurface** (`cq.Face.makeRuledSurface(edge1, edge2)`)  
   在两个边之间生成直纹面（ruled surface）。每个对应的参数点用直线连接。

2. **MakeNSidedSurface / BRepFill_Filling** (`cq.Face.makeNSidedSurface(edges, points)`)  
   从 N 条边界边填充曲面。边界必须形成闭合回路。内部可添加约束点。

3. **MakeFromWires** (`cq.Face.makeFromWires(wire)`)  
   从共面的闭合 Wire 创建平面面片。仅适用于平面 Wire。

4. **Loft** (`cq.Solid.makeLoft(wires)`)  
   在多个截面 Wire 之间放样生成实体或面。

5. **BRepOffsetAPI_MakeFilling**  
   与 makeNSidedSurface 相同的底层算法，但允许更灵活的参数控制（容差、阶数等）。

### 曲面加厚为实体的方法

1. **BRepOffset_MakeOffset (Skin 模式)**:  
   `BRepOffsetAPI_MakeOffsetShape.PerformByJoin(face, thickness, ...)`  
   将面沿法向偏移，生成偏移面 + 侧面，缝合成壳 → 实体。

2. **Extrude (固定方向拉伸)**:  
   `cq.Workplane("XY").add(face).extrude(height)`  
   仅适用于平面面片沿固定方向拉伸。对于非平面曲面，拉伸方向不一致。

3. **BRepPrimAPI_MakePrism**:  
   沿任意方向向量拉伸面为实体。方向固定，不能随曲面法向变化。

### 本项目的实现选择

本项目采用 **分层构建策略**：
- **Blade 层**：在 XY 平面创建外表面-内偏置之间的 2D 截面，沿 Z 向拉伸
- **Endwall 层**：在 XY 平面创建 blade 曲线 → 倒圆末端 → 扩展曲线之间的 2D 截面，沿 -Z 拉伸
- 两层在 z=0 处融合（fuse）

这种方法避免了直接在 3D 中填充非平面曲面和曲面法向加厚的复杂性，
同时保证了实体几何的有效性（CadQuery is Valid 验证通过）。

## 6. 验证结果

- **连接线-法向夹角**：直线段上 0.00e+00°（完美垂直）
- **倒圆方向**：内凹（中点从 1.41r 修正到 0.59r）
- **铺层带实体**：45/45 有效（CadQuery is Valid 验证通过）

## 7. 输出文件

| 文件 | 说明 |
|------|------|
| `QYModel.py` | 主建模脚本（已修复） |
| `visualize_ply.py` | 可视化脚本 |
| `vane_blade_three_parts_negative_endwall_with_ply_curves.step` | STEP 导出（UG 可导入） |
| `ply_division_2d_topview.png` | 2D 顶视图 + 局部放大 |
| `ply_division_3d_view.png` | 3D 立体视图 |
| `history.md` | 本文件 |

## 8. 运行方式

```bash
# 完整运行（STEP + 可视化）
D:/anaconda3/envs/env_cad/python.exe G:/study/project/THFB/20260507QY/visualize_ply.py

# 仅生成 STEP
D:/anaconda3/envs/env_cad/python.exe -c "
from QYModel import Params, VaneBladeAndEndwallBuilder
# ... (see main() in QYModel.py)
"
```
