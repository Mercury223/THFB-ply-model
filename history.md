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

### 1.3 铺层带实体加厚 — 三次迭代修复 (2026-05-09)

**第一次修复**：endwall 面边界改为两段式路径 (blade → endwall_start → expanded)，与 side_a/side_b 一致。

**第二次修复 — 倒圆段缺失**：发现 band 实体缺少 fillet 段（z=0~r）。新增 fillet 段：
- Arc bands: 四分之一圆环截面绕 Z 轴回转
- 使用 OCP `BRepPrimAPI_MakeRevol`（CadQuery 的 `Workplane.revolve` 有 `Standard_OutOfRange` bug）

**第三次修复 — 厚向与直段错误 (最终版)**：
- **厚向修正**: 内弧半径从 `r-t` 改为 `r+t`。外弧半径 r，内弧半径 r+t，同心（center=R+r, r）。内弧终点 (R-t, r) 和 (R+r, -t)，向叶片内侧/缘板下方加厚，与 blade/endwall 厚向一致
- **Straight bands**: 截面放 band 起点 s0，沿 tangent 方向 Prism 拉伸 seg_len
- **Transition bands**: 在 arc 边界处拆分为 straight + arc 子段，分别生成后 fuse
- **自检验证**: 新增 `_validate_band_solids`，对每个 band 在 blade / fillet / endwall 三区域采样检测点是否在体内

**CadQuery 可用曲面+加厚 API (已确认)**:
- `Face.makeNSidedSurface(edges, points)` — N 边填充曲面
- `Face.makeRuledSurface(e1, e2)` — 直纹面
- `Face.thicken(thickness)` — 曲面加厚为实体
  - 但 `makeNSidedSurface` 对复杂 3D 边界会失败 ("BRep_API: command not done")，因独立 `makeLine` 边不共享顶点
  - `Face.thicken` 需要封闭壳输入，单面加厚返回非实体

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
| `build_ply_band_solids` | 重写 | 三段式 (blade+fillet+endwall) 融合, transition band 拆分 |
| `_band_arc_angles_deg` | **新增方法** | band 弧长→角度转换 |
| `_make_fillet_solid_arc` | **新增方法** | OCP BRepPrimAPI_MakeRevol 回转, 内弧半径 r+t |
| `_make_fillet_solid_straight` | **新增方法** | BRepPrimAPI_MakePrism 沿切线拉伸 |
| `_make_fillet_solid_transition` | **新增方法** | 拆分 strad/arc 子段后 fuse |
| `_validate_band_solids` | **新增方法** | blade/fillet/endwall 三区域采样自检 |
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

采用 **三段分层构建 + 回转/拉伸** 策略：
- **Blade 层** (z=r..blade_height)：XY 平面 2D 截面沿 +Z 拉伸 (同旧方案)
- **Fillet 层** (z=0..r)：
  - Arc bands: 四分之一圆环截面 (XZ 面) 用 `BRepPrimAPI_MakeRevol` 绕 Z 回转
  - Straight bands: 同截面用 `BRepPrimAPI_MakePrism` 沿切线拉伸
  - Transition bands: 拆分 straight+arc 子段各自生成
  - 内弧半径 r+t > r，保证厚向与 blade/endwall 一致 (向叶片内侧/缘板下方)
- **Endwall 层** (z=-thickness..0)：XY 平面 2D 截面沿 -Z 拉伸，边界含 endwall_start 折点

Fillet 截面构建 (XZ 面):
```
外弧: center=(R+r, r), radius=r,  (R, r) → (R+r, 0)
内弧: center=(R+r, r), radius=r+t, (R+r, -t) → (R-t, r)
```
两弧同心，内弧半径更大（r+t > r），从而实现向 blade 内侧 / endwall 下方的偏置。

## 6. 验证结果

- **连接线-法向夹角**：直线段上 0.00e+00°（完美垂直）
- **倒圆方向**：内凹（中点从 1.41r 修正到 0.59r）
- **铺层带实体**：45/45 有效（CadQuery is Valid 验证通过）
- **自检 (blade+fillet+endwall 三区采样)**：VALIDATION PASSED — all 45 bands OK

## 7. GitHub 仓库

**仓库地址**: https://github.com/Mercury223/THFB-ply-model

### 推送经验

- **认证**: `gh auth login --with-token` (从 stdin 传入 token)
- **推送**: 直接 `git push origin master` 即可，不需要任何 proxy 或特殊配置
- **失败模式**: 不要用 `git config http.proxy` 设置代理 — 反而会导致连接失败
- VPN 连接后直接推送，浏览器能打开 GitHub 就说明 git 也能推
- 曾误判为网络问题反复尝试 proxy，实际是 push 已完成返回 "Everything up-to-date" 但被误读为超时

### Commit 历史
```
abec853 docs: update history.md with fillet solid implementation details
085f94f fix: use OCP BRepPrimAPI_MakeRevol for fillet solid generation
81df6a6 feat: add fillet solid to ply band construction via revolve
0ead9e0 fix: ply band solids side boundaries now follow surface connector path
```

## 8. 输出文件

| 文件 | 说明 |
|------|------|
| `QYModel.py` | 主建模脚本（已修复） |
| `visualize_ply.py` | 可视化脚本 |
| `vane_blade_three_parts_negative_endwall_with_ply_curves.step` | STEP 导出（UG 可导入） |
| `ply_division_2d_topview.png` | 2D 顶视图 + 局部放大 |
| `ply_division_3d_view.png` | 3D 立体视图 |
| `history.md` | 本文件 |

## 9. 运行方式

```bash
# 完整运行（STEP + 可视化）
D:/anaconda3/envs/env_cad/python.exe G:/study/project/THFB/20260507QY/visualize_ply.py

# 仅生成 STEP
D:/anaconda3/envs/env_cad/python.exe -c "
from QYModel import Params, VaneBladeAndEndwallBuilder
# ... (see main() in QYModel.py)
"
```

---

## 10. Band Solid 重构 (2026-05-11)

### 问题

1. **叶身被错误分割为 band 片段**：每个 band 的 blade 段（z=r..blade_height）各自独立，实际叶身应为连续整体
2. **分割位置错误**：band 从 z=0（倒圆-缘板交界）分开，应在 z=r（倒圆-叶身交界）分开
3. **侧面未沿 side_a/side_b**：endwall 面为环形扇区（内外同角度），应为梯形（内宽外窄），使弧长宽度基本一致

### 修复

**build_ply_band_solids 重写**：
- 移除 blade sub-solid 段（叶身不再分配到各 band）
- Band 实体 = fillet solid (z=0..r) + endwall solid (z=-thickness..0)
- 内边界（倒圆底部，R+r）使用叶身角度（分母=R，宽）
- 外边界（扩展曲线，R+offset）使用扩展角度（分母=R+offset，窄）
- endwall 面变为梯形：内弧（叶身角度）→外弧（扩展角度），侧边为直线连接
- 过渡 band：弧段内外角同（因过渡点固定），直线段并行

**_validate_band_solids 更新**：
- 移除 blade region 检测点（z > r 区域 band 不再包含）
- 仅检测 fillet + endwall 区域

**分段判定容差**：恢复为 1e-9（从错误修改的 0.1）

### 验证结果

- **12 bands (20°步长)**：12/12 生成，VALIDATION PASSED (fillet + endwall)
- **45 bands (5°步长)**：45/45 生成，VALIDATION PASSED (fillet + endwall)
- 叶身为连续整体（final_fillet_body），band 仅包裹在倒圆+缘板外侧

### Commit

```
fix: remove blade sub-solid from bands, trapezoidal endwall face

- Bands now span z=-thickness..z=r (fillet + endwall only)
- Inner boundary uses blade angles (wider), outer uses expanded angles (narrower)
- Endwall face is trapezoidal for roughly constant arc-length band width
- Blade remains one continuous body (not split into bands)
- Restore 1e-9 tolerance for segment classification
```

---

## 11. 恢复三层 band 设计 (2026-05-11)

### 问题

上一版错误地移除了 blade 层，导致叶身部分完全没有铺层带。

### 修复

**build_ply_band_solids**：恢复 blade sub-solid 层，三层融合：
- **blade 层** (z=r..blade_height)：叶身外表面等弧长段向内偏置(叶身角度)，沿 Z 拉伸
- **fillet 层** (z=0..r)：倒圆截面回转/拉伸 (叶身角度)
- **endwall 层** (z=-thickness..0)：梯形面，内边界叶身角度(宽)→外边界扩展角度(窄)

三层曲线（叶身外廓 / 倒圆底边 R+r / 扩展曲线 R+offset）上各自取等弧长段：
- 叶身外廓：弧长 = s1-s0，角度 = 弧长/R
- 倒圆底边：弧长 = 等弧长，角度 = 弧长/(R+r)
- 扩展曲线：弧长 = 等弧长，角度 = 弧长/(R+offset)
- 侧面连接三个不同角度跨度的端点，保持弧长宽度基本一致

**验证恢复**：blade region 检测点重新加入，三层检测通过。

### 验证结果

- **12 bands (20°步长)**：12/12，VALIDATION PASSED (blade + fillet + endwall)
- **45 bands (5°步长)**：45/45，VALIDATION PASSED (blade + fillet + endwall)

### Commit

```
fix: restore blade layer in ply bands with correct blade-curve angles

- Three-layer band design: blade (blade angles) + fillet (blade angles) +
  endwall (inner=blade angles, outer=expanded angles)
- Equal arc-length segments on blade profile, fillet boundary, and expanded curve
- Side faces connect wide (blade) to narrow (expanded) angular spans
- Trapezoidal endwall face preserves roughly constant arc-length band width
```

---

## 12. 过渡 band 外边界角度修正 (2026-05-11)

### 问题

过渡 band（跨直线段-圆弧段边界）的外边界角度使用了 blade 弧长 (s1-s0) 除以 (R+offset)，但扩展曲线上的 band 中心因弧段缩放比例不同会发生偏移，导致外边界起点/终点不在预期位置。

### 修复

**过渡 band 外边界角度**：改用扩展曲线上的 arc length 位置 (q0_len, q1_len) 计算角度：
- Lower transition: `θ_out0 = θ_low + (max(q0_len, arc_start) - arc_start) / (R+offset)`
- Upper transition: `θ_out1 = θ_low + (min(q1_len, expanded_arc_end) - arc_start) / (R+offset)`

**端壁面构造**：根据 q0/q1 是否在圆弧区域内，判断外边界为全弧还是折线+弧：
- Lower transition: q0 >= arc_start → 外边界全弧 (q0→q1)；否则折线+弧
- Upper transition: q1 <= expanded_arc_end → 外边界全弧 (q0→q1)；否则弧+折线

### 验证结果

- **12 bands (20°步长)**：12/12，VALIDATION PASSED
- **45 bands (5°步长)**：45/45，VALIDATION PASSED（visualize_ply.py 验证）

### Commit

```
fix: compute transition band outer angles from expanded curve positions

- Use q0_len/q1_len (expanded curve arc length) for outer angles, not s0/s1
- Handle all-arc outer boundary case when q positions fall in arc region
- Endwall outer boundary adapts to all-arc or polyline+arc based on q position
```

---

## 13. 参考线分叉点修正：从倒圆-缘板交界移到叶身-倒圆交界 (2026-05-11)

### 问题

side_a / side_b 参考线的分叉点位于倒圆-缘板交界 (z=0, 半径 R+r)，xy 位置在倒圆曲线上（叶身向外扩充 R）。

正确位置应在叶身-倒圆交界 (z=r, 半径 R)，xy 位置在叶身曲线上。

**根因**：`_connector_polyline_pts` 中，side_a/side_b 参考线先沿倒圆曲面走到 z=0 再分别指向各自的扩展曲线端点。两条线在 z=0 之前完全相同，分叉点位于倒圆底部 (R+r)。

### 修复

**`_connector_polyline_pts`**：移除沿倒圆曲面的路径段。参考线路径改为：
```
blade 顶面 → 叶身-倒圆交界(z=r) → 直接到扩展曲线端点(z=0)
```
两条相邻 band 的 side_a/side_b 从 z=r 开始分别指向各自的扩展曲线端点 (q0_{i+1} ≠ q1_i)，分叉点位于叶身曲线上的 (R, θ(s), r)。

### 验证

- 分叉点 xy 坐标位于叶身外廓曲线 (R=33)，而非倒圆曲线 (R+r=35)
- **12 bands (20°)**：12/12，VALIDATION PASSED
- **45 bands (5°)**：45/45，VALIDATION PASSED

### Commit

```
fix: connector wire bifurcation at z=r on blade curve, not z=0 on fillet curve

- _connector_polyline_pts now goes directly from z=r to expanded endpoint
- side_a/side_b diverge at blade-fillet junction (z=r, radius R)
- No longer follows fillet quarter-circle to z=0 before diverging
```
