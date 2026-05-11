# 对话记录与迭代历史 — 2026-05-11

## 迭代概览

| 迭代 | Commit | 主要变更 |
|------|--------|----------|
| 1 | `a0efa36` | 初版：移除 blade sub-solid，梯形 endwall 面 |
| 2 | `c3f027e` | 恢复 blade 三层设计，叶身角度/扩展角度分离 |
| 3 | `0d8661f` | 过渡 band 外边界角度从 q 位置计算 |
| 4 | `a82d0bc` | 参考线分叉点从 z=0 移到 z=r (但直接用直线连接) |
| 5 | `f12b1fd` | 参考线螺旋投影 + arc band 用 ThruSections loft |

---

## 当前代码架构说明

### 1. 整体几何结构

```
                    叶身 (blade)
                   z = blade_height (80mm)
                   │
    blade 外表面     │  ← blade sub-solid: XY面沿+Z挤出
    (半径 R=33)     │
                   z = r (2mm) ← 叶身-倒圆交界 ★分叉点★
                   │
    ┌──────────────┤
    │ 倒圆 (fillet) │  ← fillet sub-solid: ThruSections loft (arc)
    │ 四分之一圆弧  │    或 BRepPrimAPI_MakePrism (straight)
    │ R → R+r      │    或 split straight+arc (transition)
    └──────────────┤
                   z = 0 ← 倒圆-缘板交界
    ┌──────────────┤
    │ 缘板(endwall)│  ← endwall sub-solid: XY梯形面沿-Z挤出
    │ R+r → R+off  │
    └──────────────┤
                   z = -thickness (-1mm)
```

### 2. 三条曲线

沿 Z 轴俯视，每层有一条曲线：

| 曲线 | 位置 | 弧段半径 | 说明 |
|------|------|----------|------|
| 叶身外廓 | blade 外表面 | R=33 | blade profile (直+弧+直) |
| 倒圆底边 | z=0, 沿法向偏置 r | R+r=35 | 叶身外廓向外偏置倒圆半径 |
| 扩展曲线 | z=0, 扩展偏置 | R+offset=93 | 倒圆底边再向外扩展到 offset |

等弧长分段：每条曲线上取 blade_seg_len 长的段，弧段角度依次为 L/R、L/(R+r)、L/(R+offset)，依次变窄。

### 3. 角度体系

每个 band (s0→s1，扩展端点 q0→q1) 有两套角度：

| 角度 | 分母 | 用途 | 弧段例子 (L=16.23mm) |
|------|------|------|---------------------|
| θ_in (叶身角) | R=33 | blade 层 + fillet 层内边界 + endwall 内边界 | 28.2° span |
| θ_out (扩展角) | R+offset=93 | endwall 外边界 + fillet 侧边终点 | 10.0° span |

```python
# 纯弧段 (on_arc):
θ_in0, θ_in1 = _band_arc_angles_deg(s0, s1)          # 分母 R
θ_out0 = θ_center - blade_seg_len/(2*(R+offset))     # 分母 R+offset
θ_out1 = θ_center + blade_seg_len/(2*(R+offset))

# 过渡段 (is_lower_trans / is_upper_trans):
# 内角：同叶身角，用 _band_arc_angles_deg
# 外角：从 q0_len / q1_len (扩展曲线位置) 计算
```

### 4. Band 实体三层结构

每个 band 由三段融合 (fuse) 而成：

#### 4a. Blade 层 (z=r → blade_height)

- 2D 截面：叶身外廓 (R, θ_in0→θ_in1) → 径向向内 → 内廓 (R-t, θ_in1→θ_in0) → 径向向外
- 沿 +Z 挤出 (blade_height - r)，平移到 z=r
- **侧面是径向面** (叶身角度)

#### 4b. Fillet 层 (z=0 → z=r)

- **Arc bands**: `_make_fillet_solid_loft` — ThruSections loft
  - 6 层截面从 z=r 到 z=0
  - 内边界：沿四分之一圆环 (内弧半径 r+t)，走叶身角度 (恒定)
  - 外边界：跟随四分之一圆弧 (外弧半径 r)，**侧边角度螺旋过渡**
  - θ_side(z) = θ_in + (θ_out - θ_in) × (r-z)/r
  - 结果：侧面在 z=r 处与 blade 层对齐 (叶身角)，在 z=0 处到达扩展角
  - ★ 相邻 band 侧面从 z=r 开始分叉

- **Straight bands**: `_make_fillet_solid_straight` — BRepPrimAPI_MakePrism
  - 四分之一圆环截面沿切线方向拉伸
  - 侧面跟随法向 (直线段无螺旋)

- **Transition bands**: `_make_fillet_solid_transition`
  - 拆分为 straight + arc 子段，各自生成后 fuse

#### 4c. Endwall 层 (z=-thickness → z=0)

- 2D 梯形截面：
  - 内弧：R+r, θ_in0→θ_in1 (叶身角度，较宽)
  - 侧边 A：从 (R+r, θ_in0) 到 (R+offset, θ_out0)
  - 外弧：R+offset, θ_out1→θ_out0 (扩展角度，较窄)
  - 侧边 B：从 (R+offset, θ_out1) 到 (R+r, θ_in1)
- 沿 -Z 挤出 thickness
- **侧面是非径向垂直面** (梯形侧边)

#### 4d. 过渡 band 端壁处理

- Lower transition: 内侧直+弧 (叶身角) / 外侧全弧或折线+弧 (扩展角)
- Upper transition: 内侧弧+直 (叶身角) / 外侧全弧或弧+折线 (扩展角)
- q0/q1 在弧段的判断决定外侧是否全弧

### 5. 参考线 (Connector Wire)

`_connector_polyline_pts` 生成 side_a/side_b 参考线：

**弧段路径** (spiral):
```
blade顶 (z=blade_height) 
  → blade-倒圆交界 (z=r, R, θ_blade)  ← ★分叉点
  → 倒圆面螺旋 (z=r→0, R+rad(z), θ_blade→θ_exp)
  → 缘板顶面 (z=0, R+offset, θ_exp)
  → 缘板底面 (z=-thickness, R+offset, θ_exp)
```

**非弧段路径** (沿法向):
```
blade顶 → blade-倒圆交界 → 沿法向走四分之一圆弧倒圆面
  → 缘板顶面 (R+r) → 扩展曲线 → 缘板底面
```

**分叉机制**：
- 相邻 band i 的 side_b 和 band i+1 的 side_a 共享 blade 段 (z≥r)
- 从 z=r 开始，各自的螺旋终点不同: q1_i ≠ q0_{i+1}
- 两条线从 z=r 处分叉，螺旋到各自的扩展角度终点

### 6. 关键参数

```python
blade_height = 80.0      # 叶身高度
outer_radius R = 33.0    # 外侧圆弧半径
blade_thickness = 4.0    # 叶身壁厚 (结构，非 band 厚度)
root_fillet_radius r = 2.0  # 倒圆半径
endwall_height = 20.0    # 缘板厚度
ply_expand_offset = 60.0 # 扩展偏置距离
ply_layer_thickness = 1.0 # 铺层带可视化厚度
ply_tangent_angle_step = 20.0°  # 分段步长 (12 bands)
```

### 7. 几何数据

```
叶身外侧总长: 194.71mm
  - 下直线: 36.0mm  (s: 0→36)
  - 圆弧:   126.7mm (s: 36→162.7, R=33, 角度=-90°→130°=220°)
  - 上直线: 32.0mm  (s: 162.7→194.7)

扩展曲线总长: 425.09mm
  - 下直线: 36.0mm (同长)
  - 圆弧:   357.1mm (R=93, 220°)
  - 上直线: 32.0mm (同长)

12 bands: 每段 blade_seg_len = 194.71/12 ≈ 16.23mm
  - 纯下直: bands 0-1
  - 下过渡: band 2 (跨直-弧)
  - 纯弧:   bands 3-9
  - 上过渡: band 10 (跨弧-直)
  - 纯上直: band 11
```

### 8. 验证

`_validate_band_solids` 对每个 band 在 blade / fillet / endwall 三区域采样检测：
- Blade: 向内 t/2, z = blade_h*0.7 (叶身中上部)
- Fillet: 内外弧中点, z = 倒圆中间高度
- Endwall: 端壁中间深度

---

## 已知限制 (待改进)

1. **Transition band fillet**: 仍用 `_make_fillet_solid_transition` (revolve+prism 拆分)，侧面是径向的，未像 arc band 一样用 ThruSections loft
2. **Straight band fillet**: 用 `BRepPrimAPI_MakePrism`，侧面沿切线方向，直线段本身无螺旋需求
3. **Endwall 侧面与 fillet 侧面在 z=0 处有转折**：fillet 侧面是螺旋曲面，endwall 侧面是垂直平面，在 z=0 交界处不连续
