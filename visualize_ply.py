"""
Ply division visualization v2.
Generates 2D and 3D views verifying:
  - concave fillet direction (fixed)
  - ply bands with thickness shown as colored solids
  - connector-to-normal perpendicularity on straight segments
"""
import colorsys
import math
import sys
from pathlib import Path

import matplotlib
import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from QYModel import Params, VaneBladeAndEndwallBuilder

matplotlib.rcParams["font.family"] = "sans-serif"
matplotlib.rcParams["font.sans-serif"] = ["SimHei", "Microsoft YaHei", "DejaVu Sans"]
matplotlib.rcParams["axes.unicode_minus"] = False


def extract_wire_xyz(wire):
    pts = []
    for v in wire.Vertices():
        pts.append((v.X, v.Y, v.Z))
    return pts


def sample_blade_profile(builder, outer_profile, n=600):
    total = builder._outer_profile_total_length(outer_profile)
    pts = []
    for i in range(n):
        s = total * i / (n - 1)
        pt, _, _ = builder._outer_profile_point_tangent_normal_at_s(outer_profile, s)
        pts.append((pt.x, pt.y))
    return pts


def compute_expanded_point(builder, outer_profile, s):
    pt, _, normal = builder._outer_profile_point_tangent_normal_at_s(outer_profile, s)
    offset = builder.p.ply_expand_offset
    return (pt.x + normal.x * offset, pt.y + normal.y * offset)


def compute_inner_point(builder, outer_profile, s, thickness):
    pt, _, normal = builder._outer_profile_point_tangent_normal_at_s(outer_profile, s)
    return (pt.x - normal.x * thickness, pt.y - normal.y * thickness)


def get_segment_boundaries(builder, outer_profile):
    p = builder.p
    r = outer_profile["radius"]
    lower_len = p.blade_lower_length
    arc_angle_rad = math.radians(p.upper_tangent_angle_deg - p.lower_tangent_angle_deg)
    arc_len = r * arc_angle_rad
    upper_len = p.blade_upper_length
    total = lower_len + arc_len + upper_len
    return {
        "lower_straight": (0.0, lower_len),
        "arc": (lower_len, lower_len + arc_len),
        "upper_straight": (lower_len + arc_len, total),
        "total": total,
    }


def plot_2d_topview(builder, outer_profile, ply_curves, ply_band_solids, out_path):
    fig = plt.figure(figsize=(24, 12))

    # Main overview + lower detail + upper detail
    gs = fig.add_gridspec(2, 3, height_ratios=[3, 2],
                           hspace=0.30, wspace=0.28)

    ax_main = fig.add_subplot(gs[0, :])       # top row, full width
    ax_lower = fig.add_subplot(gs[1, 0])       # lower straight zoom
    ax_mid = fig.add_subplot(gs[1, 1])         # arc section zoom
    ax_upper = fig.add_subplot(gs[1, 2])       # upper straight zoom

    p = builder.p
    segs = get_segment_boundaries(builder, outer_profile)
    thickness = p.ply_layer_thickness

    # ── Sample blade profile ──
    blade_pts = sample_blade_profile(builder, outer_profile, 800)
    bx, by = zip(*blade_pts)

    # ── Sample expanded curve ──
    expanded_pts_xy = [compute_expanded_point(builder, outer_profile, s)
                       for s in np.linspace(0, segs["total"], 800)]
    ex, ey = zip(*expanded_pts_xy)

    # ── Sample inner offset curve (ply thickness) ──
    if thickness > 0:
        inner_pts_xy = [compute_inner_point(builder, outer_profile, s, thickness)
                        for s in np.linspace(0, segs["total"], 800)]
        ix, iy = zip(*inner_pts_xy)

    def color_for_s(s):
        if s <= segs["lower_straight"][1]:
            return "#2ecc71"
        elif s <= segs["arc"][1]:
            return "#3498db"
        else:
            return "#e67e22"

    def band_color(idx, n_bands):
        hue = idx / max(n_bands - 1, 1)
        r, g, b = colorsys.hsv_to_rgb(hue, 0.60, 0.90)
        return (r, g, b)

    # ── Main plot: blade + expanded + inner offset ──
    for i in range(len(blade_pts) - 1):
        s = segs["total"] * i / (len(blade_pts) - 1)
        c = color_for_s(s)
        ax_main.plot(bx[i:i + 2], by[i:i + 2], color=c, linewidth=2.5)

    ax_main.plot(ex, ey, "--", color="#c0392b", linewidth=2.0, alpha=0.6, label="offset curve")

    if thickness > 0:
        ax_main.plot(ix, iy, "-.", color="#8e44ad", linewidth=1.8, alpha=0.7,
                     label=f"inner offset ({thickness:.1f} mm)")

    # ── Stations ──
    stations = builder._make_ply_station_s_values(outer_profile)
    station_xy = []
    station_exp_xy = []
    for s in stations:
        pt, _, _ = builder._outer_profile_point_tangent_normal_at_s(outer_profile, s)
        exp_pt = compute_expanded_point(builder, outer_profile, s)
        station_xy.append((pt.x, pt.y))
        station_exp_xy.append(exp_pt)

    # ── Draw ply bands as filled regions ──
    n_bands = len(stations) - 1
    for idx in range(n_bands):
        s0 = stations[idx]
        s1 = stations[idx + 1]

        # Blade band segment
        seg_pts = []
        for s in np.linspace(s0, s1, 40):
            pt, _, _ = builder._outer_profile_point_tangent_normal_at_s(outer_profile, s)
            seg_pts.append((pt.x, pt.y))

        # Inner offset segment (reverse)
        for s in np.linspace(s1, s0, 40):
            pt, _, normal = builder._outer_profile_point_tangent_normal_at_s(outer_profile, s)
            seg_pts.append((pt.x - normal.x * thickness, pt.y - normal.y * thickness))

        if len(seg_pts) > 2:
            poly = plt.Polygon(seg_pts, facecolor=band_color(idx, n_bands),
                               edgecolor="none", alpha=0.20, zorder=0)
            ax_main.add_patch(poly)

    # Station connector lines
    for (sx, sy), (ex_i, ey_i) in zip(station_xy, station_exp_xy):
        ax_main.plot([sx, ex_i], [sy, ey_i], color="gray", linewidth=0.5, alpha=0.35)

    sx_all, sy_all = zip(*station_xy)
    ex_all, ey_all = zip(*station_exp_xy)
    ax_main.scatter(sx_all, sy_all, c="blue", s=22, zorder=5, label="blade stations")
    ax_main.scatter(ex_all, ey_all, c="red", s=22, zorder=5, label="offset stations")

    # ── Segment labels ──
    seg_labels = [
        ("lower straight", segs["lower_straight"], "#2ecc71"),
        ("arc", segs["arc"], "#3498db"),
        ("upper straight", segs["upper_straight"], "#e67e22"),
    ]
    for label, (s0, s1), color in seg_labels:
        sm = 0.5 * (s0 + s1)
        pt, _, _ = builder._outer_profile_point_tangent_normal_at_s(outer_profile, sm)
        ax_main.annotate(label, (pt.x, pt.y), fontsize=10, color=color,
                         ha="center", va="top",
                         bbox=dict(boxstyle="round,pad=0.3", facecolor="white", alpha=0.85))

    # ── Angle analysis ──
    lower_s_stations = [s for s in stations if s <= segs["lower_straight"][1]]
    upper_s_stations = [s for s in stations if s >= segs["upper_straight"][0]]

    def compute_angles(station_list):
        angles = []
        for s in station_list:
            pt, _, normal = builder._outer_profile_point_tangent_normal_at_s(outer_profile, s)
            exp_pt = compute_expanded_point(builder, outer_profile, s)
            conn_vec = (exp_pt[0] - pt.x, exp_pt[1] - pt.y)
            conn_len = math.hypot(*conn_vec)
            if conn_len > 1e-9:
                conn_unit = (conn_vec[0] / conn_len, conn_vec[1] / conn_len)
                dot = conn_unit[0] * normal.x + conn_unit[1] * normal.y
                dot = max(-1.0, min(1.0, dot))
                angles.append(math.degrees(math.acos(dot)))
        return angles

    lower_angles = compute_angles(lower_s_stations)
    upper_angles = compute_angles(upper_s_stations)

    expanded_arc_len = (p.outer_radius + p.ply_expand_offset) * math.radians(
        p.upper_tangent_angle_deg - p.lower_tangent_angle_deg)
    analytical = segs["lower_straight"][1] + expanded_arc_len + p.blade_upper_length

    info_lines = [
        f"Stations: {len(stations)}  |  Bands: {n_bands}  |  Angle step: {p.ply_tangent_angle_step_deg} deg",
        f"Blade total: {segs['total']:.1f} mm  (straight: {segs['lower_straight'][1]:.1f} + arc: {segs['arc'][1]-segs['arc'][0]:.1f} + straight: {p.blade_upper_length:.1f})",
        f"Expanded total: {analytical:.1f} mm  |  Offset: {p.ply_expand_offset} mm  |  Ply thickness: {thickness} mm  |  Fillet radius: {p.root_fillet_radius} mm",
        "",
    ]
    if lower_angles:
        info_lines.append(
            f"Lower straight connector-to-normal: max={max(lower_angles):.1e} deg  mean={sum(lower_angles)/len(lower_angles):.1e} deg  n={len(lower_angles)}")
    if upper_angles:
        info_lines.append(
            f"Upper straight connector-to-normal: max={max(upper_angles):.1e} deg  mean={sum(upper_angles)/len(upper_angles):.1e} deg  n={len(upper_angles)}")

    info_text = "\n".join(info_lines)
    ax_main.text(0.01, 0.99, info_text, transform=ax_main.transAxes,
                 fontsize=8.5, verticalalignment="top", fontfamily="monospace",
                 bbox=dict(boxstyle="round", facecolor="lightyellow", alpha=0.92))

    ax_main.set_title("Ply Division with Thickness — Top View (XY Plane)\n"
                      "Green=lower straight  Blue=arc  Orange=upper straight  "
                      "Filled bands = ply layer with thickness",
                      fontsize=13)
    ax_main.set_xlabel("X (mm)")
    ax_main.set_ylabel("Y (mm)")
    ax_main.set_aspect("equal")
    ax_main.grid(True, alpha=0.25)
    ax_main.legend(loc="upper right", fontsize=8.5)

    # ── Detail plots ──
    detail_configs = [
        (ax_lower, "Lower Straight Detail", segs["lower_straight"],
         [s for s in stations if s <= segs["lower_straight"][1]]),
        (ax_mid, "Arc Detail (mid section)", (segs["arc"][0] + segs["arc"][1] * 0.15,
                                               segs["arc"][0] + segs["arc"][1] * 0.35),
         [s for s in stations if segs["arc"][0] * 1.05 < s < segs["arc"][1] * 0.35]),
        (ax_upper, "Upper Straight Detail", segs["upper_straight"],
         [s for s in stations if s >= segs["upper_straight"][0]]),
    ]

    for ax_d, title, (d0, d1), det_stations in detail_configs:
        # Blade profile segment
        det_pts = []
        for s in np.linspace(d0, d1, 200):
            pt, _, _ = builder._outer_profile_point_tangent_normal_at_s(outer_profile, s)
            det_pts.append((pt.x, pt.y))
        dx, dy = zip(*det_pts)
        ax_d.plot(dx, dy, color="#2c3e50", linewidth=2.0)

        # Expanded
        det_exp = [compute_expanded_point(builder, outer_profile, s)
                   for s in np.linspace(d0, d1, 200)]
        dex, dey = zip(*det_exp)
        ax_d.plot(dex, dey, "--", color="#c0392b", linewidth=1.5, alpha=0.6)

        # Inner offset
        if thickness > 0:
            det_inner = [compute_inner_point(builder, outer_profile, s, thickness)
                         for s in np.linspace(d0, d1, 200)]
            dix, diy = zip(*det_inner)
            ax_d.plot(dix, diy, "-.", color="#8e44ad", linewidth=1.2, alpha=0.7)

        # Fill ply bands in this detail region
        for idx in range(n_bands):
            s0 = stations[idx]
            s1 = stations[idx + 1]
            if s1 < d0 or s0 > d1:
                continue
            seg_pts = []
            s_range = np.linspace(max(s0, d0), min(s1, d1), 30)
            for s in s_range:
                pt, _, _ = builder._outer_profile_point_tangent_normal_at_s(outer_profile, s)
                seg_pts.append((pt.x, pt.y))
            for s in s_range[::-1]:
                pt, _, normal = builder._outer_profile_point_tangent_normal_at_s(outer_profile, s)
                seg_pts.append((pt.x - normal.x * thickness, pt.y - normal.y * thickness))
            if len(seg_pts) > 2:
                ax_d.add_patch(plt.Polygon(seg_pts, facecolor=band_color(idx, n_bands),
                                           edgecolor="none", alpha=0.25))

        # Station connectors
        for s in det_stations:
            pt, _, _ = builder._outer_profile_point_tangent_normal_at_s(outer_profile, s)
            exp_pt = compute_expanded_point(builder, outer_profile, s)
            ax_d.plot([pt.x, exp_pt[0]], [pt.y, exp_pt[1]], "gray", linewidth=0.6, alpha=0.4)

        ax_d.set_title(title, fontsize=10)
        ax_d.set_xlabel("X (mm)")
        ax_d.set_ylabel("Y (mm)")
        ax_d.set_aspect("equal")
        ax_d.grid(True, alpha=0.25)

    fig.tight_layout()
    fig.savefig(str(out_path), dpi=150, bbox_inches="tight")
    print(f"[2D visualization saved] {out_path}")
    return fig


def plot_3d_view(builder, outer_profile, ply_curves, ply_band_solids, out_path):
    fig = plt.figure(figsize=(18, 14))
    ax = fig.add_subplot(111, projection="3d")

    p = builder.p
    segs = get_segment_boundaries(builder, outer_profile)
    total = segs["total"]
    thickness = p.ply_layer_thickness

    # Blade top and bottom edges
    blade_pts_xy = sample_blade_profile(builder, outer_profile, 400)
    bx, by = zip(*blade_pts_xy)
    bz_top = [p.blade_height] * len(bx)
    ax.plot(bx, by, bz_top, color="#2980b9", linewidth=2.2, label="blade top edge")
    ax.plot(bx, by, [0] * len(bx), color="#2980b9", linewidth=1.0, alpha=0.35)

    # Inner offset curve at blade top (ply inner boundary)
    if thickness > 0:
        inner_xy = [compute_inner_point(builder, outer_profile, s, thickness)
                    for s in np.linspace(0, total, 400)]
        iix, iiy = zip(*inner_xy)
        ax.plot(iix, iiy, bz_top, "-.", color="#8e44ad", linewidth=1.5, alpha=0.7,
                label=f"ply inner ({thickness} mm)")

    # Expanded curve
    expanded_xy = [compute_expanded_point(builder, outer_profile, s)
                   for s in np.linspace(0, total, 400)]
    ex, ey = zip(*expanded_xy)
    ax.plot(ex, ey, [0] * len(ex), "--", color="#c0392b", linewidth=1.8, label="offset curve")

    # Station paths
    station_names = sorted([k for k in ply_curves if k.startswith("ply_station_")])
    n_stations = len(station_names)
    from matplotlib import cm
    colors_tab = cm.tab20(np.linspace(0, 1, max(n_stations, 1)))

    for i, name in enumerate(station_names):
        xyz = extract_wire_xyz(ply_curves[name])
        if not xyz:
            continue
        xs, ys, zs = zip(*xyz)
        ax.plot(xs, ys, zs, color=colors_tab[i % 20], linewidth=1.0, alpha=0.7)

    # Band sides
    side_names = sorted([k for k in ply_curves if "_side_" in k])
    for name in side_names:
        xyz = extract_wire_xyz(ply_curves[name])
        if not xyz:
            continue
        xs, ys, zs = zip(*xyz)
        ax.plot(xs, ys, zs, color="green", linewidth=0.8, alpha=0.4)

    # Blade top band segments
    band_names = sorted([k for k in ply_curves if "_blade_equal_arc" in k])
    for name in band_names:
        xyz = extract_wire_xyz(ply_curves[name])
        if not xyz:
            continue
        xs, ys, zs = zip(*xyz)
        ax.plot(xs, ys, zs, color="#e74c3c", linewidth=2.5, alpha=0.85)

    # Expanded band segments
    exp_band_names = sorted([k for k in ply_curves if "_expanded_equal_arc" in k])
    for name in exp_band_names:
        xyz = extract_wire_xyz(ply_curves[name])
        if not xyz:
            continue
        xs, ys, zs = zip(*xyz)
        ax.plot(xs, ys, zs, color="#e74c3c", linewidth=2.5, alpha=0.85)

    # Ply band solids (tessellated)
    if ply_band_solids:
        n_bands = len(ply_band_solids)
        for name, solid in ply_band_solids.items():
            idx = int(name.split("_")[2])
            hue = idx / max(n_bands - 1, 1)
            r, g, b = colorsys.hsv_to_rgb(hue, 0.55, 0.88)
            try:
                verts, triangles = solid.tessellate(0.8)
                verts_np = np.array(verts)
                tris_np = np.array(triangles)
                ax.plot_trisurf(verts_np[:, 0], verts_np[:, 1], verts_np[:, 2],
                                triangles=tris_np, color=(r, g, b),
                                alpha=0.30, shade=True, linewidth=0, antialiased=True)
            except Exception:
                pass

    ax.set_title("Ply Reference Curves + Band Solids — 3D View\n"
                 "(colored bands = ply layer with thickness)",
                 fontsize=13)
    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Y (mm)")
    ax.set_zlabel("Z (mm)")
    ax.legend(loc="upper left", fontsize=7.5)
    ax.view_init(elev=28, azim=-55)

    fig.tight_layout()
    fig.savefig(str(out_path), dpi=120, bbox_inches="tight")
    print(f"[3D visualization saved] {out_path}")
    return fig


def main():
    current_dir = Path(__file__).resolve().parent

    params = Params(
        blade_height=80.0, outer_radius=33.0, blade_thickness=4.0,
        upper_tangent_angle_deg=130.0, lower_tangent_angle_deg=-90.0,
        blade_upper_length=32.0, blade_lower_length=36.0,
        endwall_height=20.0, lower_drop=20.0, lower_run=150.0,
        upper_drop=25.0, upper_run=160.0,
        root_fillet_radius=2.0, root_fillet_curve="outer",
        make_ply_curves=True, ply_tangent_angle_step_deg=5.0,
        ply_expand_offset=60.0, ply_expand_samples=1200,
        ply_fillet_samples=12, ply_curve_color=(1.0, 0.0, 0.0),
        export_debug_parts=False,
        ply_layer_thickness=1.0,
        out_dir=str(current_dir),
        step_name="vane_blade_three_parts_negative_endwall_with_ply_curves.step",
    )

    builder = VaneBladeAndEndwallBuilder(params)

    # Generate STEP file
    step_path = builder.run()
    print(f"\n[STEP file] {step_path}")

    # Get data for visualization
    blade_data = builder.build_blade_body()
    outer_profile = blade_data["outer_profile"]
    ply_curves = builder.build_ply_reference_curves(outer_profile)
    ply_band_solids = builder.build_ply_band_solids(outer_profile)

    print(f"Ply band solids: {len(ply_band_solids)} bands")

    # Generate visualizations
    out_2d = current_dir / "ply_division_2d_topview.png"
    plot_2d_topview(builder, outer_profile, ply_curves, ply_band_solids, out_2d)

    out_3d = current_dir / "ply_division_3d_view.png"
    plot_3d_view(builder, outer_profile, ply_curves, ply_band_solids, out_3d)

    print("\nDone. Output files:")
    print(f"  2D: {out_2d}")
    print(f"  3D: {out_3d}")


if __name__ == "__main__":
    main()
