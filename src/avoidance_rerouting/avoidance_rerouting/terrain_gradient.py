"""
Terrain traversability analysis from LiDAR point clouds.

Rasterizes a 3D point cloud into an elevation grid, computes slope per cell,
and visualizes traversability with PyVista. Supports PLY input or synthetic terrain.
"""
import argparse

import numpy as np


def load_ply(path):
    """Load a PLY file, return Nx3 numpy array."""
    points = []
    in_header = True
    vertex_count = 0
    with open(path, 'r') as f:
        for line in f:
            line = line.strip()
            if in_header:
                if line.startswith('element vertex'):
                    vertex_count = int(line.split()[-1])
                if line == 'end_header':
                    in_header = False
                continue
            parts = line.split()
            if len(parts) >= 3:
                points.append([float(parts[0]), float(parts[1]), float(parts[2])])
            if len(points) >= vertex_count:
                break
    return np.array(points)


def generate_synthetic_terrain(num_points=200000):
    """Generate test terrain with hills, ridges, a crater, plateau, and rocks."""
    rng = np.random.default_rng(42)

    grid_size = 20.0
    n_side = int(np.sqrt(num_points))
    x = np.linspace(-grid_size / 2, grid_size / 2, n_side)
    y = np.linspace(-grid_size / 2, grid_size / 2, n_side)
    xx, yy = np.meshgrid(x, y)
    xx = xx.flatten()
    yy = yy.flatten()

    xx += rng.normal(0, 0.02, len(xx))
    yy += rng.normal(0, 0.02, len(yy))

    zz = np.zeros_like(xx)
    zz += 0.5 * np.sin(xx * 0.5) * np.cos(yy * 0.4)
    zz += 2.0 * np.exp(-((xx - 3.0) ** 2) / 0.5)

    r_crater = np.sqrt((xx + 3) ** 2 + (yy - 2) ** 2)
    zz -= 1.5 * np.exp(-(r_crater ** 2) / 2.0)

    plateau_mask = (xx > 5) & (xx < 8) & (yy > -3) & (yy < 3)
    zz[plateau_mask] = 1.5
    ramp_mask = (xx > 4) & (xx <= 5) & (yy > -3) & (yy < 3)
    zz[ramp_mask] = (xx[ramp_mask] - 4.0) * 1.5

    for _ in range(15):
        cx, cy = rng.uniform(-8, 8, 2)
        rock_r = np.sqrt((xx - cx) ** 2 + (yy - cy) ** 2)
        zz += rng.uniform(0.3, 0.8) * np.exp(-(rock_r ** 2) / rng.uniform(0.05, 0.2))

    zz += rng.normal(0, 0.01, len(zz))
    return np.column_stack([xx, yy, zz])


def build_elevation_grid(points, resolution, obstacle_height=0.5, obstacle_max_height=1.0):
    """Rasterize point cloud into 2D elevation grid with obstacle detection.

    Ground = 10th percentile of Z per cell. Obstacle = Z spread > obstacle_height
    among points within obstacle_max_height of ground (ignores canopy).
    """
    x, y, z = points[:, 0], points[:, 1], points[:, 2]
    x_min, x_max = x.min(), x.max()
    y_min, y_max = y.min(), y.max()

    nx = int(np.ceil((x_max - x_min) / resolution)) + 1
    ny = int(np.ceil((y_max - y_min) / resolution)) + 1

    xi = np.clip(((x - x_min) / resolution).astype(int), 0, nx - 1)
    yi = np.clip(((y - y_min) / resolution).astype(int), 0, ny - 1)

    cell_idx = xi * ny + yi
    elevation = np.full((nx, ny), np.nan)
    obstacle_mask = np.zeros((nx, ny), dtype=bool)

    order = np.argsort(cell_idx)
    sorted_z = z[order]
    sorted_cells = cell_idx[order]
    split_points = np.searchsorted(sorted_cells, np.arange(nx * ny))
    split_points = np.append(split_points, len(sorted_z))

    for k in range(nx * ny):
        start, end = split_points[k], split_points[k + 1]
        if start == end:
            continue
        cell_z = sorted_z[start:end]
        i, j = divmod(k, ny)
        ground = np.percentile(cell_z, 10)
        elevation[i, j] = ground
        near_ground = cell_z[cell_z - ground <= obstacle_max_height]
        if len(near_ground) > 0 and (near_ground.max() - near_ground.min()) > obstacle_height:
            obstacle_mask[i, j] = True

    x_centers = x_min + (np.arange(nx) + 0.5) * resolution
    y_centers = y_min + (np.arange(ny) + 0.5) * resolution
    return elevation, x_centers, y_centers, obstacle_mask


def _fill_nans(elevation):
    """Fill NaN cells by nearest-neighbor interpolation. Returns filled copy."""
    filled = elevation.copy()
    nan_mask = np.isnan(filled)
    if nan_mask.any():
        from scipy.ndimage import distance_transform_edt
        indices = distance_transform_edt(nan_mask, return_distances=False, return_indices=True)
        filled = filled[tuple(indices)]
    return filled, nan_mask


def compute_slope(elevation, resolution):
    """Compute slope (degrees) per cell using max neighbor elevation difference."""
    filled, nan_mask = _fill_nans(elevation)
    nx, ny = filled.shape
    max_diff = np.zeros_like(filled)
    for di, dj in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
        shifted = np.full_like(filled, np.nan)
        src_i = slice(max(0, -di), nx - max(0, di))
        src_j = slice(max(0, -dj), ny - max(0, dj))
        dst_i = slice(max(0, di), nx - max(0, -di))
        dst_j = slice(max(0, dj), ny - max(0, -dj))
        shifted[dst_i, dst_j] = filled[src_i, src_j]
        diff = np.abs(filled - shifted) / resolution
        valid = ~np.isnan(diff)
        max_diff[valid] = np.maximum(max_diff[valid], diff[valid])

    slope_deg = np.degrees(np.arctan(max_diff))
    slope_deg[nan_mask] = np.nan
    return slope_deg


def mark_enclosed_regions(slope_deg, max_slope):
    """Mark traversable cells not reachable from grid edges as undrivable."""
    from scipy.ndimage import label

    traversable = (~np.isnan(slope_deg)) & (slope_deg < max_slope)
    labeled, _ = label(traversable)

    # Find connected components that touch any grid edge
    edge_labels = set()
    edge_labels.update(labeled[0, :])
    edge_labels.update(labeled[-1, :])
    edge_labels.update(labeled[:, 0])
    edge_labels.update(labeled[:, -1])
    edge_labels.discard(0)

    enclosed = traversable & ~np.isin(labeled, list(edge_labels))
    enclosed_count = np.count_nonzero(enclosed)
    if enclosed_count > 0:
        slope_deg[enclosed] = max_slope
        print(f"Marked {enclosed_count} enclosed cells as undrivable")

    return slope_deg


def apply_rover_footprint(elevation, obstacle_mask, resolution, rover_size, max_slope):
    """Check if rover can sit at each cell using 4-corner tilt + obstacle check."""
    nx, ny = elevation.shape
    h = int(np.round(rover_size / (2.0 * resolution)))

    def shifted_elev(di, dj):
        result = np.full((nx, ny), np.nan)
        src_i = slice(max(0, -di), nx - max(0, di))
        src_j = slice(max(0, -dj), ny - max(0, dj))
        dst_i = slice(max(0, di), nx - max(0, -di))
        dst_j = slice(max(0, dj), ny - max(0, -dj))
        result[dst_i, dst_j] = elevation[src_i, src_j]
        return result

    z_fl = shifted_elev(-h, -h)
    z_fr = shifted_elev(-h,  h)
    z_bl = shifted_elev( h, -h)
    z_br = shifted_elev( h,  h)

    side_dist = rover_size
    diag_dist = rover_size * np.sqrt(2)

    def pair_slope(z1, z2, dist):
        return np.degrees(np.arctan(np.abs(z1 - z2) / dist))

    max_tilt = np.fmax(pair_slope(z_fl, z_fr, side_dist),
              np.fmax(pair_slope(z_bl, z_br, side_dist),
              np.fmax(pair_slope(z_fl, z_bl, side_dist),
              np.fmax(pair_slope(z_fr, z_br, side_dist),
              np.fmax(pair_slope(z_fl, z_br, diag_dist),
                      pair_slope(z_fr, z_bl, diag_dist))))))

    all_valid = ~(np.isnan(z_fl) | np.isnan(z_fr) | np.isnan(z_bl) | np.isnan(z_br))

    from scipy.ndimage import maximum_filter
    obs_under_rover = maximum_filter(obstacle_mask.astype(np.float64), size=2*h+1) > 0.5

    return all_valid & (max_tilt < max_slope) & ~obs_under_rover


def analyze_terrain(points, resolution=0.2, max_slope=35.0, rover_size=0.0, smoothing=0.0):
    """Run full analysis pipeline, return results dict."""
    print(f"Points: {len(points)}")
    print(f"Resolution: {resolution}m, Max slope: {max_slope} deg")

    elevation, x_centers, y_centers, obstacle_mask = build_elevation_grid(points, resolution)
    nx, ny = elevation.shape
    valid_cells = np.count_nonzero(~np.isnan(elevation))
    print(f"Grid: {nx}x{ny} ({valid_cells} cells, {np.count_nonzero(obstacle_mask)} obstacles)")

    if smoothing > 0:
        from scipy.ndimage import gaussian_filter
        filled, nan_mask = _fill_nans(elevation)
        sigma_cells = smoothing / resolution
        elevation = gaussian_filter(filled, sigma=sigma_cells)
        elevation[nan_mask] = np.nan
        print(f"Smoothing: sigma={smoothing}m ({sigma_cells:.1f} cells)")

    slope_deg = compute_slope(elevation, resolution)

    valid_slopes = slope_deg[~np.isnan(slope_deg)]
    traversable_count = int(np.sum(valid_slopes < max_slope))
    total = len(valid_slopes)
    pct = 100.0 * traversable_count / total if total > 0 else 0
    print(f"Slope: mean={np.mean(valid_slopes):.1f}, max={np.max(valid_slopes):.1f}, "
          f"median={np.median(valid_slopes):.1f} deg")
    print(f"Traversable (<{max_slope} deg): {traversable_count}/{total} ({pct:.1f}%)")

    terrain_slope = slope_deg.copy()
    slope_deg[obstacle_mask] = max_slope
    slope_deg = mark_enclosed_regions(slope_deg, max_slope)

    rover_traversable = None
    if rover_size > 0:
        rover_traversable = apply_rover_footprint(
            elevation, obstacle_mask, resolution, rover_size, max_slope
        )
        valid_mask = ~np.isnan(slope_deg)
        rover_blocked = np.count_nonzero(~rover_traversable & valid_mask & (slope_deg < max_slope))
        if rover_blocked > 0:
            print(f"Rover ({rover_size}m) blocked {rover_blocked} additional cells")
        total = int(np.sum(valid_mask))
        traversable_count = int(np.sum(rover_traversable[valid_mask]))
        pct = 100.0 * traversable_count / total if total > 0 else 0
        print(f"Rover-aware traversable: {traversable_count}/{total} ({pct:.1f}%)")

    return {
        'elevation': elevation,
        'slope_deg': slope_deg,
        'terrain_slope': terrain_slope,
        'obstacle_mask': obstacle_mask,
        'rover_traversable': rover_traversable,
        'x_centers': x_centers,
        'y_centers': y_centers,
        'pct': pct,
    }


def analyze_and_visualize(points, resolution=0.2, max_slope=35.0, **kwargs):
    """Analyze terrain and show interactive PyVista visualization."""
    import pyvista as pv
    from matplotlib.colors import LinearSegmentedColormap

    smoothing = kwargs.get('smoothing', 0.0)
    rover_size = kwargs.get('rover_size', 0.0)
    show_contours = kwargs.get('show_contours', True)

    result = analyze_terrain(points, resolution, max_slope, rover_size, smoothing)
    elevation = result['elevation']
    slope_deg = result['slope_deg']
    terrain_slope = result['terrain_slope']
    obstacle_mask = result['obstacle_mask']
    rover_traversable = result['rover_traversable']
    x_centers = result['x_centers']
    y_centers = result['y_centers']
    pct = result['pct']
    nx, ny = elevation.shape

    # Map each point to its grid cell
    px, py, pz = points[:, 0], points[:, 1], points[:, 2]
    pi = np.clip(((px - x_centers[0]) / resolution).astype(int), 0, nx - 1)
    pj = np.clip(((py - y_centers[0]) / resolution).astype(int), 0, ny - 1)

    ground_z = elevation[pi, pj]
    nan_ground = np.isnan(ground_z)
    ground_z[nan_ground] = pz[nan_ground]
    height_above_ground = pz - ground_z

    safe_ratio = min(15.0 / max_slope, 0.5)
    slope_cmap = LinearSegmentedColormap.from_list('terrain_slope', [
        (0.0,               (0.55, 0.55, 0.55)),
        (safe_ratio,        (0.55, 0.55, 0.55)),
        (safe_ratio + 0.01, (1.0, 1.0, 0.2)),
        (0.65,              (1.0, 0.5, 0.0)),
        (0.8,               (0.9, 0.1, 0.0)),
        (1.0,               (0.5, 0.0, 0.3)),
    ])

    obstacle_color = np.array([0.4, 0.0, 0.5, 1.0])
    is_ground = height_above_ground < 0.3
    point_slope = terrain_slope[pi, pj]

    norm_slope = np.clip(point_slope / max_slope, 0.0, 1.0)
    norm_slope[np.isnan(norm_slope)] = 0.0
    rgba = slope_cmap(norm_slope)

    if rover_traversable is not None:
        rover_blocked_pts = is_ground & ~rover_traversable[pi, pj]
        rgba[rover_blocked_pts] = slope_cmap(1.0)
    rgba[~is_ground] = obstacle_color

    pcd = pv.PolyData(points)
    pcd['rgba'] = (rgba * 255).astype(np.uint8)
    point_slope_display = point_slope.copy()
    point_slope_display[~is_ground] = np.nan
    pcd['slope_deg'] = point_slope_display

    plotter = pv.Plotter(window_size=[1400, 900])
    plotter.set_background('black')
    plotter.add_mesh(pcd, scalars='rgba', rgba=True, point_size=2,
                     render_points_as_spheres=False, show_scalar_bar=False)

    # Scalar bar via invisible dummy mesh
    dummy = pv.PolyData(points[:1])
    dummy['slope_deg'] = np.array([0.0])
    plotter.add_mesh(dummy, scalars='slope_deg', cmap=slope_cmap,
                     clim=[0, max_slope], point_size=0.001, opacity=0.0,
                     scalar_bar_args={'title': 'Slope (degrees)', 'color': 'white',
                                      'title_font_size': 14, 'label_font_size': 12,
                                      'n_labels': 6})

    rover_label = f"Rover: {rover_size}m | " if rover_size > 0 else ""
    plotter.add_text(
        f"{rover_label}Traversable: {pct:.0f}% | Grid: {nx}x{ny} @ {resolution}m | "
        f"Max slope: {max_slope} deg",
        position='upper_left', font_size=10, color='white')

    if rover_size > 0:
        half = rover_size / 2.0
        base_corners = np.array([
            [-half, -half, 0], [ half, -half, 0],
            [ half,  half, 0], [-half,  half, 0],
        ])
        lines = np.array([2, 0, 1, 2, 1, 2, 2, 2, 3, 2, 3, 0])
        rover_box = pv.PolyData(base_corners.copy(), lines=lines)
        plotter.add_mesh(rover_box, color='cyan', line_width=3)

        x_range = (float(px.min()), float(px.max()))
        y_range = (float(py.min()), float(py.max()))
        z_range = (float(pz.min()), float(pz.max()))
        rover_state = {
            'x': np.median(px), 'y': np.median(py),
            'z': float(np.median(pz)), 'rot': 0.0,
        }

        def update_rover_box():
            angle = np.radians(rover_state['rot'])
            cos_a, sin_a = np.cos(angle), np.sin(angle)
            rot = base_corners.copy()
            rot[:, 0] = base_corners[:, 0] * cos_a - base_corners[:, 1] * sin_a + rover_state['x']
            rot[:, 1] = base_corners[:, 0] * sin_a + base_corners[:, 1] * cos_a + rover_state['y']
            rot[:, 2] = rover_state['z']
            rover_box.points = rot

        def on_x(v): rover_state['x'] = v; update_rover_box()
        def on_y(v): rover_state['y'] = v; update_rover_box()
        def on_z(v): rover_state['z'] = v; update_rover_box()
        def on_rot(v): rover_state['rot'] = v; update_rover_box()

        update_rover_box()
        sx0, sx1 = 0.7, 0.95
        plotter.add_slider_widget(on_x, x_range, value=rover_state['x'],
                                  title='X', pointa=(sx0, 0.90), pointb=(sx1, 0.90),
                                  style='modern', color='cyan', fmt='%.1f')
        plotter.add_slider_widget(on_y, y_range, value=rover_state['y'],
                                  title='Y', pointa=(sx0, 0.78), pointb=(sx1, 0.78),
                                  style='modern', color='cyan', fmt='%.1f')
        plotter.add_slider_widget(on_z, z_range, value=rover_state['z'],
                                  title='Z', pointa=(sx0, 0.66), pointb=(sx1, 0.66),
                                  style='modern', color='cyan', fmt='%.1f')
        plotter.add_slider_widget(on_rot, (0, 360), value=0.0,
                                  title='Rot', pointa=(sx0, 0.54), pointb=(sx1, 0.54),
                                  style='modern', color='cyan', fmt='%.0f\u00b0')

    # Traversability contours (batched into single mesh)
    if show_contours:
        from skimage.measure import find_contours
        if rover_traversable is not None:
            drivable = rover_traversable.astype(float)
        else:
            drivable = ((~obstacle_mask) & (~np.isnan(slope_deg)) & (slope_deg < max_slope)).astype(float)

        mean_elev = float(np.nanmean(elevation))
        all_pts, all_cells = [], []
        offset = 0
        for contour in find_contours(drivable, level=0.5):
            if len(contour) < 3:
                continue
            cx = x_centers[0] + contour[:, 0] * resolution
            cy = y_centers[0] + contour[:, 1] * resolution
            ci = np.clip(contour[:, 0].astype(int), 0, nx - 1)
            cj = np.clip(contour[:, 1].astype(int), 0, ny - 1)
            cz = elevation[ci, cj].copy()
            cz[np.isnan(cz)] = mean_elev
            cz += 0.1
            pts = np.column_stack([cx, cy, cz])
            n = len(pts)
            cells = np.empty((n - 1) * 3, dtype=int)
            cells[0::3] = 2
            cells[1::3] = np.arange(n - 1) + offset
            cells[2::3] = np.arange(1, n) + offset
            all_pts.append(pts)
            all_cells.append(cells)
            offset += n

        if all_pts:
            poly = pv.PolyData(np.vstack(all_pts), lines=np.concatenate(all_cells))
            plotter.add_mesh(poly, color='lime', line_width=2)
            print(f"Drew {len(all_pts)} contour lines")

    plotter.add_axes()
    plotter.camera.clipping_range = (0.01, 1000.0)
    plotter.enable_fly_to_right_click()
    plotter.show()


def main():
    parser = argparse.ArgumentParser(description="Terrain traversability from LiDAR point clouds")
    parser.add_argument('--ply', type=str, default=None, help='PLY point cloud file')
    parser.add_argument('--resolution', type=float, default=0.2, help='Grid cell size in meters')
    parser.add_argument('--max-slope', type=float, default=20.0, help='Undrivable slope threshold (deg)')
    parser.add_argument('--rover-size', type=float, default=0.0, help='Rover footprint in meters (0=off)')
    parser.add_argument('--smoothing', type=float, default=0.0, help='Gaussian smoothing sigma in meters')
    parser.add_argument('--no-contours', action='store_true', help='Hide traversability contours')
    args = parser.parse_args()

    if args.ply:
        print(f"Loading {args.ply}")
        points = load_ply(args.ply)
    else:
        print("Generating synthetic terrain...")
        points = generate_synthetic_terrain()

    analyze_and_visualize(
        points,
        resolution=args.resolution,
        max_slope=args.max_slope,
        rover_size=args.rover_size,
        smoothing=args.smoothing,
        show_contours=not args.no_contours,
    )


if __name__ == '__main__':
    main()
