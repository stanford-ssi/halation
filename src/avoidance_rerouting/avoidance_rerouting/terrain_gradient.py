"""
Terrain Gradient / Traversability Analysis from LiDAR Point Clouds

Takes a 3D point cloud, creates an elevation grid at configurable resolution,
computes the slope gradient at each cell, and colors every point directly:
ground points use a gray→yellow→red spectrum by slope, obstacle points
(height >= 0.3m above ground) are colored dark purple.

Can load PLY files from the pointcloud_accumulator or generate synthetic
terrain for testing.

Usage:
    # Synthetic demo terrain
    python terrain_gradient.py

    # With a PLY file from the accumulator
    python terrain_gradient.py --ply ~/halation/pointclouds/scan_20260305_120000.ply

    # Adjust resolution and max slope
    python terrain_gradient.py --resolution 0.15 --max-slope 30
"""
import argparse
import numpy as np


def load_ply(path):
    """Load a PLY file and return Nx3 numpy array of points."""
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
    """
    Generate a synthetic terrain point cloud with flat areas, gentle slopes,
    steep ridges, a crater, a plateau, and random rocks.
    """
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

    # Gentle rolling hills
    zz += 0.5 * np.sin(xx * 0.5) * np.cos(yy * 0.4)

    # Steep ridge along x=3
    zz += 2.0 * np.exp(-((xx - 3.0) ** 2) / 0.5)

    # Crater/depression
    r_crater = np.sqrt((xx + 3) ** 2 + (yy - 2) ** 2)
    zz -= 1.5 * np.exp(-(r_crater ** 2) / 2.0)

    # Flat plateau
    plateau_mask = (xx > 5) & (xx < 8) & (yy > -3) & (yy < 3)
    zz[plateau_mask] = 1.5

    # Ramp to plateau
    ramp_mask = (xx > 4) & (xx <= 5) & (yy > -3) & (yy < 3)
    ramp_t = (xx[ramp_mask] - 4.0) / 1.0
    zz[ramp_mask] = ramp_t * 1.5

    # Random rocks
    for _ in range(15):
        cx, cy = rng.uniform(-8, 8, 2)
        rock_r = np.sqrt((xx - cx) ** 2 + (yy - cy) ** 2)
        zz += rng.uniform(0.3, 0.8) * np.exp(-(rock_r ** 2) / rng.uniform(0.05, 0.2))

    zz += rng.normal(0, 0.01, len(zz))

    return np.column_stack([xx, yy, zz])


def build_elevation_grid(points, resolution, obstacle_height=0.5):
    """
    Rasterize a point cloud into a 2D ground elevation grid with obstacle detection.

    For each cell:
    - Uses a low percentile of Z (10th) as the ground estimate (filters out canopy)
    - If the Z range (max - min) exceeds obstacle_height, marks the cell as
      having an obstacle (tree, bush, rock) — still uses ground Z for the mesh
      but returns an obstacle mask for those cells.

    Returns:
        elevation: 2D ground elevation array (NaN where no points)
        x_centers, y_centers: cell center coordinates
        obstacle_mask: 2D bool array (True = tall obstacle in this cell)
    """
    x, y, z = points[:, 0], points[:, 1], points[:, 2]

    x_min, x_max = x.min(), x.max()
    y_min, y_max = y.min(), y.max()

    nx = int(np.ceil((x_max - x_min) / resolution)) + 1
    ny = int(np.ceil((y_max - y_min) / resolution)) + 1

    xi = np.clip(((x - x_min) / resolution).astype(int), 0, nx - 1)
    yi = np.clip(((y - y_min) / resolution).astype(int), 0, ny - 1)

    # Collect Z values per cell for percentile and range computation
    cell_idx = xi * ny + yi
    elevation = np.full((nx, ny), np.nan)
    obstacle_mask = np.zeros((nx, ny), dtype=bool)

    # Group points by cell
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
        # Ground estimate: 10th percentile (below canopy)
        elevation[i, j] = np.percentile(cell_z, 10)
        # Obstacle detection: large Z range means something tall is here
        z_range = cell_z.max() - cell_z.min()
        if z_range > obstacle_height:
            obstacle_mask[i, j] = True

    x_centers = x_min + (np.arange(nx) + 0.5) * resolution
    y_centers = y_min + (np.arange(ny) + 0.5) * resolution

    return elevation, x_centers, y_centers, obstacle_mask


def compute_slope(elevation, resolution):
    """
    Compute slope angle (degrees) at each grid cell.

    Uses the max absolute elevation difference to any of the 4 neighbors
    divided by resolution. This correctly flags both sides of a steep edge
    (e.g. a cliff top AND cliff bottom show as steep, not just the face
    between them).

    Also returns dz/dx and dz/dy from np.gradient for arrow directions.
    """
    filled = elevation.copy()
    nan_mask = np.isnan(filled)
    if nan_mask.any():
        from scipy.ndimage import distance_transform_edt
        indices = distance_transform_edt(
            nan_mask, return_distances=False, return_indices=True
        )
        filled = filled[tuple(indices)]

    # Gradient vectors for arrow direction
    grad_x, grad_y = np.gradient(filled, resolution)

    # Max neighbor difference for coloring — catches both sides of edges
    nx, ny = filled.shape
    max_diff = np.zeros_like(filled)
    for di, dj in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
        # Shifted array
        shifted = np.full_like(filled, np.nan)
        src_i = slice(max(0, -di), nx - max(0, di))
        src_j = slice(max(0, -dj), ny - max(0, dj))
        dst_i = slice(max(0, di), nx - max(0, -di))
        dst_j = slice(max(0, dj), ny - max(0, -dj))
        shifted[dst_i, dst_j] = filled[src_i, src_j]
        diff = np.abs(filled - shifted) / resolution
        valid_diff = ~np.isnan(diff)
        max_diff[valid_diff] = np.maximum(max_diff[valid_diff], diff[valid_diff])

    slope_deg = np.degrees(np.arctan(max_diff))
    slope_deg[nan_mask] = np.nan

    return slope_deg, grad_x, grad_y


def mark_enclosed_regions(slope_deg, max_slope):
    """
    Flood-fill from grid edges through traversable cells (slope < max_slope).
    Any traversable cell NOT reachable from the edge is enclosed by steep
    terrain and gets marked as undrivable (slope set to max_slope).

    Modifies slope_deg in-place and returns the updated array.
    """
    traversable = (~np.isnan(slope_deg)) & (slope_deg < max_slope)

    # Create a reachable mask by flood-filling from all edge cells
    nx, ny = slope_deg.shape
    reachable = np.zeros((nx, ny), dtype=bool)

    # Seed: all traversable cells on any edge
    seeds = []
    for i in range(nx):
        for j in [0, ny - 1]:
            if traversable[i, j]:
                seeds.append((i, j))
    for j in range(ny):
        for i in [0, nx - 1]:
            if traversable[i, j]:
                seeds.append((i, j))

    # BFS flood fill
    from collections import deque
    queue = deque(seeds)
    for i, j in seeds:
        reachable[i, j] = True

    while queue:
        ci, cj = queue.popleft()
        for di, dj in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            ni, nj = ci + di, cj + dj
            if 0 <= ni < nx and 0 <= nj < ny and traversable[ni, nj] and not reachable[ni, nj]:
                reachable[ni, nj] = True
                queue.append((ni, nj))

    # Mark enclosed traversable cells as undrivable
    enclosed = traversable & ~reachable
    enclosed_count = np.count_nonzero(enclosed)
    if enclosed_count > 0:
        slope_deg[enclosed] = max_slope
        print(f"Marked {enclosed_count} enclosed cells as undrivable")

    return slope_deg


def analyze_and_visualize(points, resolution=0.2, max_slope=35.0, **kwargs):
    """Full pipeline: point cloud -> elevation grid -> slope -> colored point cloud."""
    import pyvista as pv
    from matplotlib.colors import LinearSegmentedColormap

    print(f"Points: {len(points)}")
    print(f"Resolution: {resolution}m, Max slope threshold: {max_slope} deg")

    # Build elevation grid (ground-filtered, with obstacle detection)
    elevation, x_centers, y_centers, obstacle_mask = build_elevation_grid(
        points, resolution
    )
    nx, ny = elevation.shape
    valid_cells = np.count_nonzero(~np.isnan(elevation))
    obstacle_count = np.count_nonzero(obstacle_mask)
    print(f"Elevation grid: {nx}x{ny} "
          f"({valid_cells} cells with data, {obstacle_count} obstacle cells)")

    # Compute slope and gradient vectors
    slope_deg, _, _ = compute_slope(elevation, resolution)

    # Statistics
    valid_slopes = slope_deg[~np.isnan(slope_deg)]
    traversable_count = int(np.sum(valid_slopes < max_slope))
    total = len(valid_slopes)
    pct = 100.0 * traversable_count / total if total > 0 else 0
    print(f"Mean slope: {np.mean(valid_slopes):.1f} deg, "
          f"Max: {np.max(valid_slopes):.1f} deg, "
          f"Median: {np.median(valid_slopes):.1f} deg")
    print(f"Traversable (<{max_slope} deg): {traversable_count}/{total} "
          f"cells ({pct:.1f}%)")

    # Save terrain-only slope (before obstacle overrides) for ground point coloring
    terrain_slope = slope_deg.copy()

    # Mark obstacle cells as undrivable on the 2D slope grid
    slope_deg[obstacle_mask] = max_slope

    # Mark enclosed regions as undrivable
    slope_deg = mark_enclosed_regions(slope_deg, max_slope)

    # --- Color every point in the raw point cloud ---
    px, py, pz = points[:, 0], points[:, 1], points[:, 2]
    pi = np.clip(((px - x_centers[0]) / resolution).astype(int), 0, nx - 1)
    pj = np.clip(((py - y_centers[0]) / resolution).astype(int), 0, ny - 1)

    # Height above ground for each point
    ground_z = elevation[pi, pj]
    nan_ground = np.isnan(ground_z)
    ground_z[nan_ground] = pz[nan_ground]
    height_above_ground = pz - ground_z

    # Custom slope colormap: gray -> yellow -> red
    safe_ratio = min(15.0 / max_slope, 0.5)
    slope_cmap = LinearSegmentedColormap.from_list('terrain_slope', [
        (0.0,               (0.55, 0.55, 0.55)),  # gray - safe
        (safe_ratio,        (0.55, 0.55, 0.55)),  # gray up to 15 deg
        (safe_ratio + 0.01, (1.0, 1.0, 0.2)),     # yellow - caution
        (0.65,              (1.0, 0.5, 0.0)),      # orange - risky
        (0.8,               (0.9, 0.1, 0.0)),      # red - dangerous
        (1.0,               (0.5, 0.0, 0.3)),      # dark magenta - cliff
    ])

    # Assign RGBA to every point
    obstacle_color = np.array([0.4, 0.0, 0.5, 1.0])  # dark magenta/purple
    ground_threshold = 0.3  # meters above ground

    is_ground = height_above_ground < ground_threshold
    point_slope = terrain_slope[pi, pj]  # actual terrain slope (not obstacle-overridden)

    # Normalize slope to [0, 1] for colormap lookup
    norm_slope = np.clip(point_slope / max_slope, 0.0, 1.0)
    # Handle NaN slopes (no grid data) — treat as flat
    norm_slope[np.isnan(norm_slope)] = 0.0

    # Map all points through slope colormap, then override obstacles
    rgba = slope_cmap(norm_slope)  # (N, 4)
    rgba[~is_ground] = obstacle_color

    # Build PyVista point cloud with RGBA colors
    pcd = pv.PolyData(points)
    # PyVista expects 0-255 uint8 RGBA
    pcd['rgba'] = (rgba * 255).astype(np.uint8)
    # Also store slope for scalar bar (ground points only, NaN for obstacles)
    point_slope_display = point_slope.copy()
    point_slope_display[~is_ground] = np.nan
    pcd['slope_deg'] = point_slope_display

    # Set up the plotter
    plotter = pv.Plotter(window_size=[1400, 900])
    plotter.set_background('black')

    # Add the colored point cloud
    plotter.add_mesh(
        pcd,
        scalars='rgba',
        rgba=True,
        point_size=2,
        render_points_as_spheres=False,
        show_scalar_bar=False,
    )

    # Add a scalar bar manually using a dummy mesh with the slope colormap
    dummy = pv.PolyData(points[:1])
    dummy['slope_deg'] = np.array([0.0])
    plotter.add_mesh(
        dummy,
        scalars='slope_deg',
        cmap=slope_cmap,
        clim=[0, max_slope],
        point_size=0.001,
        opacity=0.0,
        scalar_bar_args={
            'title': 'Slope (degrees)',
            'color': 'white',
            'title_font_size': 14,
            'label_font_size': 12,
            'n_labels': 6,
        },
    )

    plotter.add_text(
        f"Traversable: {pct:.0f}% | Grid: {nx}x{ny} @ {resolution}m | "
        f"Max slope: {max_slope} deg",
        position='upper_left',
        font_size=10,
        color='white',
    )
    plotter.add_axes()
    plotter.camera.clipping_range = (0.01, 1000.0)
    plotter.enable_fly_to_right_click()
    plotter.show()


def main():
    parser = argparse.ArgumentParser(
        description="Terrain gradient/traversability from LiDAR point clouds"
    )
    parser.add_argument(
        '--ply', type=str, default=None,
        help='Path to a PLY point cloud file (uses synthetic terrain if omitted)'
    )
    parser.add_argument(
        '--resolution', type=float, default=0.2,
        help='Grid cell size in meters (default: 0.2)'
    )
    parser.add_argument(
        '--max-slope', type=float, default=35.0,
        help='Slope angle (deg) that maps to full red (default: 35)'
    )
    parser.add_argument(
        '--show-points', action='store_true',
        help='Overlay raw point cloud on the mesh'
    )
    args = parser.parse_args()

    if args.ply:
        print(f"Loading point cloud from {args.ply}")
        points = load_ply(args.ply)
    else:
        print("Generating synthetic terrain for demo...")
        points = generate_synthetic_terrain()

    analyze_and_visualize(
        points,
        resolution=args.resolution,
        max_slope=args.max_slope,
        show_points=args.show_points,
    )


if __name__ == '__main__':
    main()
