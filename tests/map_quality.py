"""Test-only grid geometry oracle. Nothing here is imported by runtime nodes."""
import numpy as np


def decode_pixels(pixels, metadata):
    """Decode trinary PGM using its YAML, including the unknown-grey boundary."""
    assert metadata['mode'] == 'trinary' and metadata['negate'] == 0
    probability = 1. - np.flipud(np.asarray(pixels, dtype=float))/255.
    return np.where(probability >= metadata['occupied_thresh'], 100,
                    np.where(probability <= metadata['free_thresh'], 0, -1))


def quality(grid, resolution, origin):
    # map begins at the fresh rover's base pose; no best-fit truth alignment.
    yy, xx = np.indices(grid.shape)
    x = origin[0] + (xx + .5)*resolution - 2.5
    y = origin[1] + (yy + .5)*resolution - 1.5
    occupied = np.column_stack((x[grid >= 65], y[grid >= 65]))
    if len(occupied) < 100:
        raise AssertionError('Too few occupied cells to represent the room')
    segments = {
        'south': (-3.94, -2.94, 3.94, -2.94),
        'north': (-3.94, 2.94, 3.94, 2.94),
        'west': (-3.94, -2.94, -3.94, 2.94),
        'east': (3.94, -2.94, 3.94, 2.94),
        'table_south': (-.75, .25, .75, .25),
        'table_north': (-.75, .95, .75, .95),
        'table_west': (-.75, .25, -.75, .95),
        'table_east': (.75, .25, .75, .95),
    }
    distances, coverage = [], {}
    for name, (x1, y1, x2, y2) in segments.items():
        a, b = np.array([x1, y1]), np.array([x2, y2])
        delta = b-a
        fraction = np.clip((occupied-a)@delta / (delta@delta), 0, 1)
        distances.append(np.linalg.norm(occupied-(a+fraction[:, None]*delta), axis=1))
        samples = np.linspace(a, b, int(np.linalg.norm(delta)/resolution)+1)
        nearest = np.linalg.norm(samples[:, None, :]-occupied[None, :, :], axis=2).min(axis=1)
        coverage[name] = float(np.mean(nearest <= .15))
    errors = np.array(distances).min(axis=0)
    interior = (x > -3.84) & (x < 3.84) & (y > -2.84) & (y < 2.84)
    # A laser slice cannot classify the occluded solid interior of the table.
    interior &= ~((x > -.85) & (x < .85) & (y > .15) & (y < 1.05))
    result = {
        'resolution_m': resolution, 'width': grid.shape[1], 'height': grid.shape[0],
        'origin_xy_m': list(origin), 'occupied_cells': len(occupied),
        'free_cells': int(np.sum((grid >= 0) & (grid <= 25))),
        'unknown_cells': int(np.sum(grid < 0)),
        'interior_known_fraction': float(np.mean(grid[interior] >= 0)),
        'interior_free_fraction': float(np.mean((grid[interior] >= 0) & (grid[interior] <= 25))),
        'occupied_surface_rmse_m': float(np.sqrt(np.mean(errors**2))),
        'occupied_surface_p95_m': float(np.quantile(errors, .95)),
        'surface_coverage_within_15cm': coverage,
        'oracle_alignment': 'world_xy = map_xy + (-2.5, -1.5); yaw=0; no fitted alignment',
        'scope': 'M6 wall inner faces and table at the LiDAR plane; low crates excluded',
    }
    assert result['interior_known_fraction'] >= .90, result
    assert result['occupied_surface_p95_m'] <= .15, result
    assert all(coverage[k] >= .90 for k in ('south', 'north', 'west', 'east')), result
    return result
