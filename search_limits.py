"""Shared hard limits for the adaptive search session and its view history."""

MAX_OBSERVATIONS = 16
SEARCH_RADIUS_M = 30.
GRID_RESOLUTION_M = 1.


def grid_dimensions(radius, resolution):
    """Bounded current grid plus the historical replay profile; no arbitrary allocation."""
    profiles = {(12., .5): 48, (SEARCH_RADIUS_M, GRID_RESOLUTION_M): 60}
    if type(radius) not in (int,float) or type(resolution) not in (int,float):
        raise ValueError('invalid visibility grid profile')
    side = profiles.get((radius,resolution))
    if side is None:
        raise ValueError('unsupported visibility grid profile')
    return side,side


def validate_grid(grid):
    rows,cols = grid_dimensions(grid.get('radius_m'),grid.get('resolution_m'))
    if (type(grid.get('rows')) is not int or type(grid.get('cols')) is not int
            or grid['rows'] != rows or grid['cols'] != cols):
        raise ValueError('grid dimensions do not match its bounded physical profile')
    return rows*cols
