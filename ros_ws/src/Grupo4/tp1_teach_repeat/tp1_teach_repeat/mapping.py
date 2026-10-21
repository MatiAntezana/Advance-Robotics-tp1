"""Grilla derivada para presentación; conserva intactos los scans y sus poses."""
import numpy as np
from .core import transform_points


def reference_grid(frames, resolution=.04):
    if resolution <= 0:
        raise ValueError('La resolución debe ser positiva')
    scans = [(np.asarray(f['pose']), transform_points(f['points'], f['pose'])[:, :2])
             for f in frames if len(f['points']) >= 20]
    if not scans:
        return None
    points = np.concatenate([p for _, p in scans] + [np.array([p[:2] for p, _ in scans])])
    origin = np.floor((points.min(axis=0)-.20)/resolution)*resolution
    width, height = (np.ceil((points.max(axis=0)+.20-origin)/resolution).astype(int)+1)
    if width*height > 4_000_000:
        raise ValueError('Mapa demasiado grande: aumentar map_resolution')
    hits = np.zeros(width*height, dtype=np.int32)
    misses = np.zeros_like(hits)
    for pose, cloud in scans:
        start = np.floor((pose[:2]-origin)/resolution).astype(int)
        cells = np.floor((cloud-origin)/resolution).astype(int)
        endpoints = np.unique(cells[:, 1]*width+cells[:, 0])
        rays = []
        for end in cells:
            count = int(np.max(np.abs(end-start)))
            if count:
                ray = np.rint(np.linspace(start, end, count+1)).astype(int)[:-1]
                rays.append(ray[:, 1]*width+ray[:, 0])
        # Cada keyframe vota una vez por celda, no una vez por rayo.
        free = np.setdiff1d(np.unique(np.concatenate(rays)), endpoints) if rays else []
        hits[endpoints] += 1
        misses[free] += 1
    evidence = .85*hits - .40*misses
    grid = np.full(width*height, -1, dtype=np.int8)
    grid[evidence < -.3] = 0
    # Con deriva histórica, rayos de otras poses pueden cruzar una superficie
    # real. Conservar impactos repetidos: la vista no debe borrar esos obstáculos.
    grid[hits >= min(2, len(scans))] = 100
    return grid.reshape(height, width), origin
