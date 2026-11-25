"""Patrón de cruces de la bandera, adaptado del aporte detector_señales.py.

Solo geometría; el supervisor filtra QR, confirma imágenes y decide la etapa.
No interpreta este patrón por sí solo como una llegada a A/B.
"""
import cv2
import numpy as np

META_WIDTH = 480
META_MAX_CORNERS = 150
META_QUALITY = 0.012
META_MIN_DISTANCE = 4
META_RADII = (3, 5, 8, 12)
META_SAMPLES = 32
META_MIN_CONTRAST = 65
META_MIN_HARMONIC = 0.67
META_MIN_VALID_RADII = 2
META_MIN_CORNERS = 8
META_NEIGHBOR_RATIO = 1.9
META_ANGLE_TOL = 35

META_ANGLES = np.arange(META_SAMPLES, dtype=np.float32) * (2*np.pi/META_SAMPLES)
META_UNIT = np.stack((np.cos(META_ANGLES), np.sin(META_ANGLES)), axis=1)
META_COS2 = np.cos(2*META_ANGLES)
META_SIN2 = np.sin(2*META_ANGLES)
META_OFFSETS = np.asarray(META_RADII, np.float32)[:, None, None]*META_UNIT[None, :, :]


def META_x_corners(gray):
    corners = cv2.goodFeaturesToTrack(gray, META_MAX_CORNERS, META_QUALITY,
                                     META_MIN_DISTANCE, blockSize=3)
    if corners is None:
        return np.empty((0, 2), np.float32), np.empty(0), np.empty(0)
    pts = corners.reshape(-1, 2)
    h, w = gray.shape
    # Muestreo bilineal vectorizado de todas las esquinas y radios.
    xy = pts[:, None, None, :] + META_OFFSETS[None, :, :, :]
    inside = ((xy[..., 0] >= 0) & (xy[..., 0] < w-1) &
              (xy[..., 1] >= 0) & (xy[..., 1] < h-1)).all(axis=2)
    samples = cv2.remap(gray, xy[..., 0].reshape(-1, META_SAMPLES),
                       xy[..., 1].reshape(-1, META_SAMPLES),
                       cv2.INTER_LINEAR).reshape(len(pts), len(META_RADII), META_SAMPLES)
    samples = samples.astype(np.float32)
    lo, hi = samples.min(axis=2), samples.max(axis=2)
    contrast = hi-lo
    binary = samples > (lo+hi)[..., None]*0.5
    changes = np.count_nonzero(binary != np.roll(binary, 1, axis=2), axis=2)
    # Los cuadrantes opuestos deben tener intensidades similares.
    opposite = np.mean(binary == np.roll(binary, META_SAMPLES//2, axis=2), axis=2)
    duty = binary.mean(axis=2)
    signal = samples - samples.mean(axis=2, keepdims=True)
    real = signal @ META_COS2
    imag = signal @ META_SIN2
    harmonic = np.clip(2*(real**2+imag**2) /
                       (META_SAMPLES*np.sum(signal**2, axis=2)+1e-6), 0, 1)
    valid = (inside & (contrast >= META_MIN_CONTRAST) & (changes == 4) &
             (opposite >= .68) & (duty >= .25) & (duty <= .75) &
             (harmonic >= META_MIN_HARMONIC))
    keep = valid.sum(axis=1) >= META_MIN_VALID_RADII
    score = np.where(valid, harmonic, 0)
    best = score.argmax(axis=1)
    rows = np.arange(len(pts))
    # Direccion de un borde de la X; la otra esta aproximadamente a 90 grados.
    axes = .5*np.arctan2(imag[rows, best], real[rows, best])+np.pi/4
    confidence = score.max(axis=1)
    return pts[keep], axes[keep], confidence[keep]


def META_group_grid(pts, axes):
    """Conectar vecinos coherentes con los ejes locales de cada X."""
    n = len(pts)
    if n < META_MIN_CORNERS:
        return np.empty(0, dtype=int), []
    delta = pts[None, :, :] - pts[:, None, :]
    dist = np.linalg.norm(delta, axis=2)
    np.fill_diagonal(dist, np.inf)
    nearest = dist.min(axis=1)
    angle = np.arctan2(delta[..., 1], delta[..., 0])
    # Distancia angular al eje mas cercano, modulo 90 grados.
    err = np.abs((angle-axes[:, None]+np.pi/4) % (np.pi/2)-np.pi/4)
    aligned = (err < np.deg2rad(META_ANGLE_TOL))
    link = (dist <= META_NEIGHBOR_RATIO*np.maximum(nearest[:, None], nearest[None, :]))
    link &= aligned & aligned.T
    # Conservar solo enlaces cercanos de ambas direcciones.
    graph = [[] for _ in range(n)]
    for i, j in zip(*np.where(np.triu(link, 1))):
        graph[i].append(j)
        graph[j].append(i)
    seen = set()
    best = []
    for start in range(n):
        if start in seen:
            continue
        stack, group = [start], []
        seen.add(start)
        while stack:
            u = stack.pop()
            group.append(u)
            for v in graph[u]:
                if v not in seen:
                    seen.add(v)
                    stack.append(v)
        if len(group) >= META_MIN_CORNERS:
            # Exigir cruces con vecinos en ambos ejes, no una sola cadena.
            junctions = 0
            for u in group:
                directions = []
                for v in graph[u]:
                    d = (angle[u, v]-axes[u]) % np.pi
                    directions.append(int(np.round(d/(np.pi/2))) % 2)
                junctions += len(set(directions)) == 2
            if junctions >= 2 and len(group) > len(best):
                best = group
    ids = np.asarray(best, dtype=int)
    links = [(i, j) for i in best for j in graph[i] if i < j]
    return ids, links


def detect_meta(gray):
    """Entrada gris; devuelve puntos, grupo, enlaces y confianza."""
    pts, axes, scores = META_x_corners(gray)
    group, links = META_group_grid(pts, axes)
    confidence = float(scores[group].mean()) if len(group) else 0.
    return pts, group, links, confidence


