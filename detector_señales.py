"""Camara: Canny -> detector de circulos interiores -> flechas por segmentos.
Uso: python detector_integrado_camara.py [--camera 0] [--mirror] [--width 640]
         [--diameter 55] [--arrow-r 0] [--early-k 0] [--track] [--vote] [--confirm 2]
         [--show-edges]
Dependencias: pip install opencv-python numpy scipy

--diameter : diametro minimo de circulo en px DEL ANCHO DE TRABAJO (55 a 640 px).
             Al bajar --width escalar proporcionalmente (480 -> 41, 384 -> 33).
--arrow-r  : (opt-in, 0=off) si el radio del circulo supera este valor, la etapa de
             flecha trabaja sobre el recorte reescalado a ese radio (recalcula Canny).
--early-k  : (opt-in, 0=off) rechaza el circulo sin Hough si el ROI tiene menos de
             k*radio pixeles de borde.
--track    : (opt-in) asocia circulos entre frames y filtra (cx, cy, r) con Kalman; si el
             detector falla, sostiene el circulo hasta TRACK_MAX_MISS frames por prediccion.
--vote     : (opt-in) voto temporal de la direccion por track (VOTE_ALPHA, VOTE_THRESH).
--confirm  : frames necesarios para confirmar un track (rechaza falsos positivos de 1 frame).
"""
import argparse
import time
import math
import json
from concurrent.futures import ThreadPoolExecutor
import cv2
import numpy as np
from scipy.spatial import cKDTree

GAUSSIAN_KERNEL = 5
GAUSSIAN_SIGMA = 1.0
CANNY_LOW = 50
CANNY_HIGH = 130
CIRCLE_CANNY_LOW = 65
CIRCLE_CANNY_HIGH = 150
CLOSE_ITER = 0
OPEN_ITER = 0
ROI_FACTOR = 0.80
COLOR = 'azul'  # 'rojo' o 'azul'
QR_EVERY = 3
QR_WIDTH = 480
QR_TTL = 0.35
COLOR_MIN_FRACTION = 0.50
WHITE_MIN_FRACTION = 0.55
COLOR_SAMPLE_FACTOR = 0.92

# HSV de OpenCV: H en [0,179], S y V en [0,255].
RED_H_MAX = 12
RED_H_MIN = 168
BLUE_H_MIN = 90
BLUE_H_MAX = 135
COLOR_MIN_S = 75
COLOR_MIN_V = 55
WHITE_MAX_S = 55
WHITE_MIN_V = 125

DIAMETER = 55
MAX_OBJECTS = 1
MAX_RATIO = 4.0
USE_HOUGH = False

# Agrupación de duplicados
MIN_RADIUS_RATIO = 0.70
MAX_CENTER_OFFSET = 0.10
MIN_SCORE_RATIO = 0.70

# Búsqueda del círculo interior
INNER_MIN_RATIO = 0.70
INNER_MAX_RATIO = 0.96
INNER_MIN_SUPPORT = 0.55
INNER_MIN_COVERAGE = 0.80
INNER_CENTER_SEARCH = 4

MIN_CONFIDENCE = 0.40
STOP_MIN_DOMINANCE = .68
STOP_MAX_PERP_RATIO = .32
STOP_MIN_SPAN = 1.05
STOP_MIN_WIDTH = .10
STOP_MAX_WIDTH = .65
STOP_CLASSES = ('IZQUIERDA', 'DERECHA', 'STOP')
ANGLE_TOL = 18
JOIN_RATIO = 0.085

DEBUG = False

# Seguimiento / voto temporal (opt-in: --track, --vote)
TRACK_GATE = 0.5          # distancia maxima de asociacion, en radios
TRACK_RATIO_MIN = 0.6     # cociente minimo de radios para asociar
TRACK_MAX_MISS = 5        # frames sin asociacion antes de descartar el track
TRACK_MEAS_VAR = 4.0      # varianza de medida (px^2)
TRACK_Q_POS = 8.0         # ruido de proceso en cx, cy (generoso por ego-motion)
TRACK_Q_RAD = 1.0         # ruido de proceso en r
TRACK_Q_VEL = 4.0         # ruido de proceso en velocidades
VOTE_ALPHA = 0.7          # memoria del voto (mayor = mas inercia)
VOTE_THRESH = 0.30        # |voto| minimo para emitir direccion
COS30 = math.cos(math.radians(30))
COS32 = math.cos(math.radians(32))

# Constantes precalculadas
_K3 = np.ones((3, 3), np.uint8)
_CROSS5 = cv2.getStructuringElement(cv2.MORPH_CROSS, (5, 5))
_ELLIPSE3 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
_CLAHE = cv2.createCLAHE(clipLimit=2., tileGridSize=(8, 8))

_ANG = np.arange(144, dtype=np.float32) * (2 * np.pi / 144)
_COS, _SIN = np.cos(_ANG), np.sin(_ANG)
_OFFS = np.array([(dx, dy)
                  for dy in range(-INNER_CENTER_SEARCH, INNER_CENTER_SEARCH + 1, 2)
                  for dx in range(-INNER_CENTER_SEARCH, INNER_CENTER_SEARCH + 1, 2)],
                 np.float32)
_PEN = .002 * np.hypot(_OFFS[:, 0], _OFFS[:, 1])
_OFFS_I = _OFFS.astype(np.int32)


def canny_pipeline(frame, gray=None, smooth=None):
    if gray is None:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    if smooth is None:
        smooth = cv2.GaussianBlur(gray, (GAUSSIAN_KERNEL, GAUSSIAN_KERNEL), GAUSSIAN_SIGMA)
    edges = cv2.Canny(smooth, CANNY_LOW, CANNY_HIGH)
    if CLOSE_ITER:
        edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, _K3, iterations=CLOSE_ITER)
    if OPEN_ITER:
        edges = cv2.morphologyEx(edges, cv2.MORPH_OPEN, _K3, iterations=OPEN_ITER)
    return edges, gray


# ============================================================
# ERROR GEOMÉTRICO DE ELIPSE
# ============================================================

def ellipse_error(contour, ellipse):
    pts = contour.reshape(-1, 2)
    if len(pts) > 120:
        pts = pts[::max(1, len(pts) // 120)]

    (cx, cy), (w, h), angle = ellipse
    if min(w, h) < 1:
        return 1.0, 1.0, 0.0

    xy = pts.astype(np.float32) - np.array((cx, cy), np.float32)
    th = math.radians(angle)
    ca, sa = math.cos(th), math.sin(th)
    u = (ca * xy[:, 0] + sa * xy[:, 1]) / (w * 0.5)
    v = (-sa * xy[:, 0] + ca * xy[:, 1]) / (h * 0.5)

    residual = np.abs(np.sqrt(u * u + v * v) - 1.0)
    bins = np.floor(((np.arctan2(v, u) + math.pi) / (2 * math.pi)) * 24).astype(np.int32) % 24
    coverage = np.unique(bins).size / 24.0
    return float(np.median(residual)), float(np.quantile(residual, 0.85)), coverage


# ============================================================
# SOPORTE CIRCULAR DE BORDES (solo Hough opcional)
# ============================================================

def circ_edge_support(edges, x, y, r, n=144):
    if r < 2:
        return 0.0, 0.0
    angles = np.arange(n, dtype=np.float32) * (2 * np.pi / n)
    xx = np.rint(x + r * np.cos(angles)).astype(np.int32)
    yy = np.rint(y + r * np.sin(angles)).astype(np.int32)
    H, W = edges.shape
    good = (xx >= 0) & (xx < W) & (yy >= 0) & (yy < H)
    if good.sum() < n * 0.80:
        return 0.0, 0.0

    hits = np.zeros(n, dtype=bool)
    ind = np.nonzero(good)[0]
    for dx, dy in ((0, 0), (1, 0), (-1, 0), (0, 1), (0, -1),
                   (2, 0), (-2, 0), (0, 2), (0, -2)):
        sx, sy = xx[ind] + dx, yy[ind] + dy
        inside = (sx >= 0) & (sx < W) & (sy >= 0) & (sy < H)
        hits[ind[inside]] |= edges[sy[inside], sx[inside]] > 0

    coverage = sum(np.any(c) for c in np.array_split(hits, 12)) / 12.0
    return float(hits[good].mean()), float(coverage)


# ============================================================
# AGRUPACIÓN DE DUPLICADOS
# ============================================================

def ellipse_radius(candidate):
    (_, _), (w, h), _ = candidate["ellipse"]
    return max(w, h) / 2.0


def duplicate(a, b):
    (ax, ay), _, _ = a["ellipse"]
    (bx, by), _, _ = b["ellipse"]
    ra, rb = ellipse_radius(a), ellipse_radius(b)
    if min(ra, rb) < 1:
        return False
    return (math.hypot(ax - bx, ay - by) <= MAX_CENTER_OFFSET * max(ra, rb)
            and min(ra, rb) / max(ra, rb) >= MIN_RADIUS_RATIO)


def select_best_duplicate(group):
    best = max(group, key=lambda d: d["score"])
    (bx, by), _, _ = best["ellipse"]
    best_radius = ellipse_radius(best)
    best_score = best["score"]
    score_margin = max(abs(best_score), 0.1) * (1.0 - MIN_SCORE_RATIO)

    valid = []
    for candidate in group:
        (cx, cy), _, _ = candidate["ellipse"]
        radius = ellipse_radius(candidate)
        if radius < best_radius * MIN_RADIUS_RATIO:
            continue
        if radius > best_radius * 1.02:
            continue
        if math.hypot(cx - bx, cy - by) > best_radius * MAX_CENTER_OFFSET:
            continue
        if candidate["score"] < best_score - score_margin:
            continue
        valid.append(candidate)

    if not valid:
        return best
    return min(valid, key=lambda d: (ellipse_radius(d), -d["score"]))


def select_unique_candidates(candidates, max_objects):
    ordered = sorted(candidates, key=lambda d: d["score"], reverse=True)
    groups = []
    for candidate in ordered:
        for group in groups:
            if duplicate(candidate, group[0]):
                group.append(candidate)
                break
        else:
            groups.append([candidate])

    selected = []
    for group in groups:
        chosen = select_best_duplicate(group)
        selected.append(chosen)
        if DEBUG and len(group) > 1:
            print(f"Grupo de {len(group)} duplicados: radio elegido={ellipse_radius(chosen):.1f}")
    return selected[:max_objects]


# ============================================================
# BÚSQUEDA DEL CÍRCULO INTERIOR
# ============================================================

def find_inner_circle(expanded, ellipse, relaxed=False):
    """expanded = cv2.dilate(edges, cruz 5x5), calculado una vez por frame.
    Vectorizado sobre los 25 centros x todos los radios; desempate idéntico
    al bucle original (centro dy-externo/dx-interno, luego radio menor)."""
    (cx, cy), (w, h), _ = ellipse
    if max(w, h) / max(1, min(w, h)) > 1.20:
        return None

    R = (w + h) / 4.0
    min_r = max(5, int(R * (.60 if relaxed else INNER_MIN_RATIO)))
    max_r = int(R * (.90 if relaxed else INNER_MAX_RATIO))
    if max_r <= min_r:
        return None

    H, W = expanded.shape
    radii = np.arange(min_r, max_r + 1, dtype=np.float32)
    # Los desplazamientos de centro son enteros pares: rint(c + d + x) == rint(c + x) + d,
    # asi que las posiciones de muestreo se calculan una vez y los 25 centros son sumas enteras.
    xb = np.rint(cx + radii[:, None] * _COS).astype(np.int32)                 # (R,144)
    yb = np.rint(cy + radii[:, None] * _SIN).astype(np.int32)
    m = INNER_CENTER_SEARCH
    if xb.min() - m >= 0 and xb.max() + m < W and yb.min() - m >= 0 and yb.max() + m < H:
        shift = _OFFS_I[:, 1] * W + _OFFS_I[:, 0]
        idx = (yb * W + xb)[None] + shift[:, None, None]                         # (C,R,144)
        hits = expanded.ravel()[idx] > 0
        count = 144
    else:
        xx = xb[None] + _OFFS_I[:, 0, None, None]
        yy = yb[None] + _OFFS_I[:, 1, None, None]
        good = (xx >= 0) & (xx < W) & (yy >= 0) & (yy < H)
        hits = good & (expanded[np.clip(yy, 0, H - 1), np.clip(xx, 0, W - 1)] > 0)
        count = good.sum(2)
    support = hits.sum(2) / np.maximum(count, 1)
    sectors = hits.reshape(len(_OFFS), len(radii), 12, 12).any(3)
    coverage = sectors.mean(2)
    scores = .85 * support + .15 * coverage - _PEN[:, None]
    valid = ((count >= 144 * .80) & (coverage >= (.66 if relaxed else INNER_MIN_COVERAGE))
             & (support >= (.48 if relaxed else INNER_MIN_SUPPORT)))
    if relaxed:
        # Una barra puede romper arcos opuestos, pero debe quedar evidencia
        # circular distribuida en los cuatro cuadrantes, no solo una recta.
        valid &= sectors.reshape(len(_OFFS), len(radii), 4, 3).any(3).all(2)
    scores = np.where(valid, scores, -np.inf)

    c, k = np.unravel_index(int(np.argmax(scores)), scores.shape)
    if scores[c, k] == -np.inf:
        return None

    r = float(radii[k])
    if DEBUG:
        print(f"Circulo interior encontrado: radio={r:.1f}, score={scores[c, k]:.3f}")
    return ((cx + float(_OFFS[c, 0]), cy + float(_OFFS[c, 1])), (2.0 * r, 2.0 * r), 0.0)


# ============================================================
# DETECTOR DE CÍRCULOS
# ============================================================

class Detector:
    def __init__(self, diameter=55, max_objects=5, hough=False, max_ratio=4.0):
        self.diameter = diameter
        self.max_objects = max_objects
        self.hough = hough
        self.max_ratio = max_ratio

    def detect(self, frame, roi=None, external_edges=None):
        H0, W0 = frame.shape[:2]
        ox = oy = 0

        if roi is not None:
            rx, ry, rw, rh = roi
            ox, oy = max(0, int(rx)), max(0, int(ry))
            xe, ye = min(W0, int(rx + rw)), min(H0, int(ry + rh))
            if xe - ox < self.diameter or ye - oy < self.diameter:
                return []
            frame = frame[oy:ye, ox:xe]

        # gray/blur solo si se necesitan (Canny interno o Hough)
        gray = smooth = None
        if external_edges is None or self.hough:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            smooth = cv2.GaussianBlur(gray, (5, 5), 1.05)
        edges = external_edges if external_edges is not None else cv2.Canny(smooth, CIRCLE_CANNY_LOW, CIRCLE_CANNY_HIGH)

        contours, _ = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        candidates = []
        d = self.diameter
        min_area = math.pi * (d / 2) ** 2 * 0.35
        Hf, Wf = frame.shape[:2]

        for c in contours:
            if len(c) < 16:
                continue
            x, y, bw, bh = cv2.boundingRect(c)
            if min(bw, bh) < d * 0.57 or max(bw, bh) < d:
                continue
            if max(bw, bh) / max(1, min(bw, bh)) > self.max_ratio * 1.4:
                continue

            area = abs(cv2.contourArea(c))
            if area < min_area or area < bw * bh * 0.30:
                continue

            try:
                ell = cv2.fitEllipse(c)
            except cv2.error:
                continue

            (cx, cy), (ew, eh), angle = ell
            if not np.isfinite([cx, cy, ew, eh, angle]).all():
                continue
            if min(ew, eh) < d / self.max_ratio or max(ew, eh) < d:
                continue
            if max(ew, eh) / max(1, min(ew, eh)) > self.max_ratio:
                continue
            if cx < -ew * 0.1 or cy < -eh * 0.1 or cx > Wf + ew * 0.1 or cy > Hf + eh * 0.1:
                continue

            ell_area = math.pi * ew * eh * 0.25
            if not (0.62 < area / max(ell_area, 1.0) < 1.2):
                continue

            med, q85, cov = ellipse_error(c, ell)
            if med > 0.048 or q85 > 0.105 or cov < 0.66:
                continue

            quality = cov - 2 * med - 1.3 * q85
            score = quality + min(0.25, math.log1p(ell_area) * 0.020)
            candidates.append({"ellipse": ((cx + ox, cy + oy), (ew, eh), angle),
                               "score": score, "kind": "contour"})

        # ---- Hough opcional ----
        if self.hough:
            circles = cv2.HoughCircles(
                smooth, cv2.HOUGH_GRADIENT_ALT, dp=1.5,
                minDist=max(42, int(d * 0.85)), param1=150, param2=0.68,
                minRadius=max(10, d // 2), maxRadius=int(min(gray.shape) * 0.48))
            if circles is not None:
                for x, y, r in circles[0][:25]:
                    trials = [(circ_edge_support(edges, x, y, r * f), r * f)
                              for f in (0.90, 0.95, 1.0, 1.05, 1.10)]
                    (support, coverage), r = max(
                        trials, key=lambda z: z[0][0] + 0.08 * z[0][1])
                    if support < 0.58 or coverage < 0.75:
                        continue
                    candidates.append({
                        "ellipse": ((float(x + ox), float(y + oy)),
                                    (float(2 * r), float(2 * r)), 0.0),
                        "score": 0.75 + 0.25 * support + 0.10 * coverage,
                        "kind": "hough"})

        selected = select_unique_candidates(candidates, self.max_objects)
        if not selected:
            return []

        # ---- refinamiento al círculo interior (dilatación una vez por frame) ----
        expanded = cv2.dilate(edges, _CROSS5)
        refined = []
        for candidate in selected:
            (cx, cy), axes, angle = candidate["ellipse"]
            inner = find_inner_circle(expanded, ((cx - ox, cy - oy), axes, angle))
            if inner is None:
                inner = find_inner_circle(expanded, ((cx - ox, cy - oy), axes, angle), relaxed=True)
            if inner is not None:
                (ix, iy), inner_axes, inner_angle = inner
                candidate = candidate.copy()
                candidate["ellipse"] = ((ix + ox, iy + oy), inner_axes, inner_angle)
                candidate["kind"] = "inner"
            refined.append(candidate)
        return refined


# ============================================================
# DETECTOR DE FLECHA (opera sobre bordes ya enmascarados al círculo)
# ============================================================

def unit(v):
    return v / max(math.hypot(v[0], v[1]), 1e-9)


def angle_diff(a, b):
    d = np.abs(a - b) % 180
    return np.minimum(d, 180 - d)


def cross(a, b):
    return float(a[0] * b[1] - a[1] * b[0])


def segments_from_edges(edges, radius):
    minimum = max(5, round(radius * .055))
    lines = cv2.HoughLinesP(edges, 1, np.pi / 180,
                            threshold=max(6, min(15, round(radius * .13))),
                            minLineLength=minimum,
                            maxLineGap=max(4, round(radius * .08)))
    if lines is None:
        return np.empty((0, 2, 2), np.float32)
    s = lines.reshape(-1, 2, 2).astype(np.float32)
    lengths = np.linalg.norm(s[:, 1] - s[:, 0], axis=1)
    return s[lengths >= minimum]


def principal_directions(seg):
    """Eje dominante ponderado por longitud + eje exactamente perpendicular."""
    delta = seg[:, 1] - seg[:, 0]
    angles = np.degrees(np.arctan2(delta[:, 1], delta[:, 0])) % 180
    lengths = np.linalg.norm(delta, axis=1)
    hist = np.bincount(np.rint(angles).astype(int) % 180, weights=lengths, minlength=180)
    ext = np.concatenate((hist[-ANGLE_TOL:], hist, hist[:ANGLE_TOL]))
    scores = np.zeros(180, dtype=float)
    for k in range(-ANGLE_TOL, ANGLE_TOL + 1):          # roll(hist,k)[i] == ext[ANGLE_TOL+i-k]
        scores += ext[ANGLE_TOL - k:ANGLE_TOL - k + 180] * (1 - abs(k) / (ANGLE_TOL + 1))
    scores += .65 * np.concatenate((scores[90:], scores[:90]))
    first = int(np.argmax(scores))
    second = (first + 90) % 180
    diffs = np.column_stack((angle_diff(angles, first), angle_diff(angles, second)))
    owner = np.argmin(diffs, axis=1)
    groups = [(owner == i) & (diffs[:, i] <= ANGLE_TOL) for i in range(2)]
    return (first, second), groups, lengths


def bands(seg, direction, radius):
    v = np.array([np.cos(np.deg2rad(direction)), np.sin(np.deg2rad(direction))])
    n = np.array([-v[1], v[0]])
    offsets = seg.mean(axis=1) @ n
    order = np.argsort(offsets)
    clusters = []
    tol = max(1.5, .035 * radius)
    for idx in order:
        if not clusters or offsets[idx] - offsets[clusters[-1][-1]] > tol:
            clusters.append([idx])
        else:
            clusters[-1].append(idx)
    out = []
    for ids in clusters:
        ss = seg[ids]
        lengths = np.linalg.norm(ss[:, 1] - ss[:, 0], axis=1)
        t = ss.reshape(-1, 2) @ v
        out.append(dict(offset=np.average(offsets[ids], weights=lengths),
                        length=lengths.sum(), lo=t.min(), hi=t.max()))
    pairs = []
    for i, a in enumerate(out):
        for b in out[i + 1:]:
            width = abs(a['offset'] - b['offset'])
            overlap = min(a['hi'], b['hi']) - max(a['lo'], b['lo'])
            if not (.045 * radius <= width <= .50 * radius and overlap >= -.06 * radius):
                continue
            support = (a['length'] + b['length']) * (
                .5 + .5 * min(a['length'], b['length']) / max(a['length'], b['length']))
            pairs.append(dict(sides=(a['offset'], b['offset']),
                              offset=(a['offset'] + b['offset']) / 2,
                              lo=min(a['lo'], b['lo']), hi=max(a['hi'], b['hi']),
                              support=support, v=v, n=n))
    return sorted(pairs, key=lambda x: -x['support'])[:5]


def _pairs(points, r):
    """Pares (i<j) de puntos a distancia <= r, como arrays."""
    pr = cKDTree(points).query_pairs(r, output_type='ndarray')
    return pr[:, 0], pr[:, 1]


def candidate_tips(seg, dirs, radius):
    """Busca puntas en segmentos originales y cadenas unidas, sin exigir cierre.
    Versión vectorizada: misma lógica y mismos filtros que el bucle original."""
    seg = seg.astype(np.float64)
    gap_limit = max(16., JOIN_RATIO * radius)

    # --- reconstruir tramos quebrados: unir extremos cercanos y casi colineales ---
    all_seg = seg
    if len(seg) > 1:
        ends = seg.reshape(-1, 2)
        i, j = _pairs(ends, gap_limit)
        si, sj = i >> 1, j >> 1
        m = si != sj
        i, j, si, sj = i[m], j[m], si[m], sj[m]
        if len(i):
            far1, far2 = seg[si, 1 - (i & 1)], seg[sj, 1 - (j & 1)]
            v1, v2 = ends[i] - far1, far2 - ends[j]
            n1, n2 = np.hypot(v1[:, 0], v1[:, 1]), np.hypot(v2[:, 0], v2[:, 1])
            ok = np.minimum(n1, n2) >= 3
            den = np.where(ok, n1 * n2, 1.0)
            ok &= np.abs((v1 * v2).sum(1) / den) >= COS30
            d12 = far2 - far1
            ok &= np.hypot(d12[:, 0], d12[:, 1]) <= .65 * radius
            if ok.any():
                all_seg = np.concatenate((seg, np.stack((far1[ok], far2[ok]), axis=1)))

    d = all_seg[:, 1] - all_seg[:, 0]
    lengths = np.hypot(d[:, 0], d[:, 1])
    ids = np.where((lengths >= .07 * radius) & (lengths <= .55 * radius))[0]
    if len(ids) > 180:
        ids = ids[np.argsort(lengths[ids])[-180:]]
    valid = all_seg[ids]
    if len(valid) < 2:
        return []
    endpoints = valid.reshape(-1, 2)
    i, j = _pairs(endpoints, gap_limit)
    si, sj = i >> 1, j >> 1
    keep = si != sj
    i, j, si, sj = i[keep], j[keep], si[keep], sj[keep]
    if not len(i):
        return []

    ei, ej = endpoints[i], endpoints[j]
    gap = np.hypot(ei[:, 0] - ej[:, 0], ei[:, 1] - ej[:, 1])
    # cada par de segmentos se evalua solo con sus extremos mas cercanos
    A, B = valid[si], valid[sj]                                        # (m,2,2)
    dd = np.hypot(A[:, :, None, 0] - B[:, None, :, 0], A[:, :, None, 1] - B[:, None, :, 1])
    keep = gap <= dd.min(axis=(1, 2)) + 1e-4
    i, j, si, sj, ei, ej, gap = (x[keep] for x in (i, j, si, sj, ei, ej, gap))
    if not len(i):
        return []

    tip = (ei + ej) * .5
    a = valid[si, 1 - (i & 1)] - tip
    b = valid[sj, 1 - (j & 1)] - tip
    la, lb = np.hypot(a[:, 0], a[:, 1]), np.hypot(b[:, 0], b[:, 1])
    keep = np.minimum(la, lb) >= .07 * radius
    tip, a, b, la, lb, gap = (x[keep] for x in (tip, a, b, la, lb, gap))
    if not len(tip):
        return []

    ang = np.degrees(np.arccos(np.clip((a * b).sum(1) / (la * lb), -1, 1)))
    keep = (ang >= 30) & (ang <= 115)
    tip, a, b, la, lb, gap, ang = (x[keep] for x in (tip, a, b, la, lb, gap, ang))
    if not len(tip):
        return []

    s = a / la[:, None] + b / lb[:, None]
    inward = s / np.maximum(np.hypot(s[:, 0], s[:, 1]), 1e-9)[:, None]
    ca = inward[:, 0] * a[:, 1] - inward[:, 1] * a[:, 0]
    cb = inward[:, 0] * b[:, 1] - inward[:, 1] * b[:, 0]
    heading = -inward
    align = np.zeros(len(tip))
    for t in dirs:
        align = np.maximum(align, np.abs(heading[:, 0] * math.cos(math.radians(t))
                                         + heading[:, 1] * math.sin(math.radians(t))))
    ab = a - b
    keep = ((np.hypot(inward[:, 0], inward[:, 1]) >= .1) & (ca * cb < 0)
            & (align >= COS32) & (np.hypot(ab[:, 0], ab[:, 1]) >= .07 * radius))
    tip, heading, la, lb, gap, ang, align = (
        x[keep] for x in (tip, heading, la, lb, gap, ang, align))
    if not len(tip):
        return []

    lmin = np.minimum(la, lb)
    score = (.25 * lmin / np.maximum(la, lb) + .30 * align
             + .20 * np.exp(-.5 * ((ang - 65) / 30) ** 2)
             + .15 * np.exp(-gap / max(gap_limit / 2, 1))
             + .10 * np.minimum(1., lmin / (.20 * radius)))
    order = np.argsort(-score, kind='stable')

    thr = .06 * radius
    pts = tip[order].tolist()
    chosen, taken = [], []
    for k, (x, y) in enumerate(pts):
        if all(math.hypot(x - ux, y - uy) > thr for ux, uy in taken):
            taken.append((x, y))
            chosen.append(order[k])
            if len(chosen) >= 20:
                break
    return [dict(p=tip[k], heading=heading[k], score=float(score[k])) for k in chosen]


def detect_stop(center, radius, seg, dirs, groups, lengths):
    """Barra centrada: dos bordes paralelos que atraviesan la ROI.

    Solo reutiliza segmentos Hough y sus pesos; no ejecuta otro detector.
    """
    support = [lengths[g].sum() for g in groups]
    idx = int(np.argmax(support))
    dominant = float(support[idx])
    concentration = dominant / max(float(lengths.sum()), 1e-9)
    perp_ratio = float(support[1 - idx]) / max(dominant, 1e-9)
    if concentration < STOP_MIN_DOMINANCE or perp_ratio > STOP_MAX_PERP_RATIO:
        return None
    # Ajustar el eje al promedio axial de los segmentos dominantes.
    ss = seg[groups[idx]] - center
    weights = lengths[groups[idx]]
    if len(ss) < 2:
        return None
    delta = ss[:, 1] - ss[:, 0]
    angles = np.arctan2(delta[:, 1], delta[:, 0])
    theta = .5 * np.arctan2(np.sum(weights * np.sin(2 * angles)),
                            np.sum(weights * np.cos(2 * angles)))
    v = np.array([np.cos(theta), np.sin(theta)])
    n = np.array([-v[1], v[0]])
    offsets = ss.mean(axis=1) @ n
    order = np.argsort(offsets)
    clusters = []
    tol = max(2., .035 * radius)
    for j in order:
        if not clusters or offsets[j] - offsets[clusters[-1][0]] > tol:
            clusters.append([j])
        else:
            clusters[-1].append(j)
    edges = []
    for ids in clusters:
        off = float(np.average(offsets[ids], weights=weights[ids]))
        spans = np.sort(ss[ids] @ v, axis=1)
        # Fusionar intervalos: no contar lineas Hough superpuestas dos veces.
        intervals = sorted(spans.tolist())
        lo, hi = intervals[0]
        covered = 0.
        for a, b in intervals[1:]:
            if a > hi + .07 * radius:
                covered += hi - lo
                lo, hi = a, b
            else:
                hi = max(hi, b)
        covered += hi - lo
        left, right = float(spans.min()), float(spans.max())
        if covered >= .85 * radius and left <= -.45 * radius and right >= .45 * radius:
            edges.append((off, left, right, covered))
    best = None
    for i, a in enumerate(edges):
        for b in edges[i + 1:]:
            width = (b[0] - a[0]) / radius
            midpoint = abs(a[0] + b[0]) / (2 * radius)
            overlap = (min(a[2], b[2]) - max(a[1], b[1])) / radius
            if not (STOP_MIN_WIDTH <= width <= STOP_MAX_WIDTH and midpoint <= .16
                    and a[0] < 0 < b[0] and overlap >= STOP_MIN_SPAN):
                continue
            score = (.45 * concentration + .30 * min(1., overlap / 1.6)
                     + .25 * max(0., 1. - midpoint / .16))
            if best is None or score > best['confidence']:
                best = dict(direction='STOP', confidence=float(score),
                            dominance=concentration, perpendicular_ratio=perp_ratio)
    return best


def detect(im, segments=None, allow_stop=True, stop_only=False):
    center, radius, edges = im
    if edges is None:
        return None, None, None, None
    seg = segments_from_edges(edges, radius) if segments is None else segments
    if len(seg) < 2:
        return None, seg, None, None
    dirs, groups, lengths = principal_directions(seg)
    if allow_stop:
        stop = detect_stop(center, radius, seg, dirs, groups, lengths)
        if stop is not None:
            return stop, seg, dirs, groups
    if stop_only:
        return None, seg, dirs, groups
    if len(seg) < 5:
        return None, seg, dirs, groups
    support = [lengths[g].sum() for g in groups]
    if min(support) < .15 * radius:
        return None, seg, dirs, groups
    tip_candidates = candidate_tips(seg, dirs, radius)
    if not tip_candidates:
        return None, seg, dirs, groups
    bs = [bands(seg[g], dirs[k], radius) for k, g in enumerate(groups)]
    dvec = [(math.cos(math.radians(t)), math.sin(math.radians(t))) for t in dirs]
    best = None
    for tip in tip_candidates:
        # Poda exacta: tip_candidates está ordenado por score desc y
        # score_final <= .35*tip_score + .65 (el factor de protrusión es <= 1).
        if best is not None and .35 * tip['score'] + .65 <= best['confidence']:
            break
        heading = tip['heading']
        idx = 0 if (abs(heading[0] * dvec[0][0] + heading[1] * dvec[0][1])
                    >= abs(heading[0] * dvec[1][0] + heading[1] * dvec[1][1])) else 1
        tip_proj = float(np.dot(tip['p'], heading))
        for head in bs[idx]:
            hv, hn = head['v'], head['n']
            hv_h = float(np.dot(hv, heading))
            neck_front = max(float(head['lo'] * hv_h), float(head['hi'] * hv_h))
            protrusion = tip_proj - neck_front
            if protrusion < -0.10 * radius or protrusion > 0.85 * radius:
                continue
            if abs(float(np.dot(tip['p'], hn) - head['offset'])) > .22 * radius:
                continue
            for stem in bs[1 - idx]:
                sv = stem['v']
                det = -hv[0] * sv[1] + sv[0] * hv[1]
                if abs(det) < .2:
                    continue
                rhs = stem['n'] * stem['offset'] - hn * head['offset']
                t = (-rhs[0] * sv[1] + sv[0] * rhs[1]) / det
                bend = t * hv + hn * head['offset']
                to_tip = tip['p'] - bend
                if math.hypot(to_tip[0], to_tip[1]) < .12 * radius \
                        or np.dot(unit(to_tip), heading) < .55:
                    continue
                th, ts = bend @ hv, bend @ sv
                ext = .40 * radius
                if not (head['lo'] - ext <= th <= head['hi'] + ext
                        and stem['lo'] - ext <= ts <= stem['hi'] + ext):
                    continue
                end = max((bend + (stem[k] - ts) * sv for k in ('lo', 'hi')),
                          key=lambda p: np.linalg.norm(p - bend))
                if np.linalg.norm(end - bend) < .12 * radius:
                    continue
                u, w = bend - end, tip['p'] - bend
                turn = np.degrees(np.arccos(np.clip(np.dot(unit(u), unit(w)), -1, 1)))
                if not 35 <= turn <= 145:
                    continue
                score = (.35 * tip['score'] + .25 * min(1, head['support'] / (2 * radius))
                         + .25 * min(1, stem['support'] / (2 * radius))
                         + .15 * np.exp(-.5 * ((turn - 90) / 35) ** 2))
                score *= .80 + .20 * min(1., max(0., protrusion) / (.18 * radius))
                if best is None or score > best['confidence']:
                    best = dict(tip=tip['p'], bend=bend, end=end, confidence=float(score),
                                direction='DERECHA' if cross(u, w) > 0 else 'IZQUIERDA')
    if best is None or best['confidence'] < MIN_CONFIDENCE:
        return None, seg, dirs, groups
    return best, seg, dirs, groups


def classify_background(frame, ellipse, color=COLOR):
    """Mide porcentajes HSV dentro de la elipse original, sin incluir su borde."""
    (cx, cy), (w, h), angle = ellipse
    margin = max(w, h) * .5 + 2
    H, W = frame.shape[:2]
    x0, y0 = max(0, int(cx-margin)), max(0, int(cy-margin))
    x1, y1 = min(W, int(math.ceil(cx+margin))), min(H, int(math.ceil(cy+margin)))
    if x1 <= x0 or y1 <= y0:
        return None, {}
    crop = frame[y0:y1, x0:x1]
    mask = np.zeros(crop.shape[:2], np.uint8)
    local = ((cx-x0, cy-y0), (w*COLOR_SAMPLE_FACTOR, h*COLOR_SAMPLE_FACTOR), angle)
    cv2.ellipse(mask, local, 255, -1)
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)[mask != 0]
    if not len(hsv):
        return None, {}
    Hh, S, V = hsv.T
    chromatic = (S >= COLOR_MIN_S) & (V >= COLOR_MIN_V)
    fractions = {
        'rojo': float(np.mean(chromatic & ((Hh <= RED_H_MAX) | (Hh >= RED_H_MIN)))),
        'azul': float(np.mean(chromatic & (Hh >= BLUE_H_MIN) & (Hh <= BLUE_H_MAX))),
        'blanco': float(np.mean((S <= WHITE_MAX_S) & (V >= WHITE_MIN_V))),
    }
    if fractions['blanco'] >= WHITE_MIN_FRACTION:
        return 'STOP', fractions
    if fractions[color] >= COLOR_MIN_FRACTION:
        return 'FLECHA', fractions
    return None, fractions


def extract_roi(edges, ellipse, factor=ROI_FACTOR):
    (cx, cy), (w, h), _ = ellipse
    roi_radius = min(w, h) * .5 * factor
    mask = np.zeros(edges.shape, dtype=np.uint8)
    cv2.circle(mask, (round(cx), round(cy)), round(roi_radius), 255, -1)
    return (np.array([cx, cy], dtype=np.float32), roi_radius,
            cv2.bitwise_and(edges, mask))


def _recovery_edges(gray):
    gray = _CLAHE.apply(gray)
    e = cv2.Canny(cv2.GaussianBlur(gray, (3, 3), .7), 30, 85)
    return cv2.morphologyEx(e, cv2.MORPH_CLOSE, _ELLIPSE3)


def detect_robust(frame, edges, ellipse, recovery_cache=None, arrow_r=0, early_k=0, gray=None, mode='FLECHA'):
    """Trabaja solo sobre el recorte del círculo. Cascada: ROI .80 -> ROI .94 ->
    ROI .94 con bordes de recuperación. Opcionalmente reescala el recorte."""
    if recovery_cache is None:
        recovery_cache = {}
    (cx, cy), (w, h), angle = ellipse
    R = min(w, h) * .5
    H, W = edges.shape
    x0, y0 = max(0, int(math.floor(cx - R - 3))), max(0, int(math.floor(cy - R - 3)))
    x1, y1 = min(W, int(math.ceil(cx + R + 4))), min(H, int(math.ceil(cy + R + 4)))
    if x1 <= x0 or y1 <= y0:
        return np.array([cx, cy]), R, None, None, None, None
    offset = np.array([x0, y0], dtype=np.float32)

    s = 1.0
    gray_small = None
    base = edges[y0:y1, x0:x1]
    local = ((cx - x0, cy - y0), (w, h), angle)
    if arrow_r and R > arrow_r * 1.15:
        s = arrow_r / R
        g = (gray if gray is not None else cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))[y0:y1, x0:x1]
        gray_small = cv2.resize(g, None, fx=s, fy=s, interpolation=cv2.INTER_AREA)
        base = cv2.Canny(cv2.GaussianBlur(gray_small, (GAUSSIAN_KERNEL, GAUSSIAN_KERNEL),
                                          GAUSSIAN_SIGMA), CANNY_LOW, CANNY_HIGH)
        local = (((cx - x0) * s, (cy - y0) * s), (w * s, h * s), angle)

    def out(center, radius, result, seg, dirs, groups):
        if result is not None:
            for key in ('tip', 'bend', 'end'):
                if key in result:
                    result[key] = result[key] / s + offset
        return (center / s + offset, radius / s, result,
                None if seg is None else seg / s + offset, dirs, groups)

    seg = dirs = groups = None
    for i, (factor, recover) in enumerate(((ROI_FACTOR, False), (.94, False), (.94, True))):
        if recover:
            if gray_small is not None:
                source = _recovery_edges(gray_small)
            else:
                if 'edges' not in recovery_cache:
                    recovery_cache['edges'] = _recovery_edges(
                        gray if gray is not None else cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
                source = recovery_cache['edges'][y0:y1, x0:x1]
        else:
            source = base
        center, radius, masked = extract_roi(source, local, factor)
        if i == 0 and early_k and cv2.countNonZero(masked) < early_k * radius:
            return out(center, radius, None, None, None, None)
        result, seg, dirs, groups = detect((center, radius, masked),
                                          allow_stop=(mode == 'STOP'), stop_only=(mode == 'STOP'))
        if result is not None:
            return out(center, radius, result, seg, dirs, groups)
        # Recuperar segmentos de contornos parcialmente cerrados.
        contours, _ = cv2.findContours(masked, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        pieces = []
        for c in contours:
            if cv2.arcLength(c, True) < .5 * radius:
                continue
            poly = cv2.approxPolyDP(c, max(1., .022 * radius), True).reshape(-1, 2)
            if len(poly) < 3:
                continue
            pieces.extend(zip(poly, np.roll(poly, -1, axis=0)))
        if pieces:
            extra = np.asarray(pieces, np.float32)
            extra = extra[np.linalg.norm(extra[:, 1] - extra[:, 0], axis=1)
                          >= max(4., .045 * radius)]
            combined = np.concatenate((seg, extra)) if seg is not None else extra
            if len(combined) > 100:
                lengths = np.linalg.norm(combined[:, 1] - combined[:, 0], axis=1)
                combined = combined[np.argsort(lengths)[-100:]]
            result, seg, dirs, groups = detect((center, radius, masked), combined,
                                              allow_stop=(mode == 'STOP'), stop_only=(mode == 'STOP'))
            if result is not None:
                return out(center, radius, result, seg, dirs, groups)
    return out(center, radius, None, seg, dirs, groups)


def draw_marker(frame, point, color, name, dy=0):
    x, y = np.rint(point).astype(int)
    cv2.circle(frame, (x, y), 6, color, -1, cv2.LINE_AA)
    cv2.circle(frame, (x, y), 9, (0, 0, 0), 1, cv2.LINE_AA)
    cv2.putText(frame, name, (x + 9, y - 9 + dy),
                cv2.FONT_HERSHEY_SIMPLEX, .46, color, 2, cv2.LINE_AA)


# ============================================================
# SEGUIMIENTO (Kalman sobre cx, cy, r) Y VOTO TEMPORAL DE DIRECCION
# ============================================================

class Track:
    _next_id = 1

    def __init__(self, cx, cy, r, smooth, raw=None):
        self.raw = raw
        self.id = Track._next_id
        Track._next_id += 1
        self.smooth = smooth
        self.hits, self.misses = 1, 0
        self.vote = np.zeros(3, dtype=np.float32)
        self.pos = np.array([cx, cy, r], np.float64)
        if smooth:
            kf = cv2.KalmanFilter(6, 3)
            F = np.eye(6, dtype=np.float32)
            F[0, 3] = F[1, 4] = F[2, 5] = 1.0
            kf.transitionMatrix = F
            kf.measurementMatrix = np.hstack((np.eye(3), np.zeros((3, 3)))).astype(np.float32)
            kf.processNoiseCov = np.diag([TRACK_Q_POS, TRACK_Q_POS, TRACK_Q_RAD,
                                          TRACK_Q_VEL, TRACK_Q_VEL, TRACK_Q_VEL]).astype(np.float32)
            kf.measurementNoiseCov = (np.eye(3) * TRACK_MEAS_VAR).astype(np.float32)
            kf.errorCovPost = np.diag([TRACK_MEAS_VAR] * 3 + [100.] * 3).astype(np.float32)
            kf.statePost = np.array([[cx], [cy], [r], [0], [0], [0]], np.float32)
            self.kf = kf

    def predict(self):
        if self.smooth:
            p = self.kf.predict()
            self.pos = p[:3, 0].astype(np.float64)
        self.pos[2] = max(self.pos[2], 5.0)

    def update(self, cx, cy, r, raw=None):
        self.raw = raw
        if self.smooth:
            self.kf.correct(np.array([[cx], [cy], [r]], np.float32))
            self.pos = self.kf.statePost[:3, 0].astype(np.float64)
        else:
            self.pos = np.array([cx, cy, r], np.float64)
        self.hits += 1
        self.misses = 0

    def miss(self):
        self.misses += 1
        self.vote *= VOTE_ALPHA
        if self.smooth:   # sin correct(), OpenCV predice desde statePost: mantener la prediccion
            self.kf.statePost = self.kf.statePre.copy()
            self.kf.errorCovPost = self.kf.errorCovPre.copy()

    def ellipse(self):
        cx, cy, r = self.pos
        return ((float(cx), float(cy)), (2 * float(r), 2 * float(r)), 0.0)

    def observe(self, result):
        """Actualiza el voto con el resultado de la flecha de este frame (None = sin evidencia)."""
        if result is None:
            self.vote *= VOTE_ALPHA
            return
        self.vote *= VOTE_ALPHA
        self.vote[STOP_CLASSES.index(result['direction'])] += (1 - VOTE_ALPHA) * result['confidence']

    def stable(self):
        i = int(np.argmax(self.vote))
        return STOP_CLASSES[i] if self.vote[i] >= VOTE_THRESH else None


class Tracker:
    def __init__(self, smooth=True, confirm=2):
        self.smooth, self.confirm = smooth, confirm
        self.tracks = []

    def step(self, circles):
        """Asocia los circulos detectados a los tracks. Devuelve [(track, ellipse, detectado)]
        solo para tracks confirmados (hits >= confirm); con smooth tambien los que van
        'en vuelo' por prediccion durante unos frames."""
        for t in self.tracks:
            t.predict()
        dets = []
        for obj in circles:
            (cx, cy), (w, h), _ = obj['ellipse']
            dets.append((cx, cy, min(w, h) * .5, obj['ellipse']))
        pairs = []
        for ti, t in enumerate(self.tracks):
            for di, (cx, cy, r, _) in enumerate(dets):
                rr = max(t.pos[2], r)
                dist = math.hypot(t.pos[0] - cx, t.pos[1] - cy)
                if dist <= TRACK_GATE * rr and min(t.pos[2], r) / rr >= TRACK_RATIO_MIN:
                    pairs.append((dist / rr, ti, di))
        pairs.sort()
        used_t, used_d = set(), set()
        for _, ti, di in pairs:
            if ti in used_t or di in used_d:
                continue
            used_t.add(ti)
            used_d.add(di)
            cx, cy, r, raw = dets[di]
            self.tracks[ti].update(cx, cy, r, raw)
        for ti, t in enumerate(self.tracks):
            if ti not in used_t:
                t.miss()
        self.tracks = [t for t in self.tracks if t.misses <= TRACK_MAX_MISS]
        for di, (cx, cy, r, raw) in enumerate(dets):
            if di not in used_d:
                self.tracks.append(Track(cx, cy, r, self.smooth, raw))
        out = []
        for t in self.tracks:
            if t.hits < self.confirm:
                continue
            seen = t.misses == 0
            if seen or self.smooth:
                out.append((t, t.ellipse() if self.smooth else t.raw, seen))
        return out



class CameraSource:
    """Una sola captura, interfaz read()/release()."""
    def __init__(self, index, width):
        self.cap = cv2.VideoCapture(index)
        if not self.cap.isOpened():
            self.cap.release()
            raise RuntimeError(f'No se pudo abrir la camara {index}')
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    def read(self):
        return self.cap.read()

    def release(self):
        self.cap.release()


class ROS2Source:
    """Recibe sensor_msgs/Image BGR mediante ROS 2 y cv_bridge."""
    def __init__(self, topic):
        import rclpy
        from sensor_msgs.msg import Image
        from cv_bridge import CvBridge
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        self.rclpy = rclpy
        rclpy.init()
        self.node = rclpy.create_node('vision_senales_qr')
        self.bridge = CvBridge()
        self.frame = None
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub = self.node.create_subscription(Image, topic, self._receive, qos)

    def _receive(self, msg):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.node.get_logger().warning(str(exc))

    def read(self):
        self.frame = None
        while self.rclpy.ok() and self.frame is None:
            self.rclpy.spin_once(self.node, timeout_sec=0.1)
        return self.frame is not None, self.frame

    def release(self):
        self.node.destroy_node()
        if self.rclpy.ok():
            self.rclpy.shutdown()


# Hiperparametros
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


def META_annotate(frame, pts, group, links, confidence):
    for p in pts:
        cv2.circle(frame, tuple(np.rint(p).astype(int)), 3, (0, 165, 255), -1)
    for i, j in links:
        cv2.line(frame, tuple(np.rint(pts[i]).astype(int)),
                 tuple(np.rint(pts[j]).astype(int)), (0, 180, 0), 1, cv2.LINE_AA)
    if len(group):
        region = np.rint(pts[group]).astype(np.int32)
        hull = cv2.convexHull(region)
        cv2.polylines(frame, [hull], True, (0, 255, 0), 2)
        for p in region:
            cv2.circle(frame, tuple(p), 3, (0, 255, 0), -1)
        x, y, _, _ = cv2.boundingRect(hull)
        cv2.putText(frame, f'META  score:{confidence:.2f}', (x, max(65, y-10)),
                    cv2.FONT_HERSHEY_SIMPLEX, .60, (0, 255, 0), 2)
    return frame

META_EVERY = 3
META_TTL = .35


class QRWorker:
    """Un hilo y como maximo una tarea en vuelo: no hay cola de frames."""
    def __init__(self, every=QR_EVERY, width=QR_WIDTH, multi=False, mirror=False,
                 meta_every=META_EVERY, meta_width=META_WIDTH):
        self.every, self.width, self.multi = every, width, multi
        self.mirror = mirror
        self.pool = ThreadPoolExecutor(max_workers=1)
        self.detector = cv2.QRCodeDetector()
        self.future = None
        self.latest = None
        self.latest_meta = None
        self.meta_every, self.meta_width = meta_every, meta_width
        self.due = {'qr': 1, 'meta': 2}
        self.kind = None

    def _detect(self, gray, timestamp, index):
        h, w = gray.shape
        if self.mirror:
            gray = cv2.flip(gray, 1)
        if w > self.width:
            gray = cv2.resize(gray, (self.width, max(1, round(h*self.width/w))),
                              interpolation=cv2.INTER_AREA)
        sx, sy = w/gray.shape[1], h/gray.shape[0]
        results = []
        if self.multi:
            ok, values, points, _ = self.detector.detectAndDecodeMulti(gray)
            pairs = zip(values, points) if ok and points is not None else ()
        else:
            value, points, _ = self.detector.detectAndDecode(gray)
            pairs = [(value, points)] if points is not None else ()
        for value, pts in pairs:
            pts = np.asarray(pts, dtype=np.float32).reshape(-1, 2)
            pts *= (sx, sy)
            if self.mirror:
                pts[:, 0] = w - 1 - pts[:, 0]
            results.append({'data': value, 'corners': pts.tolist()})
        return timestamp, index, (h, w), results

    def _detect_meta(self, gray, timestamp, index):
        start = time.perf_counter()
        h, w = gray.shape
        if w > self.meta_width:
            gray = cv2.resize(gray, (self.meta_width, max(1, round(h*self.meta_width/w))),
                              interpolation=cv2.INTER_AREA)
        pts, group, links, confidence = detect_meta(gray)
        pts *= (w/gray.shape[1], h/gray.shape[0])
        return timestamp, index, (h, w), pts, group, links, confidence, (time.perf_counter()-start)*1000

    def step(self, gray, index, timestamp):
        fresh_qr = fresh_meta = None
        if self.future is not None and self.future.done():
            try:
                result = self.future.result()
                if self.kind == 'qr':
                    self.latest = fresh_qr = result
                else:
                    self.latest_meta = fresh_meta = result
            except Exception as exc:
                print(f'Error {self.kind}: {exc}', file=__import__('sys').stderr)
                if self.kind == 'qr':
                    self.latest = None
                else:
                    self.latest_meta = None
            self.future = None
        if self.future is None:
            due = [kind for kind in self.due if index >= self.due[kind]]
            if due:
                self.kind = min(due, key=self.due.get)
                period = self.every if self.kind == 'qr' else self.meta_every
                task = self._detect if self.kind == 'qr' else self._detect_meta
                self.due[self.kind] = index + period
                self.future = self.pool.submit(task, gray, timestamp, index)
        return fresh_qr, fresh_meta

    def draw(self, out):
        if self.latest_meta is not None:
            ts, _, shape, pts, group, links, confidence, ms = self.latest_meta
            if out.shape[:2] == shape and time.monotonic()-ts <= META_TTL:
                META_annotate(out, pts, group, links, confidence)
                cv2.putText(out, f'Meta: {ms:.1f} ms  X:{len(pts)}', (8, 80),
                            cv2.FONT_HERSHEY_SIMPLEX, .45, (0, 255, 0), 1, cv2.LINE_AA)
        if self.latest is None:
            return
        timestamp, _, shape, results = self.latest
        if out.shape[:2] != shape or time.monotonic() - timestamp > QR_TTL:
            return
        for item in results:
            pts = np.rint(item['corners']).astype(np.int32)
            cv2.polylines(out, [pts], True, (255, 0, 255), 2, cv2.LINE_AA)
            x, y = pts.min(axis=0)
            text = 'QR: ' + (item['data'][:60] or 'sin decodificar')
            cv2.putText(out, text, (max(0, x), max(80, y-8)),
                        cv2.FONT_HERSHEY_SIMPLEX, .5, (255, 0, 255), 2, cv2.LINE_AA)

    def close(self):
        self.pool.shutdown(wait=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--camera', type=int, default=0)
    parser.add_argument('--ros-topic', default=None, help='Topic ROS 2 sensor_msgs/Image')
    parser.add_argument('--headless', action='store_true', help='Sin ventanas; resultados JSON')
    parser.add_argument('--qr-every', type=int, default=QR_EVERY)
    parser.add_argument('--qr-width', type=int, default=QR_WIDTH)
    parser.add_argument('--meta-every', type=int, default=META_EVERY)
    parser.add_argument('--meta-width', type=int, default=META_WIDTH)
    parser.add_argument('--qr-multi', action='store_true')
    parser.add_argument('--opencv-threads', type=int, default=1)
    parser.add_argument('--color', choices=('rojo', 'azul'), default=COLOR)
    parser.add_argument('--width', type=int, default=640)
    parser.add_argument('--diameter', type=int, default=DIAMETER)
    parser.add_argument('--arrow-r', type=float, default=0)
    parser.add_argument('--early-k', type=float, default=0)
    parser.add_argument('--track', action='store_true',
                        help='Kalman sobre (cx, cy, r) + asociacion; mantiene el circulo unos frames si falla')
    parser.add_argument('--vote', action='store_true',
                        help='voto temporal de la direccion (requiere seguimiento por asociacion)')
    parser.add_argument('--confirm', type=int, default=2,
                        help='frames consecutivos para confirmar un track (solo con --track/--vote)')
    parser.add_argument('--mirror', action='store_true')
    parser.add_argument('--show-edges', action='store_true')
    args = parser.parse_args()
    if min(args.qr_every, args.qr_width, args.meta_every, args.meta_width, args.width, args.opencv_threads) < 1:
        parser.error('frecuencia, ancho y numero de hilos deben ser positivos')
    cv2.setNumThreads(args.opencv_threads)
    cap = ROS2Source(args.ros_topic) if args.ros_topic else CameraSource(args.camera, args.width)
    qr = QRWorker(args.qr_every, args.qr_width, args.qr_multi, args.mirror,
                  args.meta_every, args.meta_width)
    frame_index = 0
    detector = Detector(diameter=args.diameter, max_objects=MAX_OBJECTS,
                        hough=USE_HOUGH, max_ratio=MAX_RATIO)
    tracker = Tracker(smooth=args.track, confirm=args.confirm) if (args.track or args.vote) else None
    fps = 0.0
    last = time.perf_counter()
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print('No se pudo leer un fotograma')
                break
            if args.mirror:
                frame = cv2.flip(frame, 1)
            h, w = frame.shape[:2]
            if w > args.width:
                frame = cv2.resize(frame, (args.width, round(h * args.width / w)),
                                   interpolation=cv2.INTER_AREA)
            frame_index += 1
            timestamp = time.monotonic()
            sign_results = []
            t0 = time.perf_counter()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            fresh_qr, fresh_meta = qr.step(gray, frame_index, timestamp)
            # Realzar el color seleccionado SOLO para localizar círculos.
            channel = cv2.extractChannel(frame, 2 if args.color == 'rojo' else 0)
            circle_gray = cv2.max(gray, channel)
            smooth_circle = cv2.GaussianBlur(circle_gray, (5, 5), 1.05)
            circle_edges = cv2.Canny(smooth_circle, CIRCLE_CANNY_LOW, CIRCLE_CANNY_HIGH)
            t1 = time.perf_counter()
            circles = detector.detect(frame, external_edges=circle_edges)
            t2 = time.perf_counter()
            recovery_cache = {}
            out = frame.copy()
            if tracker is None:
                items = [(None, obj['ellipse'], True) for obj in circles]
            else:
                items = tracker.step(circles)
            # Validar color antes de calcular bordes sensibles o segmentos.
            valid_items = []
            for trk, ellipse, seen in items:
                cv2.ellipse(out, ellipse, (0, 255, 0) if seen else (0, 200, 255), 2, cv2.LINE_AA)
                mode, fractions = classify_background(frame, ellipse, args.color)
                if trk is not None:
                    # No arrastrar votos incompatibles con el fondo actual.
                    if mode is None:
                        trk.vote[:] = 0
                    elif mode == 'STOP':
                        trk.vote[:2] = 0
                    else:
                        trk.vote[2] = 0
                if mode is not None:
                    valid_items.append((trk, ellipse, seen, mode))
            edges = circle_edges
            if valid_items:
                smooth = cv2.GaussianBlur(gray, (5, 5), 1.05)
                edges, _ = canny_pipeline(frame, gray=gray, smooth=smooth)
            # Sin círculos (ni tracks) no se ejecuta nada de la etapa de flecha.
            for trk, ellipse, seen, mode in valid_items:
                center, radius, result, seg, dirs, groups = detect_robust(
                    frame, edges, ellipse, recovery_cache, args.arrow_r, args.early_k, gray, mode)
                if trk is not None and args.vote:
                    trk.observe(result)
                if result is not None:
                    sign_results.append({'direction': result['direction'],
                                         'confidence': float(result['confidence']),
                                         'center': center.tolist(),
                                         'radius': float(radius)})
                    if seg is not None and dirs is not None:
                        for group, color in zip(groups, ((0, 0, 255), (255, 0, 0))):
                            for a, b in seg[group].astype(int):
                                cv2.line(out, tuple(a), tuple(b), color, 2, cv2.LINE_AA)
                    if result['direction'] != 'STOP':
                        tip, bend, end = result['tip'], result['bend'], result['end']
                        cv2.polylines(out, [np.rint([end, bend, tip]).astype(np.int32)],
                                      False, (255, 255, 0), 2, cv2.LINE_AA)
                        draw_marker(out, end, (255, 0, 0), 'EXTREMO')
                        draw_marker(out, bend, (0, 165, 255), 'ESQUINA')
                        draw_marker(out, tip, (0, 255, 255), 'PUNTA')
                label = None
                if trk is not None and args.vote:
                    label = f"#{trk.id} {trk.stable() or '...'}  v:{trk.vote.max():.2f}"
                elif result is not None:
                    label = f"{result['direction']}  conf: {result['confidence']:.2f}"
                if label:
                    cx, cy = center
                    pos = (max(5, int(cx - radius)), max(65, int(cy - radius - 12)))
                    cv2.putText(out, label, pos, cv2.FONT_HERSHEY_SIMPLEX,
                                .6, (255, 255, 255), 2, cv2.LINE_AA)
            now = time.perf_counter()
            instant = 1.0 / max(now - last, 1e-6)
            fps = instant if fps == 0 else .85 * fps + .15 * instant
            last = now
            cv2.rectangle(out, (0, 0), (210, 65), (20, 20, 20), -1)
            cv2.putText(out, f'FPS: {fps:.1f}', (12, 29),
                        cv2.FONT_HERSHEY_SIMPLEX, .8, (255, 255, 255), 2, cv2.LINE_AA)
            cv2.putText(out, f'Canny: {(t1-t0)*1000:.1f} ms', (8, 45),
                        cv2.FONT_HERSHEY_SIMPLEX, .40, (255, 255, 255), 1, cv2.LINE_AA)
            cv2.putText(out, f'Circ: {(t2-t1)*1000:.1f}  Flecha: {(now-t2)*1000:.1f} ms', (8, 59),
                        cv2.FONT_HERSHEY_SIMPLEX, .40, (255, 255, 255), 1, cv2.LINE_AA)
            if args.headless:
                payload = {'frame': frame_index, 'timestamp': timestamp,
                           'signs': sign_results, 'fps': round(fps, 2)}
                if fresh_qr is not None:
                    qt, qi, _, results = fresh_qr
                    payload['qr'] = {'frame': qi, 'timestamp': qt, 'results': results}
                if fresh_meta is not None:
                    mt, mi, _, pts, group, links, confidence, ms = fresh_meta
                    payload['meta'] = {'frame': mi, 'timestamp': mt,
                                       'detected': bool(len(group)),
                                       'confidence': confidence,
                                       'corners': pts[group].tolist(),
                                       'ms': round(ms, 2)}
                print(json.dumps(payload, ensure_ascii=False), flush=True)
            else:
                qr.draw(out)
                cv2.imshow('Detector de senales, QR y META - Camara', out)
                if args.show_edges:
                    cv2.imshow('Canny', edges)
                    cv2.imshow('Canny circulos', circle_edges)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
    finally:
        cap.release()
        qr.close()
        if not args.headless:
            cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
