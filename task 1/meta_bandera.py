"""Detección de meta (bandera a cuadros negro/blanco) y QR asociado, sin ROS."""

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

_META_ANGLES = np.arange(META_SAMPLES, dtype=np.float32) * (2 * np.pi / META_SAMPLES)
_META_UNIT = np.stack((np.cos(_META_ANGLES), np.sin(_META_ANGLES)), axis=1)
_META_COS2 = np.cos(2 * _META_ANGLES)
_META_SIN2 = np.sin(2 * _META_ANGLES)
_META_OFFSETS = np.asarray(META_RADII, np.float32)[:, None, None] * _META_UNIT[None, :, :]


def _meta_x_corners(gray):
    corners = cv2.goodFeaturesToTrack(
        gray, META_MAX_CORNERS, META_QUALITY, META_MIN_DISTANCE, blockSize=3
    )
    if corners is None:
        return np.empty((0, 2), np.float32), np.empty(0), np.empty(0)
    pts = corners.reshape(-1, 2)
    h, w = gray.shape
    xy = pts[:, None, None, :] + _META_OFFSETS[None, :, :, :]
    inside = (
        (xy[..., 0] >= 0) & (xy[..., 0] < w - 1) & (xy[..., 1] >= 0) & (xy[..., 1] < h - 1)
    ).all(axis=2)
    samples = cv2.remap(
        gray,
        xy[..., 0].reshape(-1, META_SAMPLES),
        xy[..., 1].reshape(-1, META_SAMPLES),
        cv2.INTER_LINEAR,
    ).reshape(len(pts), len(META_RADII), META_SAMPLES).astype(np.float32)
    lo, hi = samples.min(axis=2), samples.max(axis=2)
    contrast = hi - lo
    binary = samples > (lo + hi)[..., None] * 0.5
    changes = np.count_nonzero(binary != np.roll(binary, 1, axis=2), axis=2)
    opposite = np.mean(binary == np.roll(binary, META_SAMPLES // 2, axis=2), axis=2)
    duty = binary.mean(axis=2)
    signal = samples - samples.mean(axis=2, keepdims=True)
    real = signal @ _META_COS2
    imag = signal @ _META_SIN2
    harmonic = np.clip(
        2 * (real ** 2 + imag ** 2) / (META_SAMPLES * np.sum(signal ** 2, axis=2) + 1e-6),
        0,
        1,
    )
    valid = (
        inside
        & (contrast >= META_MIN_CONTRAST)
        & (changes == 4)
        & (opposite >= 0.68)
        & (duty >= 0.25)
        & (duty <= 0.75)
        & (harmonic >= META_MIN_HARMONIC)
    )
    keep = valid.sum(axis=1) >= META_MIN_VALID_RADII
    score = np.where(valid, harmonic, 0)
    best = score.argmax(axis=1)
    rows = np.arange(len(pts))
    axes = 0.5 * np.arctan2(imag[rows, best], real[rows, best]) + np.pi / 4
    confidence = score.max(axis=1)
    return pts[keep], axes[keep], confidence[keep]


def _meta_group_grid(pts, axes):
    n = len(pts)
    if n < META_MIN_CORNERS:
        return np.empty(0, dtype=int), []
    delta = pts[None, :, :] - pts[:, None, :]
    dist = np.linalg.norm(delta, axis=2)
    np.fill_diagonal(dist, np.inf)
    nearest = dist.min(axis=1)
    angle = np.arctan2(delta[..., 1], delta[..., 0])
    err = np.abs((angle - axes[:, None] + np.pi / 4) % (np.pi / 2) - np.pi / 4)
    aligned = err < np.deg2rad(META_ANGLE_TOL)
    link = dist <= META_NEIGHBOR_RATIO * np.maximum(nearest[:, None], nearest[None, :])
    link &= aligned & aligned.T
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
            junctions = 0
            for u in group:
                directions = []
                for v in graph[u]:
                    d = (angle[u, v] - axes[u]) % np.pi
                    directions.append(int(np.round(d / (np.pi / 2))) % 2)
                junctions += len(set(directions)) == 2
            if junctions >= 2 and len(group) > len(best):
                best = group
    ids = np.asarray(best, dtype=int)
    links = [(i, j) for i in best for j in graph[i] if i < j]
    return ids, links


class DetectorMetaBandera:
    """Detecta tablero de cuadros negros y blancos (bandera de meta)."""

    def __init__(self, ancho=META_WIDTH, min_puntos=META_MIN_CORNERS, min_confianza=0.68):
        self.ancho = int(ancho)
        self.min_puntos = int(min_puntos)
        self.min_confianza = float(min_confianza)

    def detectar(self, imagen):
        if (not isinstance(imagen, np.ndarray) or imagen.dtype != np.uint8
                or imagen.ndim != 3 or imagen.shape[2] != 3 or min(imagen.shape[:2]) < 24):
            raise ValueError('Se requiere imagen BGR uint8')
        h, w = imagen.shape[:2]
        escala = min(1.0, self.ancho / w)
        if escala < 1.0:
            reducida = cv2.resize(imagen, (max(1, round(w * escala)), max(1, round(h * escala))),
                                  interpolation=cv2.INTER_AREA)
        else:
            reducida = imagen
        gris = cv2.cvtColor(reducida, cv2.COLOR_BGR2GRAY)
        pts, axes, scores = _meta_x_corners(gris)
        grupo, _ = _meta_group_grid(pts, axes)
        if len(grupo) < self.min_puntos:
            return dict(detectada=False, confianza=0.0, puntos=0, caja=None)
        confianza = float(scores[grupo].mean())
        if confianza < self.min_confianza:
            return dict(detectada=False, confianza=confianza, puntos=len(grupo), caja=None)
        reg = pts[grupo] / escala
        x0, y0 = np.min(reg, axis=0)
        x1, y1 = np.max(reg, axis=0)
        caja = [int(x0), int(y0), max(1, int(x1 - x0)), max(1, int(y1 - y0))]
        return dict(detectada=True, confianza=confianza, puntos=len(grupo), caja=caja)


class ConsultaMetaBandera:
    """Confirma bandera de meta en N fotogramas nuevos consecutivos."""

    def __init__(self, confirmaciones=3, detector=None):
        self.confirmaciones = int(confirmaciones)
        self.detector = detector or DetectorMetaBandera()
        self.ultima_marca = None
        self.cuenta = 0
        self.ultimo = None

    def reiniciar(self):
        self.cuenta = 0
        self.ultimo = None

    def actualizar(self, imagen, marca_ns, ahora_ns):
        if imagen is None or type(marca_ns) is not int or not 0 <= ahora_ns - marca_ns <= 1_000_000_000:
            self.cuenta = 0
            self.ultimo = dict(detectada=False, confirmada=False, confianza=0.0, caja=None, cuenta=0)
            return self.ultimo
        if self.ultima_marca is not None and marca_ns <= self.ultima_marca:
            return self.ultimo or dict(detectada=False, confirmada=False, confianza=0.0, caja=None, cuenta=0)
        self.ultima_marca = marca_ns
        res = self.detector.detectar(imagen)
        if res['detectada']:
            self.cuenta = min(self.confirmaciones, self.cuenta + 1)
        else:
            self.cuenta = 0
        self.ultimo = dict(
            detectada=res['detectada'],
            confirmada=self.cuenta >= self.confirmaciones,
            confianza=res['confianza'],
            caja=res['caja'],
            cuenta=self.cuenta,
        )
        return self.ultimo


def decodificar_qr_asociado(imagen, caja_rostro=None):
    """Decodifica el QR cercano a la caja facial ganadora (o el más próximo si hay varios)."""
    if imagen is None or imagen.ndim != 3 or imagen.size == 0:
        return None
    det = cv2.QRCodeDetector()
    try:
        ok, datos, puntos, _ = det.detectAndDecodeMulti(imagen)
    except cv2.error:
        ok, datos, puntos = False, (), None
    if ok and datos:
        candidatos = []
        for idx, texto in enumerate(datos):
            txt = (texto or '').strip()
            if not txt:
                continue
            dist = 0.0
            if caja_rostro is not None and puntos is not None and idx < len(puntos):
                fx, fy, fw, fh = caja_rostro
                fcx, fcy = fx + fw * 0.5, fy + fh * 0.5
                qcx, qcy = np.mean(puntos[idx], axis=0)
                dist = float(np.hypot(qcx - fcx, qcy - fcy))
            candidatos.append((dist, txt))
        if candidatos:
            candidatos.sort(key=lambda item: item[0])
            return candidatos[0][1]
    texto_unico, _, _ = det.detectAndDecode(imagen)
    texto_unico = (texto_unico or '').strip()
    return texto_unico or None
