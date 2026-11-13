"""Exploración por fronteras sobre OccupancyGrid y evasión reactiva LiDAR (sin ROS)."""

from collections import deque
from dataclasses import dataclass
from math import atan2, hypot, isfinite, pi, radians
import numpy as np


MAX_VEL_LINEAL = 0.306
MAX_VEL_ANGULAR = 1.90


@dataclass(frozen=True)
class Frontera:
    x: float
    y: float
    tamano: int
    distancia: float
    costo: float


@dataclass(frozen=True)
class EstadoLidar:
    dist_frente: float
    dist_izquierda: float
    dist_derecha: float
    peligro_parada: bool
    precaucion: bool
    lecturas_validas: int


def normalizar_angulo(angulo: float) -> float:
    """Normaliza un ángulo en radianes al rango [-pi, pi]."""
    while angulo > pi:
        angulo -= 2.0 * pi
    while angulo < -pi:
        angulo += 2.0 * pi
    return angulo


def dilatar_ocupacion(ocupados: np.ndarray, radio_celdas: int) -> np.ndarray:
    """Dilata obstáculos booleanos con radio Manhattan/Chebyshev sin depender de SciPy."""
    if radio_celdas <= 0:
        return ocupados.copy()
    alto, ancho = ocupados.shape
    resultado = ocupados.copy()
    pad = np.pad(ocupados, radio_celdas, mode="constant", constant_values=False)
    for dy in range(-radio_celdas, radio_celdas + 1):
        for dx in range(-radio_celdas, radio_celdas + 1):
            if dy * dy + dx * dx > radio_celdas * radio_celdas:
                continue
            y0 = radio_celdas + dy
            x0 = radio_celdas + dx
            resultado |= pad[y0:y0 + alto, x0:x0 + ancho]
    return resultado


def detectar_fronteras(
    grilla: np.ndarray,
    resolucion: float,
    origen_x: float,
    origen_y: float,
    robot_x: float,
    robot_y: float,
    robot_yaw: float = 0.0,
    radio_seguridad_celdas: int = 4,
    min_tamano_cluster: int = 4,
    lista_negra: list[tuple[float, float]] | None = None,
    radio_exclusion_m: float = 0.45,
) -> list[Frontera]:
    """Encuentra fronteras seguras y alcanzables en un mapa de ocupación 2D.

    Convención OccupancyGrid:
      -1 = desconocido, 0..49 = libre, 50..100 = ocupado.
    """
    if grilla.ndim != 2 or grilla.size == 0 or resolucion <= 0:
        return []

    desconocido = grilla < 0
    libre = (grilla >= 0) & (grilla < 50)
    ocupado = grilla >= 50

    zona_bloqueada = dilatar_ocupacion(ocupado, radio_seguridad_celdas)

    # Celdas libres con algún vecino 4-conectado desconocido
    pad_desc = np.pad(desconocido, 1, mode="constant", constant_values=False)
    vecino_desconocido = (
        pad_desc[:-2, 1:-1]
        | pad_desc[2:, 1:-1]
        | pad_desc[1:-1, :-2]
        | pad_desc[1:-1, 2:]
    )
    es_frontera = libre & vecino_desconocido & (~zona_bloqueada)

    alto, ancho = grilla.shape
    visitado = np.zeros_like(es_frontera, dtype=bool)
    ys, xs = np.where(es_frontera)

    fronteras: list[Frontera] = []
    vecinos = ((-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1))
    bloqueados = lista_negra or []

    for y_ini, x_ini in zip(ys.tolist(), xs.tolist()):
        if visitado[y_ini, x_ini]:
            continue
        cola = deque([(y_ini, x_ini)])
        visitado[y_ini, x_ini] = True
        celdas: list[tuple[int, int]] = []

        while cola:
            cy, cx = cola.popleft()
            celdas.append((cy, cx))
            for dy, dx in vecinos:
                ny, nx = cy + dy, cx + dx
                if 0 <= ny < alto and 0 <= nx < ancho:
                    if not visitado[ny, nx] and es_frontera[ny, nx]:
                        visitado[ny, nx] = True
                        cola.append((ny, nx))

        if len(celdas) < min_tamano_cluster:
            continue

        # Elegir la celda real del clúster más cercana al centroide (siempre libre y segura)
        media_y = sum(c[0] for c in celdas) / len(celdas)
        media_x = sum(c[1] for c in celdas) / len(celdas)
        celda_y, celda_x = min(
            celdas, key=lambda c: (c[0] - media_y) ** 2 + (c[1] - media_x) ** 2
        )

        wx = origen_x + (celda_x + 0.5) * resolucion
        wy = origen_y + (celda_y + 0.5) * resolucion

        if any(hypot(wx - bx, wy - by) <= radio_exclusion_m for bx, by in bloqueados):
            continue

        dist = hypot(wx - robot_x, wy - robot_y)
        if dist < 0.25:
            continue

        ang_meta = atan2(wy - robot_y, wx - robot_x)
        dif_ang = abs(normalizar_angulo(ang_meta - robot_yaw))
        costo = dist + 0.25 * dif_ang - 0.08 * min(len(celdas), 25)
        fronteras.append(Frontera(x=wx, y=wy, tamano=len(celdas), distancia=dist, costo=costo))

    fronteras.sort(key=lambda f: f.costo)
    return fronteras


def analizar_lidar(
    ranges: list[float] | np.ndarray,
    angle_min: float,
    angle_increment: float,
    range_min: float = 0.12,
    range_max: float = 12.0,
    distancia_parada_m: float = 0.36,
    distancia_precaucion_m: float = 0.65,
    angulo_cono_frontal_deg: float = 35.0,
    angle_offset_rad: float = 0.0,
) -> EstadoLidar:
    """Analiza lecturas del LiDAR por sectores frontal, izquierdo y derecho en base_link."""
    if len(ranges) == 0 or angle_increment == 0:
        return EstadoLidar(0.0, 0.0, 0.0, True, True, 0)

    cono = radians(angulo_cono_frontal_deg)
    izq_min, izq_max = radians(20.0), radians(95.0)
    der_min, der_max = -radians(95.0), -radians(20.0)

    frente: list[float] = []
    izquierda: list[float] = []
    derecha: list[float] = []
    validas = 0

    for i, r in enumerate(ranges):
        val = float(r)
        if not isfinite(val) or val < range_min or val > range_max:
            continue
        validas += 1
        ang = normalizar_angulo(angle_min + i * angle_increment + angle_offset_rad)
        if abs(ang) <= cono:
            frente.append(val)
        if izq_min <= ang <= izq_max:
            izquierda.append(val)
        if der_min <= ang <= der_max:
            derecha.append(val)

    if validas == 0:
        return EstadoLidar(0.0, 0.0, 0.0, True, True, 0)

    d_frente = min(frente) if frente else range_max
    d_izq = min(izquierda) if izquierda else range_max
    d_der = min(derecha) if derecha else range_max

    return EstadoLidar(
        dist_frente=d_frente,
        dist_izquierda=d_izq,
        dist_derecha=d_der,
        peligro_parada=d_frente <= distancia_parada_m,
        precaucion=d_frente <= distancia_precaucion_m,
        lecturas_validas=validas,
    )


def calcular_velocidad_evasion(
    estado: EstadoLidar,
    velocidad_avance: float = 0.18,
    velocidad_giro: float = 0.85,
) -> tuple[float, float]:
    """Devuelve (linear_x, angular_z) segura para prueba reactiva de esquive de cajas."""
    vel_x = max(0.0, min(float(velocidad_avance), MAX_VEL_LINEAL))
    vel_w = max(0.1, min(float(velocidad_giro), MAX_VEL_ANGULAR))

    if estado.lecturas_validas == 0:
        return (0.0, 0.0)

    sentido = 1.0 if estado.dist_izquierda >= estado.dist_derecha else -1.0

    if estado.peligro_parada:
        return (0.0, sentido * vel_w)
    if estado.precaucion:
        return (vel_x * 0.45, sentido * vel_w * 0.75)

    # Pequeña corrección centradora si una caja o pared está muy cerca de un costado
    if estado.dist_izquierda < 0.32 and estado.dist_derecha > estado.dist_izquierda + 0.08:
        return (vel_x * 0.8, -0.35 * vel_w)
    if estado.dist_derecha < 0.32 and estado.dist_izquierda > estado.dist_derecha + 0.08:
        return (vel_x * 0.8, 0.35 * vel_w)

    return (vel_x, 0.0)
