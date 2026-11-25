"""Evasión derivada de paz_y_rostros: sectores LiDAR y decisiones sin ROS.

Base: task_autonoma/exploracion.py, revisada el 13/11/2026.
Los sectores sin observación suficiente son desconocidos, nunca espacio libre.
"""

from dataclasses import asdict, dataclass
import json
from math import ceil, isfinite, isnan, pi, radians
from pathlib import Path


@dataclass(frozen=True)
class ConfigEvasion:
    velocidad_avance: float = 0.10
    velocidad_giro: float = 0.40
    distancia_parada_m: float = 0.32
    distancia_precaucion_m: float = 0.45
    distancia_lateral_m: float = 0.28
    angulo_cono_frontal_deg: float = 35.0
    edad_maxima_s: float = 0.6
    cobertura_minima: float = 0.30
    fraccion_errores_maxima: float = 0.10

    def __post_init__(self):
        for nombre, valor in asdict(self).items():
            if isinstance(valor, bool) or not isinstance(valor, (int, float)) or not isfinite(valor):
                raise ValueError(f'{nombre} debe ser un número finito')
        if not 0 < self.velocidad_avance <= 0.306:
            raise ValueError('velocidad_avance fuera de (0, 0.306] m/s')
        if not 0 < self.velocidad_giro <= 1.90:
            raise ValueError('velocidad_giro fuera de (0, 1.90] rad/s')
        if not 0 < self.distancia_parada_m < self.distancia_precaucion_m:
            raise ValueError('Las distancias deben cumplir 0 < parada < precaución')
        if not 0 < self.distancia_lateral_m <= self.distancia_parada_m:
            raise ValueError('La distancia lateral debe ser positiva y no superar parada')
        if not 20 <= self.angulo_cono_frontal_deg <= 60:
            raise ValueError('El cono frontal debe estar entre 20 y 60 grados')
        if not 0 < self.edad_maxima_s <= 1.0:
            raise ValueError('edad_maxima_s fuera de (0, 1] segundos')
        if not 0.30 <= self.cobertura_minima <= 1.0:
            raise ValueError('cobertura_minima fuera de [0.30, 1]')
        if not 0 <= self.fraccion_errores_maxima <= 0.10:
            raise ValueError('fraccion_errores_maxima fuera de [0, 0.10]')


def cargar_config(ruta):
    datos = json.loads(Path(ruta).read_text(encoding='utf-8'))
    if not isinstance(datos, dict):
        raise ValueError('La configuración debe ser un objeto JSON')
    return ConfigEvasion(**datos)


@dataclass(frozen=True)
class CoberturaSector:
    nombre: str
    total: int
    utiles: int
    sin_retorno: int
    nan: int
    fuera_rango: int
    cobertura_minima: float = 0.30
    fraccion_errores_maxima: float = 0.10

    @property
    def suficiente(self):
        return (self.utiles >= max(3, ceil(self.total * self.cobertura_minima))
                and self.nan + self.fuera_rango <= self.total * self.fraccion_errores_maxima)


@dataclass(frozen=True)
class EstadoLidar:
    dist_frente: float
    dist_izquierda: float
    dist_derecha: float
    peligro_parada: bool
    precaucion: bool
    lecturas_validas: int
    observacion_valida: bool
    sectores: tuple[CoberturaSector, ...] = ()
    proximidad_no_cuantificable: bool = False


def resumir_cobertura(estado):
    if not estado.sectores:
        return 'geometría del escaneo inválida'
    resumen = []
    for sector in estado.sectores:
        porcentaje = 100 * sector.utiles / sector.total if sector.total else 0.0
        marca = 'OK' if sector.suficiente else 'INSUFICIENTE'
        resumen.append(
            f'{sector.nombre}={sector.utiles}/{sector.total} ({porcentaje:.0f}%, {marca}; '
            f'mín={100 * sector.cobertura_minima:.0f}%; '
            f'inf={sector.sin_retorno}, NaN={sector.nan}, fuera={sector.fuera_rango})')
    return ' | '.join(resumen)


def analizar_lidar(ranges, angle_min, angle_increment, range_min, range_max,
                   angle_offset_rad=0.0, cfg=None):
    cfg = cfg or ConfigEvasion()
    detenido = EstadoLidar(0.0, 0.0, 0.0, True, True, 0, False)
    geometria = (angle_min, angle_increment, range_min, range_max, angle_offset_rad)
    if (not all(isfinite(v) for v in geometria) or angle_increment == 0
            or not 0 <= range_min < range_max or len(ranges) < 3):
        return detenido

    sectores = [[], [], []]
    esperadas = [0, 0, 0]
    sin_retorno = [0, 0, 0]
    nans = [0, 0, 0]
    fuera_rango = [0, 0, 0]
    validas = 0
    demasiado_cerca = False
    cono = radians(cfg.angulo_cono_frontal_deg)
    for i, valor in enumerate(ranges):
        ang = (angle_min + i * angle_increment + angle_offset_rad + pi) % (2 * pi) - pi
        pertenece = (abs(ang) <= cono,
                     radians(20) <= ang <= radians(95),
                     -radians(95) <= ang <= -radians(20))
        # REP-117: -inf indica proximidad que el sensor no puede cuantificar.
        if valor == float('-inf') and any(pertenece):
            demasiado_cerca = True
        valido = isfinite(valor) and 0 <= valor <= range_max
        validas += int(valido)
        # Un retorno por debajo del mínimo no se descarta como si estuviera libre.
        distancia = 0.0 if valido and valor < range_min else valor
        for j, incluir in enumerate(pertenece):
            if incluir:
                esperadas[j] += 1
                if valido:
                    sectores[j].append(distancia)
                elif isnan(valor):
                    nans[j] += 1
                elif valor == float('inf'):
                    sin_retorno[j] += 1
                else:
                    fuera_rango[j] += 1

    detalles = tuple(CoberturaSector(nombre, esperadas[j], len(sectores[j]),
                                    sin_retorno[j], nans[j], fuera_rango[j],
                                    cfg.cobertura_minima, cfg.fraccion_errores_maxima)
                     for j, nombre in enumerate(('frente', 'izquierda', 'derecha')))
    cobertura = all(s.suficiente for s in detalles)
    distancias = [min(s) if s else 0.0 for s in sectores]
    return EstadoLidar(*distancias,
                       distancias[0] <= cfg.distancia_parada_m,
                       distancias[0] <= cfg.distancia_precaucion_m,
                       validas, cobertura, detalles, demasiado_cerca)


def calcular_velocidad_evasion(estado, cfg=None):
    """Avanzar recto o girar sobre el sitio; no combinar ambas órdenes."""
    cfg = cfg or ConfigEvasion()
    if not estado.observacion_valida or estado.proximidad_no_cuantificable:
        return 0.0, 0.0
    vx, vw = cfg.velocidad_avance, cfg.velocidad_giro
    sentido = 1.0 if estado.dist_izquierda >= estado.dist_derecha else -1.0
    if estado.peligro_parada:
        return 0.0, sentido * vw
    if estado.precaucion:
        return 0.0, sentido * vw * 0.75
    izq_cerca = estado.dist_izquierda < cfg.distancia_lateral_m
    der_cerca = estado.dist_derecha < cfg.distancia_lateral_m
    if izq_cerca and der_cerca:
        return 0.0, 0.0
    if izq_cerca and estado.dist_derecha > estado.dist_izquierda + 0.08:
        return 0.0, -0.35 * vw
    if der_cerca and estado.dist_izquierda > estado.dist_derecha + 0.08:
        return 0.0, 0.35 * vw
    if ((izq_cerca and estado.dist_derecha <= cfg.distancia_parada_m)
            or (der_cerca and estado.dist_izquierda <= cfg.distancia_parada_m)):
        return 0.0, 0.0
    return vx, 0.0


def decidir_scan(scan, yaw_lidar, edad_s, cfg):
    """Devuelve (vx, wz, estado, motivo); TF/recepción insuficientes detienen."""
    if scan is None or edad_s is None or not isfinite(edad_s) or not 0 <= edad_s <= cfg.edad_maxima_s:
        return 0.0, 0.0, None, 'Sin LiDAR reciente'
    if yaw_lidar is None or not isfinite(yaw_lidar):
        return 0.0, 0.0, None, 'Sin TF válido del LiDAR a base_link'
    estado = analizar_lidar(scan.ranges, scan.angle_min, scan.angle_increment,
                           scan.range_min, scan.range_max, yaw_lidar, cfg)
    vx, wz = calcular_velocidad_evasion(estado, cfg)
    if estado.proximidad_no_cuantificable:
        motivo = 'LiDAR indica proximidad no cuantificable (-inf): detenido'
    else:
        motivo = 'Evasión' if estado.observacion_valida else 'LiDAR incompleto: sectores desconocidos'
    return vx, wz, estado, motivo
