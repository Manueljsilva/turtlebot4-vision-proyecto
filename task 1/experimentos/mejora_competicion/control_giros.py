"""Giro por señal y odometría, sin ROS ni OpenCV. Soporta espera de pared frontal y cambio de tramo."""

from dataclasses import asdict, dataclass
import json
from math import atan2, cos, hypot, isfinite, radians, sin
from pathlib import Path


COLORES_TRAMO = {'ida': 'azul', 'vuelta': 'rojo'}


@dataclass(frozen=True)
class ConfigSenales:
    ancho: int = 640
    diametro_minimo_px: int = 55
    confirmaciones: int = 3
    ausencia_rearme_s: float = 1.5
    angulo_giro_deg: float = 90.0
    tolerancia_deg: float = 5.0
    tiempo_maximo_giro_s: float = 12.0
    desplazamiento_rearme_m: float = 0.4
    esperar_pared_m: float = 0.0
    tiempo_espera_pared_s: float = 6.0

    def __post_init__(self):
        for k, v in asdict(self).items():
            if type(v) not in (int, float) or not isfinite(v):
                raise ValueError(f'{k} debe ser finito')
        if (type(self.ancho) is not int or not 320 <= self.ancho <= 960
                or type(self.diametro_minimo_px) is not int
                or not 30 <= self.diametro_minimo_px <= self.ancho // 2
                or type(self.confirmaciones) is not int or not 3 <= self.confirmaciones <= 10):
            raise ValueError('Ancho, diámetro o confirmaciones fuera de rango')
        if not (1 <= self.ausencia_rearme_s <= 5 and 45 <= self.angulo_giro_deg <= 120
                and 1 <= self.tolerancia_deg <= 10 and 6 <= self.tiempo_maximo_giro_s <= 20
                and .3 <= self.desplazamiento_rearme_m <= 1):
            raise ValueError('Parámetros de giro/rearme fuera de rango')
        if not (0.0 <= self.esperar_pared_m <= 1.5 and 1.0 <= self.tiempo_espera_pared_s <= 20.0):
            raise ValueError('Parámetros de espera de pared fuera de rango')


def cargar_config_senales(ruta):
    d = json.loads(Path(ruta).read_text(encoding='utf-8'))
    if not isinstance(d, dict):
        raise ValueError('La configuración de señales debe ser un objeto')
    return ConfigSenales(**d)


def evento_valido(evento, ahora_ns):
    return (evento is None or isinstance(evento, dict)
            and set(evento) == {'id', 'direccion', 'color', 'marca_ns'}
            and type(evento['id']) is int and evento['id'] > 0
            and evento['direccion'] in ('IZQUIERDA', 'DERECHA')
            and evento['color'] in ('azul', 'rojo')
            and type(evento['marca_ns']) is int
            and 0 <= ahora_ns - evento['marca_ns'] <= 1_000_000_000)


class ControlGiro:
    """Una maniobra por evento. La salida continúa pasando por el permiso global.

    Al bloquearse no elige el sentido opuesto. Un fallo de giro se enclava hasta
    cerrar la prueba. Si esperar_pared_m > 0, memoriza la señal hasta acercarse
    a la pared frontal antes de ejecutar el giro de 90°.
    """

    def __init__(self, tramo, cfg=None):
        self.tramo = tramo
        self.color = COLORES_TRAMO[tramo]
        self.cfg = cfg or ConfigSenales()
        self.activo = None
        self.pendiente = None
        self.ultimo_id = 0
        self.ultima_posicion = None
        self.fallo = None

    def cambiar_tramo(self, tramo):
        if tramo not in COLORES_TRAMO:
            raise ValueError(f'Tramo inválido: {tramo}')
        if tramo != self.tramo:
            self.tramo = tramo
            self.color = COLORES_TRAMO[tramo]
            self.pendiente = None
            self.activo = None

    def decidir(self, evento, pose, ahora_ns, permitido, lidar, cfg_evasion):
        if self.fallo:
            return (0., 0., self.fallo)
        if self.activo and ahora_ns - self.activo['inicio_ns'] > self.cfg.tiempo_maximo_giro_s * 1e9:
            self.fallo = 'Giro sin completar a tiempo: detenido; cerrar y revisar'
            return (0., 0., self.fallo)
        if not permitido:
            return (0., 0., 'Giro en espera de permiso/sensores') if self.activo else None

        # Expirar señal memorizada si nunca apareció pared frontal en el tiempo límite.
        if (self.pendiente is not None
                and ahora_ns - self.pendiente['recibido_ns'] > self.cfg.tiempo_espera_pared_s * 1e9):
            self.pendiente = None

        if self.activo is None and evento is not None and evento['id'] > self.ultimo_id:
            self.ultimo_id = evento['id']
            if evento['color'] == self.color:
                muy_cerca = (
                    self.ultima_posicion is not None and hypot(
                        pose['x'] - self.ultima_posicion[0], pose['y'] - self.ultima_posicion[1]
                    ) < self.cfg.desplazamiento_rearme_m
                )
                if not muy_cerca:
                    self.pendiente = dict(
                        direccion=evento['direccion'],
                        sentido=1. if evento['direccion'] == 'IZQUIERDA' else -1.,
                        recibido_ns=ahora_ns,
                    )

        if self.activo is None and self.pendiente is not None:
            esperar = self.cfg.esperar_pared_m
            pared_cerca = (
                esperar <= 0.0
                or (lidar is not None and lidar.observacion_valida and lidar.dist_frente <= esperar)
            )
            if pared_cerca:
                self.activo = dict(
                    inicio_ns=ahora_ns,
                    yaw=pose['yaw'],
                    sentido=self.pendiente['sentido'],
                    direccion=self.pendiente['direccion'],
                )
                self.ultima_posicion = pose['x'], pose['y']
                self.pendiente = None

        if self.activo is None:
            return None  # La evasión conserva su propuesta mientras espera pared o no hay señal.

        a = self.activo
        delta = atan2(sin(pose['yaw'] - a['yaw']), cos(pose['yaw'] - a['yaw']))
        avance = a['sentido'] * delta
        if avance < -radians(20):
            self.fallo = 'Odometria indica giro contrario: detenido; revisar'
            return (0., 0., self.fallo)
        objetivo = radians(self.cfg.angulo_giro_deg)
        if avance > objetivo + radians(self.cfg.tolerancia_deg):
            self.fallo = 'Giro excedido segun odometria: detenido; revisar'
            return (0., 0., self.fallo)
        # La tolerancia limita el exceso; no permite reanudar antes de los 90°.
        if avance + 1e-9 >= objetivo:
            self.activo = None
            return (0., 0., 'Giro completado por odometria')
        if (lidar is None or not lidar.observacion_valida or lidar.proximidad_no_cuantificable):
            return (0., 0., 'Giro detenido: LiDAR/TF insuficientes')
        if min(lidar.dist_frente, lidar.dist_izquierda, lidar.dist_derecha) <= cfg_evasion.distancia_parada_m:
            return (0., 0., 'Giro bloqueado por obstaculo: esperando; no invierte sentido')
        restante = objetivo - avance
        piso = 0.25 if cfg_evasion.velocidad_giro >= 0.6 else 0.12
        ganancia = 1.1 if cfg_evasion.velocidad_giro >= 0.6 else 0.8
        velocidad = min(cfg_evasion.velocidad_giro, max(piso, restante * ganancia))
        return (0., a['sentido'] * velocidad, f"Giro {a['direccion']} por senal {self.color}")
