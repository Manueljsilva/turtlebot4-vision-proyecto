"""Giro por señal y odometría, sin ROS ni OpenCV. No implementa un retorno."""

from math import atan2, cos, hypot, radians, sin

from config_senales import ConfigSenales, cargar_config_senales


COLORES_TRAMO = {'ida': 'azul', 'vuelta': 'rojo'}


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
    cerrar la prueba. La holgura usa los sectores existentes, no certifica huella.
    """

    def __init__(self, tramo, cfg=None):
        self.color = COLORES_TRAMO[tramo]
        self.cfg = cfg or ConfigSenales()
        self.activo = None
        self.ultimo_id = 0
        self.ultima_posicion = None
        self.fallo = None

    def decidir(self, evento, pose, ahora_ns, permitido, lidar, cfg_evasion):
        if self.fallo:
            return (0., 0., self.fallo)
        if self.activo and ahora_ns - self.activo['inicio_ns'] > self.cfg.tiempo_maximo_giro_s * 1e9:
            self.fallo = 'Giro sin completar a tiempo: detenido; cerrar y revisar'
            return (0., 0., self.fallo)
        if not permitido:
            return (0., 0., 'Giro en espera de permiso/sensores') if self.activo else None
        if self.activo is None and evento is not None and evento['id'] > self.ultimo_id:
            self.ultimo_id = evento['id']
            if evento['color'] != self.color:
                return None
            if self.ultima_posicion is not None and hypot(
                    pose['x'] - self.ultima_posicion[0], pose['y'] - self.ultima_posicion[1]
                    ) < self.cfg.desplazamiento_rearme_m:
                return None  # Evita repetir un cartel al reaparecer después del giro.
            self.activo = dict(inicio_ns=ahora_ns, yaw=pose['yaw'],
                               sentido=1. if evento['direccion'] == 'IZQUIERDA' else -1.,
                               direccion=evento['direccion'])
            self.ultima_posicion = pose['x'], pose['y']
        if self.activo is None:
            return None  # La evasión conserva su propuesta.
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
        velocidad = min(cfg_evasion.velocidad_giro, max(.12, restante * .8))
        return (0., a['sentido'] * velocidad, f"Giro {a['direccion']} por senal {self.color}")
