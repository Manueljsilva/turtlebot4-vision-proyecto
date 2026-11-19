"""Máquina de estados de misión completa: Ida (azul) -> Meta B (barrido 4 fotos) -> Vuelta (rojo) -> Meta A."""

from dataclasses import dataclass
from math import atan2, cos, hypot, radians, sin


@dataclass
class ConfigMision:
    distancia_exclusion_meta_m: float = 1.2
    angulo_barrido_deg: float = 360.0
    angulo_giro_retorno_deg: float = 180.0
    salir_anticipado_confianza: float = 0.72
    salir_anticipado_confirmaciones: int = 3
    max_imagenes_b: int = 4


class MisionMeta:
    """Coordina etapas de ida, barrido 360° en B, giro de retorno y vuelta hasta A."""

    def __init__(self, cfg=None):
        self.cfg = cfg or ConfigMision()
        self.etapa = 'ESPERANDO'
        self.tramo = 'ida'
        self.orden = 'ESPERAR'
        self.pose_a = None
        self.pose_b = None
        self._ultimo_yaw = None
        self._barrido_acum_rad = 0.0
        self._retorno_acum_rad = 0.0
        self._confirmaciones_rapidas = 0
        self.mejor_afinidad = None
        self.mejor_resultado = None
        self.mejor_yaw = None
        self.palabra_qr = None
        self.vistas_distintas = 0
        self._ultimo_angulo_foto = None

    def iniciar(self, pose_actual=None):
        self.etapa = 'IDA'
        self.tramo = 'ida'
        self.orden = 'BUSCAR'
        self.pose_a = dict(pose_actual) if pose_actual else None
        self.pose_b = None
        self._ultimo_yaw = None
        self._barrido_acum_rad = 0.0
        self._retorno_acum_rad = 0.0
        self._confirmaciones_rapidas = 0
        self.mejor_afinidad = None
        self.mejor_resultado = None
        self.mejor_yaw = None
        self.palabra_qr = None
        self.vistas_distintas = 0
        self._ultimo_angulo_foto = None

    @staticmethod
    def _distancia(p1, p2):
        if not p1 or not p2:
            return 0.0
        return hypot(p1['x'] - p2['x'], p1['y'] - p2['y'])

    @staticmethod
    def _delta_yaw(y1, y2):
        return atan2(sin(y2 - y1), cos(y2 - y1))

    def _registrar_rostro_en_b(self, resultado_facial, pose, qr_texto=None):
        if resultado_facial is None or resultado_facial.get('motivo') in (
            'SIN_ROSTRO', 'VARIOS_ROSTROS', 'ROSTRO_NO_UTIL', 'SIN_IMAGEN_RECIENTE', 'IMAGEN_INVALIDA'
        ):
            return False
        afinidad = resultado_facial.get('afinidad_objetivo')
        if afinidad is None:
            return False

        # Contar vistas angulares separadas (>= 45°) como referencia de las 4 imágenes.
        if pose is not None:
            if (self._ultimo_angulo_foto is None
                    or abs(self._delta_yaw(self._ultimo_angulo_foto, pose['yaw'])) >= radians(45)):
                self.vistas_distintas += 1
                self._ultimo_angulo_foto = pose['yaw']

        if self.mejor_afinidad is None or afinidad > self.mejor_afinidad:
            self.mejor_afinidad = float(afinidad)
            self.mejor_resultado = dict(resultado_facial)
            self.mejor_yaw = pose['yaw'] if pose else None
            if qr_texto:
                self.palabra_qr = qr_texto
        elif qr_texto and self.palabra_qr is None and resultado_facial.get('objetivo_confirmado'):
            self.palabra_qr = qr_texto

        if (resultado_facial.get('objetivo_confirmado')
                and afinidad >= self.cfg.salir_anticipado_confianza):
            self._confirmaciones_rapidas += 1
        else:
            self._confirmaciones_rapidas = 0
        return True

    def actualizar(self, pose, meta_res, resultado_facial, qr_texto=None):
        if self.etapa in ('ESPERANDO', 'FINALIZADO'):
            return self.orden

        if self.etapa == 'IDA':
            if self.pose_a is None and pose is not None:
                self.pose_a = dict(pose)
            lejos_de_a = (
                self.pose_a is None
                or pose is None
                or self._distancia(pose, self.pose_a) >= self.cfg.distancia_exclusion_meta_m
            )
            if meta_res and meta_res.get('confirmada') and lejos_de_a:
                self.etapa = 'BARRIDO_B'
                self.pose_b = dict(pose) if pose else None
                self._ultimo_yaw = pose['yaw'] if pose else 0.0
                self._barrido_acum_rad = 0.0
                self.orden = 'GIRAR_BARRIDO'
            else:
                self.orden = 'BUSCAR'
            return self.orden

        if self.etapa == 'BARRIDO_B':
            if pose is not None and self._ultimo_yaw is not None:
                self._barrido_acum_rad += abs(self._delta_yaw(self._ultimo_yaw, pose['yaw']))
                self._ultimo_yaw = pose['yaw']
            elif pose is not None:
                self._ultimo_yaw = pose['yaw']

            vio_rostro = self._registrar_rostro_en_b(resultado_facial, pose, qr_texto=qr_texto)
            completo_360 = self._barrido_acum_rad >= radians(self.cfg.angulo_barrido_deg)
            confirmo_claro = (
                self._confirmaciones_rapidas >= self.cfg.salir_anticipado_confirmaciones
                and self.mejor_resultado is not None
            )
            if completo_360 or confirmo_claro:
                self.etapa = 'GIRO_RETORNO_B'
                self._retorno_acum_rad = 0.0
                self._ultimo_yaw = pose['yaw'] if pose else None
                self.tramo = 'vuelta'
                self.orden = 'GIRAR_RETORNO'
            else:
                self.orden = 'GIRAR_BARRIDO_LENTO' if vio_rostro else 'GIRAR_BARRIDO'
            return self.orden

        if self.etapa == 'GIRO_RETORNO_B':
            if pose is not None and self._ultimo_yaw is not None:
                self._retorno_acum_rad += abs(self._delta_yaw(self._ultimo_yaw, pose['yaw']))
                self._ultimo_yaw = pose['yaw']
            elif pose is not None:
                self._ultimo_yaw = pose['yaw']

            if self._retorno_acum_rad >= radians(self.cfg.angulo_giro_retorno_deg):
                self.etapa = 'VUELTA'
                self.tramo = 'vuelta'
                self.orden = 'BUSCAR'
            else:
                self.orden = 'GIRAR_RETORNO'
            return self.orden

        if self.etapa == 'VUELTA':
            lejos_de_b = (
                self.pose_b is None
                or pose is None
                or self._distancia(pose, self.pose_b) >= self.cfg.distancia_exclusion_meta_m
            )
            if meta_res and meta_res.get('confirmada') and lejos_de_b:
                self.etapa = 'FINALIZADO'
                self.orden = 'ENCONTRADO'
            else:
                self.orden = 'BUSCAR'
            return self.orden

        return self.orden
