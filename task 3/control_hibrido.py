"""Arbitraje autónomo/gestos de Task 3, sin ROS ni percepción de manos."""
import fcntl
import json
from math import pi

from control_busqueda import reciente
from control_meta import ReposoOdometria
from control_mision import ControlMision, normalizar

MODOS = ('AUTONOMO', 'GESTOS')
GESTOS = ('DETENER', 'AVANZAR', 'IZQUIERDA', 'DERECHA')
EDAD_GESTO_NS = 500_000_000
CAMPOS = {'modo', 'modo_ns', 'gesto', 'gesto_ns'}


def adquirir_bloqueos(rutas=('/tmp/proyecto2-evasion-control.lock', '/tmp/proyecto2-puente-gestos.lock')):
    """Ambos controles existentes quedan excluidos; liberar todo si uno falla."""
    archivos = []
    try:
        for ruta in rutas:
            archivo = open(ruta, 'a')
            archivos.append(archivo)
            fcntl.flock(archivo, fcntl.LOCK_EX | fcntl.LOCK_NB)
        return archivos
    except BaseException:
        for archivo in archivos:
            archivo.close()
        raise


class ControlHibrido(ControlMision):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.modo = self.modo_solicitado = 'AUTONOMO'
        self.modo_ns = 0
        self.cambio_modo = False
        self.reposo_modo = ReposoOdometria()
        self.gesto = 'DETENER'
        self.gesto_ns = None
        self.gestos_desde_ns = 0
        self.giro_manual = False

    def recibir(self, linea, ahora):
        try:
            d = json.loads(linea)
            if not isinstance(d, dict) or len(linea) > 2500:
                raise ValueError('Protocolo híbrido inválido')
            presentes = CAMPOS & d.keys()
            extras = {k: d.pop(k) for k in presentes}
            if presentes and presentes != CAMPOS:
                raise ValueError('Faltan campos de modo/gestos')
            if extras:
                token = extras['modo_ns']
                marca = extras['gesto_ns']
                if (extras['modo'] not in MODOS or extras['gesto'] not in GESTOS
                        or type(token) is not int or not 0 <= token <= ahora
                        or d.get('orden') != 'REINICIAR' and (token < self.modo_ns
                            or token == self.modo_ns and extras['modo'] != self.modo_solicitado)
                        or marca is not None and (type(marca) is not int or not 0 <= marca <= ahora)):
                    raise ValueError('Modo o gesto inválido')
            anterior = self.secuencia
            super().recibir(json.dumps(d), ahora)
            if self.secuencia == anterior:
                self.gesto, self.gesto_ns = 'DETENER', None
                return
            if d['orden'] == 'REINICIAR':
                # El reinicio ya llama __init__; no restaurar muestras anteriores.
                return
            if extras:
                if extras['modo_ns'] > self.modo_ns:
                    if not reciente(extras['modo_ns'], ahora, 500_000_000):
                        raise ValueError('Cambio de modo caducado')
                    self.modo_ns = extras['modo_ns']
                    self.modo_solicitado = extras['modo']
                    self.cambio_modo = True
                    self.reposo_modo = ReposoOdometria()
                    self.limpiar_intenciones()
                marca = extras['gesto_ns']
                if (marca is not None and marca == self.gesto_ns and extras['gesto'] != self.gesto):
                    raise ValueError('Una muestra no puede cambiar de gesto')
                if marca is not None and (self.gesto_ns is None or marca >= self.gesto_ns):
                    self.gesto, self.gesto_ns = extras['gesto'], marca
                elif marca is None:
                    self.gesto, self.gesto_ns = 'DETENER', None
                else:
                    raise ValueError('Muestra gestual retrocedida')
            else:
                self.gesto, self.gesto_ns = 'DETENER', None
            if not self.autonomia_activa():
                self.senal = self.senal_por_procesar = None
        except (ValueError, TypeError, KeyError, OverflowError):
            self.actividad_ns = None
            self.gesto, self.gesto_ns = 'DETENER', None

    def limpiar_intenciones(self):
        girando_previo = (self.giro_manual or self.giros.activo is not None
                          or self.evasion is not None or self._girando_ruta is not None)
        self.giros = self.clase_giros('vuelta' if self.fase == 'VUELTA' else 'ida', self.cfg_giros_mision)
        self.senal = self.senal_por_procesar = self.ultima_senal_recibida = None
        self.conflicto_senal = self.evasion = self.escape_atras = None
        self.freno_flecha = self.bloqueo_aproximacion = self.salida_flecha = None
        self.aproximacion_ns = self.revalidacion_ns = self.aproximacion_meta_ns = None
        self._girando_ruta = self._ultimo_giro_ruta_pos = None
        self._paso_corto_hasta_ns = 0
        # Conservar la incertidumbre del último movimiento hasta recibir pose nueva.
        self.giro_manual = girando_previo

    def autonomia_activa(self):
        return self.modo == 'AUTONOMO' and not self.cambio_modo

    def preparar_modo(self, ahora, pose, pose_ns):
        if not self.cambio_modo:
            return None
        self.giro_manual = False
        if pose_ns < self.modo_ns or not self.reposo_modo.actualizar(pose, pose_ns, ahora):
            return 0., 0., 'Cambio de modo: frenando; esperar 0.7 s de odometria en reposo'
        self.modo = self.modo_solicitado
        self.cambio_modo = False
        self.gestos_desde_ns = ahora
        self.gesto, self.gesto_ns = 'DETENER', None
        if self.fase == 'BUSCAR_B':
            self.inicio_barrido_ns, self.yaw_barrido = ahora, self.yaw_total
        if self.fase == 'ORIENTAR_SALIDA':
            self.yaw_salida = self.yaw_total+normalizar(self.yaw_entrada_b+pi-pose['yaw'])
            self.yaw_inicio_salida, self.inicio_orientacion_ns = self.yaw_total, ahora
            self.sentido_salida = 1. if self.yaw_salida >= self.yaw_total else -1.
        return 0., 0., 'Modo '+self.modo+' preparado; esperar una orden nueva'

    def seguir_pose(self, ahora, pose, marca):
        # Mantener la protección de incertidumbre angular también en navegación manual.
        if self.giro_manual and self.pose_ns is not None and marca is not None and marca-self.pose_ns > 2_000_000_000:
            self.fallar('Corte odometrico durante giro manual; referencia incierta', ahora)
        return super().seguir_pose(ahora, pose, marca)

    def movimiento_manual(self, ahora, lidar, cfg, solo_giro=False):
        if self.autonomia_activa():
            return None
        self.giro_manual = False
        if self.orden in ('VERIFICAR', 'QR_LISTO'):
            return 0., 0., 'Objetivo localizado: verificar rostro/QR sin girar'
        limite = max(self.gestos_desde_ns, self.desde_ns or 0)
        if (self.cambio_modo or not reciente(self.gesto_ns, ahora, EDAD_GESTO_NS)
                or self.gesto_ns <= limite):
            return 0., 0., 'Gestos: esperar mano nueva y reciente'
        if self.gesto == 'AVANZAR' and not solo_giro:
            return self.aproximar(ahora, lidar, cfg, 'Avance recto por pasillo')
        if self.gesto in ('IZQUIERDA', 'DERECHA'):
            bloqueo = self.bloqueo_giro(lidar, cfg)
            if bloqueo:
                return 0., 0., 'Gesto bloqueado: '+bloqueo
            self.giro_manual = True
            velocidad = min(cfg.velocidad_giro, .25) if solo_giro else cfg.velocidad_giro
            return 0., velocidad*(1 if self.gesto == 'IZQUIERDA' else -1), 'Gesto '+self.gesto
        return 0., 0., 'Gestos: detenido'+('; en B solo se permite girar' if solo_giro else '')
