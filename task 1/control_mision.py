"""Ensayo guiado A-B-A: estados y salida única, sin ROS ni modelos de visión.

La odometría es local. Rojo guía el regreso; no se reproduce una ruta ni hay SLAM.
La bandera B se coloca al fondo, frente al sentido de entrada (ensayo acordado).
"""

from dataclasses import asdict, dataclass
import json
from math import atan2, cos, hypot, isfinite, pi, sin
from pathlib import Path

from control_busqueda import CONFIRMACIONES_VISUALES, EDAD_ACTIVIDAD_NS, EDAD_CAMARA_NS, EDAD_POSE_NS, reciente
from control_giros import ControlGiro, evento_valido
from control_meta import ReposoOdometria, misma_caja
from evasion_lidar import calcular_velocidad_evasion


def normalizar(a):
    return atan2(sin(a), cos(a))


@dataclass(frozen=True)
class ConfigMision:
    separacion_meta_m: float = .6
    ausencia_meta_s: float = 1.5
    radio_llegada_a_m: float = .6
    frente_meta_m: float = .6
    tiempo_maximo_mision_s: float = 900.
    tiempo_orientacion_s: float = 45.
    tiempo_aproximacion_s: float = 30.
    velocidad_aproximacion_m_s: float = .05
    margen_salida_evasion_m: float = .04

    def __post_init__(self):
        limites = ((.4, 2.), (1., 5.), (.2, 1.), (.4, .8), (60., 1800.), (20., 90.), (10., 120.),
                   (.02, .10), (.02, .08))
        if not all(type(v) in (int, float) and isfinite(v) and a <= v <= b
                   for v, (a, b) in zip(asdict(self).values(), limites)):
            raise ValueError('Configuracion de mision fuera de limites')


def cargar_config_mision(ruta):
    d = json.loads(Path(ruta).read_text(encoding='utf-8'))
    if not isinstance(d, dict):
        raise ValueError('La configuracion de mision debe ser un objeto')
    return ConfigMision(**d)


class MetaNueva:
    """Ausencia en imágenes nuevas antes de aceptar otra meta; voto no repetido."""

    def __init__(self, desde_ns, cfg):
        self.desde_ns, self.cfg = desde_ns, cfg
        self.ultima = self.ausente_desde = None
        self.salida_visual = False
        self.votos, self.caja = 0, None

    def caducar(self):
        self.ausente_desde = None
        self.votos, self.caja = 0, None

    def actualizar(self, caja, marca, admisible):
        if marca <= self.desde_ns or self.ultima is not None and marca <= self.ultima:
            return False
        if self.ultima is not None and marca-self.ultima > EDAD_CAMARA_NS:
            self.caducar()
        self.ultima = marca
        if caja is None:
            if self.ausente_desde is None:
                self.ausente_desde = marca
            if marca-self.ausente_desde >= self.cfg.ausencia_meta_s*1e9:
                self.salida_visual = True
        else:
            self.ausente_desde = None
        valida = (self.salida_visual and admisible and caja is not None
                  and abs(caja[0]+caja[2]/2-.5) <= .2)
        # misma_caja trabaja en píxeles; conservar la escala normalizada en transporte.
        actual = [v*1000 for v in caja] if valida else None
        self.votos = (min(CONFIRMACIONES_VISUALES, self.votos+1)
                      if misma_caja(actual, self.caja) else 1) if valida else 0
        self.caja = actual
        return self.votos >= CONFIRMACIONES_VISUALES


class ControlMision:
    """No habilita vuelta antes del QR ni llegada A sin separación y pose inicial."""

    def __init__(self, cfg_senales, cfg=None, clase_giros=ControlGiro):
        self.cfg = cfg or ConfigMision()
        self.cfg_senales = cfg_senales
        self.clase_giros = clase_giros
        self.fase, self.desde_ns = 'ESPERAR', None
        self.motivo = 'Elegir persona y pulsar Buscar'
        self.orden, self.secuencia = 'ESPERAR', -1
        self.reinicio_ns = 0
        self.actividad_ns = self.camara_ns = self.solicitud_ns = None
        self.senal = self.meta = self.qr_ns = None
        self.qr_votos = 0
        self.a = self.b = self.yaw_entrada_b = None
        self.qr_confirmado = False
        self.pose = self.pose_ns = None
        self.yaw_total = 0.
        self.recuperando = False
        self.reposo_odom = ReposoOdometria()
        self.reposo = ReposoOdometria()
        self.busqueda_desde_ns = None
        self.inicio_barrido_ns = self.yaw_barrido = None
        self.yaw_salida = self.inicio_orientacion_ns = None
        self.yaw_inicio_salida = self.sentido_salida = None
        self.aproximacion_ns = self.revalidacion_ns = None
        self.giros = clase_giros('ida', cfg_senales)
        if not hasattr(self.giros, 'pendiente') or cfg_senales.esperar_pared_m <= 0:
            raise ValueError('La mision requiere el controlador con espera de pared')
        self.meta_nueva = None
        self.diagnostico_meta = 'Esperando inicio en A'
        self.conflicto_senal = None
        self.evasion = None

    def cambiar(self, fase, ahora, motivo):
        self.fase, self.desde_ns, self.motivo = fase, ahora, motivo
        self.reposo = ReposoOdometria()
        if fase not in ('IDA', 'FALLO'):
            self.evasion = None

    def fallar(self, motivo, ahora):
        if self.fase not in ('FALLO', 'FIN'):
            self.cambiar('FALLO', ahora, motivo)

    def recibir(self, linea, ahora):
        try:
            d = json.loads(linea)
            base = {'version', 'secuencia', 'orden', 'actividad_ns', 'camara_ns'}
            extras = {'senal', 'meta', 'qr_ns', 'qr_votos'}
            if (len(linea) > 2048 or not isinstance(d, dict) or set(d) not in (base, base|extras, base|extras|{'reinicio_ns'})
                    or type(d['version']) is not int or d['version'] != 1
                    or type(d['secuencia']) is not int or d['secuencia'] <= self.secuencia
                    or d['orden'] not in ('ESPERAR', 'INICIAR', 'OBSERVAR', 'VERIFICAR', 'QR_LISTO', 'PARAR', 'REINICIAR')
                    or not reciente(d['actividad_ns'], ahora, EDAD_ACTIVIDAD_NS)
                    or (d['camara_ns'] is not None and type(d['camara_ns']) is not int)
                    or not evento_valido(d.get('senal'), ahora)):
                raise ValueError('Mensaje invalido')
            if set(d) == base and d['orden'] not in ('ESPERAR', 'PARAR'):
                raise ValueError('Falta percepcion de mision')
            meta = d.get('meta')
            if meta is not None and (not isinstance(meta, list) or len(meta) != 4
                    or not all(type(v) in (int, float) and isfinite(v) and 0 <= v <= 1 for v in meta)
                    or min(meta[2:]) <= 0 or meta[0]+meta[2] > 1.001 or meta[1]+meta[3] > 1.001):
                raise ValueError('Meta invalida')
            if ((d.get('qr_ns') is not None and type(d['qr_ns']) is not int)
                    or type(d.get('qr_votos', 0)) is not int or not 0 <= d.get('qr_votos', 0) <= CONFIRMACIONES_VISUALES):
                raise ValueError('Confirmacion QR invalida')
            # PARAR también llega desde el cierre compartido sin identificador
            # de intento. Siempre puede detener; nunca habilita movimiento.
            reinicio = self.reinicio_ns if d['orden'] == 'PARAR' else d.get('reinicio_ns', 0)
            if type(reinicio) is not int or reinicio < self.reinicio_ns:
                raise ValueError('Intento anterior o reinicio invalido')
            if d['orden'] == 'REINICIAR':
                if (reinicio <= 0 or d['camara_ns'] is not None or meta is not None
                        or d.get('senal') is not None or d.get('qr_ns') is not None or d.get('qr_votos', 0)):
                    raise ValueError('Reinicio no puede incluir percepcion anterior')
                if reinicio > self.reinicio_ns:
                    if not reciente(reinicio, ahora, EDAD_ACTIVIDAD_NS):
                        raise ValueError('Solicitud de reinicio caducada')
                    # Solo un clic nuevo borra el intento. Repetir su latido no
                    # reinicia el reposo, el secuenciador ni crea otro publicador.
                    secuencia = self.secuencia
                    self.__init__(self.cfg_senales, self.cfg, self.clase_giros)
                    self.secuencia, self.reinicio_ns = secuencia, reinicio
                    self.cambiar('REINICIAR', ahora, 'Reiniciando: detener y confirmar reposo')
            elif reinicio != self.reinicio_ns or (self.fase == 'REINICIAR' and d['orden'] not in ('PARAR', 'ESPERAR')):
                raise ValueError('Esperar confirmacion del reinicio')
            self.secuencia, self.orden = d['secuencia'], d['orden']
            self.actividad_ns, self.camara_ns = d['actividad_ns'], d['camara_ns']
            self.senal, self.meta = d.get('senal'), meta
            self.qr_ns, self.qr_votos = d.get('qr_ns'), d.get('qr_votos', 0)
            if self.orden == 'INICIAR' and self.solicitud_ns is None:
                self.solicitud_ns = self.actividad_ns
                if self.reinicio_ns and self.fase == 'ESPERAR':
                    self.pose = self.pose_ns = None  # Puede haberse recolocado entre intentos.
            if self.orden == 'PARAR':
                self.fallar('Detenido por interfaz', ahora)
        except (ValueError, TypeError, KeyError, OverflowError):
            self.actividad_ns = self.camara_ns = None
            self.senal = self.meta = None

    def seguir_pose(self, ahora, pose, marca):
        if (pose is None or not reciente(marca, ahora, EDAD_POSE_NS)
                or not all(isfinite(pose[k]) for k in ('x', 'y', 'yaw'))):
            return False
        if self.pose is not None:
            if any(pose[k] != self.pose[k] for k in ('frame_id', 'child_frame_id')):
                self.fallar('Cambio de marco de odometria', ahora)
            if marca != self.pose_ns:
                dt = marca-self.pose_ns
                dyaw = normalizar(pose['yaw']-self.pose['yaw'])
                dxy = hypot(pose['x']-self.pose['x'], pose['y']-self.pose['y'])
                if dt < 0 or abs(dyaw) > .5 or dxy > .25:
                    self.fallar(f'Salto odometrico: dt={dt/1e9:.2f}s dxy={dxy:.3f} dyaw={dyaw:.3f}', ahora)
                elif dt > EDAD_POSE_NS:
                    if (self.fase in ('BUSCAR_B', 'ORIENTAR_SALIDA') or self.giros.activo is not None
                            or self.evasion is not None) and dt > 2_000_000_000:
                        self.fallar('Corte odometrico >2s durante giro: angulo incierto', ahora)
                    self.recuperando, self.reposo_odom = True, ReposoOdometria()
                self.yaw_total += dyaw
        self.pose, self.pose_ns = dict(pose), marca
        if self.recuperando and self.reposo_odom.actualizar(pose, marca, ahora):
            self.recuperando = False
        return self.fase != 'FALLO'

    @staticmethod
    def libre(lidar, cfg):
        return (lidar is not None and lidar.observacion_valida and not lidar.proximidad_no_cuantificable
                and min(lidar.dist_frente, lidar.dist_izquierda, lidar.dist_derecha) > cfg.distancia_parada_m)

    def diagnostico_giro(self, ahora, lidar):
        def distancia(campo):
            valor=getattr(lidar,campo,None) if lidar and lidar.observacion_valida else None
            return valor if valor is not None and isfinite(valor) else None
        pendiente=self.giros.pendiente
        activo=self.giros.activo
        return dict(color=self.giros.color, recibido=dict(self.senal) if self.senal else None,
            pendiente=dict(id=self.giros.ultimo_id,direccion=pendiente['direccion'],
                edad_revalidacion_s=(ahora-pendiente['recibido_ns'])/1e9) if pendiente else None,
            activo=activo['direccion'] if activo else None, conflicto=self.conflicto_senal,
            evasion=dict(self.evasion) if self.evasion else None,
            frente_m=distancia('dist_frente'), izquierda_m=distancia('dist_izquierda'),
            derecha_m=distancia('dist_derecha'), disparo_m=self.cfg_senales.esperar_pared_m)

    def evadir(self, ahora, lidar, cfg):
        """Conservar el lado del giro frontal hasta salir con margen, sin arcos."""
        if lidar is None or not lidar.observacion_valida or lidar.proximidad_no_cuantificable:
            return 0., 0., 'Evasion detenida: LiDAR/TF insuficientes'
        vx, wz = calcular_velocidad_evasion(lidar, cfg)
        if self.evasion is None and lidar.dist_frente <= cfg.distancia_precaucion_m:
            self.evasion=dict(sentido=1. if lidar.dist_izquierda >= lidar.dist_derecha else -1.,
                              inicio_ns=ahora, yaw=self.yaw_total)
        if self.evasion is None:
            return vx, wz, 'Correccion lateral sobre el sitio' if wz else 'Avance recto con evasion'
        e=self.evasion
        avance=e['sentido']*(self.yaw_total-e['yaw'])
        if (ahora-e['inicio_ns'] > self.cfg_senales.tiempo_maximo_giro_s*1e9
                or avance >= self.cfg_senales.angulo_giro_deg*pi/180 or avance < -.35):
            self.fallar('Evasion sin salida o giro incoherente: detenido; revisar antes de reiniciar',ahora)
            return 0., 0., self.motivo
        if lidar.dist_frente > cfg.distancia_precaucion_m+self.cfg.margen_salida_evasion_m:
            self.evasion=None
            return 0., 0., 'Frente libre con margen: terminar giro evasivo antes de avanzar'
        lateral=lidar.dist_izquierda if e['sentido'] > 0 else lidar.dist_derecha
        if lateral <= cfg.distancia_parada_m:
            return 0., 0., 'Evasion bloqueada hacia el lado elegido: detenido sin invertir'
        velocidad=cfg.velocidad_giro if lidar.dist_frente <= cfg.distancia_parada_m else cfg.velocidad_giro*.75
        lado='IZQUIERDA' if e['sentido'] > 0 else 'DERECHA'
        return 0., e['sentido']*velocidad, f'Evasion {lado} mantenida hasta frente libre con margen'

    def decidir(self, ahora, pose, pose_ns, lidar, cfg, fin=False):
        def cero(motivo):
            return 0., 0., motivo
        if self.fase == 'REINICIAR':
            if fin:
                self.fallar('Control cerrado durante reinicio', ahora)
            elif (not reciente(self.actividad_ns, ahora, EDAD_ACTIVIDAD_NS) or self.orden != 'REINICIAR'
                    or pose_ns is None or pose_ns < self.reinicio_ns):
                self.reposo = ReposoOdometria()
                self.motivo = 'Reiniciando: esperar control y odometria nuevos; detenido'
            elif self.reposo.actualizar(pose, pose_ns, ahora):
                self.pose, self.pose_ns = dict(pose), pose_ns
                self.cambiar('ESPERAR', ahora, 'Reinicio confirmado. Colocar en A, elegir persona y pulsar Buscar')
                self.orden = 'ESPERAR'
            return cero(self.motivo)
        self.diagnostico_meta = 'Meta solo se confirma durante ida o vuelta'
        if self.reinicio_ns and self.fase == 'ESPERAR' and self.solicitud_ns is None:
            # Sin intento activo no hay continuidad de ruta que conservar.
            # Recolocar el robot no autoriza avanzar ni registra A por sí solo.
            self.pose = self.pose_ns = None
        pose_ok = self.seguir_pose(ahora, pose, pose_ns)  # También durante pausa de cámara/GUI.
        if fin:
            self.fallar('Control cerrado', ahora)
        if self.fase in ('FALLO', 'FIN'):
            return cero(self.motivo)
        if self.solicitud_ns is not None and ahora-self.solicitud_ns > self.cfg.tiempo_maximo_mision_s*1e9:
            self.fallar('Tiempo total de ensayo agotado', ahora)
            return cero(self.motivo)
        if self.fase == 'BUSCAR_B' and ahora-self.inicio_barrido_ns >= 90_000_000_000:
            self.fallar('Barrido agotado (90 s)', ahora)
        if self.inicio_orientacion_ns is not None and self.fase in ('ORIENTAR_SALIDA', 'FRENAR_SALIDA'):
            if ahora-self.inicio_orientacion_ns > self.cfg.tiempo_orientacion_s*1e9:
                self.fallar('Orientacion de salida agotada', ahora)
        if self.fase == 'FALLO':
            return cero(self.motivo)
        if not reciente(self.actividad_ns, ahora, EDAD_ACTIVIDAD_NS):
            self.diagnostico_meta = 'Esperando actividad reciente'
            if self.meta_nueva:
                self.meta_nueva.caducar()
            return cero('Sin actividad reciente: detenido')
        if self.orden in ('ESPERAR', 'REINICIAR'):
            self.diagnostico_meta = 'Esperando Buscar'
            return cero('Esperando Buscar')
        if not reciente(self.camara_ns, ahora, EDAD_CAMARA_NS):
            self.diagnostico_meta = 'Esperando imagen reciente'
            if self.meta_nueva:
                self.meta_nueva.caducar()
            return cero('Sin camara reciente: detenido')
        if not pose_ok or self.recuperando:
            self.diagnostico_meta = 'Esperando odometria valida/estable'
            if self.meta_nueva:
                self.meta_nueva.caducar()
            return cero('Esperando odometria reciente y 0.7s estables tras cortes')
        if self.fase == 'ESPERAR':
            if self.solicitud_ns is None or pose_ns < self.solicitud_ns:
                return cero('Esperando odometria posterior a Buscar para registrar A')
            self.a = dict(pose)
            self.cambiar('IDA', ahora, 'A registrado en odometria; seguir azul')
            self.meta_nueva = MetaNueva(ahora, self.cfg)
            return cero(self.motivo)
        if self.fase in ('IDA', 'VUELTA'):
            origen = self.a if self.fase == 'IDA' else self.b
            separado = hypot(pose['x']-origen['x'], pose['y']-origen['y']) >= self.cfg.separacion_meta_m
            cerca_a = hypot(pose['x']-self.a['x'], pose['y']-self.a['y']) <= self.cfg.radio_llegada_a_m
            admisible = (separado and (self.fase == 'IDA' or cerca_a) and self.libre(lidar, cfg)
                         and lidar.dist_frente <= self.cfg.frente_meta_m and self.giros.activo is None)
            confirmada = self.meta_nueva.actualizar(self.meta, self.camara_ns, admisible)
            if self.meta is None:
                self.diagnostico_meta = 'Sin bandera detectada en la imagen'
            elif not self.meta_nueva.salida_visual:
                self.diagnostico_meta = f'Bandera vista; falta ausencia previa >= {self.cfg.ausencia_meta_s:.1f}s'
            elif not separado:
                self.diagnostico_meta = f'Bandera vista; separarse >= {self.cfg.separacion_meta_m:.2f}m del origen del tramo'
            elif self.fase == 'VUELTA' and not cerca_a:
                self.diagnostico_meta = f'Bandera vista; aun fuera del radio de A ({self.cfg.radio_llegada_a_m:.2f}m)'
            elif abs(self.meta[0]+self.meta[2]/2-.5) > .2:
                self.diagnostico_meta = 'Bandera vista; centrarla en la camara'
            elif self.giros.activo is not None:
                self.diagnostico_meta = 'Bandera vista; completar primero el giro por flecha'
            elif not self.libre(lidar, cfg):
                self.diagnostico_meta = 'Bandera vista; LiDAR/TF o espacio de giro insuficientes'
            elif lidar.dist_frente > self.cfg.frente_meta_m:
                self.diagnostico_meta = f'Bandera vista; frente {lidar.dist_frente:.2f}m > {self.cfg.frente_meta_m:.2f}m'
            else:
                self.diagnostico_meta = f'Bandera candidata: {self.meta_nueva.votos}/{CONFIRMACIONES_VISUALES} imagen nueva valida'
            if confirmada:
                self.diagnostico_meta = 'Meta confirmada en una imagen nueva valida'
                if self.fase == 'IDA':
                    self.b = dict(pose)
                    self.cambiar('FRENAR_B', ahora, 'Meta B confirmada; frenar antes de buscar')
                else:
                    self.cambiar('FRENAR_A', ahora, 'Meta nueva cerca de A; confirmar reposo final')
                return cero(self.motivo)
            # No reutilizar flechas anteriores al cambio de tramo o caducadas.
            if self.cfg_senales.esperar_pared_m <= cfg.distancia_parada_m+.05:
                self.fallar('Disparo de flecha sin margen suficiente sobre parada: revisar configuracion',ahora)
                return cero(self.motivo)
            ev = self.senal if self.senal and self.senal['marca_ns'] > self.desde_ns else None
            if not evento_valido(ev, ahora):
                ev = None
            # Solo imágenes nuevas que vuelven a confirmar el mismo cartel renuevan
            # la espera. El latido y la repetición de un paquete no lo hacen.
            if self.giros.pendiente:
                if ahora-self.aproximacion_ns > self.cfg.tiempo_aproximacion_s*1e9:
                    self.fallar('Aproximacion a flecha agotada: detenido', ahora)
                    return cero(self.motivo)
                if ev and ev['id'] == self.giros.ultimo_id and ev['color'] == self.giros.color:
                    if ev['direccion'] != self.giros.pendiente['direccion']:
                        self.conflicto_senal=dict(id=ev['id'],color=ev['color'],
                            esperada=self.giros.pendiente['direccion'],observada=ev['direccion'],
                            marca_ns=ev['marca_ns'])
                        self.fallar(f"Cambio de sentido del cartel pendiente: {self.conflicto_senal['esperada']} "
                                    f"-> {ev['direccion']} (id {ev['id']}): detenido", ahora)
                        return cero(self.motivo)
                    if self.revalidacion_ns is None or ev['marca_ns'] > self.revalidacion_ns:
                        self.giros.pendiente['recibido_ns'] = ev['marca_ns']
                        self.revalidacion_ns = ev['marca_ns']
            tenia_pendiente = self.giros.pendiente is not None
            giro = self.giros.decidir(ev, pose, ahora, True, lidar, cfg)
            if self.giros.pendiente and not tenia_pendiente:
                self.aproximacion_ns = ahora
                self.revalidacion_ns = ev['marca_ns'] if ev else None
            elif self.giros.pendiente is None:
                self.aproximacion_ns = self.revalidacion_ns = None
            if tenia_pendiente and self.giros.pendiente is None and self.giros.activo is None and giro is None:
                self.fallar('Flecha pendiente caducada: cerrar y revisar antes de continuar', ahora)
                return cero(self.motivo)
            if self.giros.fallo:
                self.fallar(self.giros.fallo, ahora)
            if giro is not None:
                self.evasion=None
                return giro
            if self.giros.pendiente:
                self.evasion=None
                if not self.libre(lidar,cfg):
                    return cero('Flecha pendiente: aproximacion bloqueada por LiDAR/TF o espacio reducido')
                velocidad=(min(cfg.velocidad_avance,self.cfg.velocidad_aproximacion_m_s)
                           if lidar.dist_frente <= cfg.distancia_precaucion_m else cfg.velocidad_avance)
                return velocidad, 0., 'Flecha pendiente: aproximar recto; lento en precaucion hasta el disparo'
            return self.evadir(ahora,lidar,cfg)
        if self.fase == 'FRENAR_B':
            if self.reposo.actualizar(pose, pose_ns, ahora):
                self.yaw_entrada_b = pose['yaw']
                self.busqueda_desde_ns = self.inicio_barrido_ns = ahora
                self.yaw_barrido = self.yaw_total
                self.cambiar('BUSCAR_B', ahora, 'Frenado confirmado; buscar persona y su QR')
            return cero(self.motivo)
        if self.fase == 'BUSCAR_B':
            if self.orden == 'QR_LISTO':
                if (self.qr_votos == CONFIRMACIONES_VISUALES and reciente(self.qr_ns, ahora, EDAD_CAMARA_NS)
                        and self.qr_ns > self.busqueda_desde_ns):
                    self.qr_confirmado = True
                    self.cambiar('FRENAR_QR', ahora, 'QR confirmado; preparar salida sin avance')
                return cero(self.motivo)
            barrido = self.yaw_total-self.yaw_barrido
            if barrido < -.08 or barrido >= 2*pi-.025:
                self.fallar('Barrido contrario o vuelta completa sin QR confirmado', ahora)
                return cero(self.motivo)
            if self.orden == 'VERIFICAR':
                return cero('Objetivo localizado; verificar QR sin girar')
            if not self.libre(lidar, cfg):
                return cero('Barrido bloqueado por LiDAR/TF o espacio reducido')
            return 0., min(.25, cfg.velocidad_giro), 'Barrido sobre el sitio'
        if self.fase == 'FRENAR_QR':
            if self.reposo.actualizar(pose, pose_ns, ahora):
                self.yaw_salida = self.yaw_total+normalizar(self.yaw_entrada_b+pi-pose['yaw'])
                self.yaw_inicio_salida = self.yaw_total
                self.sentido_salida = 1. if self.yaw_salida >= self.yaw_total else -1.
                self.inicio_orientacion_ns = ahora
                self.cambiar('ORIENTAR_SALIDA', ahora, 'Orientar a entrada B +180 grados')
            return cero(self.motivo)
        if self.fase == 'ORIENTAR_SALIDA':
            error = self.yaw_salida-self.yaw_total
            if (self.sentido_salida*(self.yaw_total-self.yaw_inicio_salida) < -.35
                    or self.sentido_salida*error < -5*pi/180):
                self.fallar('Giro de salida contrario o excedido: detenido', ahora)
                return cero(self.motivo)
            if abs(error) <= 5*pi/180:
                self.cambiar('FRENAR_SALIDA', ahora, 'Salida orientada; confirmar reposo antes de rojo')
                return cero(self.motivo)
            if abs(error) > pi+.15:
                self.fallar('Orientacion de salida incoherente', ahora)
                return cero(self.motivo)
            if not self.libre(lidar, cfg):
                return cero('Orientacion bloqueada: conservar QR y detener')
            return 0., (1 if error > 0 else -1)*min(.25, cfg.velocidad_giro, max(.12, abs(error)*.8)), self.motivo
        if self.fase == 'FRENAR_SALIDA':
            if self.reposo.actualizar(pose, pose_ns, ahora):
                self.giros = self.clase_giros('vuelta', self.cfg_senales)
                self.cambiar('VUELTA', ahora, 'Salida preparada; seguir rojo')
                self.meta_nueva = MetaNueva(ahora, self.cfg)
            return cero(self.motivo)
        if self.fase == 'FRENAR_A':
            if self.reposo.actualizar(pose, pose_ns, ahora):
                self.cambiar('FIN', ahora, 'Meta A y reposo confirmados; mostrar palabra guardada')
            return cero(self.motivo)
        self.fallar('Estado de mision desconocido', ahora)
        return cero(self.motivo)
