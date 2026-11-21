"""Ensayo guiado A-B-A: estados y salida única, sin ROS ni modelos de visión.

La odometría es local. Rojo guía el regreso; no se reproduce una ruta ni hay SLAM.
La bandera B se coloca al fondo, frente al sentido de entrada (ensayo acordado).
"""

from dataclasses import asdict, dataclass, is_dataclass, replace
import json
from math import atan2, cos, hypot, isfinite, pi, sin
from pathlib import Path
from types import SimpleNamespace

from control_busqueda import CONFIRMACIONES_VISUALES, EDAD_ACTIVIDAD_NS, EDAD_CAMARA_NS, EDAD_POSE_NS, reciente
from control_giros import ControlGiro, evento_valido
from control_meta import ReposoOdometria, misma_caja
from evasion_lidar import calcular_velocidad_evasion
from holgura_giro import distancia_libre_recta


def normalizar(a):
    return atan2(sin(a), cos(a))


@dataclass(frozen=True)
class ConfigMision:
    separacion_meta_m: float = .6
    ausencia_meta_s: float = 1.5
    radio_llegada_a_m: float = .6
    frente_meta_m: float = .4
    tiempo_maximo_mision_s: float = 900.
    tiempo_orientacion_s: float = 45.
    tiempo_aproximacion_s: float = 30.
    velocidad_aproximacion_m_s: float = .05
    margen_salida_evasion_m: float = .04
    radio_giro_m: float = .22
    margen_giro_m: float = .03
    margen_aproximacion_m: float = .02
    tolerancia_disparo_m: float = .005

    def __post_init__(self):
        limites = ((.4, 2.), (1., 5.), (.2, 1.), (.4, .8), (60., 1800.), (20., 90.), (10., 120.),
                   (.02, .10), (.02, .08), (.15, .5), (.02, .10), (.01, .10), (0., .01))
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
        self.senal_por_procesar = self.ultima_senal_recibida = None
        self.qr_votos = 0
        self.qr_pendiente = None
        self.qr_reintentar_ns = None
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
        self.aproximacion_meta_ns = None
        # El disparo nominal conserva su unidad/rango LiDAR. La tolerancia de
        # 5 mm evita exigir los últimos milímetros que el margen de avance veta.
        # No modifica radio, holgura ni la copia original del compañero.
        # Adaptación exclusiva de misión: la primera flecha aceptada se guarda
        # hasta el umbral, con plazo total acotado; no exige seguir viéndola.
        # Los perfiles/validación de los ensayos aislados siguen intactos.
        parametros = asdict(cfg_senales)
        parametros.update(esperar_pared_m=cfg_senales.esperar_pared_m+self.cfg.tolerancia_disparo_m,
                          tiempo_espera_pared_s=self.cfg.tiempo_aproximacion_s,
                          desplazamiento_rearme_m=0.)
        # Solo misión: ID nuevo y rearme visual sustituyen el mínimo de 0.40 m.
        # No cambiar los perfiles ni los bytes de los ensayos originales.
        self.cfg_giros_mision = SimpleNamespace(**parametros)
        self.giros = clase_giros('ida', self.cfg_giros_mision)
        if not hasattr(self.giros, 'pendiente') or cfg_senales.esperar_pared_m <= 0:
            raise ValueError('La mision requiere el controlador con espera de pared')
        self.meta_nueva = None
        self.diagnostico_meta = 'Esperando inicio en A'
        self.conflicto_senal = None
        self.evasion = None
        self.holgura = None
        self.freno_flecha = None
        self.bloqueo_aproximacion = None
        self.salida_flecha = None
        self.ultimo_giro_completado = None

    def cambiar(self, fase, ahora, motivo):
        self.fase, self.desde_ns, self.motivo = fase, ahora, motivo
        self.reposo = ReposoOdometria()
        if fase != 'BUSCAR_B':
            self.qr_pendiente = None
            self.qr_reintentar_ns = None
        if fase not in ('IDA', 'FALLO'):
            self.evasion = None
        if fase not in ('IDA','VUELTA','FALLO'):
            self.freno_flecha = self.bloqueo_aproximacion = None
            self.aproximacion_meta_ns = None
            self.senal_por_procesar = None
            self.aproximacion_ns = self.revalidacion_ns = None
            self.salida_flecha = None

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
            if (self.fase == 'BUSCAR_B' and self.orden == 'QR_LISTO'
                    and self.busqueda_desde_ns is not None
                    and self.qr_votos == CONFIRMACIONES_VISUALES
                    and self.qr_ns is not None
                    and self.qr_ns > self.busqueda_desde_ns):
                if reciente(self.qr_ns, ahora, EDAD_CAMARA_NS):
                    self.qr_pendiente = self.qr_ns
                    self.qr_reintentar_ns = None
                elif self.qr_ns <= ahora and self.qr_pendiente is None:
                    self.qr_reintentar_ns = self.qr_ns
            if self.orden == 'INICIAR' and self.solicitud_ns is None:
                self.solicitud_ns = self.actividad_ns
                if self.reinicio_ns and self.fase == 'ESPERAR':
                    self.pose = self.pose_ns = None  # Puede haberse recolocado entre intentos.
            if self.orden == 'PARAR':
                self.fallar('Detenido por interfaz', ahora)
            if (self.fase in ('IDA','VUELTA') and self.orden == 'OBSERVAR'
                    and self.senal is not None and self.senal['marca_ns'] > self.desde_ns
                    and self.senal['color'] == self.giros.color):
                # Acusar recepción antes de comprobar sensores: recordar una
                # intención no autoriza movimiento. Un fotograma posterior sin
                # flecha no debe borrarla durante una pausa de odometría.
                ev = self.senal
                self.ultima_senal_recibida = dict(ev)
                guardada = self.giros.pendiente or self.senal_por_procesar
                if guardada and ev['direccion'] != guardada['direccion']:
                    self.conflicto_senal=dict(id=ev['id'],color=ev['color'],
                        esperada=guardada['direccion'],observada=ev['direccion'],marca_ns=ev['marca_ns'])
                    self.fallar(f"Cambio de sentido del cartel pendiente: {guardada['direccion']} "
                                f"-> {ev['direccion']} (id {ev['id']}): detenido",ahora)
                elif (not guardada and self.giros.activo is None and ev['id'] > self.giros.ultimo_id):
                    self.senal_por_procesar = dict(ev)
                    self.aproximacion_ns = ahora
                if self.giros.activo is None:
                    self.revalidacion_ns = ev['marca_ns']
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
    def observado(lidar):
        return (lidar is not None and lidar.observacion_valida and not lidar.proximidad_no_cuantificable
                and all(isfinite(v) and v > 0 for v in
                        (lidar.dist_frente, lidar.dist_izquierda, lidar.dist_derecha)))

    def libre(self, lidar, cfg):
        return self.bloqueo_giro(lidar, cfg) is None

    def bloqueo_giro(self, lidar, cfg):
        """Disco barrido centrado en base_link, independiente de parada del avance."""
        if not self.observado(lidar):
            return 'LiDAR/TF insuficientes'
        h=self.holgura
        if h is None:
            return 'Sin contorno 360 grados y TF completo para girar'
        if not h.valida:
            return h.motivo
        if (h.edad_s is None or not isfinite(h.edad_s) or not 0 <= h.edad_s <= cfg.edad_maxima_s
                or h.distancia_minima_m is None or not isfinite(h.distancia_minima_m)):
            return 'Contorno de giro invalido o caducado'
        limite=self.cfg.radio_giro_m+self.cfg.margen_giro_m
        if h.distancia_minima_m <= limite:
            return f'distancia al centro={h.distancia_minima_m:.3f}m <= radio+margen={limite:.3f}m'
        return None

    def bloqueo_avance(self, lidar, cfg):
        """Protección frontal y lateral en base_link para el avance recto."""
        if not self.observado(lidar):
            return 'LiDAR/TF insuficientes'
        h = self.holgura
        if h is None:
            return 'Sin contorno 360 grados y TF completo para avanzar'
        if not h.valida:
            return h.motivo
        dist = h.distancia_avance_m if h.distancia_avance_m is not None else h.distancia_minima_m
        if (h.edad_s is None or not isfinite(h.edad_s) or not 0 <= h.edad_s <= cfg.edad_maxima_s
                or dist is None or not isfinite(dist)):
            return 'Contorno de avance invalido o caducado'
        limite = self.cfg.radio_giro_m + self.cfg.margen_giro_m
        if dist <= limite:
            return f'distancia frontal/lateral al centro={dist:.3f}m <= radio+margen={limite:.3f}m'
        return None

    def diagnostico_giro(self, ahora, lidar, cfg=None):
        def distancia(campo):
            valor=getattr(lidar,campo,None) if lidar and lidar.observacion_valida else None
            return valor if valor is not None and isfinite(valor) else None
        pendiente=self.giros.pendiente
        activo=self.giros.activo
        salida=self.salida_flecha
        return dict(color=self.giros.color,
            recibido=dict(self.ultima_senal_recibida) if self.ultima_senal_recibida else None,
            por_procesar=dict(self.senal_por_procesar) if self.senal_por_procesar else None,
            rearme=None, rearme_por_distancia=False,
            pendiente=dict(id=self.giros.ultimo_id,direccion=pendiente['direccion'],
                edad_memoria_s=(ahora-self.aproximacion_ns)/1e9,
                edad_ultima_deteccion_s=(ahora-self.revalidacion_ns)/1e9
                    if self.revalidacion_ns is not None else None,
                limite_memoria_s=self.cfg.tiempo_aproximacion_s) if pendiente else None,
            activo=activo['direccion'] if activo else None, conflicto=self.conflicto_senal,
            avance_deg=(activo['sentido']*(self.yaw_total-activo['yaw_total'])*180/pi
                        if activo and self.pose else None),
            objetivo_deg=None, criterio_fin='salida_libre',
            frente_salida_m=(cfg.distancia_precaucion_m+self.cfg.margen_salida_evasion_m if cfg else None),
            ultimo_completado=self.ultimo_giro_completado,
            salida_recta=(dict(reposo_confirmado=salida['confirmado'],
                corrigiendo=bool(salida.get('correccion')),
                distancia_m=hypot(self.pose['x']-salida['x'],self.pose['y']-salida['y']) if self.pose else None,
                adaptable=True,
                edad_s=(ahora-salida['inicio_ns'])/1e9) if salida else None),
            evasion=dict(self.evasion) if self.evasion else None,
            frente_m=distancia('dist_frente'), izquierda_m=distancia('dist_izquierda'),
            derecha_m=distancia('dist_derecha'), disparo_m=self.cfg_senales.esperar_pared_m,
            tolerancia_disparo_m=self.cfg.tolerancia_disparo_m,
            disparo_efectivo_m=self.giros.cfg.esperar_pared_m,
            frenado_flecha_confirmado=bool(self.freno_flecha and self.freno_flecha['confirmado']),
            bloqueo_aproximacion=self.bloqueo_aproximacion,
            limite_giro_m=self.cfg.radio_giro_m+self.cfg.margen_giro_m,
            radio_robot_m=self.cfg.radio_giro_m, margen_giro_m=self.cfg.margen_giro_m,
            distancia_centro_m=self.holgura.distancia_minima_m if self.holgura and self.holgura.valida else None,
            bloqueo_giro=self.bloqueo_giro(lidar,cfg) if cfg else None)

    def aproximar(self, ahora, lidar, cfg, etiqueta, lento=False):
        """Avance recto protegido; no rebaja la parada general de evasión."""
        self.evasion=None
        bloqueo=self.bloqueo_giro(lidar,cfg)
        if bloqueo:
            return 0.,0.,etiqueta+': aproximacion bloqueada; '+bloqueo
        velocidad=(min(cfg.velocidad_avance,self.cfg.velocidad_aproximacion_m_s)
                   if lento or lidar.dist_frente <= cfg.distancia_precaucion_m else cfg.velocidad_avance)
        radio=self.cfg.radio_giro_m+self.cfg.margen_giro_m
        recorrido=distancia_libre_recta(self.holgura,radio)
        margen=self.cfg.margen_aproximacion_m+velocidad*(self.holgura.edad_s+.1)
        # En ejecución real se comprueba el disco trasladado hacia delante.
        # Los dobles antiguos sin puntos conservan el veto radial conservador.
        distancia=self.holgura.distancia_minima_m if recorrido is None else recorrido
        limite=radio+margen if recorrido is None else margen
        if distancia <= limite:
            medida='centro' if recorrido is None else 'recorrido libre'
            motivo=(f'{medida}={distancia:.3f}m <= {limite:.3f}m; frente={lidar.dist_frente:.3f}m')
            self.bloqueo_aproximacion=dict(motivo=motivo,marca_ns=ahora,
                distancia_centro_m=self.holgura.distancia_minima_m,limite_m=limite,
                recorrido_libre_m=recorrido,margen_recorrido_m=margen)
            return 0.,0.,etiqueta+': margen de aproximacion insuficiente; '+motivo
        self.bloqueo_aproximacion=None
        accion=('avanzar recto con holgura; salida adaptable' if etiqueta == 'Salida tras flecha'
                else 'aproximar recto hasta el umbral; sin giro evasivo')
        return velocidad,0.,etiqueta+': '+accion

    def salir_de_flecha(self, ahora, pose, pose_ns, lidar, cfg, solo_reposo=False):
        """Reposo y salida protegida; un obstáculo puede devolver el control a evasión."""
        s=self.salida_flecha
        if s is None:
            return None
        error=normalizar(s['yaw_objetivo']-pose['yaw'])
        desvio=abs(error)*180/pi
        if desvio > 20.:
            self.fallar(f'Desviacion tras giro de flecha ({desvio:.1f} grados): detenido',ahora)
            return 0.,0.,self.motivo
        if desvio > self.cfg_senales.tolerancia_deg+1e-6 and not s.get('correccion'):
            s['correccion']=dict(inicio_ns=ahora)
            s['confirmado'],s['reposo']=False,ReposoOdometria()
            return 0.,0.,f'Corregir rumbo tras flecha ({desvio:.1f} grados): frenar antes de girar'
        correccion=s.get('correccion')
        if correccion and ahora-correccion['inicio_ns'] > self.cfg_senales.tiempo_maximo_giro_s*1e9:
            self.fallar('Correccion de rumbo agotada: detenido; revisar espacio y odometria',ahora)
            return 0.,0.,self.motivo
        if not s['confirmado']:
            s['confirmado']=s['reposo'].actualizar(pose,pose_ns,ahora)
            return 0.,0.,'Salida tras flecha: frenar y confirmar reposo 0.7s antes de avanzar'
        if correccion:
            if desvio <= self.cfg_senales.tolerancia_deg*.5:
                s['correccion']=None
                s['confirmado'],s['reposo']=False,ReposoOdometria()
                return 0.,0.,'Rumbo recuperado: confirmar reposo antes de avanzar'
            bloqueo=self.bloqueo_giro(lidar,cfg)
            if bloqueo:
                return 0.,0.,'Correccion de rumbo bloqueada: '+bloqueo
            velocidad=min(.12,cfg.velocidad_giro,max(.04,abs(error)*.8))
            return 0.,velocidad if error>0 else -velocidad,f'Corregir rumbo tras flecha: {desvio:.1f} grados restantes'
        if solo_reposo:
            return None
        # Preferir recto si una corrección lateral sería innecesaria para la
        # huella trasladada. No exigir una distancia: una pared/diagonal que
        # bloquee ese recorrido puede necesitar evasión antes de avanzar 40 cm.
        recto=self.aproximar(ahora,lidar,cfg,'Salida tras flecha',lento=True)
        if not self.observado(lidar) or not self.libre(lidar,cfg):
            return recto
        vx_normal,wz_normal=calcular_velocidad_evasion(lidar,cfg)
        if recto[0] > 0:
            if vx_normal > 0 and wz_normal == 0:
                self.salida_flecha=None  # Navegación normal ya puede continuar.
            return recto
        if wz_normal:
            self.salida_flecha=None
            self.bloqueo_aproximacion=None
            vx,wz,motivo=self.evadir(ahora,lidar,cfg)
            return vx,wz,'Salida tras flecha: recto bloqueado; '+motivo
        return recto

    def salida_libre(self, lidar, cfg):
        """Frente despejado y recorrido de avance protegido antes de acabar un giro."""
        if (not self.observado(lidar)
                or lidar.dist_frente <= cfg.distancia_precaucion_m+self.cfg.margen_salida_evasion_m
                or self.bloqueo_giro(lidar,cfg) is not None
                or self.bloqueo_avance(lidar,cfg) is not None):
            return False
        radio=self.cfg.radio_giro_m+self.cfg.margen_giro_m
        recorrido=distancia_libre_recta(self.holgura,radio)
        if recorrido is None:
            dist=self.holgura.distancia_avance_m
            recorrido=(dist if dist is not None else self.holgura.distancia_minima_m)-radio
        margen=self.cfg.margen_aproximacion_m+cfg.velocidad_avance*(self.holgura.edad_s+.1)
        return recorrido > margen

    def giro_por_salida(self, evento, pose, ahora, permitido, lidar, cfg, cfg_admision):
        """Reutiliza admisión/memoria originales; la rotación termina por LiDAR."""
        if self.giros.activo is None:
            giro=self.giros.decidir(evento,pose,ahora,permitido,lidar,cfg_admision)
            if self.giros.activo is not None:
                self.giros.activo['yaw_total']=self.yaw_total
            return giro
        if not permitido:
            return 0.,0.,'Giro por flecha: esperando frenado/sensores'
        a=self.giros.activo
        avance=a['sentido']*(self.yaw_total-a['yaw_total'])
        if avance < -20*pi/180:
            self.giros.fallo='Odometria indica giro contrario: detenido; revisar'
            return 0.,0.,self.giros.fallo
        if self.salida_libre(lidar,cfg):
            self.giros.activo=None
            return 0.,0.,'Giro por flecha completado: salida libre observada con margen'
        velocidad=cfg.velocidad_giro if self.observado(lidar) and lidar.dist_frente <= cfg.distancia_parada_m else cfg.velocidad_giro*.75
        return 0.,a['sentido']*velocidad,f"Giro {a['direccion']} por senal {self.giros.color}: buscar salida libre"

    def evadir(self, ahora, lidar, cfg):
        """Conservar el lado del giro frontal hasta salir con margen, sin arcos."""
        if not self.observado(lidar):
            return 0., 0., 'Evasion detenida: LiDAR/TF insuficientes'
        vx, wz = calcular_velocidad_evasion(lidar, cfg)
        if self.evasion is None and lidar.dist_frente <= cfg.distancia_precaucion_m:
            self.evasion=dict(sentido=1. if lidar.dist_izquierda >= lidar.dist_derecha else -1.,
                              inicio_ns=ahora, yaw=self.yaw_total)
        if self.evasion is None:
            if wz and not self.libre(lidar,cfg):
                return 0.,0.,'Correccion lateral bloqueada: '+self.bloqueo_giro(lidar,cfg)
            if vx > 0:
                bloqueo = self.bloqueo_avance(lidar, cfg)
                if bloqueo:
                    return 0., 0., 'Avance bloqueado por obstaculo cercano: ' + bloqueo
                return vx, 0., 'Avance recto con evasion'
            if wz:
                return 0., wz, 'Correccion lateral sobre el sitio'
            return 0., 0., 'Evasion detenida: ambos laterales cercanos'
        e=self.evasion
        avance=e['sentido']*(self.yaw_total-e['yaw'])
        agotado = ahora-e['inicio_ns'] > self.cfg_senales.tiempo_maximo_giro_s*1e9
        if agotado or avance < -.35:
            if avance < -.35:
                causa = f'giro contrario segun odometria ({avance*180/pi:.1f} grados)'
            else:
                causa = f'plazo de {self.cfg_senales.tiempo_maximo_giro_s:.1f}s agotado'
            if e.get('ultimo_bloqueo'):
                causa += '; ultimo bloqueo: '+e['ultimo_bloqueo']
            self.fallar('Evasion sin salida: '+causa+'; detenido, revisar antes de reiniciar',ahora)
            return 0., 0., self.motivo
        if self.salida_libre(lidar,cfg):
            self.evasion=None
            return 0., 0., 'Frente libre con margen: terminar giro evasivo antes de avanzar'
        bloqueo = self.bloqueo_giro(lidar,cfg)
        if bloqueo:
            e['ultimo_bloqueo'] = bloqueo
            return 0., 0., 'Evasion bloqueada hacia el lado elegido: '+bloqueo+'; detenido sin invertir'
        velocidad=cfg.velocidad_giro if lidar.dist_frente <= cfg.distancia_parada_m else cfg.velocidad_giro*.75
        lado='IZQUIERDA' if e['sentido'] > 0 else 'DERECHA'
        return 0., e['sentido']*velocidad, f'Evasion {lado} mantenida hasta frente libre con margen'

    def decidir(self, ahora, pose, pose_ns, lidar, cfg, fin=False, holgura=None):
        self.holgura=holgura  # Solo el scan de este ciclo; un latido no lo renueva.
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
        # Retener una lectura recibida fresca permite recuperar sensores, pero
        # no conservar una confirmación caducada indefinidamente. La GUI solo
        # reabre esa lectura cuando recibe este rechazo explícito y su marca.
        if (self.fase == 'BUSCAR_B' and self.qr_pendiente is not None
                and not reciente(self.qr_pendiente, ahora, 5_000_000_000)):
            self.qr_reintentar_ns, self.qr_pendiente = self.qr_pendiente, None
        if self.solicitud_ns is not None and ahora-self.solicitud_ns > self.cfg.tiempo_maximo_mision_s*1e9:
            self.fallar('Tiempo total de ensayo agotado', ahora)
            return cero(self.motivo)
        # El plazo original incluye frenado y pausas. Un corte de cámara no
        # reinicia ni suspende el límite de la maniobra que estaba en marcha.
        if (self.giros.activo is not None and
                ahora-self.giros.activo['inicio_ns'] > self.cfg_senales.tiempo_maximo_giro_s*1e9):
            motivo=f'Giro sin completar a tiempo ({self.cfg_senales.tiempo_maximo_giro_s:.1f}s): sin salida libre'
            if self.freno_flecha and self.freno_flecha.get('ultimo_bloqueo'):
                motivo+='; ultimo bloqueo: '+self.freno_flecha['ultimo_bloqueo']
            self.fallar(motivo,ahora)
        if (self.evasion is not None and
                ahora-self.evasion['inicio_ns'] > self.cfg_senales.tiempo_maximo_giro_s*1e9):
            motivo=f'Evasion sin salida: plazo de {self.cfg_senales.tiempo_maximo_giro_s:.1f}s agotado'
            if self.evasion.get('ultimo_bloqueo'):
                motivo+='; ultimo bloqueo: '+self.evasion['ultimo_bloqueo']
            self.fallar(motivo,ahora)
        # También expira durante pausas de cámara/GUI; recuperar sensores no
        # autoriza retomar una dirección antigua fuera del plazo de maniobra.
        if self.fase in ('IDA','VUELTA'):
            for inicio, nombre in ((self.aproximacion_ns,'flecha'),(self.aproximacion_meta_ns,'meta')):
                if inicio is not None and ahora-inicio > self.cfg.tiempo_aproximacion_s*1e9:
                    motivo=f'Aproximacion a {nombre} agotada: detenido'
                    if self.bloqueo_aproximacion:
                        motivo+='; ultimo bloqueo de aproximacion: '+self.bloqueo_aproximacion['motivo']
                    self.fallar(motivo,ahora)
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
            # También ante otra señal/meta: no saltarse el frenado posterior ni
            # autorizar un segundo giro por una desviación del primero.
            salida=self.salir_de_flecha(ahora,pose,pose_ns,lidar,cfg,solo_reposo=True)
            if salida is not None:
                return salida
            origen = self.a if self.fase == 'IDA' else self.b
            separado = hypot(pose['x']-origen['x'], pose['y']-origen['y']) >= self.cfg.separacion_meta_m
            cerca_a = hypot(pose['x']-self.a['x'], pose['y']-self.a['y']) <= self.cfg.radio_llegada_a_m
            # Confirmar la bandera ordena frenar. No exigir permiso de giro para
            # aceptar la llegada: el barrido comprueba su propia holgura después.
            admisible = (separado and (self.fase == 'IDA' or cerca_a) and self.observado(lidar)
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
            elif not self.observado(lidar):
                self.diagnostico_meta = 'Bandera vista; '+self.bloqueo_giro(lidar,cfg)
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
            minimo=self.cfg.radio_giro_m+self.cfg.margen_giro_m+self.cfg.margen_aproximacion_m
            if self.cfg_senales.esperar_pared_m <= minimo:
                self.fallar(f'Disparo de flecha {self.cfg_senales.esperar_pared_m:.3f}m sin margen: '
                            f'debe superar radio+holgura+aproximacion {minimo:.3f}m; '
                            'revisar holgura y frenado antes de reducirlo',ahora)
                return cero(self.motivo)
            ev = self.senal if self.senal and self.senal['marca_ns'] > self.desde_ns else None
            if not evento_valido(ev, ahora):
                ev = None
            if self.senal_por_procesar is not None:
                # Ya se validó al recibirla; su memoria tiene plazo propio de
                # 30 s y no cambia la fecha de captura del evento original.
                ev = self.senal_por_procesar
            # No reemplazar la primera dirección ni renovar su plazo por otro
            # ID/imagen/latido. Una contradicción del color del tramo detiene.
            if self.giros.pendiente:
                if ev and ev['color'] == self.giros.color:
                    if ev['direccion'] != self.giros.pendiente['direccion']:
                        self.conflicto_senal=dict(id=ev['id'],color=ev['color'],
                            esperada=self.giros.pendiente['direccion'],observada=ev['direccion'],
                            marca_ns=ev['marca_ns'])
                        self.fallar(f"Cambio de sentido del cartel pendiente: {self.conflicto_senal['esperada']} "
                                    f"-> {ev['direccion']} (id {ev['id']}): detenido", ahora)
                        return cero(self.motivo)
                    if self.revalidacion_ns is None or ev['marca_ns'] > self.revalidacion_ns:
                        self.revalidacion_ns = ev['marca_ns']
                ev = None  # El controlador ya conserva el evento inicial.
            tenia_pendiente = self.giros.pendiente is not None
            preparando=(self.giros.activo is not None and self.freno_flecha is not None
                        and not self.freno_flecha['confirmado'])
            if (preparando and ev and ev['id'] == self.giros.ultimo_id
                    and ev['color'] == self.giros.color
                    and ev['direccion'] != self.giros.activo['direccion']):
                self.conflicto_senal=dict(id=ev['id'],color=ev['color'],
                    esperada=self.giros.activo['direccion'],observada=ev['direccion'],marca_ns=ev['marca_ns'])
                self.fallar('Cambio de sentido del cartel durante frenado: detenido',ahora)
                return cero(self.motivo)
            # El controlador original conserva admisión, evento y disparo radial.
            # Su veto antiguo por rangos sin traslación se sustituye aquí por
            # la comprobación circular completa de abajo; no editar el snapshot.
            lidar_giro = lidar
            esperar = self.giros.cfg.esperar_pared_m
            if (lidar is not None and lidar.observacion_valida
                    and esperar < lidar.dist_frente <= esperar + 1e-4):
                lidar_giro = (replace(lidar, dist_frente=esperar) if is_dataclass(lidar)
                              else SimpleNamespace(**{**vars(lidar), 'dist_frente': esperar}))
            cfg_giro=SimpleNamespace(velocidad_giro=cfg.velocidad_giro,distancia_parada_m=0.)
            activo_anterior=dict(self.giros.activo) if self.giros.activo else None
            giro = self.giro_por_salida(ev,pose,ahora,not preparando,lidar_giro,cfg,cfg_giro)
            self.senal_por_procesar = None
            if self.giros.pendiente is not None or self.giros.activo is not None:
                # El reposo anterior ya se comprobó arriba. No corregir el
                # rumbo viejo mientras se prepara una nueva flecha válida.
                self.salida_flecha=None
            if self.giros.pendiente and not tenia_pendiente:
                if self.aproximacion_ns is None:
                    self.aproximacion_ns = ahora
                if self.revalidacion_ns is None:
                    self.revalidacion_ns = ev['marca_ns'] if ev else None
            elif self.giros.pendiente is None:
                self.aproximacion_ns = self.revalidacion_ns = None
            if tenia_pendiente and self.giros.pendiente is None and self.giros.activo is None and giro is None:
                motivo='Se perdio la maniobra de flecha pendiente'
                if self.bloqueo_aproximacion:
                    motivo+='; ultimo bloqueo de aproximacion: '+self.bloqueo_aproximacion['motivo']
                self.fallar(motivo+'; detenido, revisar antes de reiniciar',ahora)
                return cero(self.motivo)
            if self.giros.fallo:
                motivo=self.giros.fallo
                if self.freno_flecha and self.freno_flecha.get('ultimo_bloqueo'):
                    motivo+='; ultimo bloqueo: '+self.freno_flecha['ultimo_bloqueo']
                self.fallar(motivo, ahora)
            if giro is not None:
                self.evasion=None
                self.aproximacion_meta_ns=None
                if self.giros.activo and self.fase != 'FALLO':
                    activo=self.giros.activo
                    if self.freno_flecha is None or self.freno_flecha['inicio_ns'] != activo['inicio_ns']:
                        self.freno_flecha=dict(inicio_ns=activo['inicio_ns'],confirmado=False,
                                               reposo=ReposoOdometria())
                    f=self.freno_flecha
                    if not f['confirmado']:
                        quieto=f['reposo'].actualizar(pose,pose_ns,ahora)
                        bloqueo=self.bloqueo_giro(lidar,cfg)
                        if bloqueo:
                            f['ultimo_bloqueo']=bloqueo
                            return cero('Flecha cercana: detenido antes del giro; '+bloqueo)
                        if not quieto:
                            return cero(f'Flecha cercana ({lidar.dist_frente:.3f}m): '
                                        'frenar y confirmar reposo 0.7s antes de girar')
                        # Fijar el inicio angular una sola vez ANTES de publicar
                        # giro. No reiniciar el plazo original de 12 s ni volver
                        # a cambiar esta referencia tras pausas/obstáculos.
                        activo['yaw']=pose['yaw']
                        activo['yaw_total']=self.yaw_total
                        self.giros.ultima_posicion=(pose['x'],pose['y'])
                        f['confirmado']=True
                        self.bloqueo_aproximacion=None
                        return cero('Reposo de flecha confirmado: girar sobre el sitio hasta salida libre')
                    bloqueo = self.bloqueo_giro(lidar,cfg)
                    if bloqueo:
                        f['ultimo_bloqueo']=bloqueo
                        return cero('Giro por flecha bloqueado: '+bloqueo+'; no invierte sentido')
                elif self.fase != 'FALLO':
                    self.freno_flecha = self.bloqueo_aproximacion = None
                    if activo_anterior and not self.giros.fallo:
                        a=activo_anterior
                        self.ultimo_giro_completado=dict(id=self.giros.ultimo_id,
                            direccion=a['direccion'],color=self.giros.color,marca_ns=ahora,
                            avance_deg=a['sentido']*(self.yaw_total-a['yaw_total'])*180/pi,
                            criterio='salida_libre')
                        self.salida_flecha=dict(inicio_ns=ahora,x=pose['x'],y=pose['y'],
                            yaw_objetivo=pose['yaw'],
                            confirmado=False,reposo=ReposoOdometria())
                return giro
            if self.giros.pendiente:
                self.aproximacion_meta_ns=None
                return self.aproximar(ahora,lidar,cfg,'Flecha guardada')
            # La evasión gira desde 0.45 m. Con bandera centrada y guardas
            # válidas se coordina un avance lento hasta 0.40 m, sin anticipar B.
            meta_encaminada=(self.meta is not None and self.meta_nueva.salida_visual and separado
                and (self.fase == 'IDA' or cerca_a) and abs(self.meta[0]+self.meta[2]/2-.5) <= .2)
            if (meta_encaminada and self.aproximacion_meta_ns is None
                    and self.observado(lidar)
                    and lidar.dist_frente <= cfg.distancia_precaucion_m+self.cfg.margen_salida_evasion_m):
                self.aproximacion_meta_ns=ahora
            salida=self.salir_de_flecha(ahora,pose,pose_ns,lidar,cfg)
            if salida is not None:
                return salida
            if self.aproximacion_meta_ns is not None:
                self.evasion=None
                if not meta_encaminada:
                    return cero('Aproximacion a meta: detener hasta volver a ver la bandera centrada')
                return self.aproximar(ahora,lidar,cfg,'Meta vista',lento=True)
            return self.evadir(ahora,lidar,cfg)
        if self.fase == 'FRENAR_B':
            if self.reposo.actualizar(pose, pose_ns, ahora):
                self.yaw_entrada_b = pose['yaw']
                self.busqueda_desde_ns = self.inicio_barrido_ns = ahora
                self.yaw_barrido = self.yaw_total
                self.cambiar('BUSCAR_B', ahora, 'Frenado confirmado; buscar persona y su QR')
            return cero(self.motivo)
        if self.fase == 'BUSCAR_B':
            qr_fresco = (self.orden == 'QR_LISTO' and self.qr_votos == CONFIRMACIONES_VISUALES
                         and reciente(self.qr_ns, ahora, EDAD_CAMARA_NS)
                         and self.qr_ns > self.busqueda_desde_ns)
            qr_retenido = (self.qr_pendiente is not None
                           and self.qr_pendiente > self.busqueda_desde_ns
                           and reciente(self.qr_pendiente, ahora, 5_000_000_000))
            if qr_fresco or qr_retenido:
                self.qr_confirmado = True
                self.qr_pendiente = None
                self.cambiar('FRENAR_QR', ahora, 'QR confirmado; preparar salida sin avance')
                return cero(self.motivo)
            if self.orden == 'QR_LISTO':
                return cero(self.motivo)
            barrido = self.yaw_total-self.yaw_barrido
            if barrido < -.08 or barrido >= 2*pi-.025:
                self.fallar('Barrido contrario o vuelta completa sin QR confirmado', ahora)
                return cero(self.motivo)
            if self.orden == 'VERIFICAR':
                return cero('Objetivo localizado; verificar QR sin girar')
            if not self.libre(lidar, cfg):
                return cero('Barrido bloqueado: '+self.bloqueo_giro(lidar,cfg)+
                            '; vision estacionaria sigue activa')
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
                return cero('Orientacion bloqueada: '+self.bloqueo_giro(lidar,cfg)+'; conservar QR y detener')
            return 0., (1 if error > 0 else -1)*min(.25, cfg.velocidad_giro, max(.12, abs(error)*.8)), self.motivo
        if self.fase == 'FRENAR_SALIDA':
            if self.reposo.actualizar(pose, pose_ns, ahora):
                self.giros = self.clase_giros('vuelta', self.cfg_giros_mision)
                self.ultima_senal_recibida = None
                self.cambiar('VUELTA', ahora, 'Salida preparada; seguir rojo')
                self.meta_nueva = MetaNueva(ahora, self.cfg)
            return cero(self.motivo)
        if self.fase == 'FRENAR_A':
            if self.reposo.actualizar(pose, pose_ns, ahora):
                self.cambiar('FIN', ahora, 'Meta A y reposo confirmados; mostrar palabra guardada')
            return cero(self.motivo)
        self.fallar('Estado de mision desconocido', ahora)
        return cero(self.motivo)
