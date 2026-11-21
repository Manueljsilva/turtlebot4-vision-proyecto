"""Ensayo de meta/rostro/QR con avance opcional y frenado antes del barrido."""

import json
from math import atan2, cos, degrees, hypot, isfinite, pi, sin

from control_busqueda import (CONFIRMACIONES_VISUALES, EDAD_ACTIVIDAD_NS, EDAD_CAMARA_NS, EDAD_POSE_NS,
                              MAX_LINEA, reciente)
from evasion_lidar import calcular_velocidad_evasion


def misma_caja(a, b):
    if a is None or b is None:
        return False
    x, y, w, h = a
    u, v, s, t = b
    inter = max(0., min(x+w, u+s)-max(x, u))*max(0., min(y+h, v+t)-max(y, v))
    return inter/max(w*h+s*t-inter, 1.) >= .35


class ReposoOdometria:
    """Reposo confirmado por >=3 paquetes nuevos, cubriendo >=0.7 s sin cortes."""

    def __init__(self):
        self.referencia = None
        self.primera_ns = self.ultima_ns = None
        self.muestras = 0

    def actualizar(self, pose, marca, ahora):
        if (pose is None or not reciente(marca, ahora, EDAD_POSE_NS)
                or not all(isfinite(pose[k]) for k in ('x', 'y', 'yaw'))):
            self.referencia, self.primera_ns, self.ultima_ns, self.muestras = None, None, None, 0
            return False
        if marca == self.ultima_ns:
            return self.muestras >= 3 and marca-self.primera_ns >= 700_000_000
        reiniciar = (self.referencia is None or marca < self.ultima_ns
                     or marca-self.ultima_ns > EDAD_POSE_NS)
        if not reiniciar:
            r = self.referencia
            delta = atan2(sin(pose['yaw']-r['yaw']), cos(pose['yaw']-r['yaw']))
            reiniciar = (any(pose[k] != r[k] for k in ('frame_id', 'child_frame_id'))
                         or hypot(pose['x']-r['x'], pose['y']-r['y']) > .005 or abs(delta) > .02)
        if reiniciar:
            self.referencia, self.primera_ns, self.muestras = dict(pose), marca, 1
        else:
            self.muestras += 1
        self.ultima_ns = marca
        return self.muestras >= 3 and marca-self.primera_ns >= 700_000_000


class SesionPruebaB:
    def __init__(self, avanzar=False):
        self.avanzar = avanzar
        self.inicio_ns = None
        self.nombre = None
        self.orden = 'ESPERAR'
        self.meta_confirmada = False
        self.busqueda_desde_ns = None
        self.ultima_marca = None
        self.caja_meta = None
        self.votos_meta = 0
        self.confirmaciones = 0
        self.palabra = None
        self.caja_cara = None
        self.candidato_qr = None
        self.motivo = 'Elegir persona y pulsar Buscar'

    def iniciar(self, nombre, ahora_ns):
        if self.inicio_ns is not None or not isinstance(nombre, str) or not nombre.strip():
            raise ValueError('Nombre ausente o ensayo ya iniciado')
        self.nombre, self.inicio_ns = nombre, ahora_ns
        self.orden = 'NAVEGAR' if self.avanzar else 'ESPERAR_META'
        self.motivo = ('Evasion hasta ver la meta; reconocimiento facial despues de frenar' if self.avanzar
                       else 'Esperando una imagen nueva valida de la bandera de meta')

    def habilitar_busqueda(self, marca_ns):
        if (self.meta_confirmada and self.busqueda_desde_ns is None and type(marca_ns) is int
                and self.inicio_ns is not None and marca_ns >= self.inicio_ns):
            self.busqueda_desde_ns = marca_ns
            self.motivo = 'Robot detenido tras la meta; buscando persona sobre el sitio'

    def actualizar(self, percepcion, caras, asociacion, motivo_qr, marca_ns, ahora_ns):
        if self.inicio_ns is None or self.orden in ('QR_LISTO', 'PARAR'):
            return
        if not reciente(marca_ns, ahora_ns, EDAD_CAMARA_NS) or marca_ns <= self.inicio_ns:
            self.votos_meta = self.confirmaciones = 0
            self.caja_meta = self.caja_cara = self.candidato_qr = None
            self.motivo = 'Esperando imagen reciente: detenido'
            return
        if self.ultima_marca is not None and marca_ns <= self.ultima_marca:
            return
        self.ultima_marca = marca_ns
        if not self.meta_confirmada:
            meta = percepcion.get('meta')
            caja = meta['caja'] if meta else None
            self.votos_meta = (min(CONFIRMACIONES_VISUALES, self.votos_meta+1)
                               if misma_caja(caja, self.caja_meta) else 1) if meta else 0
            self.caja_meta = caja
            self.motivo = f'Meta: {self.votos_meta}/{CONFIRMACIONES_VISUALES} imagen nueva valida; ' + (
                'avance con evasion' if self.avanzar else 'robot detenido')
            if self.votos_meta >= CONFIRMACIONES_VISUALES:
                self.meta_confirmada, self.orden = True, 'BARRER'
                self.motivo = 'Meta confirmada. Frenando antes de buscar rostros'
            return  # No reconocer rostros ni votar QR en la imagen que confirma la meta.
        if self.busqueda_desde_ns is None or marca_ns <= self.busqueda_desde_ns:
            return  # Esperar confirmación del frenado y un paquete posterior.
        objetivos = [c for c in caras if c['aceptado'] and c['nombre'] == self.nombre]
        if len(objetivos) != 1:
            self.orden, self.confirmaciones = 'BARRER', 0
            self.caja_cara = self.candidato_qr = None
            self.motivo = ('Varias fotos del objetivo: sin asociacion QR' if objetivos else
                           'Buscando persona objetivo; otras identidades no finalizan la prueba')
            return
        self.orden = 'VERIFICAR'  # Parar desde la primera identidad aceptada, aun sin QR.
        caja = objetivos[0]['caja']
        palabra = asociacion['texto'] if asociacion else None
        consistente = palabra is not None and palabra == self.candidato_qr and misma_caja(caja, self.caja_cara)
        self.confirmaciones = (min(CONFIRMACIONES_VISUALES, self.confirmaciones+1)
                               if consistente else 1) if palabra is not None else 0
        self.candidato_qr, self.caja_cara = palabra, caja
        self.motivo = f'Persona buscada reconocida. {motivo_qr}; QR {self.confirmaciones}/{CONFIRMACIONES_VISUALES}'
        if self.confirmaciones >= CONFIRMACIONES_VISUALES:
            self.palabra, self.orden = palabra, 'QR_LISTO'
            self.motivo = 'Rostro y QR confirmados. Ensayo B terminado; robot detenido'

    def detener(self, motivo):
        if self.orden not in ('QR_LISTO', 'PARAR'):
            self.orden, self.motivo = 'PARAR', motivo

    def reiniciar_lectura_qr(self):
        """Reabre lectura QR si caducó antes de ser confirmada por el controlador."""
        if self.meta_confirmada and self.orden == 'QR_LISTO':
            self.palabra = None
            self.confirmaciones = 0
            self.caja_cara = None
            self.candidato_qr = None
            self.orden = 'BARRER'
            self.motivo = 'Confirmacion QR caducada antes de acuse; reintentando lectura'


class ControlBarrido:
    """Evasión opcional hasta meta; después frenado y giro <=360°, sin avance."""

    def __init__(self, avanzar=False):
        self.avanzar = avanzar
        self.orden, self.secuencia = 'ESPERAR', -1
        self.actividad_ns = self.camara_ns = None
        self.inicio_ns = self.pose_ns = None
        self.pose_anterior = None
        self.angulo = 0.
        self.terminado = False
        self.motivo_final = None
        self.llegada_ns = None
        self.meta_solicitada = False
        self.reposo_freno = ReposoOdometria()
        self.reposo_recuperacion = ReposoOdometria()
        self.recuperando_odom = False
        self.detalle_odometria = 'Esperando /odom'
        self.ultimo_corte_s = None
        self.busqueda_desde_ns = None

    def recibir(self, linea, ahora):
        try:
            d = json.loads(linea)
            if (len(linea) > MAX_LINEA or not isinstance(d, dict)
                    or set(d) != {'version', 'secuencia', 'orden', 'actividad_ns', 'camara_ns'}
                    or type(d['version']) is not int or d['version'] != 1
                    or type(d['secuencia']) is not int or d['secuencia'] <= self.secuencia
                    or not isinstance(d['orden'], str)
                    or d['orden'] not in ('ESPERAR', 'ESPERAR_META', 'NAVEGAR', 'BARRER', 'VERIFICAR', 'QR_LISTO', 'PARAR')
                    or not reciente(d['actividad_ns'], ahora, EDAD_ACTIVIDAD_NS)
                    or (d['camara_ns'] is not None and type(d['camara_ns']) is not int)):
                raise ValueError('Control invalido')
            self.secuencia, self.orden = d['secuencia'], d['orden']
            self.actividad_ns, self.camara_ns = d['actividad_ns'], d['camara_ns']
            if self.orden in ('BARRER', 'VERIFICAR'):
                self.meta_solicitada = True
            if self.orden in ('QR_LISTO', 'PARAR'):
                self.finalizar('QR confirmado: detenido' if self.orden == 'QR_LISTO' else 'Detenido por interfaz')
        except (ValueError, TypeError, KeyError, OverflowError):
            self.orden, self.actividad_ns, self.camara_ns = 'ESPERAR', None, None

    def finalizar(self, motivo):
        if not self.terminado:  # Conservar el primer fallo; PARAR no debe ocultarlo.
            self.terminado, self.motivo_final = True, motivo

    def _seguir_pose(self, ahora, pose, pose_ns):
        """Seguir odometría incluso al parar por cámara/GUI: no inventar un corte."""
        if (pose is None or not reciente(pose_ns, ahora, EDAD_POSE_NS)
                or not all(isfinite(pose[k]) for k in ('x', 'y', 'yaw'))):
            return False
        if self.pose_anterior is not None:
            if any(pose[k] != self.pose_anterior[k] for k in ('frame_id', 'child_frame_id')):
                self.finalizar('Cambio de marco de odometria: cerrar y revisar')
            elif pose_ns != self.pose_ns:
                intervalo = pose_ns-self.pose_ns
                delta = atan2(sin(pose['yaw']-self.pose_anterior['yaw']), cos(pose['yaw']-self.pose_anterior['yaw']))
                distancia = hypot(pose['x']-self.pose_anterior['x'], pose['y']-self.pose_anterior['y'])
                self.detalle_odometria = f'dt_rx={intervalo/1e9:.2f}s, dxy={distancia:.3f}m, dyaw={degrees(delta):.1f}deg'
                if intervalo < 0 or abs(delta) > .5 or distancia > .25:
                    self.finalizar('Salto de odometria: '+self.detalle_odometria+'; cerrar y revisar')
                elif intervalo > EDAD_POSE_NS:
                    self.ultimo_corte_s = intervalo/1e9
                    if self.busqueda_desde_ns is not None and intervalo > 2_000_000_000:
                        self.finalizar('Corte /odom >2 s durante barrido: angulo incierto; detenido')
                    else:
                        self.recuperando_odom = True
                        self.reposo_recuperacion = ReposoOdometria()
                if self.busqueda_desde_ns is not None and not self.terminado:
                    if delta < -.08:
                        self.finalizar('Odometria indica giro contrario: cerrar y revisar')
                    else:
                        self.angulo += delta
        if self.recuperando_odom and self.reposo_recuperacion.actualizar(pose, pose_ns, ahora):
            self.recuperando_odom = False
        self.pose_anterior, self.pose_ns = dict(pose), pose_ns
        return not self.terminado

    def _vision_con_movimiento_bloqueado(self, ahora, pose, pose_ns):
        # Una avería anterior a la meta no debe bloquear toda la visión para siempre.
        # Se exige reposo nuevo, pero el fallo terminal de movimiento NO se libera.
        if (self.meta_solicitada and self.busqueda_desde_ns is None
                and reciente(self.actividad_ns, ahora, EDAD_ACTIVIDAD_NS)
                and reciente(self.camara_ns, ahora, EDAD_CAMARA_NS)
                and self.reposo_freno.actualizar(pose, pose_ns, ahora)):
            self.busqueda_desde_ns = ahora

    def decidir(self, ahora, pose, pose_ns, lidar, cfg, fin=False):
        def cero(motivo):
            return 0., 0., motivo
        if fin:
            self.finalizar('Control cerrado: detenido')
        if self.terminado:
            if not fin:
                self._vision_con_movimiento_bloqueado(ahora, pose, pose_ns)
            return cero(self.motivo_final)
        pose_valida = self._seguir_pose(ahora, pose, pose_ns) if self.pose_anterior is not None else False
        if self.terminado:
            return cero(self.motivo_final)
        if self.inicio_ns is not None and ahora-self.inicio_ns >= 90_000_000_000:
            self.finalizar('Barrido agotado (90 s): cerrar y revisar antes de repetir')
            return cero(self.motivo_final)
        if not reciente(self.actividad_ns, ahora, EDAD_ACTIVIDAD_NS):
            return cero('Interfaz/percepcion sin actividad reciente')
        if self.orden not in ('NAVEGAR', 'BARRER', 'VERIFICAR'):
            return cero('Esperando nombre y meta; sin giro')
        if not reciente(self.camara_ns, ahora, EDAD_CAMARA_NS):
            return cero('Sin camara reciente: detenido')
        if not pose_valida and not self._seguir_pose(ahora, pose, pose_ns):
            return cero('Sin odometria reciente: detenido')
        if self.recuperando_odom:
            return cero(f'Recuperando /odom tras {self.ultimo_corte_s:.2f}s: detenido hasta 0.7s de datos estables')
        if self.orden == 'NAVEGAR':
            if not self.avanzar or self.meta_solicitada:
                return cero('Avance deshabilitado o meta ya confirmada')
            if lidar is None:
                return cero('Navegacion detenida: sin LiDAR/TF recientes')
            vx, wz = calcular_velocidad_evasion(lidar, cfg)
            return vx, wz, 'Evasion hasta meta' if (vx or wz) else 'Evasion detenida: LiDAR insuficiente/proximidad'
        if self.llegada_ns is None:
            self.llegada_ns = ahora
        if self.busqueda_desde_ns is None:
            if not self.reposo_freno.actualizar(pose, pose_ns, ahora):
                return cero('Meta confirmada: frenando; esperar odometria estable 0.7 s')
            self.busqueda_desde_ns = self.inicio_ns = ahora
            self.angulo = 0.
            return cero('Frenado confirmado: comenzar busqueda con una imagen nueva')
        if self.angulo >= 2*pi-.025:
            self.finalizar('Vuelta de busqueda completa: cerrar y revisar si falta rostro/QR')
        if self.terminado:
            return cero(self.motivo_final)
        if self.orden == 'VERIFICAR':
            return cero('Persona localizada: verificar rostro y QR sin girar')
        if (lidar is None or not lidar.observacion_valida or lidar.proximidad_no_cuantificable
                or min(lidar.dist_frente, lidar.dist_izquierda, lidar.dist_derecha) <= cfg.distancia_parada_m):
            return cero('LiDAR/TF insuficientes o espacio reducido: detenido')
        return 0., min(.25, cfg.velocidad_giro), 'Barrido sobre el sitio; sin avance'
