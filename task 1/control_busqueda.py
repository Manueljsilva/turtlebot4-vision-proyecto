"""Estados de búsqueda y permisos de movimiento; sin ROS ni visión.

Los tiempos monotónicos solo son comparables entre host y su Docker local.
Registrar A no implementa planificación ni regreso a A.
"""

import json
from math import atan2, isfinite, sqrt
import re

from control_giros import evento_valido


EDAD_ACTIVIDAD_NS = 500_000_000
EDAD_CAMARA_NS = 1_000_000_000
EDAD_POSE_NS = 600_000_000
MAX_LINEA = 2048
ORDENES = {'ESPERAR', 'BUSCAR', 'VERIFICAR', 'ENCONTRADO', 'PARAR'}


def reciente(marca, ahora, limite):
    return type(marca) is int and 0 <= marca <= ahora and ahora - marca <= limite


def pose_odometria(msg):
    """Pose del child_frame_id en header.frame_id, sin inventar un origen."""
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    valores = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
    if not all(isfinite(v) for v in valores):
        return None
    if not all(re.fullmatch(r'/?[A-Za-z0-9_][A-Za-z0-9_/]*', f or '')
               for f in (msg.header.frame_id, msg.child_frame_id)):
        return None
    norma = sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
    if not isfinite(norma) or norma < 1e-12:
        return None
    x, y, z, w = (v / norma for v in (q.x, q.y, q.z, q.w))
    return dict(frame_id=msg.header.frame_id, child_frame_id=msg.child_frame_id,
                x=float(p.x), y=float(p.y),
                yaw=atan2(2*(w*z+x*y), 1-2*(y*y+z*z)))


class PuertaBusqueda:
    """Único permiso para la salida LiDAR; los fallos siempre lo revocan."""

    def __init__(self):
        self.orden = 'ESPERAR'
        self.actividad_ns = None
        self.camara_ns = None
        self.secuencia = -1
        self.punto_a = None
        self.finalizado = False
        self.frame_cambiado = False
        self.inicio_ns = None
        self.senal = None
        self.senales_activas = False

    def recibir(self, linea, ahora_ns):
        try:
            if len(linea) > MAX_LINEA:
                raise ValueError('Línea larga')
            d = json.loads(linea)
            claves = {'version', 'secuencia', 'orden', 'actividad_ns', 'camara_ns'}
            if (not isinstance(d, dict) or set(d) not in (claves, claves | {'senal'})
                    or type(d['version']) is not int or d['version'] != 1
                    or type(d['secuencia']) is not int or d['secuencia'] <= self.secuencia
                    or not isinstance(d['orden'], str) or d['orden'] not in ORDENES
                    or not reciente(d['actividad_ns'], ahora_ns, EDAD_ACTIVIDAD_NS)
                    or (d['camara_ns'] is not None and type(d['camara_ns']) is not int)
                    or not evento_valido(d.get('senal'), ahora_ns)):
                raise ValueError('Control inválido/caducado')
            self.secuencia = d['secuencia']
            self.actividad_ns, self.camara_ns, self.orden = d['actividad_ns'], d['camara_ns'], d['orden']
            self.senal, self.senales_activas = d.get('senal'), 'senal' in d
            if self.orden in ('BUSCAR', 'VERIFICAR') and self.inicio_ns is None:
                self.inicio_ns = self.actividad_ns
            if self.orden == 'ENCONTRADO' and reciente(self.camara_ns, ahora_ns, EDAD_CAMARA_NS):
                self.finalizado = True
        except (ValueError, TypeError, KeyError, OverflowError):
            self.orden, self.actividad_ns, self.camara_ns = 'PARAR', None, None
            self.senal, self.senales_activas = None, False

    def permiso(self, ahora_ns, pose, pose_ns):
        if self.finalizado:
            return False, 'Persona buscada encontrada: detenido; retorno pendiente'
        if not reciente(self.actividad_ns, ahora_ns, EDAD_ACTIVIDAD_NS):
            return False, 'Sin actividad reciente de reconocimiento'
        if self.orden in ('ESPERAR', 'PARAR'):
            return False, 'Esperando Buscar' if self.orden == 'ESPERAR' else 'Detenido'
        if (pose is None or not reciente(pose_ns, ahora_ns, EDAD_POSE_NS)
                or self.inicio_ns is None or pose_ns < self.inicio_ns):
            return False, 'Esperando odometria reciente para A'
        if self.punto_a is None:
            self.punto_a = dict(pose, capturado_monotono_ns=ahora_ns)
        if any(pose[k] != self.punto_a[k] for k in ('frame_id', 'child_frame_id')):
            self.frame_cambiado = True
        if self.frame_cambiado:
            return False, 'Cambio de marco de odometria: reiniciar prueba'
        if not reciente(self.camara_ns, ahora_ns, EDAD_CAMARA_NS):
            return False, 'Esperando camara reciente: detenido'
        if self.orden != 'BUSCAR':
            return False, 'Verificando persona: detenido'
        return True, 'Buscando con evasion LiDAR'


class SesionBusqueda:
    """Confirmar tres paquetes nuevos consecutivos, deteniéndose desde el primero."""

    def __init__(self):
        self.nombre = None
        self.inicio_ns = None
        self.ultima_marca = None
        self.confirmaciones = 0
        self.encontrado = False
        self.orden = 'ESPERAR'

    def iniciar(self, nombre, ahora_ns):
        if self.inicio_ns is not None:
            raise ValueError('La búsqueda ya comenzó; el objetivo queda fijo')
        if not isinstance(nombre, str) or not nombre.strip():
            raise ValueError('Falta persona objetivo')
        self.nombre, self.inicio_ns, self.orden = nombre, ahora_ns, 'BUSCAR'

    def actualizar(self, resultado, marca_ns, ahora_ns, a_registrado):
        if self.inicio_ns is None or self.encontrado:
            return
        if (not a_registrado or not reciente(marca_ns, ahora_ns, EDAD_CAMARA_NS)
                or marca_ns <= self.inicio_ns):
            self.confirmaciones, self.orden = 0, 'BUSCAR'
            return
        if self.ultima_marca is not None and marca_ns <= self.ultima_marca:
            return  # Un fotograma repetido o anterior no añade votos.
        self.ultima_marca = marca_ns
        if resultado['aceptado'] and resultado['nombre'] == self.nombre:
            self.confirmaciones += 1
            self.orden = 'VERIFICAR'
            if self.confirmaciones >= 3:
                self.encontrado, self.orden = True, 'ENCONTRADO'
        else:
            self.confirmaciones, self.orden = 0, 'BUSCAR'
