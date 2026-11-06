"""Ejecuta el ciclo real del puente con ROS sustituido; no usa red ni hardware."""

import sys
import threading
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

import control_gestos
from control_gestos import codificar, validar_velocidades_robot
from gestos import Orden
from puente_gestos_ros import ejecutar_ros


def ejecutar_ciclos(frames, *, real=True, publicar=True):
    estado = NS(indice=0, mono=0, ros=0, subs=1, activo=True, mensajes=[], logs=[],
                parametros={}, publicadores=0, destruido=False, temporizador=None)
    frames = list(frames) + [dict(mono=frames[-1]['mono'] + 1, ros=frames[-1]['ros'], fin=True)]

    class Entrada:
        fin = threading.Event()

        def actualizar(self, vigilante, ahora):
            frame = frames[estado.indice]
            if frame.get('fin'):
                self.fin.set()
                vigilante.detener('Enlace de webcam cerrado')
            elif 'orden' in frame:
                vigilante.recibir(codificar(frame['orden'], frame.get('captura', ahora),
                                            estado.indice), ahora)

    class Mensaje:
        def __init__(self):
            self.header = NS(stamp=None, frame_id='')
            self.twist = NS(linear=NS(x=0.0), angular=NS(z=0.0))

    class Nodo:
        def __init__(self, nombre, parameter_overrides):
            estado.parametros = {p.name: p.value for p in parameter_overrides}

        def get_parameter(self, nombre):
            return NS(value=estado.parametros[nombre])

        def get_clock(self):
            return NS(now=lambda: NS(nanoseconds=estado.ros, to_msg=lambda: estado.ros))

        def create_publisher(self, tipo, topic, profundidad):
            assert topic == '/cmd_vel'
            estado.publicadores += 1

            def publicar_mensaje(msg):
                estado.mensajes.append((msg.twist.linear.x, msg.twist.angular.z,
                                        msg.header.stamp, msg.header.frame_id))
            return NS(publish=publicar_mensaje, get_subscription_count=lambda: estado.subs)

        def create_timer(self, periodo, callback, clock):
            estado.temporizador = (periodo, clock.tipo)
            return NS()

        def get_logger(self):
            return NS(info=estado.logs.append)

        def destroy_node(self):
            estado.destruido = True

    def spin_once(nodo, timeout_sec):
        frame = frames[estado.indice]
        estado.mono, estado.ros = frame['mono'], frame['ros']
        estado.subs = frame.get('subs', 1)
        nodo.ciclo()
        estado.indice += 1

    def shutdown():
        estado.activo = False

    modulos = {
        'rclpy': NS(init=lambda args: None, ok=lambda: estado.activo,
                    spin_once=spin_once, shutdown=shutdown),
        'rclpy.node': NS(Node=Nodo),
        'rclpy.clock': NS(Clock=lambda clock_type: NS(tipo=clock_type),
                          ClockType=NS(STEADY_TIME='steady')),
        'rclpy.parameter': NS(Parameter=lambda nombre, value: NS(name=nombre, value=value)),
        'rclpy.executors': NS(ExternalShutdownException=type('Shutdown', (Exception,), {})),
        'geometry_msgs': NS(), 'geometry_msgs.msg': NS(TwistStamped=Mensaje),
    }
    with patch.dict(sys.modules, modulos), \
            patch('puente_gestos_ros.time.monotonic_ns', side_effect=lambda: estado.mono), \
            patch('puente_gestos_ros.time.sleep'):
        ejecutar_ros(Entrada(), publicar, [], robot_real=real)
    return estado


class PuenteModosTests(unittest.TestCase):
    def test_real_usa_reloj_del_sistema_sin_exigir_clock(self):
        estado = ejecutar_ciclos([dict(mono=1_000_000_000, ros=1_793_914_294_000_000_000,
                                       orden=Orden.AVANZAR)])
        self.assertFalse(estado.parametros['use_sim_time'])
        self.assertEqual(estado.temporizador, (0.05, 'steady'))
        self.assertEqual(estado.mensajes[0], (0.306, 0.0, 1_793_914_294_000_000_000, 'base_link'))
        self.assertTrue(estado.destruido)
        self.assertTrue(all(m[:2] == (0, 0) for m in estado.mensajes[-4:]))

    def test_gazebo_conserva_parada_sin_reloj_o_con_reloj_pausado(self):
        estado = ejecutar_ciclos([
            dict(mono=0, ros=0, orden=Orden.AVANZAR),
            dict(mono=100_000_000, ros=100, orden=Orden.AVANZAR),
            dict(mono=700_000_000, ros=100, orden=Orden.AVANZAR),
        ], real=False)
        self.assertTrue(estado.parametros['use_sim_time'])
        self.assertEqual([m[:2] for m in estado.mensajes[:3]], [(0, 0), (0.306, 0), (0, 0)])

    def test_real_caduca_aunque_no_haya_mas_datos_y_respeta_detener(self):
        estado = ejecutar_ciclos([
            dict(mono=0, ros=100, orden=Orden.AVANZAR),
            dict(mono=500_000_001, ros=100),
            dict(mono=600_000_000, ros=100, orden=Orden.DERECHA),
            dict(mono=700_000_000, ros=100, orden=Orden.DETENER),
        ])
        self.assertEqual([m[:2] for m in estado.mensajes[:4]],
                         [(0.306, 0), (0, 0), (0, -1.90), (0, 0)])

    def test_real_sin_suscriptores_detiene(self):
        estado = ejecutar_ciclos([dict(mono=0, ros=100, orden=Orden.AVANZAR, subs=0)])
        self.assertTrue(all(m[:2] == (0, 0) for m in estado.mensajes))

    def test_comprobacion_real_no_crea_publicador(self):
        estado = ejecutar_ciclos([dict(mono=0, ros=100, orden=Orden.AVANZAR)], publicar=False)
        self.assertEqual(estado.publicadores, 0)
        self.assertEqual(estado.mensajes, [])
        self.assertTrue(any('AVANZAR' in mensaje for mensaje in estado.logs))

    def test_velocidades_reales_aceptan_limites_documentados(self):
        tabla = dict(control_gestos.VELOCIDADES)
        tabla[Orden.AVANZAR] = (0.306, 0.0)
        tabla[Orden.IZQUIERDA] = (0.0, 1.90)
        tabla[Orden.DERECHA] = (0.0, -1.90)
        with patch.dict(control_gestos.VELOCIDADES, tabla, clear=True):
            validar_velocidades_robot()

    def test_velocidades_invalidas_o_detener_no_nulo_se_rechazan(self):
        for orden, valor in [(Orden.AVANZAR, (0.307, 0)),
                             (Orden.IZQUIERDA, (0, 1.901)),
                             (Orden.AVANZAR, (float('nan'), 0)),
                             (Orden.DERECHA, (0, float('inf'))),
                             (Orden.DETENER, (0.1, 0))]:
            with self.subTest(orden=orden, valor=valor), \
                    patch.dict(control_gestos.VELOCIDADES, {orden: valor}), \
                    self.assertRaises(ValueError):
                validar_velocidades_robot()
