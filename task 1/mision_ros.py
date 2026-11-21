"""Nodo del ensayo guiado: una salida de velocidad, sin visión en Docker."""

import argparse
import fcntl
import json
from math import degrees, hypot
from pathlib import Path
import signal
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions

from busqueda_lidar_ros import EntradaControl
from control_busqueda import pose_odometria
from config_senales import cargar_config_senales
from control_mision import ControlMision, cargar_config_mision
from evasion_lidar import cargar_config, decidir_scan
from seguridad_lidar import EvasionLidarNode


class MisionNode(EvasionLidarNode):
    def __init__(self, mover, cfg, entrada, control):
        self.entrada, self.control = entrada, control
        self.pose = self.pose_ns = None
        self.estado_ns = 0
        super().__init__(mover, cfg)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odometria,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))

    def odometria(self, msg):
        self.pose, self.pose_ns = pose_odometria(msg), time.monotonic_ns()

    def _ciclo(self):
        ahora = time.monotonic_ns()
        linea = self.entrada.tomar()
        if linea is not None:
            self.control.recibir(linea, ahora)
        scan = self._scan
        edad = None if self._recibido_ns is None else (ahora-self._recibido_ns)/1e9
        yaw = self._yaw_lidar(scan.header.frame_id) if scan is not None else None
        _, _, lidar, _ = decidir_scan(scan, yaw, edad, self.cfg)
        vx, wz, motivo = self.control.decidir(ahora, self.pose, self.pose_ns, lidar, self.cfg,
                                              self.entrada.fin.is_set() or self._terminar)
        EvasionLidarNode.publicar_velocidad(self, vx, wz)
        if ahora-self.estado_ns >= 200_000_000:
            c = self.control
            print(json.dumps(dict(version=1, recibido_ns=ahora, estado=motivo,
                fase=c.fase, fase_desde_ns=c.desde_ns, vx=vx, wz=wz, mover=self.pub_cmd is not None,
                terminado=c.fase in ('FALLO', 'FIN'), a=c.a, b=c.b,
                qr_confirmado=c.qr_confirmado, busqueda_desde_ns=c.busqueda_desde_ns,
                reinicio_ns=c.reinicio_ns,
                yaw_entrada_b=c.yaw_entrada_b,
                barrido_deg=degrees(c.yaw_total-c.yaw_barrido) if c.yaw_barrido is not None else 0.,
                distancia_a_m=hypot(self.pose['x']-c.a['x'], self.pose['y']-c.a['y']) if self.pose and c.a else None,
                votos_meta=c.meta_nueva.votos if c.meta_nueva else 0,
                motivo_meta=c.diagnostico_meta,
                orden_control=c.orden, inicio_solicitado=c.solicitud_ns is not None,
                edad_camara_s=None if c.camara_ns is None else (ahora-c.camara_ns)/1e9,
                meta_visible=c.meta is not None and c.camara_ns is not None and 0 <= ahora-c.camara_ns <= 1_000_000_000,
                salida_visual=c.meta_nueva.salida_visual if c.meta_nueva else False,
                frente_m=lidar.dist_frente if lidar and lidar.observacion_valida else None,
                disparo_m=c.cfg_senales.esperar_pared_m,
                giro=c.diagnostico_giro(ahora,lidar),
                edad_odom_s=None if self.pose_ns is None else (ahora-self.pose_ns)/1e9)), flush=True)
            self.estado_ns = ahora


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config', type=Path, required=True)
    p.add_argument('--config-senales', type=Path, required=True)
    p.add_argument('--config-mision', type=Path, required=True)
    p.add_argument('--senales', action='store_true')
    p.add_argument('--tramo', choices=('ida',), default='ida')
    p.add_argument('--mover', action='store_true')
    args = p.parse_args()
    cfg = cargar_config(args.config)
    control = ControlMision(cargar_config_senales(args.config_senales), cargar_config_mision(args.config_mision))
    bloqueo = None
    if args.mover:
        bloqueo = open('/tmp/proyecto2-evasion-control.lock', 'a')
        try:
            fcntl.flock(bloqueo, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            p.error('Cerrar los otros controles de evasion/busqueda antes del ensayo')
    entrada = EntradaControl(sys.stdin)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    nodo = None
    def detener(_s, _f):
        entrada.fin.set()
        if nodo is not None:
            nodo._terminar = True
    anteriores = {s: signal.signal(s, detener) for s in (signal.SIGINT, signal.SIGTERM)}
    try:
        nodo = MisionNode(args.mover, cfg, entrada, control)
        while rclpy.ok() and not entrada.fin.is_set():
            rclpy.spin_once(nodo, timeout_sec=.1)
    except (KeyboardInterrupt, ExternalShutdownException, BrokenPipeError):
        pass
    finally:
        entrada.fin.set()
        if nodo is not None:
            if rclpy.ok():
                for _ in range(3):
                    EvasionLidarNode.publicar_velocidad(nodo, 0., 0.)
                    time.sleep(.03)
            nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for s, anterior in anteriores.items():
            signal.signal(s, anterior)
        if bloqueo is not None:
            bloqueo.close()


if __name__ == '__main__':
    main()
