"""Salida única: evasión opcional hasta meta, frenado y barrido sin traslación."""

import argparse
import fcntl
import json
from math import degrees
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
from control_meta import ControlBarrido
from evasion_lidar import cargar_config, decidir_scan
from seguridad_lidar import EvasionLidarNode


class BarridoNode(EvasionLidarNode):
    def __init__(self, mover, cfg, entrada, avanzar=False):
        self.entrada, self.control = entrada, ControlBarrido(avanzar)
        self.pose = self.pose_ns = None
        self.intervalo_odom_s = self.max_intervalo_odom_s = None
        self.estado_ns = 0
        super().__init__(mover, cfg)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odometria,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))

    def odometria(self, msg):
        ahora = time.monotonic_ns()
        if self.pose_ns is not None:
            self.intervalo_odom_s = (ahora-self.pose_ns)/1e9
            self.max_intervalo_odom_s = max(self.max_intervalo_odom_s or 0., self.intervalo_odom_s)
        self.pose, self.pose_ns = pose_odometria(msg), ahora

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
            print(json.dumps(dict(version=1, recibido_ns=ahora, estado=motivo, vx=vx, wz=wz,
                barrido_deg=degrees(self.control.angulo), terminado=self.control.terminado,
                busqueda_desde_ns=self.control.busqueda_desde_ns,
                edad_odom_s=None if self.pose_ns is None else (ahora-self.pose_ns)/1e9,
                intervalo_odom_s=self.intervalo_odom_s,
                max_intervalo_odom_s=self.max_intervalo_odom_s,
                ultimo_corte_odom_s=self.control.ultimo_corte_s,
                detalle_odometria=self.control.detalle_odometria,
                lidar_m=None if lidar is None else dict(frente=lidar.dist_frente,
                    izquierda=lidar.dist_izquierda, derecha=lidar.dist_derecha),
                edad_lidar_s=edad,
                mover=self.pub_cmd is not None)), flush=True)
            self.estado_ns = ahora


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--mover', action='store_true')
    parser.add_argument('--avanzar-hasta-meta', action='store_true')
    args = parser.parse_args()
    cfg = cargar_config(args.config)
    bloqueo = None
    if args.mover:
        bloqueo = open('/tmp/proyecto2-evasion-control.lock', 'a')
        try:
            fcntl.flock(bloqueo, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            parser.error('Cerrar otra evasion/busqueda con movimiento antes de este ensayo')
    entrada = EntradaControl(sys.stdin)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    nodo = None
    def terminar(_s, _f):
        entrada.fin.set()
        if nodo is not None:
            nodo._terminar = True
    anteriores = {s: signal.signal(s, terminar) for s in (signal.SIGINT, signal.SIGTERM)}
    try:
        nodo = BarridoNode(args.mover, cfg, entrada, args.avanzar_hasta_meta)
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
