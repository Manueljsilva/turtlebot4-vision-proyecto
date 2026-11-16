"""Única salida de velocidad: evasión existente + permiso de reconocimiento."""

import argparse
import fcntl
import json
from pathlib import Path
import signal
import sys
import threading
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions

from control_busqueda import MAX_LINEA, PuertaBusqueda, pose_odometria
from evasion_lidar import cargar_config
from seguridad_lidar import EvasionLidarNode


class EntradaControl:
    def __init__(self, flujo):
        self.fin = threading.Event()
        self.cerrojo = threading.Lock()
        self.linea = None
        threading.Thread(target=self.leer, args=(flujo,), daemon=True).start()

    def leer(self, flujo):
        try:
            while not self.fin.is_set():
                linea = flujo.readline(MAX_LINEA + 1)
                if not linea:
                    break
                if len(linea) > MAX_LINEA:
                    break
                with self.cerrojo:
                    self.linea = linea
        finally:
            self.fin.set()

    def tomar(self):
        with self.cerrojo:
            linea, self.linea = self.linea, None
            return linea


class BusquedaLidarNode(EvasionLidarNode):
    def __init__(self, mover, cfg, entrada):
        self.entrada = entrada
        self.puerta = PuertaBusqueda()
        self.pose, self.pose_ns = None, None
        self.motivo_control = 'Esperando Buscar'
        self._estado_ns = 0
        super().__init__(mover, cfg)
        qos_odom = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odometria, qos_odom)

    def odometria(self, msg):
        self.pose = pose_odometria(msg)
        self.pose_ns = time.monotonic_ns()

    def publicar_velocidad(self, vx, wz):
        ahora = time.monotonic_ns()
        linea = self.entrada.tomar()
        if linea is not None:
            self.puerta.recibir(linea, ahora)
        permitido, self.motivo_control = self.puerta.permiso(ahora, self.pose, self.pose_ns)
        if self.entrada.fin.is_set() or self._terminar or not permitido:
            vx, wz = 0.0, 0.0
        elif vx == 0.0 and wz == 0.0:
            self.motivo_control = 'LiDAR/TF insuficientes: detenido; revisar terminal'
        super().publicar_velocidad(vx, wz)
        if ahora - self._estado_ns >= 200_000_000:
            print(json.dumps(dict(version=1, recibido_ns=ahora, a=self.puerta.punto_a,
                                  estado=self.motivo_control, vx=vx, wz=wz,
                                  mover=self.pub_cmd is not None)), flush=True)
            self._estado_ns = ahora
        return vx, wz


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--mover', action='store_true')
    args = parser.parse_args()
    cfg = cargar_config(args.config)
    bloqueo = None
    if args.mover:
        bloqueo = open('/tmp/proyecto2-evasion-control.lock', 'a')
        try:
            fcntl.flock(bloqueo, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            parser.error('Ya hay otra evasión/búsqueda con movimiento en este contenedor')
    entrada = EntradaControl(sys.stdin)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    nodo = None
    def terminar(_senal, _frame):
        entrada.fin.set()
        if nodo is not None:
            nodo._terminar = True
    anteriores = {s: signal.signal(s, terminar) for s in (signal.SIGINT, signal.SIGTERM)}
    try:
        nodo = BusquedaLidarNode(args.mover, cfg, entrada)
        while rclpy.ok() and not entrada.fin.is_set():
            rclpy.spin_once(nodo, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException, BrokenPipeError):
        pass
    finally:
        entrada.fin.set()
        if nodo is not None:
            # No escribir telemetría durante el cierre: publicar cero aunque stdout se haya cerrado.
            if rclpy.ok():
                for _ in range(3):
                    EvasionLidarNode.publicar_velocidad(nodo, 0.0, 0.0)
                    time.sleep(0.03)
            nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for s, anterior in anteriores.items():
            signal.signal(s, anterior)
        if bloqueo is not None:
            bloqueo.close()


if __name__ == '__main__':
    main()
