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
from control_giros import ControlGiro
from config_senales import cargar_config_senales
from evasion_lidar import cargar_config, decidir_scan
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
    def __init__(self, mover, cfg, entrada, giros=None):
        self.entrada = entrada
        self.puerta = PuertaBusqueda()
        self.pose, self.pose_ns = None, None
        self.motivo_control = 'Esperando Buscar'
        self._estado_ns = 0
        self.giros = giros
        self.ultima_senal_recibida = None
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
            if self.puerta.senal is not None:
                self.ultima_senal_recibida = dict(self.puerta.senal)
        permitido, self.motivo_control = self.puerta.permiso(ahora, self.pose, self.pose_ns)
        permitido = permitido and not self.entrada.fin.is_set() and not self._terminar
        if self.giros is not None and not self.puerta.senales_activas:
            permitido = False
            self.motivo_control = 'Esperando detector de senales: detenido'
        if not permitido:
            vx, wz = 0.0, 0.0
        elif vx == 0.0 and wz == 0.0:
            self.motivo_control = 'LiDAR/TF insuficientes: detenido; revisar terminal'
        diagnostico_giro = None
        if self.giros is not None:
            scan = self._scan
            edad = None if self._recibido_ns is None else (ahora-self._recibido_ns)/1e9
            yaw = self._yaw_lidar(scan.header.frame_id) if scan is not None else None
            _, _, lidar, _ = decidir_scan(scan, yaw, edad, self.cfg)
            giro = self.giros.decidir(self.puerta.senal, self.pose, ahora, permitido, lidar, self.cfg)
            if giro is not None:
                vx, wz, motivo = giro
                if permitido:
                    self.motivo_control = motivo
            if hasattr(self.giros, 'pendiente'):
                pendiente = self.giros.pendiente
                maniobra = self.giros.activo or pendiente
                fase = ('FALLO' if self.giros.fallo else 'GIRANDO' if self.giros.activo
                        else 'ESPERA_PARED' if pendiente else 'SIN_FLECHA_PENDIENTE')
                diagnostico_giro = dict(fase=fase, permitido=permitido,
                    direccion=maniobra['direccion'] if maniobra else None,
                    frente_m=lidar.dist_frente if lidar is not None and lidar.observacion_valida else None,
                    disparo_m=self.giros.cfg.esperar_pared_m,
                    pendiente_s=(ahora-pendiente['recibido_ns'])/1e9 if pendiente else None,
                    id_consumido=self.giros.ultimo_id, vx_propuesto=vx, wz_propuesto=wz,
                    publicando=self.pub_cmd is not None,
                    id_recibido=self.puerta.senal['id'] if self.puerta.senal else None,
                    ultimo_id_recibido=self.ultima_senal_recibida['id'] if self.ultima_senal_recibida else None)
        super().publicar_velocidad(vx, wz)
        if ahora - self._estado_ns >= 200_000_000:
            estado = dict(version=1, recibido_ns=ahora, a=self.puerta.punto_a,
                                  estado=self.motivo_control, vx=vx, wz=wz,
                                  mover=self.pub_cmd is not None)
            if diagnostico_giro is not None:
                estado['giro_ensayo'] = diagnostico_giro
            print(json.dumps(estado), flush=True)
            self._estado_ns = ahora
        return vx, wz


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--mover', action='store_true')
    parser.add_argument('--senales', action='store_true')
    parser.add_argument('--tramo', choices=('ida', 'vuelta'), default='ida')
    parser.add_argument('--config-senales', type=Path)
    args = parser.parse_args()
    cfg = cargar_config(args.config)
    giros = None
    if args.senales:
        if args.config_senales is None:
            parser.error('Falta --config-senales')
        giros = ControlGiro(args.tramo, cargar_config_senales(args.config_senales))
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
        nodo = BusquedaLidarNode(args.mover, cfg, entrada, giros)
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
