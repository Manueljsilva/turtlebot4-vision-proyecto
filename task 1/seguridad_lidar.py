"""Prueba LiDAR del compañero adaptada a Task 1; sin publicar salvo --mover."""

import argparse
from collections import deque
import fcntl
from math import atan2, isfinite, sqrt
from pathlib import Path
import signal
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

from evasion_lidar import cargar_config, decidir_scan, resumir_cobertura


class EvasionLidarNode(Node):
    def __init__(self, mover, cfg):
        super().__init__('evasion_lidar_node',
                         parameter_overrides=[Parameter('use_sim_time', value=False)])
        self.cfg = cfg
        self._scan = None
        self._recibido_ns = None
        self._recepciones_ns = deque(maxlen=30)
        self._ultimo_log = 0.0
        self._terminar = False
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self._scan_cb, qos)
        self.pub_cmd = self.create_publisher(TwistStamped, '/cmd_vel', 1) if mover else None
        self._reloj_monotono = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(0.1, self._ciclo, clock=self._reloj_monotono)
        self.get_logger().info(f'Evasión LiDAR: mover={mover}, reloj real; {cfg}')

    def _scan_cb(self, msg):
        self._scan = msg
        self._recibido_ns = time.monotonic_ns()
        self._recepciones_ns.append(self._recibido_ns)

    def _yaw_lidar(self, frame):
        frame = frame.lstrip('/')
        if frame == 'base_link':
            return 0.0
        if not frame:
            return None
        try:
            tf = self.tf_buffer.lookup_transform('base_link', frame, rclpy.time.Time())
        except TransformException:
            return None
        q = tf.transform.rotation
        valores = (q.x, q.y, q.z, q.w)
        if not all(isfinite(v) for v in valores):
            return None
        norma = sqrt(sum(v * v for v in valores))
        if not isfinite(norma) or norma < 1e-12:
            return None
        x, y, z, w = (v / norma for v in valores)
        return atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

    def publicar_velocidad(self, vx, wz):
        if self.pub_cmd is not None:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'
            msg.twist.linear.x = float(vx)
            msg.twist.angular.z = float(wz)
            self.pub_cmd.publish(msg)
        return vx, wz

    def _ciclo(self):
        scan = self._scan
        edad = None if self._recibido_ns is None else (time.monotonic_ns() - self._recibido_ns) / 1e9
        yaw = self._yaw_lidar(scan.header.frame_id) if scan is not None else None
        vx, wz, estado, motivo = decidir_scan(scan, yaw, edad, self.cfg)
        if self._terminar:
            vx, wz = 0.0, 0.0
        vx, wz = self.publicar_velocidad(vx, wz)
        ahora = time.monotonic()
        if ahora - self._ultimo_log >= 0.8:
            distancias = '' if estado is None else (
                f' frente={estado.dist_frente:.2f}m izq={estado.dist_izquierda:.2f}m'
                f' der={estado.dist_derecha:.2f}m')
            cobertura = '' if estado is None else f' | {resumir_cobertura(estado)}'
            recepcion = '' if edad is None else f' | edad_rx={edad:.2f}s'
            if len(self._recepciones_ns) >= 2:
                intervalo = (self._recepciones_ns[-1] - self._recepciones_ns[0]) / 1e9
                if intervalo > 0:
                    hz = (len(self._recepciones_ns) - 1) / intervalo
                    recepcion += f' rx={hz:.1f}Hz'
            self.get_logger().info(
                f'{motivo}{distancias} -> cmd=({vx:.2f}, {wz:.2f}){recepcion}{cobertura}')
            self._ultimo_log = ahora


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', default=Path(__file__).parent / 'config/evasion_lidar.json')
    parser.add_argument('--mover', action='store_true')
    args = parser.parse_args()
    try:
        cfg = cargar_config(args.config)
    except (OSError, ValueError, TypeError) as exc:
        parser.error(str(exc))
    bloqueo = None
    if args.mover:
        bloqueo = open('/tmp/proyecto2-evasion-control.lock', 'a')
        try:
            fcntl.flock(bloqueo, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            parser.error('Ya hay otra prueba de evasión con movimiento en este contenedor')

    # Mantener el contexto vivo para intentar publicar cero antes del cierre normal.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    terminar = False

    def solicitar_cierre(_numero, _frame):
        nonlocal terminar
        terminar = True
        if nodo is not None:
            nodo._terminar = True

    nodo = None
    anteriores = {s: signal.signal(s, solicitar_cierre) for s in (signal.SIGINT, signal.SIGTERM)}
    try:
        nodo = EvasionLidarNode(args.mover, cfg)
        while rclpy.ok() and not terminar:
            rclpy.spin_once(nodo, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if nodo is not None:
            if rclpy.ok():
                for _ in range(3):
                    nodo.publicar_velocidad(0.0, 0.0)
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
