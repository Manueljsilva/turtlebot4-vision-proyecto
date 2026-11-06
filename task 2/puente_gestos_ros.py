"""Recibe órdenes por stdin y controla /cmd_vel dentro de Docker.

Sin --publicar comprueba órdenes sin crear un publicador de velocidad.
--robot-real usa el reloj del sistema y no exige /clock; sin esa opcion usa Gazebo.
--sin-ros permite comprobar el transporte local sin ROS ni Docker.
"""

import argparse
import fcntl
import queue
import sys
import threading
import time

from control_gestos import (MAX_LINEA, RelojSimulado, VigilanteOrden,
                            validar_velocidades_robot, velocidades)
from gestos import Orden


class EntradaOrdenes:
    """Lectura independiente; solo conserva la última línea, no acumula órdenes."""

    def __init__(self, flujo):
        self.ultima = queue.Queue(maxsize=1)
        self.fin = threading.Event()
        self._hilo = threading.Thread(target=self._leer, args=(flujo,), daemon=True)
        self._hilo.start()

    def _guardar(self, linea):
        try:
            self.ultima.get_nowait()
        except queue.Empty:
            pass
        self.ultima.put_nowait(linea)

    def _leer(self, flujo):
        try:
            while True:
                linea = flujo.readline(MAX_LINEA + 1)
                if not linea:
                    break
                if len(linea) > MAX_LINEA:
                    self._guardar("INVALIDO")
                    while linea and not linea.endswith("\n"):
                        linea = flujo.readline(MAX_LINEA + 1)
                    continue
                self._guardar(linea)
        finally:
            self.fin.set()

    def actualizar(self, vigilante, ahora_ns):
        if self.fin.is_set():
            vigilante.detener("Enlace de webcam cerrado")
            return
        try:
            vigilante.recibir(self.ultima.get_nowait(), ahora_ns)
        except queue.Empty:
            pass


def prueba_sin_ros(entrada):
    """Mismo transporte y vigilante; no importa ROS ni publica movimiento."""
    vigilante = VigilanteOrden()
    anterior = None
    print("[PRUEBA LOCAL] Sin ROS / sin movimiento", flush=True)
    while not entrada.fin.is_set():
        ahora = time.monotonic_ns()
        entrada.actualizar(vigilante, ahora)
        orden = vigilante.vigente(ahora)
        estado = (orden, vigilante.motivo)
        if estado != anterior:
            print(f"[PRUEBA LOCAL] {orden.value} {velocidades(orden)} | {vigilante.motivo}", flush=True)
            anterior = estado
        time.sleep(0.05)
    print("[PRUEBA LOCAL] DETENER (0.0, 0.0) | Enlace cerrado", flush=True)


def ejecutar_ros(entrada, publicar, ros_args, robot_real=False):
    import rclpy
    from geometry_msgs.msg import TwistStamped
    from rclpy.clock import Clock, ClockType
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.parameter import Parameter

    class PuenteGestos(Node):
        def __init__(self):
            super().__init__("puente_gestos_webcam", parameter_overrides=[
                Parameter("use_sim_time", value=not robot_real)])
            if self.get_parameter("use_sim_time").value != (not robot_real):
                raise RuntimeError("use_sim_time no coincide con el modo seleccionado")
            self.vigilante = VigilanteOrden()
            self.reloj = None if robot_real else RelojSimulado()
            self.terminado = False
            self.anterior = None
            self.publicador = self.create_publisher(TwistStamped, "/cmd_vel", 10) if publicar else None
            # El vigilante sigue funcionando aunque /clock deje de avanzar.
            self.reloj_monotono = Clock(clock_type=ClockType.STEADY_TIME)
            self.temporizador = self.create_timer(0.05, self.ciclo, clock=self.reloj_monotono)
            self.get_logger().info("MOVIMIENTO ACTIVADO" if publicar else
                                   "COMPROBACION: no se publica velocidad")
            self.get_logger().info("ROBOT REAL: reloj del sistema, sin /clock" if robot_real else
                                   "GAZEBO: esperando reloj simulado")

        def enviar(self, orden):
            if self.publicador is None:
                return
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "base_link"
            msg.twist.linear.x, msg.twist.angular.z = velocidades(orden)
            self.publicador.publish(msg)

        def ciclo(self):
            ahora = time.monotonic_ns()
            entrada.actualizar(self.vigilante, ahora)
            orden = self.vigilante.vigente(ahora)
            motivo = self.vigilante.motivo
            if self.reloj is not None and not self.reloj.disponible(
                    self.get_clock().now().nanoseconds, ahora):
                orden, motivo = Orden.DETENER, "Esperando /clock activo de Gazebo"
            elif self.publicador is not None and self.publicador.get_subscription_count() == 0:
                orden, motivo = Orden.DETENER, "Sin suscriptores en /cmd_vel"
            self.enviar(orden)
            estado = (orden, motivo)
            if estado != self.anterior:
                self.get_logger().info(f"{orden.value}: {velocidades(orden)} | {motivo}")
                self.anterior = estado
            self.terminado = entrada.fin.is_set()

    rclpy.init(args=ros_args)
    nodo = None
    try:
        nodo = PuenteGestos()
        while rclpy.ok() and not nodo.terminado:
            rclpy.spin_once(nodo, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if nodo is not None:
            if rclpy.ok():
                for _ in range(3):
                    nodo.enviar(Orden.DETENER)
                    time.sleep(0.02)
            nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--publicar", action="store_true")
    parser.add_argument("--sin-ros", action="store_true")
    parser.add_argument("--robot-real", action="store_true")
    args, ros_args = parser.parse_known_args()
    if args.sin_ros and (args.publicar or args.robot_real or ros_args):
        parser.error("--sin-ros no admite publicación, --robot-real ni argumentos ROS")
    if args.sin_ros:
        prueba_sin_ros(EntradaOrdenes(sys.stdin))
        return
    if args.robot_real:
        validar_velocidades_robot()
    # Evitar dos copias de este puente publicando desde el mismo contenedor.
    with open("/tmp/proyecto2-puente-gestos.lock", "a") as bloqueo:
        try:
            fcntl.flock(bloqueo, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise SystemExit("Ya hay un puente de gestos abierto. Cierra la prueba anterior.")
        ejecutar_ros(EntradaOrdenes(sys.stdin), args.publicar, ros_args, args.robot_real)


if __name__ == "__main__":
    main()
