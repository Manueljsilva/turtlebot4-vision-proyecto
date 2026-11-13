"""Prueba reactiva de evasión de cajas y paredes con LiDAR (/scan -> /cmd_vel).

Sirve como verificación rápida antes de usar SLAM/Nav2 completo:
  - Sin --mover: solo imprime distancias frontal, izquierda y derecha.
  - Con --mover: avanza y esquiva obstáculos publicando TwistStamped en /cmd_vel.
"""

import argparse
from math import atan2, degrees
import time
from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

from exploracion import analizar_lidar, calcular_velocidad_evasion
from mision import cargar_config_mision


def yaw_desde_cuaternion(q) -> float:
    return atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class EvasionLidarNode(Node):
    def __init__(self, mover: bool, use_sim_time: bool, cfg_evasion: dict):
        super().__init__(
            "evasion_lidar_node",
            parameter_overrides=[Parameter("use_sim_time", value=use_sim_time)],
        )
        self.mover = mover
        self.cfg = cfg_evasion
        self._ultimo_scan: LaserScan | None = None
        self._ultimo_scan_ns: int | None = None
        self._yaw_lidar: float | None = None
        self._inicio_tf_ns: int = time.monotonic_ns()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        qos_scan = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.sub_scan = self.create_subscription(LaserScan, "/scan", self._scan_cb, qos_scan)
        self.pub_cmd = self.create_publisher(TwistStamped, "/cmd_vel", 10) if mover else None
        # Usar reloj monotono para reportar estado aunque /clock de Gazebo esté pausado.
        self._reloj_monotono = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(0.1, self._ciclo, clock=self._reloj_monotono)
        self._ultimo_log = 0.0
        self.get_logger().info(
            f"Evasión LiDAR lista (mover={mover}, use_sim_time={use_sim_time})"
        )

    def _scan_cb(self, msg: LaserScan):
        self._ultimo_scan = msg
        self._ultimo_scan_ns = time.monotonic_ns()

    def publicar_velocidad(self, lineal: float, angular: float):
        if self.pub_cmd is None:
            return
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x = float(lineal)
        msg.twist.angular.z = float(angular)
        self.pub_cmd.publish(msg)

    def _ciclo(self):
        ahora = time.monotonic()
        if self._ultimo_scan is None or self._ultimo_scan_ns is None:
            self.publicar_velocidad(0.0, 0.0)
            if ahora - self._ultimo_log >= 2.0:
                self.get_logger().info("Esperando mensajes en /scan...")
                self._ultimo_log = ahora
            return

        edad = (time.monotonic_ns() - self._ultimo_scan_ns) / 1e9
        if edad > 0.6:
            self.publicar_velocidad(0.0, 0.0)
            if ahora - self._ultimo_log >= 1.5:
                self.get_logger().warn("LiDAR caducado (>0.6s): robot detenido")
                self._ultimo_log = ahora
            return

        scan = self._ultimo_scan
        if self._yaw_lidar is None:
            frame_lidar = scan.header.frame_id.lstrip("/") or "rplidar_link"
            if frame_lidar == "base_link":
                self._yaw_lidar = 0.0
            else:
                try:
                    tf = self.tf_buffer.lookup_transform("base_link", frame_lidar, rclpy.time.Time())
                    self._yaw_lidar = yaw_desde_cuaternion(tf.transform.rotation)
                    self.get_logger().info(
                        f"TF LiDAR resuelto: base_link -> {frame_lidar} "
                        f"(offset={degrees(self._yaw_lidar):.1f} deg)"
                    )
                except TransformException:
                    if (time.monotonic_ns() - self._inicio_tf_ns) / 1e9 >= 3.0:
                        self._yaw_lidar = 0.0
                        self.get_logger().warn(
                            f"Sin TF base_link -> {frame_lidar} tras 3s; usando offset 0.0 deg"
                        )
                    else:
                        self.publicar_velocidad(0.0, 0.0)
                        if ahora - self._ultimo_log >= 2.0:
                            self.get_logger().info(f"Esperando TF base_link -> {frame_lidar}...")
                            self._ultimo_log = ahora
                        return

        estado = analizar_lidar(
            ranges=scan.ranges,
            angle_min=scan.angle_min,
            angle_increment=scan.angle_increment,
            range_min=max(0.12, float(scan.range_min)),
            range_max=min(12.0, float(scan.range_max)),
            distancia_parada_m=float(self.cfg.get("distancia_parada_m", 0.36)),
            distancia_precaucion_m=float(self.cfg.get("distancia_precaucion_m", 0.65)),
            angulo_cono_frontal_deg=float(self.cfg.get("angulo_cono_frontal_deg", 35.0)),
            angle_offset_rad=self._yaw_lidar,
        )
        vel_x, vel_w = calcular_velocidad_evasion(
            estado,
            velocidad_avance=float(self.cfg.get("velocidad_avance", 0.18)),
            velocidad_giro=float(self.cfg.get("velocidad_giro", 0.85)),
        )
        self.publicar_velocidad(vel_x, vel_w)

        if ahora - self._ultimo_log >= 0.8:
            accion = "PARADA/GIRO" if estado.peligro_parada else ("PRECAUCION" if estado.precaucion else "AVANCE")
            self.get_logger().info(
                f"[{accion}] frente={estado.dist_frente:.2f}m izq={estado.dist_izquierda:.2f}m "
                f"der={estado.dist_derecha:.2f}m -> cmd=({vel_x:.2f}, {vel_w:.2f})"
            )
            self._ultimo_log = ahora


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--modo", choices=["gazebo", "robot"], default="gazebo")
    parser.add_argument("--mover", action="store_true", help="Publica TwistStamped en /cmd_vel")
    parser.add_argument("--config", default=None, help="Ruta a mision_config.json")
    args, ros_args = parser.parse_known_args()

    cfg = cargar_config_mision(args.config).get("evasion_lidar", {})
    use_sim_time = args.modo == "gazebo"

    rclpy.init(args=ros_args)
    nodo = None
    try:
        nodo = EvasionLidarNode(mover=args.mover, use_sim_time=use_sim_time, cfg_evasion=cfg)
        rclpy.spin(nodo)
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


if __name__ == "__main__":
    main()
