#!/usr/bin/env python3
"""Orquestador de navegación autónoma (SLAM + Nav2 + Exploración + Visión) para Task A."""

import argparse
from math import atan2, cos, sin
from pathlib import Path
import time

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import OccupancyGrid, Odometry
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformException, TransformListener

from camara_ros import CamaraTurtleBot
from detector_meta_b import DetectorMetaB
from exploracion import detectar_fronteras
from mision import (
    EstadoMision,
    cargar_config_mision,
    cargar_sistema_facial,
    evaluar_frame_facial,
    votar_identidad,
)


def yaw_desde_cuaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return atan2(siny_cosp, cosy_cosp)


def crear_pose(navigator: BasicNavigator, x: float, y: float, yaw: float = 0.0) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = 0.0
    pose.pose.orientation.z = sin(yaw * 0.5)
    pose.pose.orientation.w = cos(yaw * 0.5)
    return pose


class ObservadorNavegacion(Node):
    """Suscriptor ligero a /map, /odom y TF (map -> base_link) para exploración."""

    def __init__(self, use_sim_time: bool):
        super().__init__(
            "observador_navegacion_node",
            parameter_overrides=[Parameter("use_sim_time", value=use_sim_time)],
        )
        self.mapa: OccupancyGrid | None = None
        self.odom: Odometry | None = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        qos_map = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        qos_odom = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(OccupancyGrid, "/map", self._map_cb, qos_map)
        self.create_subscription(Odometry, "/odom", self._odom_cb, qos_odom)

    def _map_cb(self, msg: OccupancyGrid):
        self.mapa = msg

    def _odom_cb(self, msg: Odometry):
        self.odom = msg

    def pose_actual(self) -> tuple[float, float, float]:
        """Devuelve (x, y, yaw) en el marco 'map' (con respaldo a (0,0,0))."""
        try:
            tf = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
            t = tf.transform.translation
            yaw = yaw_desde_cuaternion(tf.transform.rotation)
            return (float(t.x), float(t.y), float(yaw))
        except TransformException:
            return (0.0, 0.0, 0.0)

    def grilla_actual(self) -> tuple[np.ndarray, float, float, float] | None:
        if self.mapa is None:
            return None
        info = self.mapa.info
        arr = np.asarray(self.mapa.data, dtype=np.int16).reshape((info.height, info.width))
        return (
            arr,
            float(info.resolution),
            float(info.origin.position.x),
            float(info.origin.position.y),
        )


def capturar_y_reconocer(
    nodo_camara: CamaraTurtleBot,
    executor: MultiThreadedExecutor,
    sistema_facial,
    cfg_rec: dict,
    edad_maxima_s: float,
) -> dict:
    """Toma una ráfaga de frames con el robot detenido en B y vota la identidad."""
    rafaga = int(cfg_rec.get("rafaga_frames", 8))
    espera = float(cfg_rec.get("espera_entre_frames_s", 0.25))
    min_votos = int(cfg_rec.get("min_votos", 2))
    usar_clahe = bool(cfg_rec.get("usar_clahe", True))

    decisiones = []
    print(f"[VISIÓN] Capturando ráfaga de {rafaga} frames en Punto B...", flush=True)
    for idx in range(rafaga):
        executor.spin_once(timeout_sec=0.15)
        frame = nodo_camara.obtener_frame(max_edad_s=edad_maxima_s)
        if frame is not None:
            res = evaluar_frame_facial(frame, sistema_facial, usar_clahe=usar_clahe)
            decisiones.append(res)
            print(
                f"  frame {idx + 1}/{rafaga}: {res.get('nombre')} "
                f"({res.get('motivo')}, sim={res.get('similitud')})",
                flush=True,
            )
        else:
            print(f"  frame {idx + 1}/{rafaga}: sin imagen reciente", flush=True)
        time.sleep(espera)

    veredicto = votar_identidad(decisiones, min_votos=min_votos)
    print(f"[VISIÓN] Veredicto en B: {veredicto}", flush=True)
    return veredicto


def esperar_nav2_con_slam(navigator: BasicNavigator, observador: ObservadorNavegacion, executor: MultiThreadedExecutor):
    """Espera a que SLAM publique /map y Nav2 (sin AMCL) esté activo."""
    print("[NAV2] Esperando /map de slam_toolbox y activación de Nav2...", flush=True)
    # slam_toolbox es el localizador cuando mapeamos en vivo (no AMCL)
    navigator.waitUntilNav2Active(localizer="slam_toolbox")
    limite = time.monotonic() + 20.0
    while observador.grilla_actual() is None and time.monotonic() < limite:
        executor.spin_once(timeout_sec=0.2)
    if observador.grilla_actual() is None:
        print("[ALERTA] Nav2 activo pero aún no llegó /map; continuando con espera breve.", flush=True)
    else:
        print("[NAV2] SLAM + Nav2 listos.", flush=True)


def explorar_hasta_b(
    navigator: BasicNavigator,
    observador: ObservadorNavegacion,
    camara: CamaraTurtleBot,
    detector_b: DetectorMetaB,
    executor: MultiThreadedExecutor,
    cfg: dict,
) -> bool:
    """Explora fronteras del laberinto hasta detectar visualmente el Punto B."""
    cfg_exp = cfg.get("exploracion", {})
    cfg_cam = cfg.get("camara", {})
    edad_max = float(cfg_cam.get("edad_maxima_s", 1.0))
    timeout_meta = float(cfg_exp.get("timeout_meta_s", 18.0))
    tiempo_max_total = float(cfg_exp.get("tiempo_max_exploracion_s", 180.0))
    max_intentos = int(cfg_exp.get("max_intentos_meta", 25))

    lista_negra: list[tuple[float, float]] = []
    inicio_exp = time.monotonic()
    intentos = 0

    while rclpy.ok() and (time.monotonic() - inicio_exp) < tiempo_max_total and intentos < max_intentos:
        executor.spin_once(timeout_sec=0.1)
        frame = camara.obtener_frame(max_edad_s=edad_max)
        det = detector_b.evaluar_frame(frame)
        if det.detectado:
            print(f"[META B] Detectado durante exploración: {det.motivo}", flush=True)
            navigator.cancelTask()
            return True

        grilla_info = observador.grilla_actual()
        if grilla_info is None:
            time.sleep(0.2)
            continue

        grilla, res, ox, oy = grilla_info
        rx, ry, ryaw = observador.pose_actual()
        fronteras = detectar_fronteras(
            grilla=grilla,
            resolucion=res,
            origen_x=ox,
            origen_y=oy,
            robot_x=rx,
            robot_y=ry,
            robot_yaw=ryaw,
            radio_seguridad_celdas=int(cfg_exp.get("radio_seguridad_celdas", 4)),
            min_tamano_cluster=int(cfg_exp.get("min_tamano_cluster", 4)),
            lista_negra=lista_negra,
            radio_exclusion_m=float(cfg_exp.get("radio_exclusion_m", 0.45)),
        )

        if not fronteras:
            print("[EXPLORACIÓN] Sin fronteras nuevas; ejecutando giro de barrido 360°...", flush=True)
            navigator.spin(spin_dist=6.28, time_allowance=15)
            while not navigator.isTaskComplete():
                executor.spin_once(timeout_sec=0.1)
                det = detector_b.evaluar_frame(camara.obtener_frame(max_edad_s=edad_max))
                if det.detectado:
                    navigator.cancelTask()
                    print(f"[META B] Detectado en giro de barrido: {det.motivo}", flush=True)
                    return True
            if lista_negra:
                lista_negra.clear()
                continue
            return False

        objetivo = fronteras[0]
        intentos += 1
        print(
            f"[EXPLORACIÓN #{intentos}] Frontera ({objetivo.x:.2f}, {objetivo.y:.2f}) "
            f"dist={objetivo.distancia:.2f}m celdas={objetivo.tamano}",
            flush=True,
        )
        meta_pose = crear_pose(navigator, objetivo.x, objetivo.y, atan2(objetivo.y - ry, objetivo.x - rx))
        navigator.goToPose(meta_pose)
        inicio_meta = time.monotonic()

        while not navigator.isTaskComplete():
            executor.spin_once(timeout_sec=0.1)
            det = detector_b.evaluar_frame(camara.obtener_frame(max_edad_s=edad_max))
            if det.detectado:
                print(f"[META B] Detectado en ruta: {det.motivo}", flush=True)
                navigator.cancelTask()
                return True
            if (time.monotonic() - inicio_meta) > timeout_meta:
                print("[EXPLORACIÓN] Timeout en frontera; marcando en lista negra.", flush=True)
                navigator.cancelTask()
                lista_negra.append((objetivo.x, objetivo.y))
                break

        resultado = navigator.getResult()
        if resultado != TaskResult.SUCCEEDED:
            lista_negra.append((objetivo.x, objetivo.y))

    return False


def main():
    raiz_default = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--modo", choices=["gazebo", "robot"], default="gazebo")
    parser.add_argument("--meta", choices=["rostro", "aruco", "color", "coordenada"], default=None)
    parser.add_argument("--bx", type=float, default=None, help="Coordenada X de B (solo --meta coordenada)")
    parser.add_argument("--by", type=float, default=None, help="Coordenada Y de B (solo --meta coordenada)")
    parser.add_argument("--config", default=str(raiz_default / "config" / "mision_config.json"))
    parser.add_argument("--datos-dir", default=str(raiz_default.parent / "task 1" / "datos"))
    parser.add_argument("--modelos-dir", default=str(raiz_default.parent / "task 1" / "modelos"))
    parser.add_argument("--solo-reconocimiento", action="store_true", help="Prueba cámara + reconocimiento sin mover el robot")
    args, ros_args = parser.parse_known_args()

    cfg = cargar_config_mision(Path(args.config))
    if args.meta is not None:
        cfg["meta_b"]["modo"] = args.meta
    if args.bx is not None:
        cfg["meta_b"]["coordenada_x"] = args.bx
    if args.by is not None:
        cfg["meta_b"]["coordenada_y"] = args.by

    use_sim_time = args.modo == "gazebo"
    sistema_facial = cargar_sistema_facial(Path(args.datos_dir), Path(args.modelos_dir))
    print("[FACIAL] Modelos YuNet/SFace, galería y criterio cargados.", flush=True)

    rclpy.init(args=ros_args)
    cfg_cam = cfg.get("camara", {})
    edad_max = float(cfg_cam.get("edad_maxima_s", 1.0))

    camara = CamaraTurtleBot(
        topico_raw=cfg_cam.get("topico_raw", "/oakd/rgb/preview/image_raw"),
        topico_compressed=cfg_cam.get("topico_compressed", "/oakd/rgb/preview/image_raw/compressed"),
        use_sim_time=use_sim_time,
    )
    observador = ObservadorNavegacion(use_sim_time=use_sim_time)
    executor = MultiThreadedExecutor()
    executor.add_node(camara)
    executor.add_node(observador)

    navigator = None
    estado = EstadoMision.INICIO_A

    try:
        if args.solo_reconocimiento:
            print("[MODO] Solo reconocimiento facial (sin navegación).", flush=True)
            for _ in range(15):
                executor.spin_once(timeout_sec=0.1)
                if camara.obtener_frame(max_edad_s=edad_max) is not None:
                    break
            veredicto = capturar_y_reconocer(
                camara, executor, sistema_facial, cfg.get("reconocimiento", {}), edad_max
            )
            print(f"RESULTADO PRUEBA FACIAL: {veredicto['nombre']}", flush=True)
            return

        navigator = BasicNavigator()
        esperar_nav2_con_slam(navigator, observador, executor)

        ax, ay, ayaw = observador.pose_actual()
        pose_A = crear_pose(navigator, ax, ay, ayaw)
        print(f"[MISIÓN] Punto A registrado en ({ax:.2f}, {ay:.2f}).", flush=True)

        estado = EstadoMision.EXPLORANDO_HACIA_B
        detector_b = DetectorMetaB(cfg["meta_b"], extractor_facial=sistema_facial.extractor)

        if cfg["meta_b"]["modo"] == "coordenada":
            bx = float(cfg["meta_b"]["coordenada_x"])
            by = float(cfg["meta_b"]["coordenada_y"])
            byaw = float(cfg["meta_b"].get("coordenada_yaw", 0.0))
            print(f"[MISIÓN] Navegando directo a Punto B ({bx:.2f}, {by:.2f})...", flush=True)
            navigator.goToPose(crear_pose(navigator, bx, by, byaw))
            while not navigator.isTaskComplete():
                executor.spin_once(timeout_sec=0.1)
            llego_b = navigator.getResult() == TaskResult.SUCCEEDED
        else:
            llego_b = explorar_hasta_b(navigator, observador, camara, detector_b, executor, cfg)

        if not llego_b:
            estado = EstadoMision.FALLO
            print("[FALLO] No se alcanzó el Punto B dentro del límite.", flush=True)
            return

        estado = EstadoMision.RECONOCIENDO_B
        veredicto = capturar_y_reconocer(
            camara, executor, sistema_facial, cfg.get("reconocimiento", {}), edad_max
        )
        identidad = veredicto["nombre"]

        estado = EstadoMision.RETORNANDO_A
        print(f"[MISIÓN] Retornando de B hacia A ({ax:.2f}, {ay:.2f})...", flush=True)
        retorno_ok = False
        for intento in range(1, 3):
            navigator.goToPose(crear_pose(navigator, ax, ay, ayaw))
            while not navigator.isTaskComplete():
                executor.spin_once(timeout_sec=0.1)
            if navigator.getResult() == TaskResult.SUCCEEDED:
                retorno_ok = True
                break
            print(f"[ALERTA] Reintento {intento} de retorno hacia A...", flush=True)
            navigator.backup(backup_dist=0.15, backup_speed=0.08, time_allowance=5)
            while not navigator.isTaskComplete():
                executor.spin_once(timeout_sec=0.1)

        if retorno_ok:
            estado = EstadoMision.FINALIZADO
            print("==================================================", flush=True)
            print(f"RESULTADO FINAL EN A: LA PERSONA ES: {identidad}", flush=True)
            print(f"DETALLE: {veredicto}", flush=True)
            print("==================================================", flush=True)
        else:
            estado = EstadoMision.FALLO
            print(f"[FALLO] No se completó el retorno a A. Identidad registrada: {identidad}", flush=True)

    except (KeyboardInterrupt, ExternalShutdownException):
        print(f"[INTERRUMPIDO] Estado final: {estado.value}", flush=True)
    finally:
        if navigator is not None:
            try:
                navigator.cancelTask()
            except Exception:
                pass
        executor.shutdown()
        camara.destroy_node()
        observador.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
