"""Lector ROS 2 de la cámara OAK-D (soporta Image crudo y CompressedImage)."""

import time
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image


class CamaraTurtleBot(Node):
    """Suscriptor resiliente a tópicos de cámara del TurtleBot 4 / Gazebo."""

    def __init__(
        self,
        topico_raw: str = "/oakd/rgb/preview/image_raw",
        topico_compressed: str = "/oakd/rgb/preview/image_raw/compressed",
        use_sim_time: bool = False,
    ):
        super().__init__(
            "camara_turtlebot_node",
            parameter_overrides=[
                rclpy.parameter.Parameter("use_sim_time", value=use_sim_time)
            ],
        )
        self.bridge = CvBridge()
        self._ultimo_frame: np.ndarray | None = None
        self._ultimo_frame_ns: int | None = None
        self._origen_activo: str | None = None

        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.sub_raw = self.create_subscription(
            Image, topico_raw, self._raw_callback, qos_sensor
        )
        self.sub_compressed = self.create_subscription(
            CompressedImage, topico_compressed, self._compressed_callback, qos_sensor
        )

    def _raw_callback(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self._ultimo_frame = frame
        self._ultimo_frame_ns = time.monotonic_ns()
        if self._origen_activo is None:
            self._origen_activo = "raw"
            self.get_logger().info(f"Cámara conectada vía Image crudo ({frame.shape[1]}x{frame.shape[0]})")

    def _compressed_callback(self, msg: CompressedImage):
        frame = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self._ultimo_frame = frame
        self._ultimo_frame_ns = time.monotonic_ns()
        if self._origen_activo is None:
            self._origen_activo = "compressed"
            self.get_logger().info(f"Cámara conectada vía CompressedImage ({frame.shape[1]}x{frame.shape[0]})")

    def edad_frame_s(self) -> float | None:
        if self._ultimo_frame_ns is None:
            return None
        return (time.monotonic_ns() - self._ultimo_frame_ns) / 1_000_000_000.0

    def obtener_frame(self, max_edad_s: float = 1.0) -> np.ndarray | None:
        if self._ultimo_frame is None or self._ultimo_frame_ns is None:
            return None
        edad = self.edad_frame_s()
        if edad is None or edad > max_edad_s:
            return None
        return self._ultimo_frame.copy()
