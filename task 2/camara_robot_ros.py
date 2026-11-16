"""Suscriptor de cámara en Docker: solo ROS y biblioteca estándar, sin GUI.

Salida de imágenes separada del canal de velocidad; nunca publica /cmd_vel.
EOF de stdin termina el nodo, aunque la cámara no produzca imágenes.
"""

import argparse
import os
import sys
import threading
import time

from camara_robot import TOPICO_CAMARA, codificar_imagen


def vigilar_cierre(flujo, fin):
    while flujo.read(1):
        pass
    fin.set()


def ejecutar(topico):
    os.environ["RCUTILS_LOGGING_USE_STDOUT"] = "0"
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from sensor_msgs.msg import Image, CompressedImage

    fin = threading.Event()
    threading.Thread(target=vigilar_cierre, args=(sys.stdin.buffer, fin), daemon=True).start()

    class Camara(Node):
        def __init__(self):
            super().__init__("video_gestos_robot", parameter_overrides=[
                Parameter("use_sim_time", value=False)])
            self.suscripcion = None
            self.comprimida = False
            self.ultimo_envio = None
            self.create_timer(1.0, self.descubrir)
            print(f"[VIDEO] Esperando {topico}", file=sys.stderr, flush=True)

        def descubrir(self):
            if self.suscripcion is not None:
                return
            tipos = dict(self.get_topic_names_and_types()).get(topico, [])
            if tipos == ["sensor_msgs/msg/Image"]:
                tipo = Image
            elif tipos == ["sensor_msgs/msg/CompressedImage"]:
                tipo, self.comprimida = CompressedImage, True
            else:
                return
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE)
            self.suscripcion = self.create_subscription(tipo, topico, self.recibir, qos)
            print(f"[VIDEO] Recibiendo {topico}", file=sys.stderr, flush=True)

        def recibir(self, mensaje):
            ahora = time.monotonic_ns()
            if self.ultimo_envio is not None and ahora - self.ultimo_envio < 200_000_000:
                return  # Máximo 5 FPS hacia la GUI; no cambia la cámara del robot.
            self.ultimo_envio = ahora
            try:
                registro = codificar_imagen(mensaje, self.comprimida, ahora)
                sys.stdout.buffer.write(registro)
                sys.stdout.buffer.flush()
            except ValueError as exc:
                print(f"[VIDEO] {exc}", file=sys.stderr, flush=True)
            except (BrokenPipeError, OSError):
                fin.set()

    rclpy.init()
    nodo = None
    try:
        nodo = Camara()
        while rclpy.ok() and not fin.is_set():
            rclpy.spin_once(nodo, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if nodo is not None:
            nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topico", default=TOPICO_CAMARA)
    ejecutar(parser.parse_args().topico)
