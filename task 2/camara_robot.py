"""Vídeo ROS por un canal separado de las órdenes; GUI solo en el host.

El protocolo usa el reloj monotónico compartido por esta laptop y su Docker.
No guarda imágenes ni requiere OpenCV dentro del contenedor.
"""

import base64
import json
import subprocess
import threading
import time


TOPICO_CAMARA = "/oakd/rgb/preview/image_raw"
MAX_DATOS = 8 * 1024 * 1024
MAX_LINEA_VIDEO = ((MAX_DATOS + 2) // 3) * 4 + 4096
CADUCIDAD_VIDEO_NS = 1_000_000_000
CANALES = {"bgr8": 3, "rgb8": 3, "mono8": 1, "rgba8": 4, "bgra8": 4}


def codificar_imagen(mensaje, comprimida, recibido_ns):
    if not 0 < len(mensaje.data) <= MAX_DATOS:
        raise ValueError("Imagen vacia o demasiado grande")
    paquete = {"version": 1, "recibido_ns": recibido_ns,
               "encoding": "compressed" if comprimida else mensaje.encoding.lower(),
               "datos": base64.b64encode(bytes(mensaje.data)).decode("ascii")}
    if not comprimida:
        paquete.update(ancho=int(mensaje.width), alto=int(mensaje.height),
                       paso=int(mensaje.step))
    return (json.dumps(paquete, separators=(",", ":")) + "\n").encode("ascii")


def leer_imagen(linea):
    if len(linea) > MAX_LINEA_VIDEO:
        raise ValueError("Registro de video demasiado grande")
    p = json.loads(linea)
    if not isinstance(p, dict) or p.get("version") != 1 or type(p.get("recibido_ns")) is not int or p["recibido_ns"] < 0:
        raise ValueError("Registro de video invalido")
    p["datos"] = base64.b64decode(p["datos"], validate=True)
    if not 0 < len(p["datos"]) <= MAX_DATOS:
        raise ValueError("Datos de imagen invalidos")
    if p["encoding"] != "compressed":
        canales = CANALES.get(p["encoding"])
        if canales is None:
            raise ValueError("Formato de imagen no compatible")
        if any(type(p.get(k)) is not int or p[k] <= 0 for k in ("ancho", "alto", "paso")):
            raise ValueError("Dimensiones invalidas")
        if p["paso"] < p["ancho"] * canales or len(p["datos"]) != p["alto"] * p["paso"]:
            raise ValueError("Imagen truncada o paso de fila invalido")
    return p


def decodificar_imagen(p):
    import cv2
    import numpy as np

    datos = np.frombuffer(p["datos"], dtype=np.uint8)
    if p["encoding"] == "compressed":
        try:
            imagen = cv2.imdecode(datos, cv2.IMREAD_COLOR)
        except cv2.error as exc:
            raise ValueError("Imagen comprimida invalida") from exc
        if imagen is None:
            raise ValueError("No se pudo decodificar la imagen comprimida")
        return imagen
    canales = CANALES[p["encoding"]]
    # Image.step puede incluir relleno al final de cada fila.
    imagen = datos.reshape(p["alto"], p["paso"])[:, :p["ancho"] * canales]
    imagen = imagen.reshape(p["alto"], p["ancho"], canales)
    conversion = {"rgb8": cv2.COLOR_RGB2BGR, "rgba8": cv2.COLOR_RGBA2BGR,
                  "bgra8": cv2.COLOR_BGRA2BGR, "mono8": cv2.COLOR_GRAY2BGR}
    return cv2.cvtColor(imagen, conversion[p["encoding"]]) if p["encoding"] in conversion else imagen.copy()


def comando_camara(ip, dominio, contenedor, topico):
    return ["sudo", "-n", "docker", "exec", "-i",
            "-e", f"ROS_DOMAIN_ID={dominio}", "-e", "RMW_IMPLEMENTATION=rmw_fastrtps_cpp",
            "-e", "ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET", "-e", f"ROS_STATIC_PEERS={ip}",
            "-e", "RCUTILS_LOGGING_USE_STDOUT=0", contenedor, "bash", "--noprofile", "--norc",
            "-c", 'unset ROS_LOCALHOST_ONLY; source /opt/ros/jazzy/setup.bash && '
            'exec python3 -u /tmp/proyecto2-gestos/camara_robot_ros.py "$@"',
            "camara-robot", "--topico", topico]


class VideoRobot:
    def __init__(self, proceso):
        self.proceso = proceso
        self._lock = threading.Lock()
        self._ultimo = None
        self._error = ""
        self._cerrado = False
        self._imagen = None
        self._decodificado = None
        self._hilo = threading.Thread(target=self._leer, daemon=True)
        self._hilo.start()

    @classmethod
    def iniciar(cls, ip, dominio, contenedor, topico=TOPICO_CAMARA):
        proceso = subprocess.Popen(comando_camara(ip, dominio, contenedor, topico),
                                   stdin=subprocess.PIPE, stdout=subprocess.PIPE)
        return cls(proceso)

    def _leer(self):
        try:
            while not self._cerrado:
                linea = self.proceso.stdout.readline(MAX_LINEA_VIDEO + 1)
                if not linea:
                    break
                paquete = leer_imagen(linea)
                with self._lock:
                    self._ultimo = paquete
        except (ValueError, KeyError, TypeError, OSError) as exc:
            with self._lock:
                self._error = f"Video no disponible: {exc}"
        finally:
            with self._lock:
                self._error = self._error or "Camara desconectada"
            # También cierra el receptor ROS si el registro era inválido.
            try:
                self.proceso.stdin.close()
            except OSError:
                pass

    def fotograma(self, ahora_ns=None):
        ahora_ns = time.monotonic_ns() if ahora_ns is None else ahora_ns
        with self._lock:
            p, error = self._ultimo, self._error
        if error:
            return None, error
        if p is None:
            return None, "Esperando camara del robot..."
        if not 0 <= ahora_ns - p["recibido_ns"] <= CADUCIDAD_VIDEO_NS:
            return None, "Sin imagen reciente"
        if p is not self._decodificado:
            try:
                self._imagen = decodificar_imagen(p)
                self._decodificado = p
            except (ValueError, RuntimeError) as exc:
                return None, f"Imagen no compatible: {exc}"
        return self._imagen, "Camara del robot"

    def cerrar(self):
        self._cerrado = True
        try:
            self.proceso.stdin.close()  # EOF hace salir al nodo ROS incluso sin imágenes.
        except OSError:
            pass
        try:
            self.proceso.wait(timeout=2)
        except subprocess.TimeoutExpired:
            self.proceso.terminate()
            try:
                self.proceso.wait(timeout=1)
            except subprocess.TimeoutExpired:
                self.proceso.kill()
                self.proceso.wait(timeout=1)
        self._hilo.join(timeout=0.5)
        if not self._hilo.is_alive():
            self.proceso.stdout.close()


def panel_lateral(webcam, imagen, estado):
    """Panel derecho sin espejo, conservando la proporción y sin congelar vídeo viejo."""
    import cv2
    import numpy as np

    alto, ancho = webcam.shape[:2]
    panel = np.full((alto, ancho, 3), 30, dtype=np.uint8)
    if imagen is not None:
        h, w = imagen.shape[:2]
        escala = min(ancho / w, (alto - 40) / h)
        w, h = max(1, round(w * escala)), max(1, round(h * escala))
        x, y = (ancho - w) // 2, 40 + (alto - 40 - h) // 2
        panel[y:y+h, x:x+w] = cv2.resize(imagen, (w, h))
    else:
        cv2.putText(panel, estado[:65], (15, alto // 2), cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, (0, 180, 255), 1)
    cv2.putText(panel, "CAMARA DEL ROBOT", (12, 26), cv2.FONT_HERSHEY_SIMPLEX,
                0.65, (240, 240, 240), 1)
    return np.hstack((webcam, panel))
