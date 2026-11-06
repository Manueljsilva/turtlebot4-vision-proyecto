"""Órdenes y caducidad del enlace webcam–Docker, sin ROS ni MediaPipe.

El reloj monotónico se comparte entre host y contenedor en esta máquina.
No usar este protocolo entre computadoras distintas sin adaptar los relojes.
"""

from dataclasses import dataclass, field
import json
from math import isfinite

from gestos import Orden


MAX_LINEA = 1024
CADUCIDAD_S = 0.5
VELOCIDADES = {
    Orden.DETENER: (0.0, 0.0),
    Orden.AVANZAR: (0.306, 0.0),
    Orden.IZQUIERDA: (0.0, 1.90),
    Orden.DERECHA: (0.0, -1.90),
}


def velocidades(orden: Orden) -> tuple[float, float]:
    """Las órdenes de giro ya están expresadas respecto al robot."""
    return VELOCIDADES.get(orden, (0.0, 0.0))


def validar_velocidades_robot():
    """Comprueba la tabla antes de usar hardware con las protecciones activas."""
    if set(VELOCIDADES) != set(Orden) or VELOCIDADES[Orden.DETENER] != (0.0, 0.0):
        raise ValueError("La tabla debe contener las cuatro ordenes y DETENER debe ser cero")
    for orden, (lineal, angular) in VELOCIDADES.items():
        if (not isfinite(lineal) or not isfinite(angular) or
                abs(lineal) > 0.306 or abs(angular) > 1.90):
            raise ValueError(f"Velocidad de {orden.value} fuera de los limites 0.306 m/s, 1.90 rad/s")


def codificar(orden: Orden, captura_ns: int, secuencia: int) -> str:
    return json.dumps({
        "version": 1,
        "orden": orden.value,
        "captura_ns": captura_ns,
        "secuencia": secuencia,
    }, separators=(",", ":"))


@dataclass
class VigilanteOrden:
    """Solo mantiene órdenes recientes; una línea incorrecta también detiene."""

    caducidad_s: float = CADUCIDAD_S
    orden: Orden = field(default=Orden.DETENER, init=False)
    motivo: str = field(default="Esperando orden de webcam", init=False)
    _captura_ns: int | None = field(default=None, init=False)
    _ultima_secuencia: int = field(default=-1, init=False)

    def __post_init__(self):
        if not isfinite(self.caducidad_s) or self.caducidad_s <= 0:
            raise ValueError("La caducidad debe ser positiva y finita")

    def detener(self, motivo: str):
        self.orden = Orden.DETENER
        self._captura_ns = None
        self.motivo = motivo

    def recibir(self, linea: str, ahora_ns: int):
        try:
            if len(linea) > MAX_LINEA:
                raise ValueError("Linea demasiado larga")
            datos = json.loads(linea)
            if not isinstance(datos, dict) or set(datos) != {
                "version", "orden", "captura_ns", "secuencia"
            }:
                raise ValueError("Formato incorrecto")
            if type(datos["version"]) is not int or datos["version"] != 1:
                raise ValueError("Version incorrecta")
            captura = datos["captura_ns"]
            secuencia = datos["secuencia"]
            if (type(captura) is not int or captura < 0 or
                    type(secuencia) is not int or secuencia < 0):
                raise ValueError("Tiempo o secuencia invalidos")
            orden = Orden(datos["orden"])
            if secuencia <= self._ultima_secuencia:
                raise ValueError("Orden repetida o fuera de secuencia")
            self._ultima_secuencia = secuencia
            edad = ahora_ns - captura
            if edad < 0 or edad > self.caducidad_s * 1_000_000_000:
                self.detener("Orden recibida fuera de tiempo")
                return
        except (ValueError, TypeError, KeyError, OverflowError):
            self.detener("Mensaje de webcam invalido")
            return
        self.orden = orden
        self._captura_ns = captura
        self.motivo = "Orden reciente de webcam"

    def vigente(self, ahora_ns: int) -> Orden:
        if self._captura_ns is not None:
            edad = ahora_ns - self._captura_ns
            if edad < 0 or edad > self.caducidad_s * 1_000_000_000:
                self.detener("Webcam sin actualizar: orden caducada")
        return self.orden


@dataclass
class RelojSimulado:
    """Impide movimiento sin /clock o con simulación pausada/reset reciente."""

    max_pausa_s: float = CADUCIDAD_S
    _ultimo_ros_ns: int = field(default=0, init=False)
    _ultimo_avance_ns: int | None = field(default=None, init=False)

    def disponible(self, ros_ns: int, monotono_ns: int) -> bool:
        if ros_ns <= 0 or ros_ns < self._ultimo_ros_ns:
            self._ultimo_ros_ns = ros_ns
            self._ultimo_avance_ns = None
            return False
        if ros_ns > self._ultimo_ros_ns:
            self._ultimo_avance_ns = monotono_ns
        self._ultimo_ros_ns = ros_ns
        if self._ultimo_avance_ns is None:
            return False
        edad = monotono_ns - self._ultimo_avance_ns
        return 0 <= edad <= self.max_pausa_s * 1_000_000_000
