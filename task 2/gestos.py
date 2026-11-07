"""Reglas de gestos de Task 2 (Task B), sin cámara, MediaPipe ni ROS.

Los puntos se reciben normalizados como (x, y), en el orden de MediaPipe.
Se convierten a píxeles para no deformar ángulos en imágenes rectangulares.
Esta versión reconoce una mano aproximadamente de frente a la cámara.
"""

from dataclasses import dataclass, field
from enum import Enum
from math import acos, degrees, dist, isfinite
from collections.abc import Sequence


class Orden(str, Enum):
    DETENER = "DETENER"
    AVANZAR = "AVANZAR"
    IZQUIERDA = "IZQUIERDA"
    DERECHA = "DERECHA"


@dataclass(frozen=True)
class Gesto:
    orden: Orden
    motivo: str


def _angulo(a, b, c):
    """Ángulo ABC en grados; un segmento degenerado devuelve cero."""
    u = (a[0] - b[0], a[1] - b[1])
    v = (c[0] - b[0], c[1] - b[1])
    norma = dist(a, b) * dist(c, b)
    if norma < 1e-9:
        return 0.0
    coseno = max(-1.0, min(1.0, (u[0] * v[0] + u[1] * v[1]) / norma))
    return degrees(acos(coseno))


def interpretar_mano(
    puntos: Sequence[Sequence[float]] | None,
    ancho: int,
    alto: int,
) -> Gesto:
    """Interpreta una mano; ausencia, orientación dudosa o datos inválidos paran.

    - Palma abierta: los cuatro dedos largos extendidos -> DETENER.
    - Seña de la paz: índice y medio extendidos -> AVANZAR.
    - Solo índice: apuntar a los lados determina IZQUIERDA o DERECHA.
    El pulgar queda libre en esta convención.
    """
    if puntos is None:
        return Gesto(Orden.DETENER, "Sin mano")
    if ancho <= 0 or alto <= 0 or len(puntos) != 21:
        return Gesto(Orden.DETENER, "Datos de mano invalidos")
    if any(len(p) < 2 or not all(isfinite(v) for v in p[:2]) for p in puntos):
        return Gesto(Orden.DETENER, "Coordenadas invalidas")
    pixeles = [(p[0] * ancho, p[1] * alto) for p in puntos]
    muneca = pixeles[0]
    escala_palma = dist(muneca, pixeles[9])
    if escala_palma < 1e-6:
        return Gesto(Orden.DETENER, "Palma degenerada")

    estados = []
    for base in (5, 9, 13, 17):
        mcp, pip, dip, tip = pixeles[base : base + 4]
        if min(dist(mcp, pip), dist(pip, dip), dist(dip, tip)) < 0.01 * escala_palma:
            return Gesto(Orden.DETENER, "Articulaciones degeneradas")
        angulo_pip = _angulo(mcp, pip, dip)
        angulo_dip = _angulo(pip, dip, tip)
        radio_pip = dist(muneca, pip)
        radio_tip = dist(muneca, tip)
        if angulo_pip >= 155 and angulo_dip >= 150 and radio_tip > 1.1 * radio_pip:
            estados.append("extendido")
        elif angulo_pip <= 130 and radio_tip < 1.05 * radio_pip:
            estados.append("flexionado")
        else:
            estados.append("ambiguo")

    # 1. Palma completa abierta -> DETENER
    if all(estado == "extendido" for estado in estados):
        return Gesto(Orden.DETENER, "Palma abierta")

    # 2. SEÑA DE LA PAZ (Índice y Medio extendidos) -> AVANZAR
    if estados == ["extendido", "extendido", "flexionado", "flexionado"]:
        # Opcional: verificar que ambos dedos apunten razonablemente hacia arriba
        dy_indice = pixeles[8][1] - pixeles[5][1]
        dy_medio = pixeles[12][1] - pixeles[9][1]
        if dy_indice < 0 and dy_medio < 0:
            return Gesto(Orden.AVANZAR, "Gesto de paz (Avanzar)")
        return Gesto(Orden.DETENER, "Gesto de paz inclinado")

    # 3. SOLO ÍNDICE EXTENDIDO -> GIROS (IZQUIERDA / DERECHA)
    if estados == ["extendido", "flexionado", "flexionado", "flexionado"]:
        dx = pixeles[8][0] - pixeles[5][0]
        dy = pixeles[8][1] - pixeles[5][1]
        
        # Evaluar orientación horizontal con umbral estricto para evitar confusión con arriba
        if -dx >= 1.5 * abs(dy):
            return Gesto(Orden.DERECHA, "Indice hacia tu derecha")
        if dx >= 1.5 * abs(dy):
            return Gesto(Orden.IZQUIERDA, "Indice hacia tu izquierda")
        
        return Gesto(Orden.DETENER, "Indice solo apunta arriba (usa paz para avanzar)")

    return Gesto(Orden.DETENER, "Gesto no definido o dedos ambiguos")


@dataclass
class FiltroOrden:
    """Movimiento tras un gesto sostenido; detenerse tiene efecto inmediato."""

    estabilidad_s: float = 0.4
    max_intervalo_s: float = 0.5
    _candidata: Orden = field(default=Orden.DETENER, init=False)
    _inicio: float | None = field(default=None, init=False)
    _ultima_muestra: float | None = field(default=None, init=False)
    _salida: Orden = field(default=Orden.DETENER, init=False)

    def __post_init__(self):
        if not (isfinite(self.estabilidad_s) and self.estabilidad_s > 0):
            raise ValueError("estabilidad_s debe ser positiva y finita")
        if not (isfinite(self.max_intervalo_s) and self.max_intervalo_s > 0):
            raise ValueError("max_intervalo_s debe ser positivo y finito")

    def detener(self) -> Orden:
        self._candidata = Orden.DETENER
        self._inicio = None
        self._ultima_muestra = None
        self._salida = Orden.DETENER
        return self._salida

    def actualizar(self, orden: Orden, ahora: float) -> Orden:
        if not isfinite(ahora) or not isinstance(orden, Orden):
            return self.detener()
        if self._ultima_muestra is not None:
            intervalo = ahora - self._ultima_muestra
            if intervalo < 0 or intervalo > self.max_intervalo_s:
                self.detener()
        self._ultima_muestra = ahora
        if orden is Orden.DETENER:
            return self.detener()
        if orden is not self._candidata:
            self._candidata = orden
            self._inicio = ahora
            self._salida = Orden.DETENER
        elif self._inicio is not None and ahora - self._inicio >= self.estabilidad_s:
            self._salida = orden
        return self._salida

    def vigente(self, ahora: float) -> Orden:
        """Consultar la orden sin una muestra nueva: una orden vieja caduca."""
        if not isfinite(ahora) or self._ultima_muestra is None:
            return self.detener()
        edad = ahora - self._ultima_muestra
        if edad < 0 or edad > self.max_intervalo_s:
            return self.detener()
        return self._salida