"""Holgura circular para giro sobre el sitio, con escaneo completo y TF plano.

El radio incluye la parte más alejada del robot. Los puntos se trasladan desde
el LiDAR al centro base_link; +inf no se convierte en espacio libre.
"""

from dataclasses import dataclass
from math import atan2, cos, hypot, isfinite, isnan, pi, radians, sin, sqrt

from evasion_lidar import CoberturaSector


@dataclass(frozen=True)
class HolguraGiro:
    valida: bool
    distancia_minima_m: float | None
    motivo: str
    edad_s: float | None = None
    sectores: tuple[CoberturaSector, ...] = ()
    distancia_avance_m: float | None = None
    puntos_base: tuple[tuple[float, float], ...] = ()


def distancia_libre_recta(holgura, radio, radio_frente=None):
    """Recorrido de un perfil frontal hasta el primer punto observado en base_link.

    Sin `radio_frente`, conserva un disco completo de radio `radio` alrededor del
    centro. Con `radio_frente`, usa `radio` como semiancho lateral del cuerpo y
    `radio_frente` como alcance frontal mínimo al avanzar recto entre paredes.
    """
    if (holgura is None or not holgura.valida or not holgura.puntos_base
            or not isfinite(radio) or radio <= 0):
        return None
    alcance = radio if radio_frente is None else radio_frente
    if not isfinite(alcance) or alcance <= 0:
        return None
    libres = []
    for x, y in holgura.puntos_base:
        if radio_frente is None and hypot(x, y) <= radio:
            return 0.
        if x >= 0 and abs(y) <= radio:
            dx = alcance * sqrt(max(0., 1. - (y / radio) ** 2))
            libres.append(max(0., x - dx))
    return min(libres) if libres else None


def distancia_libre_atras(holgura, radio):
    """Recorrido libre hacia -x ignorando puntos delante del centro para destrabar esquinas."""
    if (holgura is None or not holgura.valida or not holgura.puntos_base
            or not isfinite(radio) or radio <= 0):
        return None
    libres = []
    for x, y in holgura.puntos_base:
        if x <= 0.05 and abs(y) <= radio:
            libres.append(max(0., -x-sqrt(max(0., radio*radio-y*y))))
    return min(libres) if libres else None


def transformacion_plana(traslacion, rotacion):
    """TF LiDAR -> base_link; rechazar inclinación >5° y datos no finitos."""
    valores = (traslacion.x, traslacion.y, traslacion.z,
               rotacion.x, rotacion.y, rotacion.z, rotacion.w)
    if not all(isfinite(v) for v in valores):
        return None
    norma = sqrt(sum(v*v for v in valores[3:]))
    if norma < 1e-12:
        return None
    x, y, z, w = (v/norma for v in valores[3:])
    if 1-2*(x*x+y*y) < cos(radians(5)):
        return None
    yaw = atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
    return traslacion.x, traslacion.y, yaw


def analizar_holgura(scan, tf_plano, edad_s, cfg):
    """Mínimo radial en base_link, con cobertura en cuatro cuadrantes del scan."""
    def desconocida(motivo, sectores=()):
        return HolguraGiro(False, None, motivo, edad_s, sectores)

    if (scan is None or edad_s is None or not isfinite(edad_s)
            or not 0 <= edad_s <= cfg.edad_maxima_s):
        return desconocida('Sin LiDAR reciente para holgura')
    if (tf_plano is None or len(tf_plano) != 3
            or not all(isfinite(v) for v in tf_plano)):
        return desconocida('Sin TF plano completo LiDAR -> base_link')
    n, paso = len(scan.ranges), scan.angle_increment
    if (not all(isfinite(v) for v in (scan.angle_min,paso,scan.range_min,scan.range_max))
            or not 0 <= scan.range_min < scan.range_max or not 0 < abs(paso) <= radians(10)
            or n < 36 or abs(n*abs(paso)-2*pi) > 2*abs(paso)):
        return desconocida('Se requiere escaneo completo de 360 grados para girar')
    tx, ty, yaw = tf_plano
    total, utiles, infs, nans, fuera = ([0]*4 for _ in range(5))
    distancias = []
    distancias_avance = []
    puntos_base = []
    proximidad = False
    cono_avance = radians(105)
    for i, r in enumerate(scan.ranges):
        ang = scan.angle_min+i*paso+yaw
        j = int(((ang+pi/4) % (2*pi))/(pi/2)) % 4
        total[j] += 1
        if isfinite(r) and 0 <= r <= scan.range_max:
            utiles[j] += 1
            if r < scan.range_min or r == 0:
                proximidad = True
            else:
                px, py = tx+r*cos(ang), ty+r*sin(ang)
                puntos_base.append((px, py))
                dist = hypot(px, py)
                distancias.append(dist)
                if abs(atan2(py, px)) <= cono_avance:
                    distancias_avance.append(dist)
        elif r == float('inf'):
            infs[j] += 1
        elif isnan(r):
            nans[j] += 1
        else:
            fuera[j] += 1
            proximidad |= r == float('-inf')
    sectores = tuple(CoberturaSector(nombre,total[j],utiles[j],infs[j],nans[j],fuera[j],
                                     cfg.cobertura_minima,cfg.fraccion_errores_maxima)
                     for j,nombre in enumerate(('frente','izquierda','detras','derecha')))
    if proximidad:
        return desconocida('Proximidad no cuantificable o bajo rango en el contorno',sectores)
    insuficientes = [s.nombre for s in sectores if not s.suficiente]
    if insuficientes:
        return desconocida('Cobertura de giro insuficiente: '+', '.join(insuficientes),sectores)
    dist_avance = min(distancias_avance) if distancias_avance else min(distancias)
    return HolguraGiro(True,min(distancias),'Contorno observado en base_link',edad_s,sectores,
                       dist_avance,tuple(puntos_base))
