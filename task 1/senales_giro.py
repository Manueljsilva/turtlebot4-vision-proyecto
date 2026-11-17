"""Detección de giros azules/rojos a partir de imágenes BGR, sin ROS ni cámara.

Reutiliza la geometría aportada por la compañera en geometria_senales.py.
No interpreta discos sin flecha, STOP, meta ni QR. No refleja la imagen.
"""

import math
import cv2
import numpy as np

import geometria_senales as geo
from control_giros import COLORES_TRAMO, ConfigSenales


class DetectorGiros:
    def __init__(self, cfg=None):
        self.cfg = cfg or ConfigSenales()
        self.circulos = geo.Detector(diameter=self.cfg.diametro_minimo_px, max_objects=6)

    def detectar(self, imagen):
        if (not isinstance(imagen, np.ndarray) or imagen.dtype != np.uint8
                or imagen.ndim != 3 or imagen.shape[2] != 3 or min(imagen.shape[:2]) < 20):
            raise ValueError('Se requiere imagen BGR uint8')
        escala = min(1., self.cfg.ancho / imagen.shape[1])
        frame = (cv2.resize(imagen, None, fx=escala, fy=escala, interpolation=cv2.INTER_AREA)
                 if escala < 1 else imagen)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        canal = cv2.max(gray, cv2.max(frame[:, :, 0], frame[:, :, 2]))
        bordes_circulo = cv2.Canny(cv2.GaussianBlur(canal, (5, 5), 1.05), 65, 150)
        circulos = self.circulos.detect(frame, external_edges=bordes_circulo)
        bordes = None
        recuperacion = {}
        resultados = []
        for obj in circulos:
            elipse = obj['ellipse']
            (cx, cy), (w, h), _ = elipse
            if cx-w/2 < 1 or cy-h/2 < 1 or cx+w/2 >= frame.shape[1]-1 or cy+h/2 >= frame.shape[0]-1:
                continue  # Un cartel cortado no autoriza una maniobra.
            for color in ('azul', 'rojo'):
                modo, fracciones = geo.classify_background(frame, elipse, color)
                if modo != 'FLECHA':
                    continue
                if bordes is None:
                    bordes, _ = geo.canny_pipeline(frame, gray=gray)
                _, _, flecha, _, _, _ = geo.detect_robust(
                    frame, bordes, elipse, recuperacion, gray=gray, mode='FLECHA')
                if flecha is not None and flecha['direction'] in ('IZQUIERDA', 'DERECHA'):
                    resultados.append(dict(color=color, direccion=flecha['direction'],
                        confianza=float(flecha['confidence']),
                        caja=[int((cx-w/2)/escala), int((cy-h/2)/escala),
                              int(w/escala), int(h/escala)]))
        return resultados


class ConsultaSenales:
    """Tres paquetes nuevos de un único cartel del tramo; sin votos predictivos."""

    def __init__(self, tramo='ida', cfg=None, detector=None):
        self.cfg = cfg or ConfigSenales()
        self.tramo, self.color = tramo, COLORES_TRAMO[tramo]
        self.detector = detector or DetectorGiros(self.cfg)
        self.ultima_marca = None
        self.candidato = None
        self.cuenta = 0
        self.id = 0
        self.armado = True
        self.ausencia_ns = None
        self.ultimo = None

    def actualizar(self, imagen, marca_ns, ahora_ns):
        if imagen is None or type(marca_ns) is not int or not 0 <= ahora_ns-marca_ns <= 1_000_000_000:
            self.candidato, self.cuenta, self.ausencia_ns = None, 0, None
            self.ultimo = None
            return dict(detecciones=[], evento=None, estado='Esperando camara reciente', confirmaciones=0)
        if self.ultima_marca is not None and marca_ns <= self.ultima_marca:
            return self.ultimo or dict(detecciones=[], evento=None, estado='Esperando imagen nueva', confirmaciones=0)
        if self.ultima_marca is not None and marca_ns-self.ultima_marca > 1_000_000_000:
            self.candidato, self.cuenta, self.ausencia_ns = None, 0, None
        self.ultima_marca = marca_ns
        detecciones = self.detector.detectar(imagen)
        candidatas = [d for d in detecciones if d['color'] == self.color]
        evento = None
        estado = f'Sin giro {self.color}; otros colores se ignoran'
        if len(candidatas) == 1:
            d = candidatas[0]
            if self.candidato is not None:
                a, b = self.candidato['caja'], d['caja']
                cerca = math.hypot((a[0]+a[2]/2)-(b[0]+b[2]/2), (a[1]+a[3]/2)-(b[1]+b[3]/2)) < .6*max(a[2:]+b[2:])
                cerca &= min(a[2], b[2])/max(a[2], b[2]) >= .6
            else:
                cerca = False
            if not cerca or self.candidato['direccion'] != d['direccion']:
                self.cuenta = 0
            self.candidato, self.ausencia_ns = d, None
            self.cuenta = min(self.cfg.confirmaciones, self.cuenta + 1)
            estado = f"{d['direccion']} {self.color}: {self.cuenta}/{self.cfg.confirmaciones}"
            if self.cuenta == self.cfg.confirmaciones:
                if self.armado:
                    self.id += 1
                    self.armado = False
                evento = dict(id=self.id, direccion=d['direccion'], color=self.color, marca_ns=marca_ns)
                estado += ' confirmada'
        else:
            self.candidato, self.cuenta = None, 0
            if len(candidatas) > 1:
                self.ausencia_ns = None
                estado = 'Varias senales del tramo: ambiguo, no ordenar giro'
            else:
                if self.ausencia_ns is None:
                    self.ausencia_ns = marca_ns
                if marca_ns-self.ausencia_ns >= self.cfg.ausencia_rearme_s*1e9:
                    self.armado = True
        self.ultimo = dict(detecciones=detecciones, evento=evento, estado=estado, confirmaciones=self.cuenta)
        return self.ultimo


def dibujar_detecciones(imagen, resultado, tramo):
    out = imagen.copy()
    for d in resultado['detecciones']:
        x, y, w, h = d['caja']
        activo = d['color'] == COLORES_TRAMO[tramo]
        color = (255, 160, 0) if d['color'] == 'azul' else (0, 80, 255)
        cv2.rectangle(out, (x, y), (x+w, y+h), color if activo else (150, 150, 150), 2)
        texto = f"{d['direccion']} {d['color']}" + ('' if activo else ' (otro tramo)')
        (ancho, alto), _ = cv2.getTextSize(texto, cv2.FONT_HERSHEY_SIMPLEX, .48, 1)
        tx = max(0, min(x, out.shape[1]-ancho-8))
        ty = y-7 if y >= alto+12 else min(out.shape[0]-4, y+alto+7)
        cv2.rectangle(out, (tx, ty-alto-4), (tx+ancho+6, ty+4), (27, 27, 27), -1)
        cv2.putText(out, texto, (tx+3, ty), cv2.FONT_HERSHEY_SIMPLEX, .48, color, 1, cv2.LINE_AA)
    return out
