"""Detector visual configurable para identificar la llegada al Punto B."""

from dataclasses import dataclass
import cv2
import numpy as np


@dataclass(frozen=True)
class DeteccionMetaB:
    detectado: bool
    modo: str
    motivo: str
    desplazamiento_x: float = 0.0  # [-1.0 izq, +1.0 der] respecto al centro de la imagen
    ancho_relativo: float = 0.0


class DetectorMetaB:
    """Detecta el objetivo B por rostro, marcador ArUco o parche de color HSV."""

    def __init__(self, config_meta: dict, extractor_facial=None):
        self.modo = str(config_meta.get("modo", "rostro")).lower()
        self.confirmaciones_requeridas = int(config_meta.get("confirmaciones_consecutivas", 2))
        self.min_ancho_rostro_px = int(config_meta.get("min_ancho_rostro_px", 45))
        self.hsv_min = np.array(config_meta.get("hsv_min", [35, 80, 60]), dtype=np.uint8)
        self.hsv_max = np.array(config_meta.get("hsv_max", [85, 255, 255]), dtype=np.uint8)
        self.min_area_color_px = int(config_meta.get("min_area_color_px", 1800))
        self.aruco_id = int(config_meta.get("aruco_id", 0))
        self.extractor_facial = extractor_facial
        self._conteo_consecutivo = 0
        self._aruco_detector = self._crear_detector_aruco(
            str(config_meta.get("aruco_dict", "DICT_4X4_50"))
        )

    @staticmethod
    def _crear_detector_aruco(nombre_dict: str):
        if not hasattr(cv2, "aruco"):
            return None
        dic_id = getattr(cv2.aruco, nombre_dict, cv2.aruco.DICT_4X4_50)
        diccionario = cv2.aruco.getPredefinedDictionary(dic_id)
        if hasattr(cv2.aruco, "ArucoDetector"):
            parametros = cv2.aruco.DetectorParameters()
            return cv2.aruco.ArucoDetector(diccionario, parametros)
        return diccionario

    def reiniciar(self):
        self._conteo_consecutivo = 0

    def evaluar_frame(self, frame: np.ndarray | None) -> DeteccionMetaB:
        """Evalúa si el frame actual muestra de forma estable el objetivo B."""
        if self.modo == "coordenada":
            return DeteccionMetaB(False, self.modo, "Modo por coordenadas fijas")

        if frame is None or frame.ndim != 3 or frame.size == 0:
            self._conteo_consecutivo = 0
            return DeteccionMetaB(False, self.modo, "Sin frame valido")

        crudo = self._evaluar_crudo(frame)
        if crudo.detectado:
            self._conteo_consecutivo += 1
        else:
            self._conteo_consecutivo = 0

        confirmado = self._conteo_consecutivo >= max(1, self.confirmaciones_requeridas)
        return DeteccionMetaB(
            detectado=confirmado,
            modo=self.modo,
            motivo=crudo.motivo if not confirmado else f"Confirmado ({self._conteo_consecutivo})",
            desplazamiento_x=crudo.desplazamiento_x,
            ancho_relativo=crudo.ancho_relativo,
        )

    def _evaluar_crudo(self, frame: np.ndarray) -> DeteccionMetaB:
        alto, ancho = frame.shape[:2]

        if self.modo == "rostro":
            if self.extractor_facial is None:
                return DeteccionMetaB(False, self.modo, "Sin extractor facial")
            analisis = self.extractor_facial.analizar_imagen(frame)
            if analisis.estado != "OK" or analisis.caja is None:
                return DeteccionMetaB(False, self.modo, f"Rostro: {analisis.estado}")
            cx, _, cw, _ = analisis.caja
            if cw < self.min_ancho_rostro_px:
                return DeteccionMetaB(False, self.modo, f"Rostro muy lejano ({cw:.0f}px)")
            centro_x = cx + cw * 0.5
            desplazamiento = (centro_x - ancho * 0.5) / max(ancho * 0.5, 1.0)
            return DeteccionMetaB(True, self.modo, "Rostro detectado", float(desplazamiento), float(cw / ancho))

        if self.modo == "aruco":
            if self._aruco_detector is None:
                return DeteccionMetaB(False, self.modo, "OpenCV sin modulo aruco")
            gris = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            if hasattr(self._aruco_detector, "detectMarkers"):
                esquinas, ids, _ = self._aruco_detector.detectMarkers(gris)
            else:
                esquinas, ids, _ = cv2.aruco.detectMarkers(gris, self._aruco_detector)
            if ids is None:
                return DeteccionMetaB(False, self.modo, "Sin marcador ArUco")
            for pts, mid in zip(esquinas, ids.flatten()):
                if int(mid) == self.aruco_id:
                    xs = pts[0, :, 0]
                    centro_x = float(np.mean(xs))
                    cw = float(np.max(xs) - np.min(xs))
                    desplazamiento = (centro_x - ancho * 0.5) / max(ancho * 0.5, 1.0)
                    return DeteccionMetaB(True, self.modo, f"ArUco {mid}", desplazamiento, cw / ancho)
            return DeteccionMetaB(False, self.modo, "ArUco distinto al objetivo")

        if self.modo == "color":
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mascara = cv2.inRange(hsv, self.hsv_min, self.hsv_max)
            mascara = cv2.morphologyEx(mascara, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
            contornos, _ = cv2.findContours(mascara, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not contornos:
                return DeteccionMetaB(False, self.modo, "Sin marcador de color")
            mayor = max(contornos, key=cv2.contourArea)
            area = cv2.contourArea(mayor)
            if area < self.min_area_color_px:
                return DeteccionMetaB(False, self.modo, f"Area color insuficiente ({area:.0f})")
            x, _, w, _ = cv2.boundingRect(mayor)
            desplazamiento = ((x + w * 0.5) - ancho * 0.5) / max(ancho * 0.5, 1.0)
            return DeteccionMetaB(True, self.modo, f"Color detectado ({area:.0f}px)", float(desplazamiento), float(w / ancho))

        return DeteccionMetaB(False, self.modo, f"Modo desconocido: {self.modo}")
