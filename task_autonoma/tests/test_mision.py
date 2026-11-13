import json
from pathlib import Path
import unittest
import cv2
import numpy as np

from detector_meta_b import DetectorMetaB
from mision import (
    EstadoMision,
    cargar_config_mision,
    cargar_sistema_facial,
    evaluar_frame_facial,
    votar_identidad,
)


RAIZ_TAREA = Path(__file__).resolve().parent.parent
RAIZ_PROYECTO = RAIZ_TAREA.parent


class ConfiguracionYArchivosTests(unittest.TestCase):
    def test_config_mision_valida(self):
        cfg = cargar_config_mision(RAIZ_TAREA / "config" / "mision_config.json")
        self.assertIn("camara", cfg)
        self.assertIn("exploracion", cfg)
        self.assertIn("meta_b", cfg)
        self.assertIn("reconocimiento", cfg)
        self.assertIn("evasion_lidar", cfg)
        self.assertEqual(EstadoMision.INICIO_A.value, "INICIO_A")

    def test_yamls_de_slam_y_nav2_existen_y_tienen_stamped(self):
        slam_txt = (RAIZ_TAREA / "config" / "slam_params.yaml").read_text(encoding="utf-8")
        nav2_txt = (RAIZ_TAREA / "config" / "nav2_params.yaml").read_text(encoding="utf-8")
        self.assertIn("scan_topic: /scan", slam_txt)
        self.assertIn("enable_stamped_cmd_vel: true", nav2_txt)
        self.assertIn("robot_radius: 0.18", nav2_txt)


class DetectorMetaBTests(unittest.TestCase):
    def test_detecta_parche_color_tras_confirmaciones_consecutivas(self):
        detector = DetectorMetaB({
            "modo": "color",
            "confirmaciones_consecutivas": 2,
            "hsv_min": [45, 100, 100],
            "hsv_max": [75, 255, 255],
            "min_area_color_px": 500,
        })
        frame = np.zeros((240, 320, 3), dtype=np.uint8)
        # Rectángulo verde puro BGR=(0,255,0) desplazado a la derecha
        cv2.rectangle(frame, (200, 80), (280, 160), (0, 255, 0), -1)

        d1 = detector.evaluar_frame(frame)
        self.assertFalse(d1.detectado)  # Solo 1 frame de 2
        d2 = detector.evaluar_frame(frame)
        self.assertTrue(d2.detectado)
        self.assertGreater(d2.desplazamiento_x, 0.0)

        # Frame negro reinicia el contador
        d3 = detector.evaluar_frame(np.zeros_like(frame))
        self.assertFalse(d3.detectado)

    def test_detecta_aruco_sintetico(self):
        if not hasattr(cv2, "aruco"):
            self.skipTest("OpenCV sin aruco")
        dic = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        marcador = cv2.aruco.generateImageMarker(dic, 3, 120)
        lienzo = np.full((240, 320, 3), 255, dtype=np.uint8)
        marcador_bgr = cv2.cvtColor(marcador, cv2.COLOR_GRAY2BGR)
        lienzo[60:180, 100:220] = marcador_bgr

        detector = DetectorMetaB({
            "modo": "aruco",
            "aruco_dict": "DICT_4X4_50",
            "aruco_id": 3,
            "confirmaciones_consecutivas": 1,
        })
        res = detector.evaluar_frame(lienzo)
        self.assertTrue(res.detectado)
        self.assertLess(abs(res.desplazamiento_x), 0.1)


class SistemaFacialYVotacionTests(unittest.TestCase):
    def test_carga_sistema_facial_real_y_rechaza_imagen_negra(self):
        datos_dir = RAIZ_PROYECTO / "task 1" / "datos"
        modelos_dir = RAIZ_PROYECTO / "task 1" / "modelos"
        if not (datos_dir / "galeria.npz").exists() or not (modelos_dir / "face_detection_yunet_2023mar.onnx").exists():
            self.skipTest("Faltan galeria.npz o modelos locales de task 1")

        sistema = cargar_sistema_facial(datos_dir, modelos_dir)
        negra = np.zeros((240, 320, 3), dtype=np.uint8)
        res = evaluar_frame_facial(negra, sistema, usar_clahe=False)
        self.assertFalse(res["aceptado"])
        self.assertEqual(res["nombre"], "NO_CONCLUYENTE")

    def test_votacion_mayoria_y_desempate(self):
        decisiones = [
            {"aceptado": True, "nombre": "Sofia_Salazar", "similitud": 0.72},
            {"aceptado": True, "nombre": "Sofia_Salazar", "similitud": 0.68},
            {"aceptado": True, "nombre": "Juan_Prochazka", "similitud": 0.65},
            {"aceptado": False, "nombre": "NO_CONCLUYENTE", "similitud": None},
        ]
        veredicto = votar_identidad(decisiones, min_votos=2)
        self.assertEqual(veredicto["nombre"], "Sofia_Salazar")
        self.assertEqual(veredicto["votos"], 2)

        sin_mayoria = votar_identidad([{"aceptado": True, "nombre": "Sofia_Salazar", "similitud": 0.7}], min_votos=2)
        self.assertEqual(sin_mayoria["nombre"], "NO_CONCLUYENTE")


if __name__ == "__main__":
    unittest.main()
