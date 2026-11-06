"""Comprueban separación de datos y decisiones; no prueban precisión general."""

from dataclasses import replace
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from modelos_rostros import MODELOS, preparar_modelos
from reconocimiento_facial import (Analisis, Criterio, ExtractorFacial, Foto, Galeria,
                                  calibrar, decidir, evaluar, inventariar,
                                  normalizar, particion_inicial, preparar_particion)


class ParticionTests(unittest.TestCase):
    def fotos(self):
        return [Foto(f"Alumno/{i}.jpg", "Alumno", f"hash-{i}") for i in range(5)]

    def test_referencias_calibracion_y_evaluacion_separadas(self):
        fotos = particion_inicial(self.fotos())
        self.assertEqual([f.rol for f in fotos], ["galeria"] * 3 + ["calibracion", "evaluacion"])
        self.assertEqual(len({f.sha256 for f in fotos}), 5)

    def test_pocos_datos_no_se_copian_entre_conjuntos(self):
        self.assertEqual([f.rol for f in particion_inicial(self.fotos()[:1])], ["galeria"])
        self.assertEqual([f.rol for f in particion_inicial(self.fotos()[:2])], ["galeria", "evaluacion"])

    def test_manifiesto_persistente_y_datos_modificados(self):
        with tempfile.TemporaryDirectory() as carpeta:
            p = Path(carpeta) / "particion.json"
            original = preparar_particion(self.fotos(), p)
            self.assertEqual(preparar_particion(self.fotos(), p), original)
            cambiadas = [replace(self.fotos()[0], sha256="otra"), *self.fotos()[1:]]
            with self.assertRaises(ValueError):
                preparar_particion(cambiadas, p)
            self.assertEqual(preparar_particion(cambiadas, p, regenerar=True)[0].sha256, "otra")

    def test_manifesto_no_puede_repetir_una_foto_entre_roles(self):
        with tempfile.TemporaryDirectory() as carpeta:
            p = Path(carpeta) / "particion.json"
            preparar_particion(self.fotos(), p)
            datos = json.loads(p.read_text())
            datos["fotos"][-1] = dict(datos["fotos"][0], rol="evaluacion")
            p.write_text(json.dumps(datos))
            with self.assertRaises(ValueError):
                preparar_particion(self.fotos(), p)

    def test_duplicados_exactos_se_detectan_antes_del_reparto(self):
        with tempfile.TemporaryDirectory() as carpeta:
            alumno = Path(carpeta) / "Persona"
            alumno.mkdir()
            (alumno / "a.jpg").write_bytes(b"misma imagen")
            (alumno / "b.jpg").write_bytes(b"misma imagen")
            with self.assertRaises(ValueError):
                inventariar(Path(carpeta))


class DecisionesTests(unittest.TestCase):
    def test_vectores_invalidos_se_rechazan(self):
        for vector in ([], [0, 0], [float("nan"), 1], [float("inf"), 0]):
            with self.assertRaises(ValueError):
                normalizar(vector)

    def test_galeria_solo_usa_referencias(self):
        analisis = []
        for rol in ("galeria", "calibracion", "evaluacion"):
            f = Foto(f"{rol}.jpg", "A", rol, rol)
            analisis.append(Analisis(f, "OK", 1, np.array([1, 0]), 0.9, 100, 1))
        galeria = Galeria.desde_analisis(analisis)
        self.assertEqual(galeria.origenes.tolist(), ["galeria.jpg"])

    def test_sin_referencia_valida_no_se_inscribe_alumno(self):
        f = Foto("a.jpg", "A", "hash", "galeria")
        with self.assertRaises(ValueError):
            Galeria.desde_analisis([Analisis(f, "SIN_ROSTRO", 0, None, None, None, 1)])

    def test_nombre_rechazo_y_ambiguedad(self):
        criterio = Criterio(0.5, 0.1)
        self.assertEqual(decidir([("A", 0.8), ("B", 0.2)], criterio)["nombre"], "A")
        self.assertFalse(decidir([("A", 0.4), ("B", 0.2)], criterio)["aceptado"])
        self.assertFalse(decidir([("A", 0.8), ("B", 0.75)], criterio)["aceptado"])
        self.assertFalse(decidir([("A", 0.8), ("B", 0.8)], Criterio(0, 0))["aceptado"])
        self.assertFalse(decidir([("A", 0.99)], criterio)["aceptado"])
        self.assertFalse(decidir([], criterio)["aceptado"])

    def test_coseno_normalizado_y_exclusion_de_identidad(self):
        galeria = Galeria([[3, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]], ["A", "A", "B", "C"])
        ranking = galeria.puntuar([2, 0, 0])
        self.assertAlmostEqual(ranking[0][1], 1)
        self.assertEqual(ranking[0][0], "A")
        self.assertNotIn("A", dict(galeria.puntuar([2, 0, 0], excluir="A")))

    def test_calibracion_no_fuerza_aceptar_impostores(self):
        casos = [
            {"real": "A", "ranking": [("A", 0.85), ("B", 0.2)],
             "ranking_sin_identidad": [("B", 0.2), ("C", 0.18)]},
            {"real": "B", "ranking": [("B", 0.75), ("C", 0.25)],
             "ranking_sin_identidad": [("C", 0.25), ("A", 0.2)]},
        ]
        criterio, resumen = calibrar(casos)
        self.assertEqual(resumen["aciertos"], 2)
        self.assertEqual(resumen["falsas_aceptaciones_simuladas"], 0)
        for caso in casos:
            self.assertFalse(decidir(caso["ranking_sin_identidad"], criterio)["aceptado"])

    def test_fallos_del_detector_no_se_ocultan_en_evaluacion(self):
        casos = [{"archivo": "a.jpg", "real": "A", "estado": "SIN_ROSTRO", "ranking": [],
                  "ranking_sin_identidad": []}]
        filas, resumen = evaluar(casos, Criterio(0.5, 0.1))
        self.assertEqual(resumen["consultas"], 1)
        self.assertEqual(resumen["rechazos"], 1)
        self.assertEqual(resumen["aciertos"], 0)
        self.assertEqual(filas[0]["motivo"], "SIN_ROSTRO")

    def test_modelo_local_corrupto_no_se_reemplaza(self):
        with tempfile.TemporaryDirectory() as carpeta:
            p = Path(carpeta) / MODELOS["yunet"]["archivo"]
            p.write_bytes(b"archivo incorrecto")
            with self.assertRaises(RuntimeError):
                preparar_modelos(Path(carpeta), permitir_descarga=False)
            self.assertEqual(p.read_bytes(), b"archivo incorrecto")


class ModelosRealesTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        carpeta = Path(__file__).resolve().parents[1] / "modelos"
        if not all((carpeta / m["archivo"]).is_file() for m in MODELOS.values()):
            raise unittest.SkipTest("Preparar modelos desde el notebook; estas pruebas no descargan")
        cls.extractor = ExtractorFacial(preparar_modelos(carpeta, permitir_descarga=False))

    def test_imagen_negra_y_entrada_invalida(self):
        a = self.extractor.analizar_imagen(np.zeros((480, 640, 3), dtype=np.uint8))
        self.assertEqual(a.estado, "SIN_ROSTRO")
        self.assertIsNone(a.descriptor)
        self.assertEqual(self.extractor.analizar_imagen(None).estado, "IMAGEN_INVALIDA")


if __name__ == "__main__":
    unittest.main()
