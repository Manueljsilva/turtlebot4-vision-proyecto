"""Contrato de órdenes y vigilancia; no evalúa precisión de manos ni ROS."""

import json
import math
import unittest

from control_gestos import RelojSimulado, VigilanteOrden, codificar, velocidades
from gestos import FiltroOrden, Orden, interpretar_mano
from tests.test_gestos import mano_sintetica


class ControlTests(unittest.TestCase):
    def test_velocidades_y_sentido_del_robot(self):
        self.assertEqual(velocidades(Orden.AVANZAR), (0.306, 0.0))
        self.assertEqual(velocidades(Orden.IZQUIERDA), (0.0, 1.90))
        self.assertEqual(velocidades(Orden.DERECHA), (0.0, -1.90))
        self.assertEqual(velocidades(Orden.DETENER), (0.0, 0.0))
        self.assertEqual(velocidades("DESCONOCIDA"), (0.0, 0.0))

    def test_lados_del_operador_hasta_velocidad_y_caducidad(self):
        # Sin espejo: derecha del operador aparece a la izquierda de la imagen.
        for angulo, orden, giro in [(-math.pi / 2, Orden.DERECHA, -1.90),
                                     (math.pi / 2, Orden.IZQUIERDA, 1.90)]:
            with self.subTest(orden=orden):
                gesto = interpretar_mano(mano_sintetica(angulo), 640, 480)
                self.assertEqual(gesto.orden, orden)
                filtro, vigilante = FiltroOrden(), VigilanteOrden()
                for secuencia, instante in enumerate((0.0, 0.2, 0.41)):
                    salida = filtro.actualizar(gesto.orden, instante)
                    captura = round(instante * 1_000_000_000)
                    vigilante.recibir(codificar(salida, captura, secuencia), captura)
                self.assertEqual(velocidades(vigilante.vigente(captura)), (0.0, giro))
                self.assertEqual(velocidades(vigilante.vigente(captura + 500_000_001)),
                                 (0.0, 0.0))

    def test_sin_datos_y_caducidad_sin_mas_mensajes(self):
        v = VigilanteOrden()
        self.assertEqual(v.vigente(0), Orden.DETENER)
        v.recibir(codificar(Orden.AVANZAR, 1_000_000_000, 0), 1_010_000_000)
        self.assertEqual(v.vigente(1_500_000_000), Orden.AVANZAR)
        self.assertEqual(v.vigente(1_500_000_001), Orden.DETENER)
        self.assertEqual(v.vigente(1_100_000_000), Orden.DETENER)

    def test_una_linea_en_cola_no_renueva_una_captura_vieja(self):
        v = VigilanteOrden()
        v.recibir(codificar(Orden.AVANZAR, 1, 0), 700_000_000)
        self.assertEqual(v.vigente(700_000_000), Orden.DETENER)

    def test_futuro_y_retroceso_de_reloj_detienen(self):
        v = VigilanteOrden()
        v.recibir(codificar(Orden.IZQUIERDA, 500, 0), 499)
        self.assertEqual(v.vigente(499), Orden.DETENER)
        v.recibir(codificar(Orden.IZQUIERDA, 500, 1), 501)
        self.assertEqual(v.vigente(499), Orden.DETENER)

    def test_perdida_de_mano_y_cambio_de_direccion_en_el_enlace(self):
        filtro, vigilante = FiltroOrden(), VigilanteOrden()
        casos = [
            (0.00, Orden.IZQUIERDA, Orden.DETENER),
            (0.20, Orden.IZQUIERDA, Orden.DETENER),
            (0.41, Orden.IZQUIERDA, Orden.IZQUIERDA),
            (0.42, Orden.DERECHA, Orden.DETENER),
            (0.65, Orden.DERECHA, Orden.DETENER),
            (0.84, Orden.DERECHA, Orden.DERECHA),
            (0.85, Orden.DETENER, Orden.DETENER),
        ]
        for secuencia, (instante, propuesta, esperada) in enumerate(casos):
            ahora = round(instante * 1_000_000_000)
            orden = filtro.actualizar(propuesta, instante)
            vigilante.recibir(codificar(orden, ahora, secuencia), ahora)
            self.assertEqual(vigilante.vigente(ahora), esperada)

    def test_repeticion_no_revive_movimiento(self):
        v = VigilanteOrden()
        linea = codificar(Orden.AVANZAR, 0, 2)
        v.recibir(linea, 0)
        self.assertEqual(v.vigente(1), Orden.AVANZAR)
        v.recibir(linea, 2)
        self.assertEqual(v.vigente(2), Orden.DETENER)

    def test_datos_invalidos_detienen_una_orden_previa(self):
        valido = json.loads(codificar(Orden.AVANZAR, 0, 1))
        invalidos = ["", "NaN", "[]", "null", "hola", "x" * 1025]
        for campo, valor in [("version", True), ("version", 2), ("captura_ns", math.nan),
                             ("captura_ns", True), ("captura_ns", -1), ("secuencia", True),
                             ("secuencia", -1), ("orden", "OTRA"), ("orden", None)]:
            invalidos.append(json.dumps(dict(valido, **{campo: valor})))
        for linea in invalidos:
            with self.subTest(linea=linea[:100]):
                v = VigilanteOrden()
                v.recibir(codificar(Orden.AVANZAR, 0, 0), 0)
                self.assertEqual(v.vigente(0), Orden.AVANZAR)
                v.recibir(linea, 1)
                self.assertEqual(v.vigente(1), Orden.DETENER)

    def test_caducidad_invalida(self):
        for valor in (0, -1, math.inf, math.nan):
            with self.assertRaises(ValueError):
                VigilanteOrden(valor)


class RelojTests(unittest.TestCase):
    def test_sin_reloj_pausa_y_reanudacion(self):
        reloj = RelojSimulado()
        self.assertFalse(reloj.disponible(0, 1))
        self.assertTrue(reloj.disponible(100, 10))
        self.assertTrue(reloj.disponible(100, 500_000_010))
        self.assertFalse(reloj.disponible(100, 500_000_011))
        self.assertTrue(reloj.disponible(101, 600_000_000))

    def test_reset_de_simulacion_no_reutiliza_el_reloj_anterior(self):
        reloj = RelojSimulado()
        self.assertTrue(reloj.disponible(200, 10))
        self.assertFalse(reloj.disponible(100, 20))
        self.assertFalse(reloj.disponible(100, 21))
        self.assertTrue(reloj.disponible(101, 30))


if __name__ == "__main__":
    unittest.main()
