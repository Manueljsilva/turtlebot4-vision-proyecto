"""Pruebas geométricas y temporales. No miden precisión con manos reales."""

import math
import unittest

from gestos import FiltroOrden, Orden, interpretar_mano


def mano_sintetica(angulo=0, palma=False, ancho=640, alto=480):
    """Mano esquemática: índice arriba y demás dedos flexionados por defecto."""
    puntos = [(0.0, 0.0)] * 21
    for base, x in zip((5, 9, 13, 17), (-30, 0, 25, 45)):
        puntos[base] = (x, -60)
        if base == 5 or palma:
            puntos[base + 1 : base + 4] = [(x, -90), (x, -115), (x, -140)]
        else:
            puntos[base + 1 : base + 4] = [(x, -90), (x + 15, -65), (x + 10, -50)]
    coseno, seno = math.cos(angulo), math.sin(angulo)
    return [
        ((ancho / 2 + x * coseno - y * seno) / ancho,
         (alto / 2 + x * seno + y * coseno) / alto)
        for x, y in puntos
    ]


class InterpretacionTests(unittest.TestCase):
    def test_direcciones_y_rechazo_diagonal(self):
        casos = [(0, Orden.AVANZAR), (-math.pi / 2, Orden.DERECHA),
                 (math.pi / 2, Orden.IZQUIERDA), (math.pi, Orden.DETENER),
                 (math.pi / 4, Orden.DETENER)]
        for angulo, esperada in casos:
            with self.subTest(angulo=angulo):
                self.assertEqual(interpretar_mano(mano_sintetica(angulo), 640, 480).orden, esperada)

    def test_palma_abierta_siempre_detiene(self):
        for angulo in (0, math.pi / 2, math.pi):
            self.assertEqual(interpretar_mano(mano_sintetica(angulo, palma=True), 640, 480).orden, Orden.DETENER)

    def test_la_relacion_de_aspecto_no_cambia_la_geometria(self):
        for ancho, alto in ((640, 480), (1280, 720), (480, 640)):
            puntos = mano_sintetica(math.pi / 2, ancho=ancho, alto=alto)
            self.assertEqual(interpretar_mano(puntos, ancho, alto).orden, Orden.IZQUIERDA)

    def test_sin_mano_datos_degenerados_y_no_finitos(self):
        parcial = mano_sintetica()
        parcial[10:13] = [parcial[9]] * 3
        for puntos in (None, [], [(0, 0)] * 21, [(math.nan, 0)] * 21, parcial):
            self.assertEqual(interpretar_mano(puntos, 640, 480).orden, Orden.DETENER)

    def test_dos_dedos_extendidos_no_son_una_orden(self):
        puntos = mano_sintetica()
        puntos[10:13] = mano_sintetica(palma=True)[10:13]
        self.assertEqual(interpretar_mano(puntos, 640, 480).orden, Orden.DETENER)


class FiltroTests(unittest.TestCase):
    def setUp(self):
        self.filtro = FiltroOrden()

    def activar(self, orden=Orden.AVANZAR):
        self.filtro.actualizar(orden, 0.0)
        self.filtro.actualizar(orden, 0.2)
        return self.filtro.actualizar(orden, 0.41)

    def test_un_destello_no_activa_movimiento(self):
        self.assertEqual(self.filtro.actualizar(Orden.AVANZAR, 0), Orden.DETENER)
        self.assertEqual(self.filtro.actualizar(Orden.AVANZAR, 0.2), Orden.DETENER)
        self.assertEqual(self.filtro.actualizar(Orden.DETENER, 0.3), Orden.DETENER)

    def test_gesto_sostenido_y_parada_inmediata(self):
        self.assertEqual(self.activar(), Orden.AVANZAR)
        self.assertEqual(self.filtro.actualizar(Orden.DETENER, 0.42), Orden.DETENER)

    def test_cambiar_de_direccion_exige_otra_confirmacion(self):
        self.activar(Orden.IZQUIERDA)
        self.assertEqual(self.filtro.actualizar(Orden.DERECHA, 0.42), Orden.DETENER)
        self.assertEqual(self.filtro.actualizar(Orden.DERECHA, 0.7), Orden.DETENER)
        self.assertEqual(self.filtro.actualizar(Orden.DERECHA, 0.83), Orden.DERECHA)

    def test_una_pausa_reinicia_la_confirmacion(self):
        self.activar()
        self.assertEqual(self.filtro.actualizar(Orden.AVANZAR, 1.1), Orden.DETENER)
        self.assertEqual(self.filtro.actualizar(Orden.AVANZAR, 1.3), Orden.DETENER)
        self.assertEqual(self.filtro.actualizar(Orden.AVANZAR, 1.51), Orden.AVANZAR)

    def test_orden_caducada_no_se_reutiliza(self):
        self.activar()
        self.assertEqual(self.filtro.vigente(0.5), Orden.AVANZAR)
        self.assertEqual(self.filtro.vigente(1.0), Orden.DETENER)
        self.assertEqual(self.filtro.actualizar(Orden.AVANZAR, 1.1), Orden.DETENER)

    def test_retroceso_del_reloj_inicia_confirmacion_nueva(self):
        self.activar()
        self.assertEqual(self.filtro.actualizar(Orden.AVANZAR, 0.1), Orden.DETENER)

    def test_entrada_invalida_no_mantiene_movimiento(self):
        self.activar()
        self.assertEqual(self.filtro.actualizar("derecha", 0.42), Orden.DETENER)
        self.assertEqual(self.filtro.actualizar(Orden.AVANZAR, math.nan), Orden.DETENER)


if __name__ == "__main__":
    unittest.main()
