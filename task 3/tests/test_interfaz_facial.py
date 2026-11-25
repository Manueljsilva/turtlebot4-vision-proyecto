"""Entrada de objetivo con ratón/teclado simulados y decisiones faciales reales."""

from types import SimpleNamespace
import unittest

import cv2
import numpy as np

from interfaz_facial import (BUSCAR, CAMPO, SALIR, Y_SUGERENCIAS, EntradaObjetivo,
                            dibujar_interfaz, mensaje_busqueda)
from prueba_facial_robot import ConsultaEnVivo, SistemaFacial
from reconocimiento_facial import Criterio, Galeria


class EntradaObjetivoTests(unittest.TestCase):
    def setUp(self):
        self.llamadas = 0
        def analizar(imagen):
            self.llamadas += 1
            return SimpleNamespace(estado='OK', descriptor=np.array([1., 0.]), caja=(1, 2, 8, 8))
        self.sistema = SistemaFacial(SimpleNamespace(analizar_imagen=analizar),
                                    Galeria([[1, 0], [0, 1]], ['Persona A', 'Persona B']),
                                    Criterio(0.5, 0.1))
        self.consulta = ConsultaEnVivo(self.sistema)
        self.entrada = EntradaObjetivo(self.consulta)
        self.imagen = np.zeros((24, 32, 3), dtype=np.uint8)

    def escribir(self, texto):
        for letra in texto:
            self.assertFalse(self.entrada.tecla(ord(letra)))

    def click(self, caja):
        x0, y0, x1, y1 = caja
        self.entrada.raton(cv2.EVENT_LBUTTONDOWN, (x0+x1)//2, (y0+y1)//2)

    def test_escribir_no_cambia_objetivo_hasta_click_buscar(self):
        self.escribir('persona a')
        self.assertIsNone(self.consulta.nombre)
        self.click(BUSCAR)
        self.assertEqual(self.consulta.nombre, 'Persona A')
        self.assertFalse(self.entrada.error)

    def test_sugerencia_rellena_y_buscar_confirma(self):
        self.escribir('sona b')
        self.assertEqual(self.entrada.sugerencias(), ['Persona B'])
        self.entrada.raton(cv2.EVENT_LBUTTONDOWN, 80, Y_SUGERENCIAS + 8)
        self.assertEqual(self.entrada.texto, 'Persona B')
        self.assertIsNone(self.consulta.nombre)
        self.click(BUSCAR)
        self.assertEqual(self.consulta.nombre, 'Persona B')

    def test_nombre_vacio_ajeno_o_parcial_no_fuerza_ni_reemplaza_objetivo(self):
        self.consulta.seleccionar_nombre('Persona B')
        for texto in ('', 'Alguien ausente', 'Persona'):
            self.entrada.texto = texto
            self.click(BUSCAR)
            self.assertTrue(self.entrada.error)
            self.assertEqual(self.consulta.nombre, 'Persona B')
        resultado = self.consulta.actualizar(self.imagen, 100, 101)
        self.assertEqual(resultado['nombre'], 'Persona A')
        self.assertFalse(resultado['objetivo_confirmado'])

    def test_cambio_de_objetivo_recompara_frame_sin_repetir_inferencia(self):
        self.consulta.seleccionar_nombre('Persona B')
        primero = self.consulta.actualizar(self.imagen, 100, 101)
        self.assertFalse(primero['objetivo_confirmado'])
        self.entrada.texto = 'Persona A'
        self.click(BUSCAR)
        segundo = self.consulta.actualizar(self.imagen, 100, 102)
        self.assertEqual(segundo['nombre'], 'Persona A')
        self.assertTrue(segundo['objetivo_confirmado'])
        self.assertEqual(self.llamadas, 1)
        self.entrada.texto = 'Persona B'
        self.click(BUSCAR)
        tercero = self.consulta.actualizar(self.imagen, 100, 103)
        self.assertFalse(tercero['objetivo_confirmado'])
        self.assertEqual(tercero['nombre'], 'Persona A')

    def test_editar_tras_confirmar_conserva_objetivo_activo_hasta_buscar(self):
        self.entrada.texto = 'Persona A'
        self.click(BUSCAR)
        self.click(CAMPO)
        self.entrada.tecla(8)
        self.entrada.tecla(ord('B'))
        self.assertEqual(self.entrada.texto, 'Persona B')
        self.assertEqual(self.consulta.nombre, 'Persona A')
        self.entrada.tecla(13)
        self.assertEqual(self.consulta.nombre, 'Persona B')

    def test_busqueda_sin_tilde_y_nombre_ambiguo(self):
        self.sistema.galeria = Galeria([[1, 0], [0, 1]], ['Érika', 'Otra persona'])
        entrada = EntradaObjetivo(ConsultaEnVivo(self.sistema))
        entrada.texto = 'erika'
        entrada.confirmar()
        self.assertEqual(entrada.consulta.nombre, 'Érika')
        self.sistema.galeria = Galeria([[1, 0], [0, 1]], ['José', 'Jose'])
        entrada = EntradaObjetivo(ConsultaEnVivo(self.sistema))
        entrada.texto = 'jose'
        entrada.confirmar()
        self.assertTrue(entrada.error)
        self.assertIsNone(entrada.consulta.nombre)

    def test_caducidad_no_confirma_rostro_anterior_al_cambiar_objetivo(self):
        self.consulta.actualizar(self.imagen, 100, 101)
        self.consulta.actualizar(None, None, 102)
        self.consulta.seleccionar_nombre('Persona A')
        resultado = self.consulta.actualizar(self.imagen, 100, 1_000_000_101)
        self.assertFalse(resultado['objetivo_confirmado'])
        self.assertFalse(resultado['aceptado'])
        mensaje, _ = mensaje_busqueda(resultado, self.consulta.nombre, False)
        self.assertIn('Esperando', mensaje)

    def test_distingue_objetivo_otro_conocido_y_rechazo(self):
        self.consulta.seleccionar_nombre('Persona A')
        resultado = self.consulta.actualizar(self.imagen, 100, 101)
        self.assertEqual(mensaje_busqueda(resultado, 'Persona A', True)[0],
                         'Reconozco a la persona que buscas.')
        self.consulta.seleccionar_nombre('Persona B')
        resultado = self.consulta.actualizar(self.imagen, 100, 102)
        self.assertEqual(mensaje_busqueda(resultado, 'Persona B', True)[0],
                         'Reconozco a otra persona; no es la que buscas.')
        self.assertEqual(resultado['nombre'], 'Persona A')
        resultado = self.consulta.actualizar(None, None, 103)
        self.assertIn('Aun no puedo identificar', mensaje_busqueda(resultado, 'Persona B', True)[0])

    def test_q_es_texto_dentro_del_campo_y_salida_fuera(self):
        self.entrada.tecla(ord('q'))
        self.assertEqual(self.entrada.texto, 'q')
        self.assertFalse(self.entrada.cerrar)
        self.entrada.foco = False
        self.assertTrue(self.entrada.tecla(ord('q')))

    def test_escape_y_boton_salir_funcionan_durante_edicion(self):
        self.assertTrue(self.entrada.tecla(27))
        self.entrada.cerrar = False
        self.click(SALIR)
        self.assertTrue(self.entrada.tecla(-1))

    def test_panel_espera_y_reconocimiento_conserva_imagen(self):
        copia = self.imagen.copy()
        for imagen in (None, self.imagen):
            resultado = self.consulta.actualizar(imagen, 100 if imagen is not None else None, 101)
            panel = dibujar_interfaz(imagen, resultado, 'Esperando camara', self.entrada)
            self.assertEqual(panel.shape, (708, 960, 3))
        self.assertTrue(np.array_equal(self.imagen, copia))


if __name__ == '__main__':
    unittest.main()
