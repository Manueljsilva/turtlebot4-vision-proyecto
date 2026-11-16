"""Imágenes sintéticas y tubería real; no abre cámara, GUI, Docker ni ROS."""

import io
import json
import subprocess
import sys
import threading
import time
from types import SimpleNamespace as NS
import unittest

import cv2
import numpy as np

from camara_robot import (CADUCIDAD_VIDEO_NS, MAX_DATOS, VideoRobot, codificar_imagen,
                          comando_camara, decodificar_imagen, leer_imagen, panel_lateral)
from camara_robot_ros import vigilar_cierre


def mensaje(datos, encoding='bgr8', ancho=1, alto=1, paso=3):
    return NS(data=datos, encoding=encoding, width=ancho, height=alto, step=paso)


class CamaraTests(unittest.TestCase):
    def test_rgb_con_relleno_de_fila_se_convierte_a_bgr(self):
        m = mensaje(bytes([255, 0, 0, 99, 0, 255, 0, 99]), 'rgb8', alto=2, paso=4)
        p = leer_imagen(codificar_imagen(m, False, 100))
        np.testing.assert_array_equal(decodificar_imagen(p), [[[0, 0, 255]], [[0, 255, 0]]])

    def test_bgr_mono_y_alpha(self):
        for encoding, datos, esperado in [('bgr8', [1, 2, 3], [1, 2, 3]),
                                           ('mono8', [7], [7, 7, 7]),
                                           ('rgba8', [1, 2, 3, 255], [3, 2, 1]),
                                           ('bgra8', [1, 2, 3, 255], [1, 2, 3])]:
            with self.subTest(encoding=encoding):
                p = leer_imagen(codificar_imagen(mensaje(bytes(datos), encoding, paso=len(datos)), False, 100))
                np.testing.assert_array_equal(decodificar_imagen(p)[0, 0], esperado)

    def test_comprimida_png_se_decodifica_en_el_host(self):
        imagen = np.full((8, 12, 3), (10, 50, 200), np.uint8)
        ok, png = cv2.imencode('.png', imagen)
        self.assertTrue(ok)
        p = leer_imagen(codificar_imagen(NS(data=png.tobytes()), True, 100))
        np.testing.assert_array_equal(decodificar_imagen(p), imagen)

    def test_datos_invalidos_no_producen_imagen(self):
        registro = json.loads(codificar_imagen(mensaje(b'\x01\x02\x03'), False, 100))
        for cambios in [{'version': 2}, {'recibido_ns': True}, {'datos': '!!!!'},
                        {'ancho': 2}, {'paso': 1}, {'encoding': '16UC1'}]:
            with self.subTest(cambios=cambios), self.assertRaises((ValueError, TypeError)):
                leer_imagen(json.dumps(dict(registro, **cambios)))
        with self.assertRaises(ValueError):
            leer_imagen('[]')
        with self.assertRaises(ValueError):
            decodificar_imagen(dict(encoding='compressed', datos=b'imagen rota'))

    def test_limite_de_datos_se_comprueba_antes_de_codificar(self):
        class Grande:
            def __len__(self):
                return MAX_DATOS + 1
        with self.assertRaises(ValueError):
            codificar_imagen(NS(data=Grande()), True, 100)

    def video_por_tuberia(self, registros):
        codigo = ('import sys; sys.stdout.buffer.write(bytes.fromhex(sys.argv[1])); '
                  'sys.stdout.buffer.flush(); sys.stdin.buffer.read()')
        proceso = subprocess.Popen([sys.executable, '-u', '-c', codigo, b''.join(registros).hex()],
                                   stdin=subprocess.PIPE, stdout=subprocess.PIPE)
        video = VideoRobot(proceso)
        self.addCleanup(video.cerrar)
        return video

    def esperar_imagen(self, video, valor=None):
        limite = time.monotonic() + 2
        while time.monotonic() < limite:
            imagen, estado = video.fotograma(100)
            if imagen is not None and (valor is None or imagen[0, 0, 0] == valor):
                return imagen
            time.sleep(0.01)
        self.fail(f'No llego la imagen: {estado}')

    def test_tuberia_descarta_imagenes_antiguas_y_cierra_por_eof(self):
        registros = [codificar_imagen(mensaje(bytes([v, 0, 0])), False, 100) for v in (1, 2, 3)]
        video = self.video_por_tuberia(registros)
        self.assertEqual(self.esperar_imagen(video, 3)[0, 0, 0], 3)
        video.cerrar()
        self.assertEqual(video.proceso.returncode, 0)
        self.assertFalse(video._hilo.is_alive())

    def test_video_viejo_o_con_reloj_futuro_no_se_muestra(self):
        video = self.video_por_tuberia([codificar_imagen(mensaje(b'\x01\x02\x03'), False, 100)])
        self.esperar_imagen(video)
        for ahora in (99, 100 + CADUCIDAD_VIDEO_NS + 1):
            imagen, estado = video.fotograma(ahora)
            self.assertIsNone(imagen)
            self.assertEqual(estado, 'Sin imagen reciente')

    def test_canal_roto_se_indica_sin_afectar_a_las_ordenes(self):
        video = self.video_por_tuberia([b'{registro roto}\n'])
        limite = time.monotonic() + 2
        while video._hilo.is_alive() and time.monotonic() < limite:
            time.sleep(0.01)
        self.assertIsNone(video.fotograma()[0])
        self.assertIn('Video no disponible', video.fotograma()[1])

    def test_paneles_laterales_no_invierten_la_imagen(self):
        webcam = np.full((240, 320, 3), (0, 50, 0), np.uint8)
        imagen = np.zeros((100, 160, 3), np.uint8)
        imagen[:, :80] = (0, 0, 255)
        imagen[:, 80:] = (255, 0, 0)
        panel = panel_lateral(webcam, imagen, 'Camara del robot')
        self.assertEqual(panel.shape, (240, 640, 3))
        np.testing.assert_array_equal(panel[:, :320], webcam)
        np.testing.assert_array_equal(panel[100, 350], [0, 0, 255])
        np.testing.assert_array_equal(panel[100, 620], [255, 0, 0])
        self.assertEqual(panel_lateral(webcam, None, 'Sin imagen reciente').shape, panel.shape)

    def test_camara_usa_red_del_robot_sin_display_ni_canal_de_velocidad(self):
        args = comando_camara('10.87.179.100', 5, 'proyecto2-sim', '/oakd/rgb/preview/image_raw')
        for valor in ['ROS_DOMAIN_ID=5', 'ROS_STATIC_PEERS=10.87.179.100',
                      'RMW_IMPLEMENTATION=rmw_fastrtps_cpp', 'RCUTILS_LOGGING_USE_STDOUT=0', '-i']:
            self.assertIn(valor, args)
        self.assertNotIn('/cmd_vel', ' '.join(args))
        self.assertNotIn('DISPLAY', ' '.join(args))

    def test_receptor_ros_se_cierra_incluso_sin_fotogramas(self):
        fin = threading.Event()
        vigilar_cierre(io.BytesIO(), fin)
        self.assertTrue(fin.is_set())
