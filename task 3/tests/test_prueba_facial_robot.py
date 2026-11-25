"""Cámara facial sin ROS ni hardware: rechazo, objetivo, frescura y conexión."""

from pathlib import Path
import io
import subprocess
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

from modelos_rostros import MODELOS
from reconocimiento_facial import Criterio, Galeria
from interfaz_facial import EntradaObjetivo, dibujar_interfaz
import prueba_facial_robot as prueba


class ExtractorDoble:
    def __init__(self):
        self.llamadas = 0
        self.estado = 'OK'
        self.descriptor = np.array([1., 0.], dtype=np.float32)

    def analizar_imagen(self, imagen):
        self.llamadas += 1
        return SimpleNamespace(estado=self.estado, descriptor=self.descriptor,
                               caja=(1, 2, 3, 4) if self.estado == 'OK' else None)


class ConsultaFacialRobotTests(unittest.TestCase):
    def setUp(self):
        self.extractor = ExtractorDoble()
        self.galeria = Galeria([[1, 0], [0, 1]], ['Persona A', 'Persona B'])
        self.sistema = prueba.SistemaFacial(self.extractor, self.galeria, Criterio(0.5, 0.1))
        self.imagen = np.zeros((32, 32, 3), dtype=np.uint8)

    def test_identidad_aceptada_se_basa_en_galeria(self):
        consulta = prueba.ConsultaEnVivo(self.sistema)
        resultado = consulta.actualizar(self.imagen, 100, 101)
        self.assertEqual(resultado['nombre'], 'Persona A')
        self.assertTrue(resultado['aceptado'])
        self.assertFalse(resultado['objetivo_confirmado'])

    def test_otro_nombre_y_rechazo_no_se_convierten_en_objetivo(self):
        consulta = prueba.ConsultaEnVivo(self.sistema, 'persona b')
        resultado = consulta.actualizar(self.imagen, 100, 101)
        self.assertEqual(resultado['nombre'], 'Persona A')
        self.assertFalse(resultado['objetivo_confirmado'])
        self.extractor.descriptor = np.array([1., 1.], dtype=np.float32)
        resultado = consulta.actualizar(self.imagen, 102, 103)
        self.assertEqual(resultado['nombre'], 'NO_CONCLUYENTE')
        self.assertFalse(resultado['objetivo_confirmado'])

    def test_objetivo_solo_se_confirma_si_coincide_identidad_aceptada(self):
        consulta = prueba.ConsultaEnVivo(self.sistema, '  PERSONA A ')
        self.assertTrue(consulta.actualizar(self.imagen, 100, 101)['objetivo_confirmado'])
        with self.assertRaises(ValueError):
            prueba.ConsultaEnVivo(self.sistema, 'Persona ausente')

    def test_sin_rostro_y_varios_rostros_conservan_rechazo(self):
        consulta = prueba.ConsultaEnVivo(self.sistema)
        for i, estado in enumerate(('SIN_ROSTRO', 'VARIOS_ROSTROS', 'ROSTRO_NO_UTIL')):
            with self.subTest(estado=estado):
                self.extractor.estado = estado
                resultado = consulta.actualizar(self.imagen, i + 1, i + 2)
                self.assertFalse(resultado['aceptado'])
                self.assertEqual(resultado['nombre'], 'NO_CONCLUYENTE')
                self.assertEqual(resultado['motivo'], estado)

    def test_misma_marca_no_repite_inferencia_ni_acumula_votos(self):
        consulta = prueba.ConsultaEnVivo(self.sistema)
        primero = consulta.actualizar(self.imagen, 100, 101)
        self.assertIs(consulta.actualizar(self.imagen, 100, 102), primero)
        self.assertEqual(self.extractor.llamadas, 1)
        consulta.actualizar(self.imagen, 103, 104)
        self.assertEqual(self.extractor.llamadas, 2)

    def test_imagen_caducada_ausente_o_futura_borra_identidad(self):
        consulta = prueba.ConsultaEnVivo(self.sistema)
        self.assertTrue(consulta.actualizar(self.imagen, 100, 101)['aceptado'])
        for imagen, marca, ahora in ((None, None, 102), (self.imagen, 100, 1_000_000_101),
                                     (self.imagen, 104, 103)):
            with self.subTest(marca=marca, ahora=ahora):
                resultado = consulta.actualizar(imagen, marca, ahora)
                self.assertEqual(resultado['nombre'], 'NO_CONCLUYENTE')
                self.assertFalse(resultado['aceptado'])
        self.assertEqual(self.extractor.llamadas, 1)

    def test_panel_no_modifica_imagen_original(self):
        imagen = self.imagen.copy()
        consulta = prueba.ConsultaEnVivo(self.sistema)
        resultado = consulta.actualizar(imagen, 100, 101)
        vista = dibujar_interfaz(imagen, resultado, 'Camara', EntradaObjetivo(consulta))
        self.assertEqual(vista.shape[1], 960)
        self.assertTrue(np.array_equal(imagen, self.imagen))


class ConexionFacialTests(unittest.TestCase):
    def setUp(self):
        self.llamadas = []

    def comando(self, args):
        self.llamadas.append(args)
        if 'inspect' in args:
            return 'true host'
        if any('ros2 topic list' in a for a in args):
            return prueba.TOPICO + ' [sensor_msgs/msg/CompressedImage]'
        if 'mktemp' in args:
            return '/tmp/proyecto2-facial.ABC12345'
        return ''

    def test_solo_suscribe_camara_y_no_copia_datos_personales(self):
        with patch.object(prueba, 'ejecutar_comando', side_effect=self.comando), \
                patch.object(prueba.subprocess, 'run'), patch.object(prueba.shutil, 'which', return_value='tool'):
            comando = prueba.preparar_camara('10.87.179.100', 5, 'proyecto2-sim', prueba.TOPICO)
        texto = ' '.join(' '.join(c) for c in self.llamadas + [comando])
        self.assertNotIn('/cmd_vel', texto)
        self.assertNotIn('/scan', texto)
        for nombre in ('ROS_DOMAIN_ID=5', 'ROS_STATIC_PEERS=10.87.179.100', '--norc', prueba.TOPICO):
            self.assertIn(nombre, comando)
        copias = [c for c in self.llamadas if 'cp' in c]
        self.assertEqual(len(copias), 2)
        self.assertEqual({Path(c[-2]).name for c in copias}, {'camara_robot.py', 'camara_robot_ros.py'})
        self.assertIn('/tmp/proyecto2-facial.ABC12345/camara_robot_ros.py', comando)

    def test_red_y_topico_incorrectos_impiden_crear_suscriptor(self):
        for caso in ('red', 'tipo'):
            def respuestas(args):
                respuesta = self.comando(args)
                if caso == 'red' and 'inspect' in args:
                    return 'true bridge'
                if caso == 'tipo' and any('ros2 topic list' in a for a in args):
                    return prueba.TOPICO + ' [std_msgs/msg/String]'
                return respuesta
            self.llamadas.clear()
            with self.subTest(caso=caso), patch.object(prueba, 'ejecutar_comando', side_effect=respuestas), \
                    patch.object(prueba.subprocess, 'run'), patch.object(prueba.shutil, 'which', return_value='tool'):
                with self.assertRaises(ValueError):
                    prueba.preparar_camara('10.87.179.100', 5, 'proyecto2-sim', prueba.TOPICO)
            self.assertFalse(any('mktemp' in c or 'cp' in c for c in self.llamadas))

    def test_argumentos_invalidos_no_contactan_docker(self):
        for args in (('999.1.1.1', 5, 'proyecto2-sim', prueba.TOPICO),
                     ('10.87.179.100', 102, 'proyecto2-sim', prueba.TOPICO),
                     ('10.87.179.100', 5, 'contenedor;algo', prueba.TOPICO),
                     ('10.87.179.100', 5, 'proyecto2-sim', 'no es topico')):
            with self.subTest(args=args), patch.object(prueba, 'ejecutar_comando') as ejecutar:
                with self.assertRaises(ValueError):
                    prueba.preparar_camara(*args)
                ejecutar.assert_not_called()

    def test_topico_aun_no_descubierto_permite_arranque_del_receptor(self):
        def respuestas(args):
            respuesta = self.comando(args)
            if any('ros2 topic list' in a for a in args):
                return '/scan [sensor_msgs/msg/LaserScan]\n/oakd/rgb/preview/image_raw [sensor_msgs/msg/Image]'
            return respuesta
        with patch.object(prueba, 'ejecutar_comando', side_effect=respuestas), \
                patch.object(prueba.subprocess, 'run'), patch.object(prueba.shutil, 'which', return_value='tool'), \
                patch('sys.stdout', new_callable=io.StringIO) as salida:
            comando = prueba.preparar_camara('10.87.179.100', 5, 'proyecto2-sim', prueba.TOPICO)
        self.assertIn('Aún no aparece', salida.getvalue())
        self.assertIn('/oakd/rgb/preview/image_raw [sensor_msgs/msg/Image]', salida.getvalue())
        self.assertEqual(comando[-1], prueba.TOPICO)  # No cambia a raw silenciosamente.

    def test_error_de_comando_conserva_la_causa_sin_volcar_lista_de_argumentos(self):
        fallo = subprocess.CompletedProcess(['docker'], 1, stdout='', stderr='No se encuentra ROS')
        with patch.object(prueba.subprocess, 'run', return_value=fallo):
            with self.assertRaisesRegex(RuntimeError, 'No se encuentra ROS'):
                prueba.ejecutar_comando(['docker', 'exec'])

    def test_tiempo_agotado_de_comprobacion_es_explicito(self):
        with patch.object(prueba.subprocess, 'run', side_effect=subprocess.TimeoutExpired(['docker'], 25)):
            with self.assertRaisesRegex(RuntimeError, 'agotó 25 s'):
                prueba.ejecutar_comando(['docker'])

    def test_formato_lista_de_topicos_admite_varios_tipos_y_omite_ruido(self):
        texto = 'aviso\n/a [sensor_msgs/msg/Image]\n/b [tipo/A, tipo/B]\n/c malformado'
        self.assertEqual(prueba.interpretar_topicos(texto),
                         {'/a': ['sensor_msgs/msg/Image'], '/b': ['tipo/A', 'tipo/B']})

    def test_arranque_lento_mantiene_la_espera_hasta_que_el_usuario_termina(self):
        galeria = Galeria([[1, 0], [0, 1]], ['Persona A', 'Persona B'])
        sistema = prueba.SistemaFacial(ExtractorDoble(), galeria, Criterio(0.5, 0.1))
        fotogramas = [(None, 'Esperando cámara'), (None, 'Esperando cámara')]
        def fotograma():
            if fotogramas:
                return fotogramas.pop(0)
            raise KeyboardInterrupt
        video = SimpleNamespace(fotograma=fotograma, cerrar=lambda: cierres.append(True))
        cierres = []
        with patch.object(prueba, 'cargar_sistema', return_value=sistema), \
                patch.object(prueba, 'preparar_camara', return_value=['cámara']), \
                patch.object(prueba, 'cargar_transporte', return_value=lambda p: video), \
                patch.object(prueba.subprocess, 'Popen') as abrir, \
                patch.object(prueba.time, 'monotonic', side_effect=[31, 301]), \
                patch.object(prueba.time, 'sleep'), \
                patch('sys.stdout', new_callable=io.StringIO) as salida, \
                patch('sys.stderr', new_callable=io.StringIO) as error:
            abrir.return_value.poll.return_value = None
            codigo = prueba.main(['--ip', '10.87.179.100', '--dominio', '5', '--sin-ventana'])
        self.assertEqual(codigo, 0)
        self.assertIn('reconocimiento en espera', salida.getvalue())
        self.assertEqual(error.getvalue(), '')
        self.assertEqual(fotogramas, [])
        self.assertEqual(cierres, [True])

    def test_interfaz_abre_esperando_registra_click_y_cierra_con_escape(self):
        galeria = Galeria([[1, 0], [0, 1]], ['Persona A', 'Persona B'])
        sistema = prueba.SistemaFacial(ExtractorDoble(), galeria, Criterio(0.5, 0.1))
        cierres = []
        video = SimpleNamespace(fotograma=lambda: (None, 'Esperando cámara'),
                                cerrar=lambda: cierres.append(True))
        with patch.object(prueba, 'cargar_sistema', return_value=sistema), \
                patch.object(prueba, 'preparar_camara', return_value=['cámara']), \
                patch.object(prueba, 'cargar_transporte', return_value=lambda p: video), \
                patch.object(prueba.subprocess, 'Popen'), \
                patch.dict(prueba.os.environ, {'DISPLAY': ':simulado'}), \
                patch.object(prueba.cv2, 'namedWindow'), patch.object(prueba.cv2, 'resizeWindow'), \
                patch.object(prueba.cv2, 'setMouseCallback') as callback, \
                patch.object(prueba.cv2, 'imshow') as mostrar, \
                patch.object(prueba.cv2, 'waitKeyEx', return_value=27), \
                patch.object(prueba.cv2, 'destroyAllWindows') as destruir, \
                patch('sys.stdout', new_callable=io.StringIO):
            codigo = prueba.main(['--ip', '10.87.179.100', '--dominio', '5'])
        self.assertEqual(codigo, 0)
        self.assertIsInstance(callback.call_args.args[1].__self__, EntradaObjetivo)
        self.assertEqual(mostrar.call_args.args[1].shape, (708, 960, 3))
        self.assertEqual(cierres, [True])
        destruir.assert_called_once()


class CargaFacialRealTests(unittest.TestCase):
    def test_criterio_y_modelos_locales_se_reutilizan_sin_recalibrar(self):
        carpeta = prueba.TAREA
        if not (carpeta / 'datos/galeria.npz').exists():
            self.skipTest('La galería personal no está en esta computadora')
        if not all((carpeta / 'modelos' / d['archivo']).is_file() for d in MODELOS.values()):
            self.skipTest('Modelos locales no preparados')
        sistema = prueba.cargar_sistema()
        resultado = prueba.ConsultaEnVivo(sistema).actualizar(np.zeros((320, 320, 3), np.uint8), 100, 101)
        self.assertEqual(resultado['nombre'], 'NO_CONCLUYENTE')
        self.assertEqual(resultado['motivo'], 'SIN_ROSTRO')


if __name__ == '__main__':
    unittest.main()
