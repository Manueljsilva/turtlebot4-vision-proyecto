"""Ventanas con dobles: arranque tardío y confirmación posterior a Buscar."""

import io
from pathlib import Path
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

import numpy as np

import busqueda_robot as app
import prueba_senales_robot as prueba
from control_giros import ConfigSenales
from evasion_lidar import ConfigEvasion
from prueba_facial_robot import SistemaFacial
from reconocimiento_facial import Criterio, Galeria
from senales_giro import ConsultaSenales


class VentanasSenalesTests(unittest.TestCase):
    def test_prueba_individual_espera_camara_sin_cargar_galeria_y_cierra(self):
        video = NS(proceso=Mock(), fotograma=Mock(return_value=(None, 'Esperando camara')), cerrar=Mock())
        video.proceso.poll.return_value = None
        with patch.object(prueba, 'preparar_camara', return_value=['camara']) as preparar, \
                patch.object(prueba, 'cargar_transporte', return_value=lambda p: video), \
                patch.object(prueba.subprocess, 'Popen'), \
                patch('prueba_facial_robot.cargar_sistema', side_effect=AssertionError('No cargar galería')), \
                patch.dict(prueba.os.environ, {'DISPLAY': ':simulado'}), \
                patch.object(prueba.cv2, 'namedWindow'), patch.object(prueba.cv2, 'imshow') as mostrar, \
                patch.object(prueba.cv2, 'waitKey', side_effect=[-1, 27]), \
                patch.object(prueba.cv2, 'getWindowProperty', return_value=1), \
                patch.object(prueba.cv2, 'destroyAllWindows') as destruir, \
                patch('sys.stdout', new_callable=io.StringIO):
            resultado = prueba.main(['--ip', '10.87.179.100', '--dominio', '5', '--tramo', 'vuelta'])
        self.assertEqual(resultado, 0)
        self.assertEqual(mostrar.call_count, 2)
        self.assertEqual(mostrar.call_args.args[1].shape, (660, 960, 3))
        preparar.assert_called_once()
        video.cerrar.assert_called_once()
        destruir.assert_called_once()

    def test_buscar_descarta_votos_previos_y_comparte_imagen_sin_cambiar_el_detector_facial(self):
        sistema = SistemaFacial(Mock(), Galeria([[1, 0], [0, 1]], ['Persona A', 'Persona B']), Criterio(.5, .1))
        sistema.extractor.analizar_imagen.return_value = NS(estado='SIN_ROSTRO')
        reloj = [10_000_000_000]
        video = NS(proceso=Mock(), cerrar=Mock(), _decodificado={})
        video.proceso.poll.return_value = None
        imagen = np.zeros((240, 320, 3), dtype='uint8')
        cuadro = [0]
        def fotograma():
            cuadro[0] += 1
            reloj[0] += 100_000_000
            video._decodificado['recibido_ns'] = reloj[0]
            return imagen, 'Recibiendo'
        video.fotograma = fotograma
        puente = Mock()
        a = dict(x=1., y=2., yaw=0., frame_id='odom', child_frame_id='base_link')
        puente.ultimo_estado.side_effect = lambda ns: dict(a=a if cuadro[0] > 1 else None, estado='Buscando')
        detector = Mock()
        detector.detectar.return_value = [dict(color='azul', direccion='DERECHA', confianza=.9, caja=[50, 50, 80, 80])]
        def consulta(tramo, cfg):
            return ConsultaSenales(tramo, cfg, detector)
        callback = {}
        def tecla(_):
            if cuadro[0] == 1:
                callback['raton'].__self__.confirmar()
            return 27 if cuadro[0] >= 5 else -1
        with tempfile.TemporaryDirectory() as carpeta:
            temporal = Path(carpeta)
            (temporal/'datos').mkdir()
            with patch.object(app, 'TAREA', temporal), \
                    patch.object(app, 'cargar_config', return_value=ConfigEvasion()), \
                    patch.object(app, 'cargar_config_senales', return_value=ConfigSenales()), \
                    patch.object(app, 'cargar_sistema', return_value=sistema), \
                    patch.object(app, 'ConsultaSenales', side_effect=consulta), \
                    patch.object(app, 'preparar_camara', return_value=['camara']), \
                    patch.object(app, 'preparar_movimiento', return_value=['control']), \
                    patch.object(app, 'cargar_transporte', return_value=lambda p: video), \
                    patch.object(app, 'PuenteBusqueda', return_value=puente), \
                    patch.object(app.subprocess, 'Popen'), \
                    patch.object(app.time, 'monotonic_ns', side_effect=lambda: reloj[0]), \
                    patch.dict(app.os.environ, {'DISPLAY': ':simulado'}), \
                    patch.object(app.cv2, 'namedWindow'), patch.object(app.cv2, 'resizeWindow'), \
                    patch.object(app.cv2, 'setMouseCallback', side_effect=lambda w, cb: callback.update(raton=cb)), \
                    patch.object(app.cv2, 'imshow'), patch.object(app.cv2, 'waitKeyEx', side_effect=tecla), \
                    patch.object(app.cv2, 'getWindowProperty', return_value=1), \
                    patch.object(app.cv2, 'destroyAllWindows'), patch('sys.stdout', new_callable=io.StringIO):
                resultado = app.main(['--ip', '10.87.179.100', '--dominio', '5', '--senales', '--nombre', 'Persona A'])
            self.assertEqual(len(list((temporal/'datos/sesiones').glob('*.json'))), 1)
        self.assertEqual(resultado, 0)
        self.assertEqual([bool(c.args[3]) for c in puente.actualizar.call_args_list], [False, False, False, True, True])
        self.assertEqual(sistema.extractor.analizar_imagen.call_count, 5)
        self.assertTrue(np.all(imagen == 0), 'Las anotaciones no deben cambiar la imagen de inferencia')
        video.cerrar.assert_called_once()
        puente.cerrar.assert_called_once()


if __name__ == '__main__':
    unittest.main()
