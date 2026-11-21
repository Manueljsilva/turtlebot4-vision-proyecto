"""Bandera y QR aportados como PNG; ventanas y transporte simulados, sin robot."""

import io
import json
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

import cv2
import numpy as np

import prueba_meta_visual as app
from percepcion_meta_qr import DetectorMetaQR
from senales_giro import DetectorGiros


SENALES = Path(__file__).resolve().parents[2]/'señales'
REFERENCIAS = all((SENALES/n).is_file() for n in ('meta.png','qr1.png','qr6.png'))


@unittest.skipUnless(REFERENCIAS, 'Requiere los PNG locales de señales')
class ReferenciasPNGTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cv2.setNumThreads(1)

    def preview(self, nombre, alto):
        original = cv2.imread(str(SENALES/nombre))
        self.assertIsNotNone(original)
        im = cv2.resize(original, (round(alto*original.shape[1]/original.shape[0]),alto), interpolation=cv2.INTER_AREA)
        fondo = np.full((240,320,3),180,np.uint8)
        x,y = (320-im.shape[1])//2,(240-alto)//2
        fondo[y:y+alto,x:x+im.shape[1]]=im
        return fondo

    def test_bandera_preview_pequena_jpeg_desenfoque_y_luz(self):
        for alto in (100,140,180):
            im = self.preview('meta.png',alto)
            _,jpeg=cv2.imencode('.jpg',im,[cv2.IMWRITE_JPEG_QUALITY,45])
            casos=(im,cv2.imdecode(jpeg,cv2.IMREAD_COLOR),cv2.GaussianBlur(im,(3,3),.8),
                   cv2.convertScaleAbs(im,alpha=.65,beta=20))
            for i,caso in enumerate(casos):
                with self.subTest(alto=alto,variante=i):
                    r=DetectorMetaQR().detectar(caso,leer_qr=False)
                    self.assertIsNotNone(r['meta'])
                    x,y,w,h=r['meta']['caja']
                    self.assertTrue(0 <= x < x+w <= 320 and 0 <= y < y+h <= 240)
                    self.assertGreaterEqual(r['meta']['esquinas'],8)
                    if alto==100:
                        self.assertEqual(r['meta']['metodo'],'geometria_ampliada')

    def test_bandera_demasiado_pequena_no_fuerza_meta(self):
        self.assertIsNone(DetectorMetaQR().detectar(self.preview('meta.png',70),leer_qr=False)['meta'])

    def test_seis_qr_reales_legibles_y_nunca_son_meta_tampoco_en_preview(self):
        detector=DetectorMetaQR()
        for i in range(1,7):
            nombre=f'qr{i}.png'
            original=cv2.imread(str(SENALES/nombre))
            with self.subTest(nombre=nombre):
                r=detector.detectar(original,leer_qr=True)
                self.assertIsNone(r['meta'])
                self.assertEqual(len(r['codigos']),1)
                self.assertTrue(r['codigos'][0]['texto'])
            for alto in (70,100,140):
                im=self.preview(nombre,alto)
                for caso in (im,cv2.GaussianBlur(im,(3,3),.8)):
                    with self.subTest(nombre=nombre,alto=alto):
                        r=detector.detectar(caso,leer_qr=False)
                        self.assertIsNone(r['meta'])
                        self.assertTrue(all(q['texto'] is None for q in r['codigos']))

    def test_flechas_png_direccion_color_y_negativos_meta(self):
        giros,meta=DetectorGiros(),DetectorMetaQR()
        for prefijo,color in (('ida','azul'),('regreso','rojo')):
            for lado in ('derecha','izquierda'):
                nombre=f'{prefijo}_{lado}.png'
                im=cv2.imread(str(SENALES/nombre))
                with self.subTest(nombre=nombre):
                    self.assertEqual([(d['color'],d['direccion']) for d in giros.detectar(im)],[(color,lado.upper())])
                    self.assertIsNone(meta.detectar(im,leer_qr=False)['meta'])
        im=cv2.imread(str(SENALES/'sincamino.png'))
        self.assertEqual(giros.detectar(im),[])
        self.assertIsNone(meta.detectar(im,leer_qr=False)['meta'])


class VentanaTests(unittest.TestCase):
    def ejecutar_video(self,video,detector,teclas,reloj):
        with patch.object(app,'preparar_camara',return_value=['camara']), \
                patch.object(app,'cargar_transporte',return_value=lambda p:video), \
                patch.object(app.subprocess,'Popen'), \
                patch.object(app,'DetectorMetaQR',return_value=detector), \
                patch('prueba_facial_robot.cargar_sistema',side_effect=AssertionError('No cargar galeria')), \
                patch.dict(app.os.environ,{'DISPLAY':':simulado'}), \
                patch.object(app.time,'monotonic_ns',side_effect=lambda:reloj[0]), \
                patch.object(app.cv2,'namedWindow'),patch.object(app.cv2,'imshow') as mostrar, \
                patch.object(app.cv2,'waitKey',side_effect=teclas), \
                patch.object(app.cv2,'getWindowProperty',return_value=1), \
                patch.object(app.cv2,'destroyAllWindows') as cerrar, \
                patch('sys.stdout',new_callable=io.StringIO) as salida:
            r=app.main(['--ip','10.87.179.100','--dominio','5'])
        self.assertEqual(r,0,salida.getvalue())
        video.cerrar.assert_called_once()
        cerrar.assert_called_once()
        return mostrar,salida.getvalue()

    def test_espera_camara_indefinidamente_sin_cargar_galeria(self):
        video=NS(proceso=Mock(),fotograma=Mock(return_value=(None,'Esperando camara')),cerrar=Mock())
        video.proceso.poll.return_value=None
        detector=Mock()
        mostrar,salida=self.ejecutar_video(video,detector,[-1,-1,27],[5_000_000_000])
        self.assertEqual(mostrar.call_count,3)
        detector.detectar.assert_not_called()
        self.assertIn('Esperando camara',salida)

    def test_paquete_repetido_no_reinfiere_y_caducado_no_muestra_meta(self):
        reloj=[5_000_000_000]
        video=NS(proceso=Mock(),cerrar=Mock(),_decodificado={'recibido_ns':reloj[0]})
        video.proceso.poll.return_value=None
        im=np.full((240,320,3),180,np.uint8)
        video.fotograma=Mock(side_effect=[(im,'Camara del robot'),(im,'Camara del robot'),(None,'Sin imagen reciente')])
        detector=Mock()
        detector.detectar.return_value=dict(meta=dict(caja=[40,40,40,80],esquinas=10),codigos=[])
        def tecla(_):
            reloj[0]+=1_100_000_000 if video.fotograma.call_count==2 else 0
            return 27 if video.fotograma.call_count==3 else -1
        mostrar,salida=self.ejecutar_video(video,detector,tecla,reloj)
        detector.detectar.assert_called_once_with(im,leer_qr=False,buscar_meta=True)
        self.assertEqual(mostrar.call_count,3)
        self.assertIn('META VISTA',salida)
        self.assertIn('Sin imagen reciente',salida)

    def test_lanzador_delega_visual_y_rechaza_mover_o_mezclar_modos(self):
        from tests.test_lanzador_evasion import LanzadorEvasionTests
        caso=LanzadorEvasionTests()
        caso.setUp()
        self.addCleanup(caso.doCleanups)
        (caso.tarea/'prueba_meta_visual.py').write_text('import json,sys\nprint(json.dumps(sys.argv[1:]))\n')
        r=caso.ejecutar('--solo-meta','--topico-camara','/oakd/rgb/preview/image_raw/compressed')
        self.assertEqual(r.returncode,0,r.stderr)
        self.assertIn('/oakd/rgb/preview/image_raw/compressed',json.loads(r.stdout))
        self.assertFalse(caso.registro.exists())
        for args in (('--mover',),('--solo-evasion',),('--mision-guiada',),('--nombre','Persona A'),('--tramo','ida')):
            with self.subTest(args=args):
                r=caso.ejecutar('--solo-meta',*args)
                self.assertEqual(r.returncode,2,r.stderr)
                self.assertFalse(caso.registro.exists())


if __name__=='__main__':
    unittest.main()
