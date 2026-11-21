"""Referencias del PDF y control sin ROS; no conecta a cámaras ni al robot."""

import json
from math import pi, radians
from pathlib import Path
import unittest
from unittest.mock import Mock

import cv2
import numpy as np

from control_busqueda import PuertaBusqueda
from control_giros import ConfigSenales, ControlGiro, cargar_config_senales
from evasion_lidar import ConfigEvasion, analizar_lidar
from senales_giro import ConsultaSenales, DetectorGiros


TAREA = Path(__file__).resolve().parents[1]
AHORA = 10_000_000_000
REFERENCIAS_DISPONIBLES = all((TAREA/f'tests/fixtures/giro_{c}_derecha.png').is_file()
                             for c in ('azul', 'rojo'))


def evento(direccion='DERECHA', color='azul', id=1, marca=AHORA):
    return dict(id=id, direccion=direccion, color=color, marca_ns=marca)


def deteccion(direccion='DERECHA', color='azul', caja=None):
    return dict(direccion=direccion, color=color, confianza=.9, caja=caja or [10, 10, 100, 100])


class DetectorTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cv2.setNumThreads(1)

    @unittest.skipUnless(REFERENCIAS_DISPONIBLES, 'Requiere los recortes locales de las señales')
    def test_referencias_pdf_ambos_colores_y_reflejos_izquierda(self):
        detector = DetectorGiros()
        for color in ('azul', 'rojo'):
            imagen = cv2.imread(str(TAREA/f'tests/fixtures/giro_{color}_derecha.png'))
            self.assertIsNotNone(imagen)
            for reflejo in (False, True):
                for factor in (1., .6):
                    with self.subTest(color=color, reflejo=reflejo, factor=factor):
                        im = cv2.flip(imagen, 1) if reflejo else imagen.copy()
                        im = cv2.resize(im, None, fx=factor, fy=factor)
                        # Iluminación más baja y artefactos JPEG, sin ajustar umbrales con estas consultas.
                        im = cv2.convertScaleAbs(im, alpha=.8)
                        _, jpeg = cv2.imencode('.jpg', im, [cv2.IMWRITE_JPEG_QUALITY, 80])
                        ds = detector.detectar(cv2.imdecode(jpeg, cv2.IMREAD_COLOR))
                        self.assertEqual([(d['color'], d['direccion']) for d in ds],
                                         [(color, 'IZQUIERDA' if reflejo else 'DERECHA')])

    def test_discos_vacios_prohibicion_meta_y_fondo_no_son_giros(self):
        detector = DetectorGiros()
        for color in ((255, 90, 0), (0, 0, 255), (255, 255, 255)):
            im = np.full((300, 300, 3), 255, np.uint8)
            cv2.circle(im, (150, 150), 105, (0, 0, 0), -1)
            cv2.circle(im, (150, 150), 88, color, -1)
            self.assertEqual(detector.detectar(im), [])
            if color == (255, 255, 255):
                cv2.line(im, (80, 80), (220, 220), (0, 0, 0), 20)
                self.assertEqual(detector.detectar(im), [])
        self.assertEqual(detector.detectar(np.zeros((300, 300, 3), np.uint8)), [])
        tablero = np.indices((300, 300)).sum(axis=0)//20 % 2 * 255
        self.assertEqual(detector.detectar(np.repeat(tablero[:, :, None], 3, axis=2).astype('uint8')), [])

    @unittest.skipUnless(REFERENCIAS_DISPONIBLES, 'Requiere los recortes locales de las señales')
    def test_preview_250_jpeg_no_invierte_flechas_pequenas_de_ambos_colores(self):
        detector=DetectorGiros()
        for color in ('azul','rojo'):
            original=cv2.imread(str(TAREA/f'tests/fixtures/giro_{color}_derecha.png'))
            for reflejo in (False,True):
                im=cv2.flip(original,1) if reflejo else original
                for lado in (65,80,100,125,155,190):
                    r=cv2.resize(im,None,fx=lado/max(im.shape[:2]),fy=lado/max(im.shape[:2]),interpolation=cv2.INTER_AREA)
                    frame=np.full((250,250,3),220,np.uint8)
                    y,x=(250-r.shape[0])//2,(250-r.shape[1])//2
                    frame[y:y+r.shape[0],x:x+r.shape[1]]=r
                    for blur in (0,3):
                        f=cv2.GaussianBlur(frame,(blur,blur),.8) if blur else frame
                        for calidad in (45,80):
                            with self.subTest(color=color,reflejo=reflejo,lado=lado,blur=blur,calidad=calidad):
                                _,jpg=cv2.imencode('.jpg',f,[cv2.IMWRITE_JPEG_QUALITY,calidad])
                                ds=detector.detectar(cv2.imdecode(jpg,cv2.IMREAD_COLOR))
                                self.assertEqual([(d['color'],d['direccion']) for d in ds],
                                    [(color,'IZQUIERDA' if reflejo else 'DERECHA')])

    @unittest.skipUnless(REFERENCIAS_DISPONIBLES, 'Requiere los recortes locales de las señales')
    def test_geometria_de_giro_admite_inclinacion_moderada_sin_espejo_de_camara(self):
        detector=DetectorGiros()
        for color in ('azul','rojo'):
            original=cv2.imread(str(TAREA/f'tests/fixtures/giro_{color}_derecha.png'))
            for reflejo in (False,True):
                im=cv2.flip(original,1) if reflejo else original
                r=cv2.resize(im,None,fx=155/max(im.shape[:2]),fy=155/max(im.shape[:2]),interpolation=cv2.INTER_AREA)
                frame=np.full((250,250,3),220,np.uint8)
                y,x=(250-r.shape[0])//2,(250-r.shape[1])//2
                frame[y:y+r.shape[0],x:x+r.shape[1]]=r
                for angulo in (-20,0,20):
                    with self.subTest(color=color,reflejo=reflejo,angulo=angulo):
                        rotada=cv2.warpAffine(frame,cv2.getRotationMatrix2D((125,125),angulo,1),
                                             (250,250),borderValue=(220,220,220))
                        self.assertEqual([(d['color'],d['direccion']) for d in detector.detectar(rotada)],
                            [(color,'IZQUIERDA' if reflejo else 'DERECHA')])

    @unittest.skipUnless(REFERENCIAS_DISPONIBLES, 'Requiere los recortes locales de las señales')
    def test_no_admite_cartel_cortado_ni_entrada_no_bgr(self):
        imagen = cv2.imread(str(TAREA/'tests/fixtures/giro_azul_derecha.png'))
        self.assertEqual(DetectorGiros().detectar(imagen[:, 80:]), [])
        with self.assertRaises(ValueError):
            DetectorGiros().detectar(np.zeros((100, 100), np.uint8))


class ConfirmacionTests(unittest.TestCase):
    def setUp(self):
        self.detector = Mock()
        self.detector.detectar.return_value = [deteccion()]
        self.q = ConsultaSenales(detector=self.detector)
        self.im = np.zeros((1, 1, 3), np.uint8)  # El doble no extrae geometría.

    def paso(self, ns, ds=None):
        if ds is not None:
            self.detector.detectar.return_value = ds
        return self.q.actualizar(self.im, ns, ns)

    def test_primera_imagen_valida_emite_evento_sin_recontar_repeticiones(self):
        self.assertEqual(self.paso(AHORA)['evento'],evento())
        for _ in range(6):
            self.assertEqual(self.paso(AHORA)['confirmaciones'], 1)
        self.assertEqual(self.detector.detectar.call_count, 1)
        self.assertEqual(self.paso(AHORA+100_000_000)['evento']['id'],1)
        self.assertEqual(self.paso(AHORA+200_000_000)['evento'], evento(marca=AHORA+200_000_000))
        self.assertEqual(self.paso(AHORA+300_000_000)['evento']['id'], 1)

    def test_ignora_color_del_otro_tramo_y_rechaza_ambiguedad(self):
        for i in range(5):
            self.assertIsNone(self.paso(AHORA+i, [deteccion(color='rojo')])['evento'])
        for i in range(5):
            r = self.paso(AHORA+10+i, [deteccion(), deteccion('IZQUIERDA')])
            self.assertIsNone(r['evento'])
            self.assertIn('ambiguo', r['estado'])
        q = ConsultaSenales('vuelta', detector=self.detector)
        self.detector.detectar.return_value = [deteccion(color='rojo')]
        for i in range(3):
            r = q.actualizar(self.im, AHORA+i, AHORA+i)
        self.assertEqual(r['evento']['color'], 'rojo')

    def test_caducidad_cambio_de_direccion_y_otro_cartel_reinician(self):
        self.paso(AHORA)
        self.paso(AHORA+1)
        self.assertEqual(self.paso(AHORA+2, [deteccion('IZQUIERDA')])['evento']['direccion'],'IZQUIERDA')
        self.assertEqual(self.paso(AHORA+3, [deteccion(caja=[400, 10, 100, 100])])['confirmaciones'], 1)
        r = self.q.actualizar(self.im, AHORA+3, AHORA+1_000_000_004)
        self.assertIsNone(r['evento'])
        self.assertEqual(self.paso(AHORA+1_100_000_000)['confirmaciones'], 1)

    def test_rearme_exige_ausencia_en_imagenes_nuevas(self):
        for i in range(3):
            self.paso(AHORA+i)
        self.paso(AHORA+100_000_000, [])
        for i in range(1, 6):
            self.paso(AHORA+100_000_000+i*350_000_000, [])
        for i in range(3):
            r = self.paso(AHORA+2_000_000_000+i, [deteccion()])
        self.assertEqual(r['evento']['id'], 2)


class GiroTests(unittest.TestCase):
    def setUp(self):
        self.cfg = ConfigEvasion()
        self.pose = dict(x=0., y=0., yaw=0.)
        self.lidar = analizar_lidar([1.]*360, -pi, 2*pi/360, .05, 12.)

    def decidir(self, control, ev=None, ahora=AHORA, permitido=True, lidar='normal'):
        return control.decidir(ev, self.pose, ahora, permitido,
                              self.lidar if lidar == 'normal' else lidar, self.cfg)

    def test_ambos_sentidos_terminan_por_angulo_y_no_por_tiempo_fijo(self):
        for direccion, signo in (('IZQUIERDA', 1), ('DERECHA', -1)):
            with self.subTest(direccion=direccion):
                c = ControlGiro('ida')
                self.pose['yaw'] = 0.
                self.assertEqual(self.decidir(c, evento(direccion))[:2], (0., signo*.4))
                self.pose['yaw'] = signo*radians(60)
                self.assertEqual(self.decidir(c, ahora=AHORA+2_000_000_000)[1]*signo, .4)
                self.pose['yaw'] = signo*radians(87)
                incompleto = self.decidir(c, ahora=AHORA+4_000_000_000)
                self.assertEqual(incompleto[0], 0.)
                self.assertGreater(incompleto[1]*signo, 0.)
                self.assertIsNotNone(c.activo)
                self.pose['yaw'] = signo*radians(90)
                self.assertEqual(self.decidir(c, ahora=AHORA+4_500_000_000)[:2], (0., 0.))
                self.assertIsNone(c.activo)
                self.assertIsNone(self.decidir(c, evento(direccion)))

    def test_yaw_cruza_pi_sin_invertir_el_giro(self):
        c = ControlGiro('ida')
        self.pose['yaw'] = radians(170)
        self.decidir(c, evento('IZQUIERDA'))
        self.pose['yaw'] = radians(-100)
        self.assertEqual(self.decidir(c)[:2], (0., 0.))
        self.assertIsNone(c.activo)

    def test_exceso_de_angulo_enclava_parada_en_vez_de_reanudar(self):
        c = ControlGiro('ida')
        self.decidir(c, evento())
        self.pose['yaw'] = -radians(96)
        self.assertEqual(self.decidir(c)[:2], (0., 0.))
        self.assertIn('excedido', c.fallo)
        self.assertEqual(self.decidir(c)[:2], (0., 0.))

    def test_falta_de_sensores_o_permiso_y_obstaculo_no_invierten_la_senal(self):
        c = ControlGiro('ida')
        self.decidir(c, evento())
        self.assertEqual(self.decidir(c, permitido=False)[:2], (0., 0.))
        self.assertEqual(self.decidir(c, lidar=None)[:2], (0., 0.))
        cerca = analizar_lidar([.25]*360, -pi, 2*pi/360, .05, 12.)
        r = self.decidir(c, lidar=cerca)
        self.assertEqual(r[:2], (0., 0.))
        self.assertIn('bloqueado', r[2])
        self.assertLess(self.decidir(c)[1], 0)

    def test_timeout_enclava_parada_y_colores_no_se_mezclan(self):
        c = ControlGiro('ida')
        self.assertIsNone(self.decidir(c, evento(color='rojo')))
        self.decidir(c, evento(id=2))
        self.assertEqual(self.decidir(c, ahora=AHORA+13_000_000_000)[:2], (0., 0.))
        self.assertIsNotNone(c.fallo)
        self.assertEqual(self.decidir(c, evento(id=3), ahora=AHORA+14_000_000_000)[:2], (0., 0.))

    def test_evento_reaparecido_necesita_desplazamiento_y_nuevo_id(self):
        c = ControlGiro('ida')
        self.decidir(c, evento())
        self.pose['yaw'] = -pi/2
        self.decidir(c)
        self.assertIsNone(self.decidir(c, evento(id=2)))
        self.pose['x'] = .5
        self.assertIsNone(self.decidir(c, evento(id=2)))
        self.assertLess(self.decidir(c, evento(id=3))[1], 0)

    def test_protocolo_rechaza_eventos_caducados_futuros_y_malformados(self):
        for ev in (evento(marca=AHORA-1_000_000_001), evento(marca=AHORA+1),
                   evento(id=True), dict(evento(), velocidad=1), evento('STOP')):
            p = PuertaBusqueda()
            d = dict(version=1, secuencia=0, orden='BUSCAR', actividad_ns=AHORA, camara_ns=AHORA, senal=ev)
            p.recibir(json.dumps(d), AHORA)
            self.assertEqual(p.orden, 'PARAR')
        p = PuertaBusqueda()
        p.recibir(json.dumps(dict(version=1, secuencia=0, orden='BUSCAR', actividad_ns=AHORA,
                                 camara_ns=AHORA, senal=evento())), AHORA)
        self.assertTrue(p.senales_activas)
        self.assertEqual(p.senal['direccion'], 'DERECHA')

    def test_configuracion_real_y_limites(self):
        self.assertEqual(cargar_config_senales(TAREA/'config/senales_giro.json').angulo_giro_deg, 90)
        self.assertEqual(cargar_config_senales(TAREA/'config/senales_giro.json').confirmaciones,1)
        for d in (dict(confirmaciones=0), dict(confirmaciones=True), dict(angulo_giro_deg=360), dict(ancho=True),
                  dict(tiempo_maximo_giro_s=float('nan'))):
            with self.assertRaises(ValueError):
                ConfigSenales(**d)


if __name__ == '__main__':
    unittest.main()
