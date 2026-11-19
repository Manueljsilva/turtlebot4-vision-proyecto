"""Percepción/estados del ensayo B; referencias del PDF y ROS simulado, sin hardware."""

import contextlib
import importlib.util
import io
import json
from math import atan2, cos, sin
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import threading
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

import cv2
import numpy as np

import prueba_meta_robot as app
from control_meta import ControlBarrido, ReposoOdometria, SesionPruebaB
from evasion_lidar import ConfigEvasion
from percepcion_meta_qr import (DetectorMetaQR, analizar_candidatos, asociar_qr,
                                detectar_regiones_fotos, texto_valido)
from prueba_facial_robot import ConsultaEnVivo, SistemaFacial
from reconocimiento_facial import Criterio, Galeria
from tests.test_busqueda_robot import cargar_ros_simulado


def cara(nombre='Persona A', x=0, aceptado=True):
    return dict(caja=[x, 0, 100, 100], aceptado=aceptado, nombre=nombre)


META = dict(meta=dict(caja=[20, 20, 100, 180]), codigos=[])
LIBRE = NS(observacion_valida=True, proximidad_no_cuantificable=False,
           peligro_parada=False, precaucion=False,
           dist_frente=1., dist_izquierda=1., dist_derecha=1.)


class AsociacionTests(unittest.TestCase):
    def region(self, x0=20, y0=20, x1=360, y1=380):
        return dict(puntos=[[x0,y0],[x1,y0],[x1,y1],[x0,y1]], caja=[x0,y0,x1-x0,y1-y0])

    def test_elige_solo_qr_del_objetivo_entre_otras_fotos(self):
        caras = [cara(), cara('Persona B', 500)]
        codigos = [dict(caja=[620, 20, 60, 60], texto='OTRO'),
                   dict(caja=[120, 20, 60, 60], texto='OBJETIVO')]
        resultado, _ = asociar_qr(caras, codigos, 'Persona A')
        self.assertEqual(resultado, dict(texto='OBJETIVO', cara=0, qr=1))

    def test_rechaza_otro_rostro_desconocido_duplicado_y_qr_compartido(self):
        q = [dict(caja=[120, 20, 60, 60], texto='PALABRA')]
        for caras in ([cara('Persona B')], [cara(aceptado=False)], [cara(), cara(x=500)],
                      [cara(), cara('Persona B', 200)]):
            self.assertIsNone(asociar_qr(caras, q, 'Persona A')[0])

    def test_rechaza_dos_qr_equivalentes_o_sin_decodificar(self):
        for codigos in ([dict(caja=[120, 20, 60, 60], texto=None)],
                        [dict(caja=[120, 20, 60, 60], texto='UNO'),
                         dict(caja=[-80, 20, 60, 60], texto='DOS')]):
            self.assertIsNone(asociar_qr([cara()], codigos, 'Persona A')[0])

    def test_texto_es_dato_literal_sin_controles_ni_limite_excedido(self):
        for texto in ('', '   ', 'a\nb', '\x1b[31m', 'a'*257, None):
            self.assertFalse(texto_valido(texto))
        self.assertTrue(texto_valido('árbol'))
        self.assertTrue(texto_valido('$(no_ejecutar)'))

    def test_varias_caras_se_extraen_en_recortes_individuales(self):
        extractor = NS(lado_maximo=960, cara_minima=32, detector=NS())
        extractor.detector.setInputSize = lambda *a: None
        extractor.detector.detect = lambda im: (None, np.array([[20, 20, 60, 60], [180, 20, 60, 60]]))
        recortes = []
        def analizar(im):
            recortes.append(im.shape)
            return NS(estado='OK', descriptor=np.array([1., 0.]))
        extractor.analizar_imagen = analizar
        sistema = SistemaFacial(extractor, Galeria([[1, 0], [0, 1]], ['Persona A', 'Persona B']), Criterio(.5, .1))
        candidatos = analizar_candidatos(np.zeros((120, 300, 3), np.uint8), sistema)
        self.assertEqual(len(candidatos), 2)
        self.assertEqual(len(recortes), 2)
        self.assertTrue(all(w < 120 for _, w, _ in recortes))
        self.assertTrue(all(c['nombre'] == 'Persona A' and c['aceptado'] for c in candidatos))

    def test_qr_pequeno_en_esquina_se_asocia_solo_con_foto_delimitada(self):
        c = dict(caja=[100,70,100,100], aceptado=True, nombre='Persona A')
        q = dict(caja=[300,320,18,18], texto='ESQUINA')
        self.assertIsNone(asociar_qr([c], [q], 'Persona A')[0])
        resultado, motivo = asociar_qr([c], [q], 'Persona A', [self.region()])
        self.assertEqual(resultado['texto'], 'ESQUINA')
        self.assertIn('misma foto', motivo)

    def test_foto_propia_prevalece_sobre_qr_de_otra_foto_mas_cercano(self):
        caras = [dict(caja=[100,70,100,100], aceptado=True, nombre='Persona A'),
                 dict(caja=[450,70,100,100], aceptado=True, nombre='Persona B')]
        codigos = [dict(caja=[365,95,35,35], texto='AJENO'),
                   dict(caja=[300,320,30,30], texto='PROPIO')]
        regiones = [self.region(), self.region(360,20,700,380)]
        resultado, _ = asociar_qr(caras, codigos, 'Persona A', regiones)
        self.assertEqual(resultado['texto'], 'PROPIO')
        self.assertEqual(resultado['qr'], 1)

    def test_panel_compartido_no_atribuye_esquina_a_una_cara(self):
        caras = [dict(caja=[70,70,80,100], aceptado=True, nombre='Persona A'),
                 dict(caja=[200,70,80,100], aceptado=True, nombre='Persona B')]
        codigos = [dict(caja=[300,320,30,30], texto='AMBIGUO')]
        self.assertIsNone(asociar_qr(caras, codigos, 'Persona A', [self.region()])[0])
        caras = caras[:1]
        codigos.append(dict(caja=[30,320,30,30], texto='OTRO'))
        self.assertIsNone(asociar_qr(caras, codigos, 'Persona A', [self.region()])[0])
        # Dos contornos solapados atribuyen el mismo código a fotos distintas:
        # no resolver esa contradicción por distancia.
        caras = [dict(caja=[100,70,100,100], aceptado=True, nombre='Persona A'),
                 dict(caja=[540,70,100,100], aceptado=True, nombre='Persona B')]
        q = [dict(caja=[300,320,30,30], texto='COMPARTIDO')]
        regiones = [self.region(), self.region(250,20,700,380)]
        self.assertIsNone(asociar_qr(caras, q, 'Persona A', regiones)[0])

    def test_contorno_de_papel_se_detecta_sin_capturas_personales(self):
        im = np.full((420,780,3), 45, np.uint8)
        cv2.rectangle(im,(20,20),(360,380),(245,245,245),-1)
        cv2.rectangle(im,(100,70),(200,170),(95,95,95),-1)
        regiones = detectar_regiones_fotos(im)
        c = dict(caja=[110,80,80,80], aceptado=True, nombre='Persona A')
        q = dict(caja=[300,320,25,25], texto='PAPEL')
        resultado, _ = asociar_qr([c], [q], 'Persona A', regiones)
        self.assertEqual(resultado['texto'], 'PAPEL')


class DetectorTests(unittest.TestCase):
    def test_qr_generado_se_lee_y_no_es_meta(self):
        qr = cv2.QRCodeEncoder_create().encode('PALABRA')
        qr = cv2.copyMakeBorder(qr, 4, 4, 4, 4, cv2.BORDER_CONSTANT, value=255)
        qr = cv2.resize(qr, (300, 300), interpolation=cv2.INTER_NEAREST)
        resultado = DetectorMetaQR().detectar(cv2.cvtColor(qr, cv2.COLOR_GRAY2BGR), True)
        self.assertIsNone(resultado['meta'])
        self.assertEqual(resultado['codigos'][0]['texto'], 'PALABRA')

    def test_recorte_con_margen_recupera_qr_pequeno_si_falla_decode_multi(self):
        qr = cv2.QRCodeEncoder_create().encode('PALABRA')
        qr = cv2.copyMakeBorder(qr, 4, 4, 4, 4, cv2.BORDER_CONSTANT, value=255)
        im = cv2.cvtColor(cv2.resize(qr, (64,64), interpolation=cv2.INTER_NEAREST), cv2.COLOR_GRAY2BGR)
        detector = DetectorMetaQR()
        original = detector.qr
        detector.qr = NS(detectMulti=original.detectMulti, detect=original.detect,
                         decodeMulti=lambda im,p:(False,(),None))
        with patch.object(detector, '_leer_recorte', wraps=detector._leer_recorte) as leer:
            resultado = detector.detectar(im, leer_qr=True, buscar_meta=False)
        self.assertEqual(resultado['codigos'][0]['texto'], 'PALABRA')
        leer.assert_called_once()

    def test_reintentos_acotados_y_nunca_reintenta_antes_de_buscar_qr(self):
        detector = DetectorMetaQR()
        puntos = np.array([[[x,20],[x+30,20],[x+30,50],[x,50]] for x in (10,70,130,190)], np.float32)
        detector.qr = NS(detectMulti=lambda im:(True,puntos), decodeMulti=lambda im,p:(False,(),None))
        im = np.full((100,260,3),255,np.uint8)
        with patch.object(detector, '_leer_recorte', return_value=None) as leer:
            detector.detectar(im, leer_qr=False, buscar_meta=False)
            leer.assert_not_called()
            detector.detectar(im, leer_qr=True, buscar_meta=False, caja_objetivo=[150,20,30,30])
            self.assertEqual(leer.call_count, 2)
        with patch.object(detector, 'qr_recorte') as recorte:
            self.assertIsNone(detector._leer_recorte(np.zeros((40,40),np.uint8), [[1,1],[5,1],[5,5],[1,5]]))
            recorte.detectAndDecode.assert_not_called()

    @unittest.skipUnless(shutil.which('pdftoppm'), 'Referencia PDF requiere Poppler')
    def test_bandera_real_positiva_y_las_tres_paginas_qr_negativas(self):
        pdf = app.TAREA.parent/'señales/señales.pdf'
        with tempfile.TemporaryDirectory() as tmp:
            subprocess.run(['pdftoppm', '-f', '7', '-l', '10', '-scale-to', '850', '-png',
                            str(pdf), str(Path(tmp)/'p')], check=True, capture_output=True)
            detector = DetectorMetaQR()
            resultados = [detector.detectar(cv2.imread(str(p)), True) for p in sorted(Path(tmp).glob('*.png'))]
        self.assertIsNotNone(resultados[0]['meta'])
        self.assertTrue(all(r['meta'] is None for r in resultados[1:]))
        self.assertTrue(all(r['codigos'] for r in resultados[1:]))


class SesionTests(unittest.TestCase):
    def setUp(self):
        self.s = SesionPruebaB()
        self.s.iniciar('Persona A', 1_000_000_000)
        self.t = 1_000_000_000

    def frame(self, meta=META, caras=(), texto=None, marca=None):
        self.t += 100_000_000
        marca = self.t if marca is None else marca
        self.s.actualizar(meta, caras, dict(texto=texto) if texto else None, 'QR', marca, self.t)

    def confirmar_meta(self):
        for _ in range(3):
            self.frame()
        self.s.habilitar_busqueda(self.t)

    def test_meta_necesita_tres_nuevas_y_rostro_previo_no_finaliza(self):
        self.frame(caras=[cara()], texto='ANTES')
        self.frame(caras=[cara()], texto='ANTES', marca=self.t)
        self.assertEqual(self.s.votos_meta, 1)
        self.assertEqual(self.s.orden, 'ESPERAR_META')
        self.frame(caras=[cara()], texto='ANTES')
        self.frame(caras=[cara()], texto='ANTES')
        self.assertTrue(self.s.meta_confirmada)
        self.assertIsNone(self.s.palabra)
        self.assertEqual(self.s.orden, 'BARRER')

    def test_ausencia_cambio_y_caducidad_reinician_meta(self):
        self.frame()
        self.frame(dict(meta=None))
        self.assertEqual(self.s.votos_meta, 0)
        self.frame()
        self.frame(dict(meta=dict(caja=[500, 20, 100, 180])))
        self.assertEqual(self.s.votos_meta, 1)
        self.frame(marca=0)
        self.assertEqual(self.s.votos_meta, 0)

    def test_pausa_en_primer_rostro_sin_qr_y_confirma_mismo_texto_tres_veces(self):
        self.confirmar_meta()
        self.frame(caras=[cara()])
        self.assertEqual(self.s.orden, 'VERIFICAR')
        self.frame(caras=[cara()], texto='UNO')
        self.frame(caras=[cara()], texto='DOS')
        self.assertEqual(self.s.confirmaciones, 1)
        self.frame(caras=[cara()], texto='DOS')
        self.frame(caras=[cara()], texto='DOS')
        self.assertEqual((self.s.orden, self.s.palabra), ('QR_LISTO', 'DOS'))
        self.frame(caras=[cara('Persona B')], texto='OTRO')
        self.assertEqual(self.s.palabra, 'DOS')

    def test_otro_rostro_repeticion_o_cambio_de_foto_no_suman_votos(self):
        self.confirmar_meta()
        self.frame(caras=[cara('Persona B')], texto='OTRO')
        self.assertEqual(self.s.orden, 'BARRER')
        self.frame(caras=[cara()], texto='UNO')
        self.frame(caras=[cara()], texto='UNO', marca=self.t)
        self.assertEqual(self.s.confirmaciones, 1)
        self.frame(caras=[cara(x=200)], texto='UNO')
        self.assertEqual(self.s.confirmaciones, 1)

    def test_nombre_fijo_y_resultado_json_local_sin_imagenes(self):
        with self.assertRaises(ValueError):
            self.s.iniciar('Persona B', self.t)
        self.confirmar_meta()
        for _ in range(3):
            self.frame(caras=[cara()], texto='árbol')
        with tempfile.TemporaryDirectory() as tmp:
            p = app.guardar_resultado(self.s, Path(tmp))
            datos = json.loads(p.read_text())
            self.assertEqual(datos['palabra_qr'], 'árbol')
            self.assertFalse(datos['regreso_a_confirmado'])
            self.assertEqual(len(list(Path(tmp).iterdir())), 1)
            self.assertNotIn('punto_a', datos)


class BarridoTests(unittest.TestCase):
    def setUp(self):
        self.c = ControlBarrido()
        self.t, self.seq = 2_000_000_000, 0
        self.cfg = ConfigEvasion()
        self.pose = dict(x=1., y=2., yaw=0., frame_id='odom', child_frame_id='base_link')

    def orden(self, orden='BARRER', camara=True):
        self.c.recibir(json.dumps(dict(version=1, secuencia=self.seq, orden=orden,
                       actividad_ns=self.t, camara_ns=self.t if camara else None)), self.t)
        self.seq += 1

    def decidir(self, **kw):
        valores = dict(ahora=self.t, pose=self.pose, pose_ns=self.t, lidar=LIBRE, cfg=self.cfg)
        valores.update(kw)
        return self.c.decidir(**valores)

    def arrancar_barrido(self):
        for _ in range(9):
            self.t += 100_000_000
            self.orden()
            self.decidir()
        self.assertIsNotNone(self.c.busqueda_desde_ns)

    def test_nunca_avanza_y_meta_camara_pose_lidar_son_obligatorios(self):
        self.assertEqual(self.decidir()[:2], (0, 0))
        for orden in ('ESPERAR', 'ESPERAR_META', 'VERIFICAR'):
            self.orden(orden)
            self.assertEqual(self.decidir()[:2], (0, 0))
        self.orden(camara=False)
        self.assertEqual(self.decidir()[:2], (0, 0))
        self.orden()
        for kw in (dict(pose=None), dict(pose_ns=self.t-700_000_000), dict(lidar=None),
                   dict(lidar=NS(**dict(vars(LIBRE), dist_derecha=.2)))):
            self.assertEqual(self.decidir(**kw)[:2], (0, 0))
        self.arrancar_barrido()
        self.assertEqual(self.decidir()[:2], (0, .25))
        self.assertFalse(hasattr(self.c, 'punto_a'))

    def test_actividad_caducada_se_detiene_aunque_llegue_latido(self):
        self.arrancar_barrido()
        self.assertGreater(self.decidir()[1], 0)
        self.t += 600_000_000
        self.assertEqual(self.decidir()[:2], (0, 0))

    def test_yaw_cruza_pi_y_termina_una_sola_vuelta(self):
        self.arrancar_barrido()
        for i in range(65):
            self.t += 100_000_000
            self.pose['yaw'] = atan2(sin(i*.1), cos(i*.1))
            self.orden()
            vx, wz, _ = self.decidir()
            self.assertEqual(vx, 0)
        self.assertTrue(self.c.terminado)
        self.assertAlmostEqual(self.c.angulo, 6.3)
        self.assertEqual(wz, 0)

    def test_timeout_eof_cambio_marco_y_discontinuidad_enclavan(self):
        for fallo in ('timeout', 'eof', 'frame', 'salto'):
            self.setUp()
            self.orden()
            self.decidir()
            self.arrancar_barrido()
            self.t += 91_000_000_000 if fallo == 'timeout' else 100_000_000
            self.orden()
            if fallo == 'frame':
                self.pose['frame_id'] = 'otro'
            elif fallo == 'salto':
                self.pose['yaw'] = 2.
            self.assertEqual(self.decidir(fin=fallo == 'eof')[:2], (0, 0))
            self.assertTrue(self.c.terminado)
            self.orden()
            self.assertEqual(self.decidir()[:2], (0, 0))

    def test_control_malformado_o_repetido_revoca_permiso(self):
        for dato in ('{}', 'null', 'no-json', 'x'*2049):
            self.orden()
            self.c.recibir(dato, self.t)
            self.assertEqual(self.decidir()[:2], (0, 0))
        self.orden('QR_LISTO')
        self.orden()
        self.assertEqual(self.decidir()[:2], (0, 0))

    def test_pausa_de_camara_no_inventa_un_corte_de_odometria(self):
        self.arrancar_barrido()
        for _ in range(12):
            self.t += 100_000_000
            self.orden(camara=False)
            self.assertEqual(self.decidir()[:2], (0., 0.))
        self.orden()
        self.assertEqual(self.decidir()[:2], (0., .25))
        self.assertFalse(self.c.terminado)

    def test_pausa_de_inferencia_mantiene_odometria_y_reanuda_solo_con_actividad(self):
        self.arrancar_barrido()
        for _ in range(12):
            self.t += 100_000_000
            self.decidir()  # Odometría sigue llegando, GUI no renueva marcas.
        self.assertFalse(self.c.terminado)
        self.assertEqual(self.decidir()[:2], (0., 0.))
        self.orden()
        self.assertEqual(self.decidir()[:2], (0., .25))

    def test_primer_motivo_de_fallo_no_se_sustituye_por_parar(self):
        self.c.finalizar('Odometria discontinua: cerrar y revisar')
        self.orden('PARAR')
        self.assertIn('Odometria discontinua', self.decidir()[2])

    def test_avance_explicito_y_frenado_impiden_reanudar_navegacion_tras_meta(self):
        self.orden('NAVEGAR')
        self.assertEqual(self.decidir()[:2], (0., 0.))
        self.c = ControlBarrido(avanzar=True)
        self.orden('NAVEGAR')
        self.assertEqual(self.decidir()[:2], (.1, 0.))
        self.t += 100_000_000
        self.orden('BARRER')
        self.assertEqual(self.decidir()[:2], (0., 0.))
        self.t += 100_000_000
        self.orden('NAVEGAR')
        self.assertEqual(self.decidir()[:2], (0., 0.))
        self.arrancar_barrido()
        self.assertEqual(self.decidir()[:2], (0., .25))

    def test_no_gira_hasta_frenar_segun_odometria_y_esperar_07_s(self):
        for _ in range(12):
            self.t += 100_000_000
            self.pose['x'] += .01
            self.orden()
            self.assertEqual(self.decidir()[:2], (0., 0.))
            self.assertIsNone(self.c.busqueda_desde_ns)
        self.arrancar_barrido()
        self.assertGreater(self.decidir()[1], 0)

    def test_navegacion_respeta_lidar_camara_odometria_y_actividad(self):
        self.c = ControlBarrido(avanzar=True)
        self.orden('NAVEGAR')
        self.assertGreater(self.decidir()[0], 0)
        for datos in (dict(lidar=None), dict(pose=None), dict(pose_ns=0),
                      dict(lidar=NS(**dict(vars(LIBRE), observacion_valida=False)))):
            self.assertEqual(self.decidir(**datos)[:2], (0., 0.))
        self.orden('NAVEGAR', camara=False)
        self.assertEqual(self.decidir()[:2], (0., 0.))

    def test_corte_rx_en_navegacion_para_recupera_reposo_y_permite_meta_y_barrido(self):
        self.c = ControlBarrido(avanzar=True)
        self.orden('NAVEGAR')
        self.assertGreater(self.decidir()[0], 0.)
        vieja = self.t
        self.t += 900_000_000
        self.orden('NAVEGAR')
        self.assertEqual(self.decidir(pose_ns=vieja)[:2], (0., 0.))
        self.assertFalse(self.c.terminado)
        # Llega un paquete nuevo tras 0.9 s; no es un salto de posición/yaw.
        self.assertEqual(self.decidir()[:2], (0., 0.))
        self.assertTrue(self.c.recuperando_odom)
        self.assertIn('Recuperando', self.decidir()[2])
        for _ in range(6):
            self.t += 100_000_000
            self.orden('NAVEGAR')
            self.assertEqual(self.decidir()[:2], (0., 0.))
        for _ in range(2):
            self.t += 100_000_000
            self.orden('NAVEGAR')
            salida = self.decidir()
        self.assertGreater(salida[0], 0.)
        self.assertFalse(self.c.recuperando_odom)
        self.assertFalse(self.c.terminado)
        self.arrancar_barrido()
        self.assertEqual(self.decidir()[:2], (0., .25))

    def test_reposo_no_se_confirma_repitiendo_el_mismo_paquete(self):
        r = ReposoOdometria()
        marca = self.t
        for i in range(12):
            self.assertFalse(r.actualizar(self.pose, marca, self.t+i*100_000_000))

    def test_recuperacion_no_se_reanuda_con_movimiento_o_cortes_repetidos(self):
        self.c = ControlBarrido(avanzar=True)
        self.orden('NAVEGAR'); self.decidir()
        self.t += 900_000_000
        self.orden('NAVEGAR'); self.decidir()
        for _ in range(12):
            self.t += 100_000_000
            self.pose['x'] += .01
            self.orden('NAVEGAR')
            self.assertEqual(self.decidir()[:2], (0., 0.))
        self.assertTrue(self.c.recuperando_odom)
        self.t += 800_000_000
        self.orden('NAVEGAR')
        self.assertEqual(self.decidir()[:2], (0., 0.))
        self.assertTrue(self.c.recuperando_odom)

    def test_corte_corto_durante_barrido_conserva_angulo_y_corte_largo_enclava(self):
        self.arrancar_barrido()
        self.t += 900_000_000
        self.pose['yaw'] += .1
        self.orden()
        self.assertEqual(self.decidir()[:2], (0., 0.))
        self.assertFalse(self.c.terminado)
        self.assertAlmostEqual(self.c.angulo, .1)
        self.arrancar_barrido()
        self.assertEqual(self.decidir()[:2], (0., .25))
        self.t += 2_100_000_000
        self.orden()
        self.assertEqual(self.decidir()[:2], (0., 0.))
        self.assertTrue(self.c.terminado)
        self.assertIn('angulo incierto', self.c.motivo_final)

    def test_salto_real_antes_de_meta_solo_habilita_vision_tras_reposo_sin_mover(self):
        self.c = ControlBarrido(avanzar=True)
        self.orden('NAVEGAR'); self.decidir()
        self.t += 100_000_000
        self.pose['x'] += 1.
        self.orden('NAVEGAR')
        self.assertEqual(self.decidir()[:2], (0., 0.))
        self.assertTrue(self.c.terminado)
        self.assertIn('Salto', self.c.motivo_final)
        self.assertIsNone(self.c.busqueda_desde_ns)
        for _ in range(9):
            self.t += 100_000_000
            self.orden('BARRER')
            self.assertEqual(self.decidir()[:2], (0., 0.))
        self.assertIsNotNone(self.c.busqueda_desde_ns)
        self.assertTrue(self.c.terminado)
        self.assertIn('Salto', self.c.motivo_final)
        self.orden('NAVEGAR')
        self.assertEqual(self.decidir()[:2], (0., 0.))
        self.t += 600_000_000
        self.assertEqual(self.decidir()[:2], (0., 0.))


class IntegracionTests(unittest.TestCase):
    def test_nodo_salida_unica_solo_giro_y_cero_con_eof_o_lidar_caducado(self):
        base = cargar_ros_simulado()
        spec = importlib.util.spec_from_file_location('barrido_simulado', app.TAREA/'barrido_ros.py')
        mod = importlib.util.module_from_spec(spec)
        reemplazos = {'busqueda_lidar_ros': base,
                      'seguridad_lidar': NS(EvasionLidarNode=base.EvasionLidarNode),
                      'rclpy': base.rclpy, 'nav_msgs.msg': NS(Odometry=base.Odometry),
                      'rclpy.executors': NS(ExternalShutdownException=RuntimeError),
                      'rclpy.qos': NS(QoSProfile=base.QoSProfile, ReliabilityPolicy=base.ReliabilityPolicy),
                      'rclpy.signals': NS(SignalHandlerOptions=base.SignalHandlerOptions)}
        with patch.dict(sys.modules, reemplazos):
            spec.loader.exec_module(mod)
        for mover in (False, True):
            entrada = NS(fin=threading.Event(), tomar=lambda: None)
            n = mod.BarridoNode(mover, ConfigEvasion(), entrada)
            self.assertEqual(len(n.publicadores), int(mover))
            t = 3_000_000_000
            n.pose = dict(x=0., y=0., yaw=0., frame_id='odom', child_frame_id='base_link')
            with contextlib.redirect_stdout(io.StringIO()):
                for seq in range(10):
                    t += 100_000_000
                    n.pose_ns = t
                    n.control.recibir(json.dumps(dict(version=1, secuencia=seq, orden='BARRER',
                        actividad_ns=t, camara_ns=t)), t)
                    with patch.object(mod.time, 'monotonic_ns', return_value=t), patch.object(mod, 'decidir_scan', return_value=(.1, .4, LIBRE, 'libre')):
                        n._ciclo()
            with patch.object(mod.time, 'monotonic_ns', return_value=t), contextlib.redirect_stdout(io.StringIO()):
                with patch.object(mod, 'decidir_scan', return_value=(0., 0., None, 'caducado')):
                    n._ciclo()
                entrada.fin.set()
                with patch.object(mod, 'decidir_scan', return_value=(.1, .4, LIBRE, 'libre')):
                    n._ciclo()
            if mover:
                ms = n.publicadores[0].mensajes
                self.assertTrue(all(m.twist.linear.x == 0 for m in ms))
                self.assertEqual([m.twist.angular.z for m in ms[-3:]], [.25, 0., 0.])

    def test_percepcion_no_analiza_caras_antes_de_meta_y_caducidad_invalida_inferencia(self):
        ensayo = app.EnsayoB(None)
        ensayo.sesion.iniciar('Persona A', 1_000_000_000)
        im = np.zeros((100, 200, 3), np.uint8)
        with patch.object(ensayo.detector, 'detectar', return_value=META), patch.object(app, 'analizar_candidatos', return_value=[]) as caras:
            for t in (1_100_000_000, 1_200_000_000, 1_300_000_000):
                with patch.object(app.time, 'monotonic_ns', return_value=t):
                    ensayo.actualizar(im, t, t)
            caras.assert_not_called()
            self.assertTrue(ensayo.sesion.meta_confirmada)
            ensayo.sesion.habilitar_busqueda(t)
            t = 1_400_000_000
            with patch.object(app.time, 'monotonic_ns', return_value=t+2_000_000_000):
                self.assertIsNone(ensayo.actualizar(im, t, t))
            self.assertEqual(ensayo.sesion.confirmaciones, 0)
            self.assertEqual(ensayo.caras, [])

    def test_interfaz_nombre_preparado_no_inicia_y_no_dice_que_registra_a(self):
        sistema = SistemaFacial(None, Galeria([[1, 0]], ['Persona A']), Criterio(.5, .1))
        ensayo = app.EnsayoB(sistema)
        entrada = app.EntradaMeta(ConsultaEnVivo(sistema, 'Persona A'), ensayo.sesion)
        self.assertIsNone(ensayo.sesion.inicio_ns)
        self.assertNotIn('registra A', entrada.aviso)
        entrada.confirmar()
        self.assertEqual(ensayo.sesion.orden, 'ESPERAR_META')
        panel = app.dibujar_prueba(None, 'Esperando camara', entrada, ensayo, None, False)
        self.assertEqual(panel.shape, (708, 960, 3))
        self.assertTrue(entrada.tecla(27))

    def test_preparacion_reutiliza_red_y_copia_solo_control_sin_datos(self):
        cmd = ['sudo', '-n', 'docker', 'exec', '-i', 'proyecto2-sim', '/tmp/x/camara.py', 'topic']
        base = ['docker', '/tmp/x/busqueda_lidar_ros.py', '--config', '/tmp/x/config.json']
        for mover in (False, True):
            with patch.object(app, 'preparar_movimiento', return_value=base.copy()) as preparar, patch.object(app, 'ejecutar_comando') as ejecutar:
                comando = app.preparar_barrido(cmd, 'proyecto2-sim', Path('cfg.json'), mover)
            preparar.assert_called_once_with(cmd, 'proyecto2-sim', Path('cfg.json'), mover)
            self.assertIn('/tmp/x/barrido_ros.py', comando)
            self.assertEqual([Path(c.args[0][-2]).name for c in ejecutar.call_args_list], ['control_meta.py', 'barrido_ros.py'])

    def test_lanzador_rechaza_mezclas_antes_de_red(self):
        script = str(app.TAREA/'ejecutar_autonomo_robot.sh')
        for args in (['--solo-evasion'], ['--evasion-reconocimiento'], ['--senales'], ['--tramo', 'vuelta'], ['--sin-ventana']):
            r = subprocess.run(['bash', script, '--prueba-meta-b', *args], capture_output=True, text=True)
            self.assertEqual(r.returncode, 2, r.stderr)

    def test_lanzador_desde_raiz_y_tarea_respeta_mover_explicito(self):
        with tempfile.TemporaryDirectory(prefix='prueba meta ') as tmp:
            raiz = Path(tmp)
            tarea = raiz/'task 1'
            tarea.mkdir()
            shutil.copyfile(app.TAREA/'ejecutar_autonomo_robot.sh', tarea/'ejecutar_autonomo_robot.sh')
            python = raiz/'.venv/bin/python'
            python.parent.mkdir(parents=True)
            python.write_text(f'#!{Path(sys.executable).resolve()}\nimport sys,json\n'
                              'if sys.argv[1] != "-c": print(json.dumps(sys.argv[1:]))\n')
            python.chmod(0o755)
            for cwd in (raiz, tarea):
                for mover, avanzar in ((False, False), (True, False), (False, True), (True, True)):
                    r = subprocess.run(['bash', str(tarea/'ejecutar_autonomo_robot.sh'),
                        '--ip', '10.87.179.100', '--dominio', '5', '--prueba-meta-b',
                        *(['--mover'] if mover else []), *(['--avanzar-hasta-meta'] if avanzar else [])],
                        cwd=cwd, capture_output=True, text=True)
                    self.assertEqual(r.returncode, 0, r.stderr)
                    comando = json.loads(r.stdout)
                    self.assertIn(str(tarea/'prueba_meta_robot.py'), comando)
                    self.assertEqual('--mover' in comando, mover)
                    self.assertEqual('--avanzar-hasta-meta' in comando, avanzar)

    def test_percepcion_sigue_tras_parada_del_barrido_y_no_busca_qr_sin_objetivo(self):
        ensayo = app.EnsayoB(None)
        ensayo.sesion.iniciar('Persona A', 1_000_000_000)
        ensayo.sesion.meta_confirmada = True
        t = 1_100_000_000
        ensayo.recibir_control(dict(recibido_ns=t, terminado=True, estado='Odometria discontinua',
                                    busqueda_desde_ns=t), t)
        self.assertNotEqual(ensayo.sesion.orden, 'PARAR')
        im = np.zeros((200, 300, 3), np.uint8)
        with patch.object(app, 'analizar_candidatos', return_value=[cara('Persona B')]), patch.object(ensayo.detector, 'detectar') as detector:
            t += 100_000_000
            with patch.object(app.time, 'monotonic_ns', return_value=t):
                ensayo.actualizar(im, t, t)
            detector.assert_not_called()
        qr = dict(meta=None, codigos=[dict(caja=[120, 20, 60, 60], texto='OBJETIVO')])
        with patch.object(app, 'analizar_candidatos', return_value=[cara()]), patch.object(ensayo.detector, 'detectar', return_value=qr):
            for _ in range(3):
                t += 100_000_000
                with patch.object(app.time, 'monotonic_ns', return_value=t):
                    ensayo.actualizar(im, t, t)
        self.assertEqual(ensayo.sesion.orden, 'QR_LISTO')
        self.assertEqual(ensayo.sesion.palabra, 'OBJETIVO')
        self.assertEqual(ensayo.fallo_movimiento, 'Odometria discontinua')

    def test_rostros_esperan_frenado_y_paquete_posterior(self):
        ensayo = app.EnsayoB(None, avanzar=True)
        ensayo.sesion.iniciar('Persona A', 1_000_000_000)
        self.assertEqual(ensayo.sesion.orden, 'NAVEGAR')
        ensayo.sesion.meta_confirmada = True
        im = np.zeros((200, 300, 3), np.uint8)
        with patch.object(app, 'analizar_candidatos', return_value=[]) as caras:
            t = 1_100_000_000
            ensayo.actualizar(im, t, t)
            caras.assert_not_called()
            ensayo.recibir_control(dict(recibido_ns=t, terminado=False, busqueda_desde_ns=t), t)
            ensayo.actualizar(im, t, t)
            caras.assert_not_called()
            t += 100_000_000
            with patch.object(app.time, 'monotonic_ns', return_value=t):
                ensayo.actualizar(im, t, t)
            caras.assert_called_once()

    def test_preparacion_requiere_opcion_en_host_y_docker_para_avanzar(self):
        with patch.object(app, 'preparar_movimiento', return_value=['docker','/tmp/x/busqueda_lidar_ros.py']), patch.object(app, 'ejecutar_comando'):
            cmd = app.preparar_barrido([], 'contenedor', Path('config'), False, avanzar=True)
        self.assertIn('--avanzar-hasta-meta', cmd)
        self.assertNotIn('--mover', cmd)

    def test_diagnostico_odometria_expone_edad_corte_y_maximo_rx(self):
        texto = app.resumen_odometria(dict(edad_odom_s=.1, max_intervalo_odom_s=1.3,
                                          ultimo_corte_odom_s=1.2))
        self.assertIn('edad=0.10s', texto)
        self.assertIn('rx_max=1.30s', texto)
        self.assertIn('corte=1.20s', texto)

    def test_diagnostico_qr_distingue_detectado_legible_y_asociacion(self):
        ensayo = app.EnsayoB(None)
        ensayo.estado_qr = 'QR sin asociar'
        ensayo.percepcion['codigos'] = [dict(caja=[0,0,20,20],texto=None),
                                        dict(caja=[30,0,20,20],texto='PALABRA')]
        resumen = ensayo.resumen_qr()
        self.assertIn('detectados=2, legibles=1', resumen)
        self.assertIn('QR sin asociar', resumen)
        self.assertIn('confirmacion=0/3', resumen)


if __name__ == '__main__':
    unittest.main()
