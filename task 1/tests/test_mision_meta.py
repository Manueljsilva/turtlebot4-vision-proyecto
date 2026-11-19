"""Pruebas de meta bandera a cuadros, espera de pared en giros y misión completa A->B->A."""

import json
from math import pi, radians
from pathlib import Path
import unittest
from unittest.mock import patch

import cv2
import numpy as np

from control_busqueda import PuertaBusqueda
from control_giros import ConfigSenales, ControlGiro, cargar_config_senales
from evasion_lidar import ConfigEvasion, analizar_lidar
from meta_bandera import ConsultaMetaBandera, DetectorMetaBandera, decodificar_qr_asociado
from mision_meta import ConfigMision, MisionMeta


TAREA = Path(__file__).resolve().parents[1]
AHORA = 20_000_000_000


def tablero_sintetico(lado=320, celdas=8):
    paso = lado // celdas
    idx = np.indices((lado, lado))
    patron = ((idx[0] // paso + idx[1] // paso) % 2) * 255
    img = np.full((lado + 80, lado + 80, 3), 180, np.uint8)
    img[40:40 + lado, 40:40 + lado] = np.repeat(patron[:, :, None], 3, axis=2).astype(np.uint8)
    return img


class MetaBanderaTests(unittest.TestCase):
    def test_detecta_bandera_cuadros_y_rechaza_lisa(self):
        det = DetectorMetaBandera()
        res_ok = det.detectar(tablero_sintetico())
        self.assertTrue(res_ok['detectada'])
        self.assertGreaterEqual(res_ok['puntos'], 8)
        self.assertIsNotNone(res_ok['caja'])

        res_lisa = det.detectar(np.full((240, 320, 3), 200, np.uint8))
        self.assertFalse(res_lisa['detectada'])

    def test_consulta_meta_exige_tres_frames_nuevos(self):
        q = ConsultaMetaBandera(confirmaciones=3)
        img = tablero_sintetico()
        self.assertFalse(q.actualizar(img, AHORA, AHORA)['confirmada'])
        # Repetir marca no suma confirmaciones
        self.assertEqual(q.actualizar(img, AHORA, AHORA)['cuenta'], 1)
        self.assertFalse(q.actualizar(img, AHORA + 100_000_000, AHORA + 100_000_000)['confirmada'])
        r3 = q.actualizar(img, AHORA + 200_000_000, AHORA + 200_000_000)
        self.assertTrue(r3['confirmada'])

    def test_decodificar_qr_elige_mas_cercano_a_rostro(self):
        img = np.zeros((200, 300, 3), np.uint8)
        pts = np.array([
            [[10, 10], [50, 10], [50, 50], [10, 50]],
            [[220, 20], [270, 20], [270, 70], [220, 70]],
        ], dtype=np.float32)
        with patch.object(cv2.QRCodeDetector, 'detectAndDecodeMulti',
                          return_value=(True, ('PALABRA_OTRA', 'PALABRA_OBJETIVO'), pts, None)):
            texto = decodificar_qr_asociado(img, caja_rostro=(215, 15, 60, 60))
        self.assertEqual(texto, 'PALABRA_OBJETIVO')


class GiroConParedYVelocidadTests(unittest.TestCase):
    def test_espera_pared_frontal_antes_de_girar_y_cambia_a_vuelta(self):
        cfg_ev = ConfigEvasion(velocidad_avance=0.22, velocidad_giro=0.90,
                               distancia_parada_m=0.34, distancia_precaucion_m=0.55)
        cfg_sen = cargar_config_senales(TAREA / 'config/senales_giro.json')
        self.assertGreater(cfg_sen.esperar_pared_m, cfg_ev.distancia_parada_m)

        ctrl = ControlGiro('ida', cfg_sen)
        pose = dict(x=0.0, y=0.0, yaw=0.0)
        lidar_lejos = analizar_lidar([1.8] * 360, -pi, 2 * pi / 360, 0.05, 12.0, cfg=cfg_ev)
        ev = dict(id=1, direccion='DERECHA', color='azul', marca_ns=AHORA)

        # Lejos de la pared: memoriza señal pero deja avanzar recto (retorna None)
        self.assertIsNone(ctrl.decidir(ev, pose, AHORA, True, lidar_lejos, cfg_ev))
        self.assertIsNotNone(ctrl.pendiente)

        # Al acercarse a <= esperar_pared_m (y > distancia_parada_m), inicia el giro de 90°
        lidar_pared = analizar_lidar([0.45 if abs(-pi + i * 2 * pi / 360) < 0.2 else 1.8 for i in range(360)],
                                     -pi, 2 * pi / 360, 0.05, 12.0, cfg=cfg_ev)
        giro = ctrl.decidir(None, pose, AHORA + 200_000_000, True, lidar_pared, cfg_ev)
        self.assertIsNotNone(giro)
        self.assertEqual(giro[0], 0.0)
        self.assertLess(giro[1], -0.5)  # Giro ágil hacia derecha

        # Completar 90° y cambiar automáticamente a tramo vuelta (rojo)
        pose['yaw'] = -pi / 2
        fin = ctrl.decidir(None, pose, AHORA + 1_500_000_000, True, lidar_pared, cfg_ev)
        self.assertEqual(fin[:2], (0.0, 0.0))
        ctrl.cambiar_tramo('vuelta')
        self.assertEqual(ctrl.color, 'rojo')

    def test_puerta_acepta_ordenes_de_barrido_y_tramo(self):
        p = PuertaBusqueda()
        linea = json.dumps(dict(version=1, secuencia=0, orden='GIRAR_BARRIDO',
                                actividad_ns=AHORA, camara_ns=AHORA, tramo='ida'))
        p.recibir(linea, AHORA)
        ok, _ = p.permiso(AHORA, dict(frame_id='odom', child_frame_id='base_link', x=0.0, y=0.0, yaw=0.0), AHORA)
        self.assertTrue(ok)
        self.assertEqual(p.tramo, 'ida')


class MisionMetaTests(unittest.TestCase):
    def test_ciclo_completo_ida_barrido_4_fotos_vuelta_a(self):
        m = MisionMeta(ConfigMision(distancia_exclusion_meta_m=1.0, salir_anticipado_confianza=0.95))
        pose = dict(x=0.0, y=0.0, yaw=0.0)
        m.iniciar(pose)
        self.assertEqual(m.etapa, 'IDA')

        # Bandera cerca de A se ignora
        self.assertEqual(m.actualizar(pose, {'confirmada': True}, None), 'BUSCAR')
        self.assertEqual(m.etapa, 'IDA')

        # Bandera lejos de A inicia BARRIDO_B
        pose = dict(x=1.5, y=0.0, yaw=0.0)
        self.assertEqual(m.actualizar(pose, {'confirmada': True}, None), 'GIRAR_BARRIDO')
        self.assertEqual(m.etapa, 'BARRIDO_B')

        # Durante el barrido de 360° ve 4 imágenes con distintas afinidades al input
        muestras = [
            (0.0, 0.31, 'Persona 1', 'QR_1'),
            (pi / 2, 0.78, 'Objetivo Real', 'SECRETO_UTEC'),
            (pi, 0.42, 'Persona 3', 'QR_3'),
            (-pi / 2, 0.39, 'Persona 4', 'QR_4'),
        ]
        for ang, af, cand, qr in muestras:
            pose = dict(x=1.5, y=0.0, yaw=ang)
            res_f = dict(nombre=cand, candidato=cand, aceptado=af > 0.6,
                         objetivo_confirmado=af > 0.6, afinidad_objetivo=af, motivo='OK')
            orden = m.actualizar(pose, None, res_f, qr_texto=qr)
            self.assertEqual(orden, 'GIRAR_BARRIDO_LENTO')

        # Completa 360° -> pasa a GIRO_RETORNO_B y cambia tramo a vuelta
        pose = dict(x=1.5, y=0.0, yaw=0.0)
        orden = m.actualizar(pose, None, None)
        self.assertEqual(orden, 'GIRAR_RETORNO')
        self.assertEqual(m.etapa, 'GIRO_RETORNO_B')
        self.assertEqual(m.tramo, 'vuelta')
        self.assertEqual(m.mejor_afinidad, 0.78)
        self.assertEqual(m.palabra_qr, 'SECRETO_UTEC')

        # Completa 180° -> pasa a VUELTA
        m.actualizar(dict(x=1.5, y=0.0, yaw=pi / 2), None, None)
        orden = m.actualizar(dict(x=1.5, y=0.0, yaw=pi), None, None)
        self.assertEqual(orden, 'BUSCAR')
        self.assertEqual(m.etapa, 'VUELTA')

        # Bandera cerca de B se ignora; al llegar cerca de A finaliza
        self.assertEqual(m.actualizar(dict(x=1.4, y=0.0, yaw=pi), {'confirmada': True}, None), 'BUSCAR')
        orden = m.actualizar(dict(x=0.1, y=0.0, yaw=pi), {'confirmada': True}, None)
        self.assertEqual(orden, 'ENCONTRADO')
        self.assertEqual(m.etapa, 'FINALIZADO')


if __name__ == '__main__':
    unittest.main()
