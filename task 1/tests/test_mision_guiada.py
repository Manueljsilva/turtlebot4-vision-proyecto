"""Ensayo integrado con sensores/visión simulados, sin red ni robot físico."""

import importlib.util
import io
import json
from dataclasses import replace
from math import pi
from pathlib import Path
import sys
import tempfile
import threading
from types import ModuleType, SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

import numpy as np

import mision_guiada_robot as app
from control_mision import ConfigMision, ControlMision, MetaNueva, normalizar
from ensayo_giros_companero import cargar_ensayo, CONFIG, CONTROL
from evasion_lidar import ConfigEvasion
from holgura_giro import HolguraGiro
from prueba_facial_robot import SistemaFacial
from reconocimiento_facial import Criterio, Galeria
from tests.test_busqueda_robot import cargar_ros_simulado
from tests import test_lanzador_evasion as lanzador


META = [.4, .2, .2, .4]


def lidar(frente=1.8, izquierda=1.8, derecha=1.8, valido=True):
    return NS(dist_frente=frente, dist_izquierda=izquierda, dist_derecha=derecha,
              observacion_valida=valido, proximidad_no_cuantificable=False,
              peligro_parada=frente <= .32, precaucion=frente <= .45)


class MisionTests(unittest.TestCase):
    def setUp(self):
        self.mod, self.cfg_s = cargar_ensayo(config=app.CONFIG_SENALES_MISION)
        self.c = ControlMision(self.cfg_s, clase_giros=self.mod.ControlGiro)
        self.cfg = ConfigEvasion()
        self.t, self.seq = 2_000_000_000, 0
        self.pose = dict(frame_id='odom', child_frame_id='base_link', x=0., y=0., yaw=0.)

    def paso(self, orden='OBSERVAR', dt=.2, meta=None, sensor=None, marca=None, qr_ns=None, votos=0,
             senal=None, reinicio=None, holgura=None, **pose):
        self.t += round(dt*1e9)
        self.pose.update(pose)
        if marca is None and orden != 'REINICIAR':
            marca = self.t
        self.c.recibir(json.dumps(dict(version=1, secuencia=self.seq, orden=orden,
            actividad_ns=self.t, camara_ns=marca, meta=meta, senal=senal,
            qr_ns=qr_ns, qr_votos=votos,
            reinicio_ns=self.c.reinicio_ns if reinicio is None else reinicio)), self.t)
        self.seq += 1
        sensor=sensor or lidar()
        if holgura is None:
            holgura=HolguraGiro(sensor.observacion_valida,
                min(sensor.dist_frente,sensor.dist_izquierda,sensor.dist_derecha),'Contorno simulado',0.)
        return self.c.decidir(self.t,self.pose,self.t,sensor,self.cfg,holgura=holgura)

    def iniciar(self):
        self.paso('INICIAR')
        self.assertEqual(self.c.fase, 'IDA')

    def llegar_b(self):
        self.iniciar()
        for i in range(12):
            self.paso(x=min(1., i*.1))
        self.paso(meta=META, sensor=lidar(.4))
        self.assertEqual(self.c.fase, 'FRENAR_B')
        for _ in range(5):
            self.paso()
        self.assertEqual(self.c.fase, 'BUSCAR_B')

    def qr_y_salida(self, solo_orientar=False):
        self.llegar_b()
        for yaw in (.2, .4, .6):
            self.paso(yaw=yaw)
        self.paso('QR_LISTO', qr_ns=self.t+200_000_000, votos=1)
        self.assertEqual(self.c.fase, 'FRENAR_QR')
        for _ in range(5):
            self.paso()
        self.assertEqual(self.c.fase, 'ORIENTAR_SALIDA')
        if solo_orientar:
            return
        for _ in range(30):
            error = self.c.yaw_salida-self.c.yaw_total
            yaw = self.pose['yaw']+max(-.2, min(.2, error))
            vx, _, _ = self.paso(yaw=normalizar(yaw))
            self.assertEqual(vx, 0.)
            if self.c.fase == 'FRENAR_SALIDA':
                break
        self.assertEqual(self.c.fase, 'FRENAR_SALIDA')
        self.assertAlmostEqual(abs(self.pose['yaw']), pi)
        for _ in range(5):
            self.paso()
        self.assertEqual(self.c.fase, 'VUELTA')

    def test_secuencia_completa_azul_b_qr_orientacion_rojo_a(self):
        self.qr_y_salida()
        self.assertEqual(self.c.giros.color, 'rojo')
        self.assertTrue(self.c.qr_confirmado)
        self.assertEqual(self.c.yaw_entrada_b, 0.)
        # El rostro estaba a 0.6 rad: no se suman 180° a esa orientación.
        self.assertNotAlmostEqual(normalizar(.6+pi), self.pose['yaw'])
        for i in range(1, 11):
            self.paso(x=max(.1, 1.-i*.1))
        self.assertEqual(self.c.fase,'VUELTA')  # Estar cerca de A sin ver bandera no finaliza.
        self.paso(meta=META, sensor=lidar(.4))
        self.assertEqual(self.c.fase, 'FRENAR_A')
        for _ in range(5):
            self.paso()
        self.assertEqual(self.c.fase, 'FIN')
        self.assertEqual(self.paso('INICIAR')[:2], (0., 0.))
        self.assertEqual(self.c.fase, 'FIN')

    def test_meta_inicial_visible_no_cuenta_como_b(self):
        self.iniciar()
        for i in range(12):
            self.paso(meta=META, sensor=lidar(.4), x=min(1., i*.1))
        self.assertEqual(self.c.fase, 'IDA')
        self.assertEqual(self.c.meta_nueva.votos, 0)
        self.assertIn('falta ausencia previa',self.c.diagnostico_meta)

    def test_reinicio_de_vuelta_borra_mision_y_espera_reposo_sin_reanudar(self):
        self.qr_y_salida()
        token=self.t+200_000_000
        self.assertEqual(self.paso('REINICIAR',reinicio=token)[:2],(0.,0.))
        self.assertEqual(self.c.fase,'REINICIAR')
        self.assertIsNone(self.c.a)
        self.assertIsNone(self.c.b)
        self.assertIsNone(self.c.yaw_entrada_b)
        self.assertIsNone(self.c.yaw_salida)
        self.assertIsNone(self.c.meta_nueva)
        self.assertFalse(self.c.qr_confirmado)
        self.assertEqual(self.c.giros.color,'azul')
        for _ in range(4):
            self.assertEqual(self.paso('REINICIAR',reinicio=token)[:2],(0.,0.))
        self.assertEqual(self.c.fase,'ESPERAR')
        self.assertIsNone(self.c.solicitud_ns)
        self.paso('REINICIAR',reinicio=token)  # Latido repetido no abre otro reinicio.
        self.assertEqual(self.c.fase,'ESPERAR')
        self.assertIsNone(self.c.a)
        self.assertEqual(self.paso('ESPERAR',x=5.)[:2],(0.,0.))
        self.assertEqual(self.c.fase,'ESPERAR')  # Recolocar quieto no hereda una ruta anterior.
        self.paso('INICIAR',x=6.)
        self.assertEqual(self.c.fase,'IDA')
        self.assertAlmostEqual(self.c.a['x'],6.)

    def test_reinicio_explicito_recupera_fin_y_fallo_sin_reusar_resultado(self):
        for fase in ('FIN','FALLO'):
            self.c.fase=fase
            self.c.qr_confirmado=True
            token=self.t+200_000_000
            self.paso('REINICIAR',reinicio=token)
            for _ in range(4):
                self.paso('REINICIAR',reinicio=token)
            self.assertEqual(self.c.fase,'ESPERAR')
            self.assertFalse(self.c.qr_confirmado)
            self.assertIsNone(self.c.a)

    def test_reinicio_no_acepta_orden_anterior_inicio_prematuro_o_token_invalido(self):
        self.iniciar()
        for token in (0,self.t-1_000_000_000,self.t+2_000_000_000):
            self.paso('REINICIAR',reinicio=token)
            self.assertEqual(self.c.fase,'IDA')
        token=self.t+200_000_000
        self.paso('REINICIAR',reinicio=token)
        self.paso('INICIAR')  # No se habilita Buscar antes del reposo/acuse.
        self.assertEqual(self.c.fase,'REINICIAR')
        self.assertIsNone(self.c.solicitud_ns)
        for _ in range(5):
            self.paso('REINICIAR',reinicio=token)
        self.assertEqual(self.c.fase,'ESPERAR')
        self.paso('QR_LISTO',reinicio=0,qr_ns=self.t+200_000_000,votos=1)
        self.assertIsNone(self.c.actividad_ns)
        self.assertIsNone(self.c.a)
        self.assertFalse(self.c.qr_confirmado)
        self.paso('INICIAR',reinicio=token+1)
        self.assertIsNone(self.c.solicitud_ns)
        self.c.recibir(json.dumps(dict(version=1,secuencia=self.seq,orden='PARAR',
            actividad_ns=self.t,camara_ns=None)),self.t)
        self.assertEqual(self.c.fase,'FALLO')  # Cierre compartido sin token también detiene.

    def test_reinicio_no_confirma_reposo_con_robot_en_movimiento_o_pose_antigua(self):
        self.iniciar()
        token=self.t+200_000_000
        self.paso('REINICIAR',reinicio=token)
        for i in range(10):
            self.assertEqual(self.paso('REINICIAR',reinicio=token,x=.01*i)[:2],(0.,0.))
        self.assertEqual(self.c.fase,'REINICIAR')
        self.t+=2_000_000_000
        self.c.decidir(self.t,self.pose,token-1,lidar(),self.cfg)
        self.assertEqual(self.c.fase,'REINICIAR')
        self.assertEqual(self.c.reposo.muestras,0)
        for _ in range(5):
            self.paso('REINICIAR',reinicio=token)
        self.assertEqual(self.c.fase,'ESPERAR')

    def test_inicio_solicitado_sin_camara_permanece_esperando_sin_registrar_a(self):
        vx,wz,motivo=self.paso('INICIAR',marca=0)
        self.assertEqual((vx,wz),(0.,0.))
        self.assertEqual(self.c.fase,'ESPERAR')
        self.assertIsNone(self.c.a)
        self.assertIsNotNone(self.c.solicitud_ns)
        self.assertEqual(self.c.orden,'INICIAR')
        self.assertIn('Sin camara reciente',motivo)
        self.assertEqual(self.c.diagnostico_meta,'Esperando imagen reciente')

    def test_meta_de_lejos_o_lateral_no_declara_b(self):
        self.iniciar()
        for i in range(12):
            self.paso(x=min(1., i*.1))
        for _ in range(4):
            self.paso(meta=META)  # Frontal 1.8 m.
        self.assertEqual(self.c.fase, 'IDA')
        self.assertIn('frente 1.80m > 0.40m',self.c.diagnostico_meta)
        for _ in range(4):
            self.paso(meta=[.05, .2, .1, .4], sensor=lidar(.4))
        self.assertEqual(self.c.fase, 'IDA')
        self.assertIn('centrarla',self.c.diagnostico_meta)

    def test_meta_467mm_aproxima_sin_giro_evasivo_y_solo_frena_a_400mm(self):
        self.iniciar()
        for i in range(12):self.paso(x=min(1.,i*.1))
        for frente in (.467,.45,.41,.401):
            self.assertEqual(self.paso(meta=META,sensor=lidar(frente))[:2],(.05,0.))
            self.assertEqual(self.c.fase,'IDA')
            self.assertIsNone(self.c.b)
            self.assertIsNone(self.c.evasion)
        self.assertEqual(self.paso(meta=META,sensor=lidar(.4))[:2],(0.,0.))
        self.assertEqual(self.c.fase,'FRENAR_B')
        for _ in range(5):self.paso()
        self.assertEqual(self.c.fase,'BUSCAR_B')

    def test_meta_en_aproximacion_pierde_vista_o_holgura_detiene_sin_girar(self):
        self.iniciar()
        for i in range(12):self.paso(x=min(1.,i*.1))
        self.paso(meta=META,sensor=lidar(.467))
        inicio=self.c.aproximacion_meta_ns
        self.assertEqual(self.paso(sensor=lidar(.44))[:2],(0.,0.))
        self.assertIsNone(self.c.evasion)
        self.assertEqual(self.paso(meta=META,sensor=lidar(.44),
            holgura=HolguraGiro(True,.27,'Lateral cercano',.2))[:2],(0.,0.))
        self.assertEqual(self.paso(meta=META,sensor=lidar(.44),marca=0)[:2],(0.,0.))
        self.assertEqual(self.paso(meta=META,sensor=lidar(.44,valido=False))[:2],(0.,0.))
        self.assertEqual(self.c.aproximacion_meta_ns,inicio)
        self.assertEqual(self.paso(meta=META,sensor=lidar(.44))[:2],(.05,0.))
        for _ in range(152):self.paso(marca=0)
        self.assertEqual(self.c.fase,'FALLO')
        self.assertIn('Aproximacion a meta agotada',self.c.motivo)

    def test_meta_vuelta_espera_400mm_y_flecha_roja_conserva_perfil_mision(self):
        self.qr_y_salida()
        self.assertEqual(self.c.giros.cfg.esperar_pared_m,.305)
        self.assertEqual(self.c.giros.cfg.tiempo_espera_pared_s,30.)
        for i in range(1,11):self.paso(x=max(.1,1.-i*.1))
        self.assertEqual(self.paso(meta=META,sensor=lidar(.467))[:2],(.05,0.))
        self.assertEqual(self.c.fase,'VUELTA')
        self.paso(meta=META,sensor=lidar(.4))
        self.assertEqual(self.c.fase,'FRENAR_A')

    def test_flecha_roja_se_guarda_al_perderla_y_activa_en_umbral_de_ida(self):
        self.qr_y_salida()
        ev=dict(id=1,direccion='DERECHA',color='rojo',marca_ns=self.t+200_000_000)
        self.paso(senal=ev,sensor=lidar(.5))
        for _ in range(40):self.paso(sensor=lidar(.38))
        self.assertIsNotNone(self.c.giros.pendiente)
        self.assertEqual(self.paso(sensor=lidar(.304))[:2],(0.,0.))
        for _ in range(5):r=self.paso(sensor=lidar(.304))
        self.assertEqual(r[:2],(0.,-.4))

    def test_sin_flecha_detectada_no_inventa_direccion_guardada(self):
        self.iniciar()
        for _ in range(40):self.paso(sensor=lidar(.5))
        self.assertIsNone(self.c.giros.pendiente)
        self.assertIsNone(self.c.giros.activo)
        self.assertIsNone(self.c.aproximacion_ns)

    def test_meta_frena_aunque_el_barrido_este_bloqueado_y_vision_sigue_activa(self):
        self.iniciar()
        for i in range(12):
            self.paso(x=min(1.,i*.1))
        estrecho=lidar(.4,.314,.307)  # Meta dentro del umbral y obstáculo trasero.
        trasera=HolguraGiro(True,.24,'Obstaculo trasero simulado',0.)
        self.assertEqual(self.paso(meta=META,sensor=estrecho,holgura=trasera)[:2],(0.,0.))
        self.assertEqual(self.c.fase,'FRENAR_B')
        for _ in range(5):
            self.assertEqual(self.paso(sensor=estrecho,holgura=trasera)[:2],(0.,0.))
        self.assertEqual(self.c.fase,'BUSCAR_B')
        vx,wz,motivo=self.paso(sensor=estrecho,holgura=trasera)
        self.assertEqual((vx,wz),(0.,0.))
        self.assertIn('distancia al centro=0.240m <= radio+margen=0.250m',motivo)
        self.assertIn('vision estacionaria sigue activa',motivo)
        self.assertEqual(self.paso('VERIFICAR',sensor=estrecho,holgura=trasera)[:2],(0.,0.))
        self.assertEqual(self.c.fase,'BUSCAR_B')
        self.paso('QR_LISTO',qr_ns=self.t+200_000_000,votos=1,sensor=estrecho,holgura=trasera)
        self.assertTrue(self.c.qr_confirmado)
        self.assertEqual(self.c.fase,'FRENAR_QR')

    def test_meta_no_omite_sensores_desconocidos_y_espacio_abierto_si_permite_barrido(self):
        self.iniciar()
        for i in range(12):
            self.paso(x=min(1.,i*.1))
        for sensor in (lidar(.4,valido=False),lidar(float('nan')),
                       lidar(.4,float('inf')),lidar(.4,0.)):
            with self.subTest(sensor=sensor):
                self.assertEqual(self.paso(meta=META,sensor=sensor)[:2],(0.,0.))
                self.assertEqual(self.c.fase,'IDA')
        self.paso(meta=META,sensor=lidar(.4))
        for _ in range(5):self.paso()
        self.assertEqual(self.c.fase,'BUSCAR_B')
        self.assertEqual(self.paso()[:2],(0.,.25))

    def test_llegada_a_puede_frenar_y_finalizar_sin_necesitar_otro_giro(self):
        self.qr_y_salida()
        for i in range(1,12):self.paso(x=max(.1,1.-i*.1))
        self.assertEqual(self.paso(meta=META,sensor=lidar(.3,.31,.31))[:2],(0.,0.))
        self.assertEqual(self.c.fase,'FRENAR_A')
        for _ in range(5):self.paso(sensor=lidar(.3,.31,.31))
        self.assertEqual(self.c.fase,'FIN')

    def test_repetir_paquete_inadmisible_no_declara_llegada_y_nuevo_valido_si(self):
        self.iniciar()
        for i in range(12):
            self.paso(x=min(1., i*.1))
        self.paso(meta=META, sensor=lidar(1.8))
        marca = self.t
        for _ in range(3):
            self.paso(meta=META, sensor=lidar(.4), marca=marca)
        self.assertEqual(self.c.meta_nueva.votos, 0)
        self.assertEqual(self.c.fase,'IDA')
        self.paso(meta=META, marca=0)
        self.assertEqual(self.c.meta_nueva.votos, 0)
        self.paso(meta=META,sensor=lidar(.4))
        self.assertEqual(self.c.fase,'FRENAR_B')
        self.assertEqual(self.c.meta_nueva.votos,1)

    def test_b_no_puede_convertirse_en_a_durante_la_salida(self):
        self.qr_y_salida()
        for _ in range(12):
            self.paso()  # Ausencia visual, pero sigue en la posición B.
        for _ in range(4):
            self.paso(meta=META, sensor=lidar(.4))
        self.assertEqual(self.c.fase, 'VUELTA')
        self.assertEqual(self.c.meta_nueva.votos, 0)

    def test_meta_lejos_de_a_no_finaliza_retorno(self):
        self.qr_y_salida()
        for i in range(12):
            self.paso(y=(i+1)*.1)
        for _ in range(4):
            self.paso(meta=META, sensor=lidar(.4))
        self.assertEqual(self.c.fase, 'VUELTA')

    def test_qr_antes_de_b_incompleto_o_caducado_no_habilita_regreso(self):
        self.iniciar()
        self.paso('QR_LISTO', qr_ns=self.t+200_000_000, votos=1)
        self.assertEqual(self.c.fase, 'IDA')
        self.setUp()
        self.llegar_b()
        for marca, votos in ((0,1), (self.t+200_000_000,0)):
            self.assertEqual(self.paso('QR_LISTO', qr_ns=marca, votos=votos)[:2], (0., 0.))
            self.assertEqual(self.c.fase, 'BUSCAR_B')
        self.assertFalse(self.c.qr_confirmado)
        self.assertEqual(self.paso('VERIFICAR')[:2], (0., 0.))

    def test_protocolo_qr_rechaza_recuentos_fuera_del_nuevo_contrato(self):
        for votos in (2,True,1.):
            with self.subTest(votos=votos):
                self.setUp()
                self.llegar_b()
                orden_anterior=self.c.orden
                self.assertEqual(self.paso('QR_LISTO',qr_ns=self.t+200_000_000,votos=votos)[:2],(0.,0.))
                self.assertEqual(self.c.fase,'BUSCAR_B')
                self.assertEqual(self.c.orden,orden_anterior)
                self.assertIsNone(self.c.actividad_ns)
                self.assertFalse(self.c.qr_confirmado)

    def test_primera_imagen_de_flecha_impide_elegir_giro_evasivo_contrario(self):
        self.iniciar()
        sistema=SistemaFacial(Mock(),Galeria([[1,0]],['Persona A']),Criterio(.5,.1))
        v=app.VisionMision(sistema,self.cfg_s)
        s=app.SesionInterfaz()
        s.iniciar('Persona A',self.t)
        marca=self.t+200_000_000
        d=dict(color='azul',direccion='IZQUIERDA',confianza=.9,caja=[60,40,100,100])
        with patch.object(app.time,'monotonic_ns',return_value=marca), \
                patch.object(v.detector_meta,'detectar',return_value=dict(meta=None,codigos=[])), \
                patch('senales_giro.DetectorGiros.detectar',return_value=[d]):
            v.actualizar(np.zeros((240,320,3),np.uint8),marca,marca,
                         dict(fase='IDA',fase_desde_ns=self.c.desde_ns),s,Mock())
        self.assertEqual(v.resultado_senal['confirmaciones'],1)
        self.assertEqual(v.senal['direccion'],'IZQUIERDA')
        # Menos espacio a izquierda: evasión elegiría derecha, señal pide izquierda.
        vx,wz,_=self.paso(senal=v.senal,sensor=lidar(.30,izquierda=.8,derecha=1.8))
        self.assertEqual((vx,wz),(0.,0.))
        self.assertEqual(self.c.giros.activo['direccion'],'IZQUIERDA')
        for _ in range(4):
            self.assertEqual(self.paso(sensor=lidar(.30,.8,1.8))[:2],(0.,0.))
        self.assertGreater(self.paso(sensor=lidar(.30,.8,1.8))[1],0.)

    def test_giro_azul_espera_30cm_con_aproximacion_lenta_y_sin_invertir(self):
        self.iniciar()
        ev = lambda: dict(id=1, direccion='IZQUIERDA', color='azul', marca_ns=self.t+200_000_000)
        self.assertEqual(self.paso(senal=ev(), sensor=lidar(.5))[:2], (.1, 0.))
        self.assertEqual(self.paso(senal=ev(), sensor=lidar(.44))[:2], (.05, 0.))
        self.assertEqual(self.paso(senal=ev(), sensor=lidar(.41))[:2], (.05, 0.))
        self.assertEqual(self.paso(senal=ev(), sensor=lidar(.5, izquierda=.2))[:2], (0., 0.))
        self.assertEqual(self.paso(senal=ev(),sensor=lidar(.31))[:2],(.05,0.))
        vx, wz, _ = self.paso(senal=ev(), sensor=lidar(.30))
        self.assertEqual((vx,wz),(0.,0.))
        for _ in range(4):
            self.assertEqual(self.paso(sensor=lidar(.30))[:2],(0.,0.))
        self.assertEqual(self.paso(sensor=lidar(.30))[:2],(0.,.4))
        self.assertEqual(self.paso(sensor=lidar(.30,derecha=.25))[:2],(0.,0.))

    def test_evasion_mantiene_lado_con_las_variaciones_laterales_del_registro(self):
        self.iniciar()
        self.assertGreater(self.paso(sensor=lidar(.418,.366,.282))[1],0.)
        inicio=self.c.evasion['inicio_ns']
        for derecha in (.404,.366,.387,.362,.411):
            vx,wz,_=self.paso(sensor=lidar(.365,.373,derecha),yaw=self.pose['yaw']+.03)
            self.assertEqual(vx,0.)
            self.assertGreater(wz,0.)
            self.assertEqual(self.c.evasion['inicio_ns'],inicio)
        self.assertEqual(self.paso(sensor=lidar(.365,.24,.5))[:2],(0.,0.))
        self.assertGreater(self.paso(sensor=lidar(.48,.373,.5))[1],0.)
        self.assertEqual(self.paso(sensor=lidar(.50,.6,.6))[:2],(0.,0.))
        self.assertIsNone(self.c.evasion)
        self.assertEqual(self.paso(sensor=lidar(.50,.6,.6))[:2],(.1,0.))

    def test_evasion_derecha_tampoco_invierte_por_un_lado_mas_libre(self):
        self.iniciar()
        self.assertLess(self.paso(sensor=lidar(.37,.36,.42))[1],0.)
        for izquierda in (.6,.7,.36):
            self.assertLess(self.paso(sensor=lidar(.37,izquierda,.42),yaw=self.pose['yaw']-.03)[1],0.)
        self.assertEqual(self.paso(sensor=lidar(.37,.6,.24))[:2],(0.,0.))

    def test_evasion_sin_salida_se_limita_por_angulo_o_tiempo(self):
        for por_angulo in (True,False):
            self.setUp()
            self.iniciar()
            self.paso(sensor=lidar(.37,.6,.5))
            for _ in range(70):
                r=self.paso(sensor=lidar(.37,.6,.5),yaw=self.pose['yaw']+(.3 if por_angulo else 0))
                if self.c.fase=='FALLO':break
            self.assertEqual(self.c.fase,'FALLO')
            self.assertEqual(r[:2],(0.,0.))
            self.assertIn('Evasion sin salida',self.c.motivo)
            self.assertIn('limite angular' if por_angulo else 'plazo de 12.0s agotado',self.c.motivo)
            self.assertEqual(self.paso(sensor=lidar(2.))[:2],(0.,0.))

    def test_31cm_permite_giro_con_radio_indicado_y_contorno_24cm_bloquea(self):
        self.iniciar()
        self.assertGreater(self.paso(sensor=lidar(.426,.311,.308))[1],0.)
        ocupado=HolguraGiro(True,.24,'Contorno ocupado',0.)
        for _ in range(65):
            vx,wz,motivo=self.paso(sensor=lidar(.426,.311,.308),holgura=ocupado)
            self.assertEqual((vx,wz),(0.,0.))
            if self.c.fase=='FALLO':break
        self.assertEqual(self.c.fase,'FALLO')
        self.assertIn('plazo de 12.0s agotado',motivo)
        self.assertIn('ultimo bloqueo: distancia al centro=0.240m <= radio+margen=0.250m',motivo)
        self.assertNotIn('giro contrario',motivo)
        d=self.c.diagnostico_giro(self.t,lidar(.426,.311,.308),self.cfg)
        self.assertEqual(d['limite_giro_m'],.25)
        self.assertIn('distancia al centro=0.240m',d['bloqueo_giro'])
        self.assertEqual(self.paso(sensor=lidar(2.))[:2],(0.,0.))

    def test_disparo_sin_margen_sobre_huella_se_rechaza(self):
        self.cfg_s=replace(self.cfg_s,esperar_pared_m=.27)
        self.c=ControlMision(self.cfg_s,clase_giros=self.mod.ControlGiro)
        self.iniciar()
        vx,wz,motivo=self.paso(sensor=lidar(.40))
        self.assertEqual((vx,wz),(0.,0.))
        self.assertEqual(self.c.fase,'FALLO')
        self.assertIn('0.270m sin margen',motivo)
        self.assertIn('radio+holgura+aproximacion 0.270m',motivo)
        self.assertEqual(self.cfg.distancia_parada_m,.32)

    def test_aproximacion_compensa_edad_y_barrido_no_usa_holgura_anterior(self):
        self.iniciar()
        ev=lambda:dict(id=1,direccion='IZQUIERDA',color='azul',marca_ns=self.t+200_000_000)
        # Mismo contorno, scan reciente permite avance lento; viejo exige más margen.
        self.assertEqual(self.paso(senal=ev(),sensor=lidar(.31),
            holgura=HolguraGiro(True,.29,'Contorno simulado',.02))[:2],(.05,0.))
        self.assertEqual(self.paso(senal=ev(),sensor=lidar(.31),
            holgura=HolguraGiro(True,.29,'Contorno simulado',.6))[:2],(0.,0.))
        self.setUp();self.llegar_b()
        self.assertGreater(self.paso()[1],0.)
        self.assertEqual(self.paso(holgura=HolguraGiro(False,None,'Sin cobertura trasera',0.))[:2],(0.,0.))
        self.c.decidir(self.t,self.pose,self.t,lidar(),self.cfg)
        self.assertIsNone(self.c.holgura)
        self.assertIn('Sin contorno',self.c.bloqueo_giro(lidar(),self.cfg))

    def test_registro_304mm_frena_y_gira_sin_exigir_mas_avance_ni_otra_imagen_de_flecha(self):
        self.iniciar()
        ev=lambda:dict(id=1,direccion='IZQUIERDA',color='azul',marca_ns=self.t+200_000_000)
        self.paso(senal=ev(),sensor=lidar(.558))
        self.paso(senal=ev(),sensor=lidar(.466))
        for _ in range(45):self.paso(sensor=lidar(.38))  # >9 s sin volver a ver la flecha.
        h=HolguraGiro(True,.264,'Contorno del registro',.12)
        vx,wz,motivo=self.paso(sensor=lidar(.304),holgura=h)
        self.assertEqual((vx,wz),(0.,0.))
        self.assertIn('confirmar reposo',motivo)
        self.assertIsNone(self.c.giros.pendiente)
        inicio=self.c.giros.activo['inicio_ns']
        for _ in range(4):
            self.assertEqual(self.paso(sensor=lidar(.304),holgura=h)[:2],(0.,0.))
        self.assertTrue(self.c.freno_flecha['confirmado'])
        self.assertEqual(self.paso(sensor=lidar(.304),holgura=h)[:2],(0.,.4))
        self.assertEqual(self.c.giros.activo['inicio_ns'],inicio)
        self.assertIsNone(self.c.senal)
        self.assertEqual(self.c.giros.ultimo_id,1)
        d=self.c.diagnostico_giro(self.t,lidar(.304),self.cfg)
        self.assertEqual(d['disparo_m'],.30)
        self.assertEqual(d['disparo_efectivo_m'],.305)
        for i in range(1,7):
            r=self.paso(sensor=lidar(.304),holgura=h,yaw=min(pi/2,i*.3))
            self.assertEqual(r[0],0.)
        self.assertIsNone(self.c.giros.activo)
        self.assertEqual(r[:2],(0.,0.))
        self.assertEqual(self.paso(sensor=lidar(.8))[:2],(.1,0.))

    def test_tolerancia_no_anticipa_giro_a_306mm_ni_quita_margen_de_avance(self):
        self.iniciar()
        ev=dict(id=1,direccion='IZQUIERDA',color='azul',marca_ns=self.t+200_000_000)
        r=self.paso(senal=ev,sensor=lidar(.306),holgura=HolguraGiro(True,.266,'Contorno',.12))
        self.assertEqual(r[:2],(0.,0.))
        self.assertIn('margen de aproximacion',r[2])
        self.assertIsNotNone(self.c.giros.pendiente)
        self.assertIsNone(self.c.giros.activo)
        self.assertEqual(self.c.cfg.radio_giro_m+self.c.cfg.margen_giro_m,.25)

    def test_flecha_espera_reposo_real_y_fija_referencia_angular_una_sola_vez(self):
        self.iniciar()
        ev=dict(id=1,direccion='DERECHA',color='azul',marca_ns=self.t+200_000_000)
        self.paso(senal=ev,sensor=lidar(.304))
        inicio=self.c.giros.activo['inicio_ns']
        for i in range(1,9):
            self.assertEqual(self.paso(sensor=lidar(.304),x=i*.006,yaw=i*.01)[:2],(0.,0.))
            self.assertFalse(self.c.freno_flecha['confirmado'])
        for _ in range(4):self.assertEqual(self.paso(sensor=lidar(.304))[:2],(0.,0.))
        self.assertTrue(self.c.freno_flecha['confirmado'])
        self.assertEqual(self.c.giros.activo['yaw'],.08)
        self.assertEqual(self.c.giros.activo['inicio_ns'],inicio)
        self.assertLess(self.paso(sensor=lidar(.304))[1],0.)

    def test_holgura_perdida_durante_frenado_o_giro_no_renueva_referencia(self):
        self.iniciar()
        ev=dict(id=1,direccion='IZQUIERDA',color='azul',marca_ns=self.t+200_000_000)
        self.paso(senal=ev,sensor=lidar(.304))
        for _ in range(5):
            self.assertEqual(self.paso(sensor=lidar(.304),
                holgura=HolguraGiro(False,None,'Sin cobertura trasera',.1))[:2],(0.,0.))
        self.assertFalse(self.c.freno_flecha['confirmado'])
        self.assertEqual(self.paso(sensor=lidar(.304))[:2],(0.,0.))
        referencia=self.c.giros.activo['yaw']
        inicio=self.c.giros.activo['inicio_ns']
        self.assertGreater(self.paso(sensor=lidar(.304))[1],0.)
        self.assertEqual(self.paso(sensor=lidar(.304),yaw=.2,
            holgura=HolguraGiro(True,.24,'Contorno ocupado',0.))[:2],(0.,0.))
        self.assertGreater(self.paso(sensor=lidar(.304))[1],0.)
        self.assertEqual(self.c.giros.activo['yaw'],referencia)
        self.assertEqual(self.c.giros.activo['inicio_ns'],inicio)

    def test_senal_lejana_caduca_con_causa_de_aproximacion_sin_giro_temprano(self):
        self.iniciar()
        ev=dict(id=1,direccion='IZQUIERDA',color='azul',marca_ns=self.t+200_000_000)
        self.paso(senal=ev,sensor=lidar(.558))
        for _ in range(152):
            r=self.paso(sensor=lidar(.32),holgura=HolguraGiro(True,.28,'Contorno',.6))
            self.assertEqual(r[:2],(0.,0.))
        self.assertEqual(self.c.fase,'FALLO')
        self.assertIn('Aproximacion a flecha agotada',self.c.motivo)
        self.assertIn('ultimo bloqueo de aproximacion',self.c.motivo)
        self.assertIsNone(self.c.giros.activo)

    def test_freno_bloqueado_agota_plazo_sin_reiniciarlo(self):
        self.iniciar()
        ev=dict(id=1,direccion='IZQUIERDA',color='azul',marca_ns=self.t+200_000_000)
        self.paso(senal=ev,sensor=lidar(.304))
        inicio=self.c.giros.activo['inicio_ns']
        for _ in range(65):
            r=self.paso(sensor=lidar(.304),holgura=HolguraGiro(True,.24,'Contorno ocupado',0.))
            self.assertEqual(r[:2],(0.,0.))
            self.assertEqual(self.c.giros.activo['inicio_ns'],inicio)
        self.assertEqual(self.c.fase,'FALLO')
        self.assertIn('Giro sin completar a tiempo',self.c.motivo)

    def test_cambio_del_cartel_durante_frenado_sigue_detenido(self):
        self.iniciar()
        ev=dict(id=1,direccion='IZQUIERDA',color='azul',marca_ns=self.t+200_000_000)
        self.paso(senal=ev,sensor=lidar(.304))
        ev.update(direccion='DERECHA',marca_ns=self.t+200_000_000)
        self.assertEqual(self.paso(senal=ev,sensor=lidar(.304))[:2],(0.,0.))
        self.assertEqual(self.c.fase,'FALLO')
        self.assertIn('durante frenado',self.c.motivo)

    def test_flecha_prevalece_y_limpia_la_memoria_evasiva(self):
        self.iniciar()
        self.assertGreater(self.paso(sensor=lidar(.37,.6,.5))[1],0.)
        ev=lambda:dict(id=1,direccion='DERECHA',color='azul',marca_ns=self.t+200_000_000)
        self.assertEqual(self.paso(senal=ev(),sensor=lidar(.43,.6,.5))[:2],(.05,0.))
        self.assertIsNone(self.c.evasion)
        self.assertEqual(self.paso(senal=ev(),sensor=lidar(.30,.6,.5))[:2],(0.,0.))
        for _ in range(4):self.assertEqual(self.paso(sensor=lidar(.30,.6,.5))[:2],(0.,0.))
        self.assertEqual(self.paso(sensor=lidar(.30,.6,.5))[:2],(0.,-.4))

    def test_corte_largo_de_odometria_durante_evasion_tambien_enclava(self):
        self.iniciar()
        self.paso(sensor=lidar(.37,.6,.5))
        self.assertIsNotNone(self.c.evasion)
        self.assertEqual(self.paso(dt=2.1,sensor=lidar(.37,.6,.5))[:2],(0.,0.))
        self.assertEqual(self.c.fase,'FALLO')
        self.assertIn('Corte odometrico >2s',self.c.motivo)

    def test_aproximacion_lenta_no_omite_espacio_o_frescura(self):
        self.iniciar()
        ev=lambda:dict(id=1,direccion='IZQUIERDA',color='azul',marca_ns=self.t+200_000_000)
        self.assertEqual(self.paso(senal=ev(),sensor=lidar(.44))[:2],(.05,0.))
        self.assertEqual(self.paso(senal=ev(),sensor=lidar(.43,.27,1.))[:2],(0.,0.))
        self.assertEqual(self.paso(senal=ev(),sensor=lidar(.43,valido=False))[:2],(0.,0.))
        self.assertEqual(self.paso(senal=ev(),sensor=lidar(.43),marca=0)[:2],(0.,0.))
        self.c.cfg=replace(self.c.cfg,radio_giro_m=.30)
        self.assertEqual(self.paso(senal=ev(),sensor=lidar(.43))[:2],(0.,0.))
        self.assertEqual(self.c.fase,'FALLO')
        self.assertIn('margen',self.c.motivo)

    def test_flecha_guardada_sin_imagen_nueva_hasta_30s_no_gira_desde_lejos(self):
        self.iniciar()
        self.paso(senal=dict(id=1, direccion='DERECHA', color='azul', marca_ns=self.t+200_000_000))
        for _ in range(33):
            self.assertEqual(self.paso()[:2],(.1,0.))
        self.assertEqual(self.c.fase,'IDA')
        self.assertEqual(self.c.giros.pendiente['direccion'],'DERECHA')
        for _ in range(120):self.paso()
        self.assertEqual(self.c.fase, 'FALLO')
        self.assertIn('Aproximacion a flecha agotada', self.c.motivo)

    def test_revalidacion_por_imagen_nueva_conserva_flecha_visible_y_limita_espera(self):
        self.iniciar()
        for _ in range(40):
            ev = dict(id=1, direccion='DERECHA', color='azul', marca_ns=self.t+200_000_000)
            self.paso(senal=ev)
        self.assertEqual(self.c.fase, 'IDA')
        self.assertIsNotNone(self.c.giros.pendiente)  # Más de seis segundos, imágenes nuevas.
        inicio=self.c.aproximacion_ns
        self.assertEqual(self.c.giros.pendiente['recibido_ns'],inicio)
        self.assertGreater(self.c.revalidacion_ns,inicio)
        for _ in range(120):
            ev['marca_ns'] = self.t+200_000_000
            self.paso(senal=ev)
        self.assertEqual(self.c.fase, 'FALLO')
        self.assertIn('Aproximacion', self.c.motivo)

    def test_cambio_de_sentido_del_mismo_cartel_pendiente_detiene(self):
        self.iniciar()
        ev = dict(id=1, direccion='DERECHA', color='azul', marca_ns=self.t+200_000_000)
        self.paso(senal=ev)
        ev.update(direccion='IZQUIERDA', marca_ns=self.t+200_000_000)
        self.assertEqual(self.paso(senal=ev)[:2], (0., 0.))
        self.assertEqual(self.c.fase, 'FALLO')
        self.assertIn('DERECHA -> IZQUIERDA',self.c.motivo)
        marca=self.c.giros.pendiente['recibido_ns']
        diagnostico=self.c.diagnostico_giro(self.t,lidar(.522))
        self.assertEqual(diagnostico['conflicto']['esperada'],'DERECHA')
        self.assertEqual(diagnostico['conflicto']['observada'],'IZQUIERDA')
        self.assertEqual(diagnostico['pendiente']['id'],1)
        self.assertEqual(diagnostico['frente_m'],.522)
        self.assertEqual(diagnostico['disparo_m'],.30)
        self.assertIsNone(diagnostico['activo'])
        self.c.diagnostico_giro(self.t+900_000_000,None)
        self.assertEqual(self.c.giros.pendiente['recibido_ns'],marca)  # Diagnosticar no revalida.
        self.paso('REINICIAR',reinicio=self.t+200_000_000)
        self.assertIsNone(self.c.conflicto_senal)

    def test_nuevo_id_no_reemplaza_flecha_guardada_ni_renueva_plazo(self):
        self.iniciar()
        ev=dict(id=1,direccion='IZQUIERDA',color='azul',marca_ns=self.t+200_000_000)
        self.paso(senal=ev)
        inicio=self.c.giros.pendiente['recibido_ns']
        for i in range(40):
            ev.update(id=i+2,marca_ns=self.t+200_000_000)
            self.paso(senal=ev)
        d=self.c.diagnostico_giro(self.t,lidar())
        self.assertEqual(d['pendiente']['id'],1)
        self.assertEqual(self.c.giros.pendiente['recibido_ns'],inicio)
        self.assertGreater(d['pendiente']['edad_memoria_s'],6.)
        self.assertEqual(d['pendiente']['edad_ultima_deteccion_s'],0.)
        ev.update(direccion='DERECHA',marca_ns=self.t+200_000_000)
        self.assertEqual(self.paso(senal=ev)[:2],(0.,0.))
        self.assertEqual(self.c.fase,'FALLO')

    def test_memoria_flecha_expira_aunque_camara_sea_antigua_y_no_reanuda(self):
        self.iniciar()
        ev=dict(id=1,direccion='IZQUIERDA',color='azul',marca_ns=self.t+200_000_000)
        self.paso(senal=ev)
        for _ in range(152):
            self.assertEqual(self.paso(marca=0)[:2],(0.,0.))
        self.assertEqual(self.c.fase,'FALLO')
        self.assertIn('Aproximacion a flecha agotada',self.c.motivo)
        self.assertEqual(self.paso(sensor=lidar(.304))[:2],(0.,0.))

    def test_flecha_en_pausa_odom_se_recibe_y_conserva_hasta_recuperar_sensores(self):
        self.iniciar()
        ev=dict(id=1,direccion='IZQUIERDA',color='azul',marca_ns=self.t+800_000_000)
        self.assertEqual(self.paso(dt=.8,senal=ev,sensor=lidar(.5))[:2],(0.,0.))
        self.assertTrue(self.c.recuperando)
        self.assertEqual(self.c.diagnostico_giro(self.t,lidar())['recibido'],ev)
        # El siguiente fotograma no contiene flecha, antes de recuperar el reposo.
        for _ in range(9):r=self.paso(sensor=lidar(.38))
        self.assertEqual(self.c.fase,'IDA')
        self.assertFalse(self.c.recuperando)
        self.assertEqual(self.c.giros.pendiente['direccion'],'IZQUIERDA')
        self.assertEqual(r[:2],(.05,0.))

    def test_evento_recibido_sin_sensores_no_renueva_30s_y_reinicio_lo_borra(self):
        self.iniciar()
        ev=dict(id=1,direccion='DERECHA',color='azul',marca_ns=self.t+800_000_000)
        self.paso(dt=.8,senal=ev)
        for _ in range(152):self.paso(marca=0)
        self.assertEqual(self.c.fase,'FALLO')
        self.assertIn('Aproximacion a flecha agotada',self.c.motivo)
        self.paso('REINICIAR',reinicio=self.t+200_000_000)
        self.assertIsNone(self.c.diagnostico_giro(self.t,lidar())['recibido'])

    def test_evento_fresco_recibido_con_camara_ausente_no_mueve_y_espera_imagen(self):
        self.iniciar()
        ev=dict(id=1,direccion='IZQUIERDA',color='azul',marca_ns=self.t+200_000_000)
        self.assertEqual(self.paso(senal=ev,marca=0,sensor=lidar(.38))[:2],(0.,0.))
        self.assertEqual(self.c.diagnostico_giro(self.t,lidar())['por_procesar'],ev)
        for _ in range(7):self.assertEqual(self.paso(marca=0)[:2],(0.,0.))
        self.assertEqual(self.paso(sensor=lidar(.38))[:2],(.05,0.))
        self.assertEqual(self.c.giros.pendiente['direccion'],'IZQUIERDA')

    def test_recepcion_no_guarda_flecha_de_otra_fase_color_o_imagen_previa(self):
        ev=dict(id=1,direccion='IZQUIERDA',color='azul',marca_ns=self.t+200_000_000)
        self.paso('ESPERAR',senal=ev)
        self.assertIsNone(self.c.ultima_senal_recibida)
        self.iniciar()
        for cambios in ({'color':'rojo'},{'marca_ns':self.c.desde_ns},
                        {'marca_ns':self.t-2_000_000_000}):
            ev=dict(id=1,direccion='IZQUIERDA',color='azul',marca_ns=self.t+200_000_000)
            ev.update(cambios)
            self.paso(senal=ev)
            self.assertIsNone(self.c.ultima_senal_recibida)
            self.assertIsNone(self.c.senal_por_procesar)
            self.assertIsNone(self.c.giros.pendiente)

    def test_perdida_de_camara_gui_y_lidar_detiene(self):
        self.iniciar()
        self.assertEqual(self.paso(marca=0)[:2], (0., 0.))
        self.assertEqual(self.paso(sensor=lidar(valido=False))[:2], (0., 0.))
        self.t += 700_000_000
        self.assertEqual(self.c.decidir(self.t, self.pose, self.t, lidar(), self.cfg)[:2], (0., 0.))

    def test_corte_coherente_recupera_quieto_y_salto_enclava(self):
        self.iniciar()
        self.assertEqual(self.paso(dt=.8)[:2], (0., 0.))
        self.assertTrue(self.c.recuperando)
        for _ in range(5):
            self.paso()
        self.assertFalse(self.c.recuperando)
        self.paso(x=1.)
        self.assertEqual(self.c.fase, 'FALLO')
        motivo = self.c.motivo
        self.paso('PARAR')
        self.assertEqual(self.c.motivo, motivo)

    def test_barrido_limites_y_corte_largo_bloquean_sin_retorno(self):
        self.llegar_b()
        self.paso(dt=2.1)
        self.assertEqual(self.c.fase, 'FALLO')
        self.assertFalse(self.c.qr_confirmado)
        self.setUp()
        self.llegar_b()
        self.c.inicio_barrido_ns = self.t-90_000_000_000
        self.paso()
        self.assertEqual(self.c.fase, 'FALLO')

    def test_en_rojo_ignora_azul_y_no_avanza_durante_giro_rojo(self):
        self.qr_y_salida()
        ev = dict(id=1, direccion='DERECHA', color='azul', marca_ns=self.t+200_000_000)
        self.paso(senal=ev, sensor=lidar(.40))
        self.assertIsNone(self.c.giros.activo)
        ev.update(id=2, color='rojo', marca_ns=self.t+200_000_000)
        self.assertEqual(self.paso(senal=ev,sensor=lidar(.30))[:2],(0.,0.))
        for _ in range(4):self.assertEqual(self.paso(sensor=lidar(.30))[:2],(0.,0.))
        self.assertEqual(self.paso(sensor=lidar(.30))[:2],(0.,-.4))

    def test_salida_bloqueada_conserva_qr_y_giro_contrario_enclava(self):
        self.qr_y_salida(solo_orientar=True)
        self.assertEqual(self.paso(sensor=lidar(.24))[:2], (0., 0.))
        self.assertTrue(self.c.qr_confirmado)
        self.assertEqual(self.c.fase, 'ORIENTAR_SALIDA')
        self.assertEqual(self.paso(yaw=.2)[:2], (0., 0.))
        self.assertEqual(self.c.fase, 'FALLO')
        self.assertIn('contrario', self.c.motivo)
        self.assertTrue(self.c.qr_confirmado)

    def test_mensajes_invalidos_y_fin_cierran_permiso(self):
        self.iniciar()
        self.c.recibir('{', self.t)
        self.assertEqual(self.c.decidir(self.t, self.pose, self.t, lidar(), self.cfg)[:2], (0., 0.))
        self.paso()
        self.assertEqual(self.c.decidir(self.t, self.pose, self.t, lidar(), self.cfg, fin=True)[:2], (0., 0.))
        self.assertEqual(self.c.fase, 'FALLO')

    def test_evasion_detiene_ante_postes_diagonales_simetricos_en_avance(self):
        from evasion_lidar import analizar_lidar
        from holgura_giro import analizar_holgura
        self.iniciar()
        for dist_poste in (.227, .219):
            with self.subTest(dist_poste=dist_poste):
                scan720 = NS(ranges=[2.5]*720, angle_min=-pi, angle_increment=2*pi/720,
                             range_min=.05, range_max=12.)
                for i in range(720):
                    deg = (-pi+i*scan720.angle_increment)*180/pi
                    if 38 <= abs(deg) <= 44:
                        scan720.ranges[i] = dist_poste
                estado = analizar_lidar(scan720.ranges, scan720.angle_min, scan720.angle_increment,
                                        scan720.range_min, scan720.range_max, cfg=self.cfg)
                holg = analizar_holgura(scan720, (0., 0., 0.), .05, self.cfg)
                vx, wz, _ = self.paso(sensor=estado, holgura=holg)
                self.assertEqual((vx, wz), (0., 0.))
        # Un obstáculo únicamente detrás no bloquea avanzar hacia adelante cuando frente está libre.
        holg_trasera = HolguraGiro(True, .24, 'Trasera', .05, (), 1.8)
        self.assertEqual(self.paso(sensor=lidar(1.8), holgura=holg_trasera)[:2], (.1, 0.))

    def test_qr_confirmado_sobrevive_corte_odometrico_y_reposo_sin_bloquearse(self):
        self.llegar_b()
        qr_marca = self.t + 200_000_000
        # Recibe QR fresco al mismo instante de un corte odométrico de 0.8 s.
        self.assertEqual(self.paso('QR_LISTO', dt=.8, qr_ns=qr_marca, votos=1)[:2], (0., 0.))
        self.assertTrue(self.c.recuperando)
        self.assertIsNotNone(self.c.qr_pendiente)
        self.assertEqual(self.c.fase, 'BUSCAR_B')
        # Tras recuperar odometría (0.7 s de reposo), qr_ns ya supera 1 s pero fue recibido fresco.
        for _ in range(5):
            self.paso('QR_LISTO', dt=.2, qr_ns=qr_marca, votos=1)
        self.assertTrue(self.c.qr_confirmado)
        self.assertEqual(self.c.fase, 'FRENAR_QR')
        for _ in range(4):
            self.paso()
        self.assertEqual(self.c.fase, 'ORIENTAR_SALIDA')

    def test_rearme_cercano_distingue_rechazo_y_no_consume_id_para_siguiente_flecha(self):
        self.iniciar()
        ev1 = dict(id=1, direccion='IZQUIERDA', color='azul', marca_ns=self.t+200_000_000)
        self.paso(senal=ev1, sensor=lidar(.30))
        for _ in range(4):
            self.paso(sensor=lidar(.30))
        for i in range(1, 7):
            self.paso(sensor=lidar(.30), yaw=min(pi/2, i*.3))
        self.assertIsNone(self.c.giros.activo)
        # Avanza 0.30 m (< 0.40 m de rearme) en pasos odométricos válidos y aparece nueva flecha DERECHA (id=2).
        self.paso(x=0., y=.15, sensor=lidar(.9))
        ev2 = lambda: dict(id=2, direccion='DERECHA', color='azul', marca_ns=self.t+200_000_000)
        self.assertEqual(self.paso(x=0., y=.30, senal=ev2(), sensor=lidar(.8))[:2], (.1, 0.))
        d = self.c.diagnostico_giro(self.t, lidar(.8), self.cfg)
        self.assertIsNone(d['pendiente'])
        self.assertIsNotNone(d['rearme'])
        self.assertEqual(d['rearme']['direccion'], 'DERECHA')
        self.assertEqual(self.c.giros.ultimo_id, 1)  # No quemó id=2.
        # Al superar 0.40 m, la misma señal id=2 se admite y al llegar a ~0.305 m (float32) gira a DERECHA.
        self.paso(x=0., y=.45, senal=ev2(), sensor=lidar(.6))
        d2 = self.c.diagnostico_giro(self.t, lidar(.6), self.cfg)
        self.assertIsNone(d2['rearme'])
        self.assertEqual(d2['pendiente']['direccion'], 'DERECHA')
        self.assertEqual(self.paso(sensor=lidar(.3050000071525574))[:2], (0., 0.))
        for _ in range(4):
            self.paso(sensor=lidar(.3050000071525574))
        vx, wz, _ = self.paso(sensor=lidar(.3050000071525574))
        self.assertEqual(vx, 0.)
        self.assertLess(wz, 0.)

    def test_segunda_flecha_se_conserva_sin_rever_cartel_durante_rearme(self):
        self.iniciar()
        self.c.giros.ultima_posicion=(0.,0.)
        for i in range(1,4):self.paso(x=i*.1)
        ev=dict(id=2,direccion='DERECHA',color='azul',marca_ns=self.t+200_000_000)
        self.paso(senal=ev,sensor=lidar(.8))
        inicio=self.c.aproximacion_ns
        self.assertEqual(self.c.senal_por_procesar,ev)
        self.assertEqual(self.c.giros.ultimo_id,0)
        self.assertEqual(self.paso(x=.35,sensor=lidar(.32))[:2],(.05,0.))
        self.assertIsNone(self.c.evasion)
        self.assertEqual(self.c.aproximacion_ns,inicio)
        self.assertEqual(self.paso(x=.35,sensor=lidar(.304))[:2],(0.,0.))
        self.assertIsNone(self.c.giros.activo)  # No omitir el desplazamiento minimo.
        self.paso(x=.45,sensor=lidar(.304))
        for _ in range(5):r=self.paso(sensor=lidar(.304))
        self.assertEqual(r[:2],(0.,-.4))

    def test_rearme_espera_acotada_sin_renovar_por_nuevas_imagenes(self):
        self.iniciar()
        self.c.giros.ultima_posicion=(0.,0.)
        self.paso(x=.15);self.paso(x=.30)
        for _ in range(153):
            ev=dict(id=2,direccion='DERECHA',color='azul',marca_ns=self.t+200_000_000)
            r=self.paso(senal=ev,sensor=lidar(.304))
            self.assertEqual(r[:2],(0.,0.))
        self.assertEqual(self.c.fase,'FALLO')
        self.assertIn('Aproximacion a flecha agotada',self.c.motivo)

    def test_qr_retenido_caduca_durante_pausa_y_pide_relectura_explicita(self):
        self.llegar_b()
        marca=self.t+200_000_000
        self.paso('QR_LISTO',qr_ns=marca,votos=1,marca=0)
        self.assertEqual(self.c.qr_pendiente,marca)
        for _ in range(30):self.paso('QR_LISTO',qr_ns=marca,votos=1,marca=0)
        self.assertIsNone(self.c.qr_pendiente)
        self.assertEqual(self.c.qr_reintentar_ns,marca)
        self.assertFalse(self.c.qr_confirmado)
        # Un frame reciente sin lectura QR nueva no rescata evidencia caducada.
        self.paso('QR_LISTO',qr_ns=marca,votos=1)
        self.assertEqual(self.c.fase,'BUSCAR_B')
        self.paso('QR_LISTO',qr_ns=self.t+200_000_000,votos=1)
        self.assertEqual(self.c.fase,'FRENAR_QR')

    def test_qr_primera_entrega_caducada_no_se_acepta_por_camara_reciente(self):
        self.llegar_b()
        marca=self.t+200_000_000
        for _ in range(8):self.paso('VERIFICAR')
        self.paso('QR_LISTO',qr_ns=marca,votos=1)
        self.assertEqual(self.c.fase,'BUSCAR_B')
        self.assertFalse(self.c.qr_confirmado)
        self.assertIsNone(self.c.qr_pendiente)
        self.assertEqual(self.c.qr_reintentar_ns,marca)


class PreparacionTests(unittest.TestCase):
    def test_reintento_qr_requiere_rechazo_explicito_de_la_misma_lectura(self):
        marca=2_000_000_000
        s=NS(palabra='sol',ultima_marca=marca)
        ensayo=NS(sesion=s,reintentar_qr=Mock())
        v=NS(ensayo=ensayo)
        for estado in (dict(fase='BUSCAR_B',qr_pendiente=False),
                       dict(fase='BUSCAR_B',qr_reintentar_ns=marca-1),
                       dict(fase='FRENAR_QR',qr_reintentar_ns=marca),
                       dict(fase='BUSCAR_B',qr_confirmado=True,qr_reintentar_ns=marca)):
            self.assertFalse(app.aplicar_reintento_qr(v,estado))
        ensayo.reintentar_qr.assert_not_called()
        self.assertTrue(app.aplicar_reintento_qr(v,dict(fase='BUSCAR_B',qr_reintentar_ns=marca)))
        ensayo.reintentar_qr.assert_called_once()

    def puente_sin_proceso(self):
        p=app.PuenteMision.__new__(app.PuenteMision)
        p.cerrojo=threading.Lock()
        p.estado=None
        return p

    def test_confirmacion_breve_no_desaparece_antes_del_envio_y_acuse(self):
        p=self.puente_sin_proceso()
        t=2_000_000_000
        ev=dict(id=1,direccion='IZQUIERDA',color='azul',marca_ns=t)
        p.actualizar('OBSERVAR',t,t,senal=ev)
        p.actualizar('OBSERVAR',t+20_000_000,t+20_000_000)  # GUI más rápida que emisor.
        self.assertEqual(p.datos['senal'],ev)
        lineas=[]
        p.proceso=NS(stdin=Mock(write=lineas.append))
        p.fin=Mock(is_set=Mock(side_effect=[False,True]))
        p.enviar()
        self.assertEqual(json.loads(lineas[0])['senal'],ev)
        self.assertEqual(json.loads(lineas[-1])['orden'],'PARAR')
        p.estado=dict(version=1,recibido_ns=t+30_000_000,reinicio_ns=0,
                      giro=dict(recibido=ev))
        p.actualizar('OBSERVAR',t+40_000_000,t+40_000_000)
        self.assertIsNone(p.datos['senal'])

    def test_reenvio_no_rejuvenece_imagen_y_caducidad_parada_o_reinicio_cancelan(self):
        t=2_000_000_000
        ev=dict(id=1,direccion='IZQUIERDA',color='azul',marca_ns=t)
        for orden,delta in (('OBSERVAR',1_000_000_001),('PARAR',20_000_000),
                            ('ESPERAR',20_000_000),('REINICIAR',20_000_000)):
            with self.subTest(orden=orden):
                p=self.puente_sin_proceso()
                p.actualizar('OBSERVAR',t,t,senal=ev)
                p.actualizar(orden,t+delta,None,reinicio_ns=t if orden=='REINICIAR' else None)
                self.assertIsNone(p.datos['senal'])
                self.assertEqual(p.datos['orden'],orden)
                self.assertIsNone(p.datos['camara_ns'])

    def test_acuse_otro_intento_o_evento_no_descarta_flecha_sin_recibir(self):
        t=2_000_000_000
        ev=dict(id=1,direccion='IZQUIERDA',color='azul',marca_ns=t)
        for acuse,token in ((dict(ev,id=2),0),(ev,123)):
            with self.subTest(acuse=acuse,token=token):
                p=self.puente_sin_proceso()
                p.actualizar('OBSERVAR',t,t,senal=ev)
                p.estado=dict(recibido_ns=t+30_000_000,reinicio_ns=token,giro=dict(recibido=acuse))
                p.actualizar('OBSERVAR',t+40_000_000,t+40_000_000)
                self.assertEqual(p.datos['senal'],ev)

    def test_puente_reenvia_nuevo_fotograma_mismo_id_y_reintento_qr_limpia_expirado(self):
        p=self.puente_sin_proceso()
        t=2_000_000_000
        ev1=dict(id=2,direccion='DERECHA',color='azul',marca_ns=t)
        p.estado=dict(version=1,recibido_ns=t+20_000_000,reinicio_ns=0,giro=dict(recibido=ev1))
        ev2=dict(id=2,direccion='DERECHA',color='azul',marca_ns=t+100_000_000)
        p.actualizar('OBSERVAR',t+100_000_000,t+100_000_000,senal=ev2)
        self.assertEqual(p.datos['senal'],ev2)
        # Reintento de QR si caducó en GUI sin acuse/pendiente del controlador.
        from prueba_meta_robot import EnsayoB
        ensayo=EnsayoB(SistemaFacial(Mock(),Galeria([[1,0]],['Persona A']),Criterio(.5,.1)))
        ensayo.sesion.iniciar('Persona A',t)
        ensayo.sesion.meta_confirmada=True
        ensayo.sesion.busqueda_desde_ns=t+100_000_000
        ensayo.sesion.orden='QR_LISTO'
        ensayo.sesion.palabra='sol'
        ensayo.reintentar_qr()
        self.assertIsNone(ensayo.sesion.palabra)
        self.assertEqual(ensayo.sesion.orden,'BARRER')

    def test_perfil_mision_30cm_separa_ensayo_48cm_y_valida_aproximacion(self):
        _,actual=cargar_ensayo(config=app.CONFIG_SENALES_MISION)
        _,aislado=cargar_ensayo()
        self.assertEqual(actual.esperar_pared_m,.30)
        self.assertEqual(aislado.esperar_pared_m,.48)
        self.assertEqual(actual.confirmaciones,1)
        for cambios in ({'velocidad_aproximacion_m_s':.2}, {'velocidad_aproximacion_m_s':float('nan')},
                        {'velocidad_aproximacion_m_s':True}, {'margen_salida_evasion_m':-.1},
                        {'radio_giro_m':0.}, {'margen_giro_m':0.}, {'margen_aproximacion_m':-.1},
                        {'tolerancia_disparo_m':.02},{'tolerancia_disparo_m':float('nan')}):
            with self.subTest(cambios=cambios), self.assertRaises(ValueError):
                ConfigMision(**cambios)

    def test_boton_reinicio_revoca_orden_bloquea_buscar_y_nuevo_intento_limpia_vision(self):
        sistema=SistemaFacial(Mock(),Galeria([[1,0],[0,1]],['Persona A','Persona B']),Criterio(.5,.1))
        _,cfg=cargar_ensayo()
        puente=app.PuenteMision.__new__(app.PuenteMision)
        puente.cerrojo=threading.Lock()
        sesion,entrada,v=app.crear_intento(sistema,cfg,puente,'Persona A')
        entrada.confirmar()
        v.ensayo.sesion.palabra='Mensaje anterior'
        v.tramo='vuelta'
        v.senal=dict(id=9)
        with patch.object(app.time,'monotonic_ns',return_value=100):
            entrada.raton(app.cv2.EVENT_LBUTTONDOWN,700,685)
            entrada.raton(app.cv2.EVENT_LBUTTONDOWN,700,685)
        self.assertEqual(puente.datos['orden'],'REINICIAR')
        self.assertIsNone(puente.datos['camara_ns'])
        self.assertEqual(puente.datos['reinicio_ns'],100)
        inicio=sesion.inicio_ns
        entrada.texto='Persona B'
        entrada.tecla(13)
        self.assertEqual(sesion.nombre,'Persona A')
        self.assertEqual(sesion.inicio_ns,inicio)
        nueva,e2,v2=app.crear_intento(sistema,cfg,puente)
        self.assertIsNone(nueva.inicio_ns)
        self.assertEqual(e2.texto,'')
        self.assertIsNone(v2.ensayo.sesion.palabra)
        self.assertIsNone(v2.senal)
        self.assertIsNone(v2.tramo)
        self.assertIs(e2.consulta.sistema,sistema)
        e2.texto='Persona B'
        e2.confirmar()
        self.assertEqual(nueva.nombre,'Persona B')
        puente.actualizar('INICIAR',200,200)
        self.assertEqual(puente.datos['reinicio_ns'],100)

    def test_resultado_grande_solo_tras_meta_a_y_qr_incluso_sin_camara(self):
        sistema=SistemaFacial(Mock(),Galeria([[1,0]],['Persona A']),Criterio(.5,.1))
        _,cfg=cargar_ensayo()
        v=app.VisionMision(sistema,cfg)
        sesion=app.SesionInterfaz()
        entrada=app.EntradaBusqueda(app.ConsultaEnVivo(sistema,'Persona A'),sesion)
        sesion.iniciar('Persona A',1)
        v.ensayo.sesion.palabra='ÁRBOL con ñ'
        estado=dict(fase='VUELTA',estado='Seguir rojo',qr_confirmado=True)
        with patch.object(app,'tarjeta_resultado',wraps=app.tarjeta_resultado) as tarjeta:
            app.dibujar(None,'Sin imagen reciente',entrada,v,estado,True)
            tarjeta.assert_not_called()
            estado.update(fase='FIN',estado='Meta A y reposo confirmados')
            panel=app.dibujar(None,'Sin imagen reciente',entrada,v,estado,True)
            tarjeta.assert_called_once_with('ÁRBOL con ñ')
            self.assertEqual(panel.shape,(708,960,3))
            self.assertTrue(np.array_equal(panel[258:650,18:942],app.tarjeta_resultado('ÁRBOL con ñ')))
            estado['qr_confirmado']=False
            tarjeta.reset_mock()
            app.dibujar(None,'Sin imagen reciente',entrada,v,estado,True)
            tarjeta.assert_not_called()

    def test_mensaje_largo_se_ajusta_sin_perder_caracteres_o_tildes(self):
        palabra=('Árbol, pingüino y niño.  '*12)[:256]
        fuente=app.fuente_resultado(28)
        lineas=app.lineas_resultado(palabra,fuente,880)
        self.assertEqual(''.join(lineas),palabra)
        self.assertTrue(all(fuente.getlength(l)<=880 for l in lineas))
        self.assertEqual(app.tarjeta_resultado(palabra).shape,(392,924,3))

    def test_cuadro_visual_meta_independiente_de_llegada_sin_reinferir_repetidos(self):
        sistema=SistemaFacial(Mock(),Galeria([[1,0]],['Persona A']),Criterio(.5,.1))
        _,cfg=cargar_ensayo()
        v=app.VisionMision(sistema,cfg)
        sesion=app.SesionInterfaz()
        t=20_000_000_000
        sesion.iniciar('Persona A',t-2000)
        estado=dict(fase='IDA',fase_desde_ns=t-1000)
        im=np.zeros((240,320,3),np.uint8)
        with patch.object(app.time,'monotonic_ns',return_value=t), \
                patch.object(v.detector_meta,'detectar',return_value=dict(meta=dict(caja=[128,48,64,96]),codigos=[])) as detector, \
                patch('senales_giro.DetectorGiros.detectar',return_value=[]):
            anotada=v.actualizar(im,t,t,estado,sesion,Mock())
            v.actualizar(im,t,t,estado,sesion,Mock())
        detector.assert_called_once()
        self.assertEqual(v.meta,[.4,.2,.2,.4])
        self.assertTrue(np.any(anotada))
        self.assertFalse(np.any(im))
        self.assertEqual(estado['fase'],'IDA')
        v.actualizar(None,None,t+2_000_000_000,estado,sesion,Mock())
        self.assertIsNone(v.meta)
        self.assertIsNone(v.deteccion_meta)

    def test_copia_solo_control_y_perfiles_al_nodo_con_publicador_opcional(self):
        cmd = ['sudo','-n','docker','exec','-i','proyecto2-sim','bash','-c','script',
               'camara-facial','/tmp/proyecto2-facial.X/camara_robot_ros.py',app.TOPICO]
        for mover in (False,True):
            with patch('busqueda_robot.ejecutar_comando', return_value='/cmd_vel [geometry_msgs/msg/TwistStamped]') as base, \
                    patch.object(app,'ejecutar_comando') as extra:
                orden = app.preparar_mision(cmd, 'proyecto2-sim', app.TAREA/'config/evasion_lidar.json', mover)
            self.assertEqual('--mover' in orden, mover)
            self.assertTrue(any(x.endswith('/mision_ros.py') for x in orden))
            self.assertIn('--config-mision', orden)
            copias = [x.args[0] for x in base.call_args_list+extra.call_args_list if 'cp' in x.args[0]]
            self.assertTrue(any(str(app.CONFIG_SENALES_MISION) == x[-2] for x in copias))
            self.assertTrue(any(str(CONTROL) == x[-2] for x in copias))
            self.assertTrue(any(str(app.TAREA/'holgura_giro.py') == x[-2] for x in copias))
            self.assertTrue(any(str(app.TAREA/'telemetria_ros.py') == x[-2] for x in copias))
            self.assertFalse(any('rostros' in x[-2] or 'galeria' in x[-2] for x in copias))

    def test_vision_no_analiza_rostros_antes_de_b_y_cambia_a_rojo(self):
        sistema = SistemaFacial(Mock(), Galeria([[1,0]], ['Persona A']), Criterio(.5,.1))
        _, cfg = cargar_ensayo()
        v = app.VisionMision(sistema, cfg)
        sesion = app.SesionInterfaz()
        ahora = 20_000_000_000
        sesion.iniciar('Persona A', ahora-1000)
        im = np.zeros((200,300,3), np.uint8)
        with patch.object(app.time, 'monotonic_ns', return_value=ahora), \
                patch.object(v.detector_meta, 'detectar', return_value=dict(meta=None,codigos=[])), \
                patch.object(v.ensayo, 'actualizar') as b, patch('senales_giro.DetectorGiros.detectar', return_value=[]):
            for fase in ('IDA','FRENAR_B','VUELTA'):
                v.actualizar(im, ahora, ahora, dict(fase=fase,fase_desde_ns=ahora-100), sesion, Mock())
            b.assert_not_called()
            self.assertEqual(v.consulta.color, 'rojo')
            v.actualizar(im, ahora, ahora, dict(fase='BUSCAR_B',busqueda_desde_ns=ahora-100), sesion, Mock())
            b.assert_called_once()
            self.assertTrue(v.ensayo.sesion.meta_confirmada)

    def test_resultado_local_separa_palabra_identidad_y_llegada(self):
        sesion = app.SesionInterfaz()
        sesion.iniciar('Persona A', 1)
        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp)/'resultado.json'
            estado = dict(fase='ORIENTAR_SALIDA', estado='Orientando', a=dict(x=1),b=dict(x=2))
            app.guardar_estado(p,sesion,estado,'árbol')
            d = json.loads(p.read_text())
            self.assertEqual((d['palabra_qr'],d['identidad_confirmada']), ('árbol','Persona A'))
            self.assertFalse(d['regreso_a_confirmado'])
            estado['fase']='FIN'
            app.guardar_estado(p,sesion,estado,'árbol')
            self.assertTrue(json.loads(p.read_text())['regreso_a_confirmado'])
            self.assertEqual(len(list(Path(tmp).iterdir())),1)

    def test_vision_b_rechaza_otra_identidad_y_acepta_primer_qr_propio_nuevo(self):
        sistema = SistemaFacial(Mock(), Galeria([[1,0]], ['Persona A']), Criterio(.5,.1))
        _, cfg = cargar_ensayo()
        v = app.VisionMision(sistema,cfg)
        sesion = app.SesionInterfaz()
        t = 20_000_000_000
        sesion.iniciar('Persona A',t-2_000_000_000)
        estado = dict(fase='BUSCAR_B',busqueda_desde_ns=t-100_000_000)
        im = np.zeros((200,300,3),np.uint8)
        cara = dict(caja=[30,30,50,70],aceptado=True,nombre='Persona B')
        codigos = [dict(caja=[90,40,30,30],texto='árbol')]
        pausa = Mock()
        with patch('prueba_meta_robot.analizar_candidatos',return_value=[cara]), \
                patch('prueba_meta_robot.detectar_regiones_fotos',return_value=[]), \
                patch.object(v.ensayo.detector,'detectar',return_value=dict(meta=None,codigos=codigos)) as leer:
            with patch.object(app.time,'monotonic_ns',return_value=t):
                v.actualizar(im,t,t,estado,sesion,pausa)
            leer.assert_not_called()
            pausa.assert_not_called()
            self.assertIsNone(v.ensayo.sesion.palabra)
            cara['nombre']='Persona A'
            for i in range(3):
                ahora=t+(i+1)*100_000_000
                with patch.object(app.time,'monotonic_ns',return_value=ahora):
                    v.actualizar(im,ahora,ahora,estado,sesion,pausa)
                self.assertEqual(v.ensayo.sesion.confirmaciones,1)
            self.assertEqual(v.ensayo.sesion.palabra,'árbol')
            self.assertEqual(v.ensayo.sesion.orden,'QR_LISTO')
            self.assertEqual(pausa.call_count,1)

    def test_relectura_qr_tras_rechazo_usa_otra_imagen_y_misma_asociacion_objetivo(self):
        sistema=SistemaFacial(Mock(),Galeria([[1,0]],['Persona A']),Criterio(.5,.1))
        _,cfg=cargar_ensayo()
        v=app.VisionMision(sistema,cfg)
        sesion=app.SesionInterfaz()
        t=20_000_000_000
        sesion.iniciar('Persona A',t-2_000_000_000)
        estado=dict(fase='BUSCAR_B',busqueda_desde_ns=t-100_000_000)
        imagen=np.zeros((200,300,3),np.uint8)
        cara=dict(caja=[30,30,50,70],aceptado=True,nombre='Persona A')
        codigos=[dict(caja=[90,40,30,30],texto='árbol')]
        with patch('prueba_meta_robot.analizar_candidatos',return_value=[cara]), \
                patch('prueba_meta_robot.detectar_regiones_fotos',return_value=[]), \
                patch.object(v.ensayo.detector,'detectar',return_value=dict(meta=None,codigos=codigos)) as leer:
            with patch.object(app.time,'monotonic_ns',return_value=t):
                v.actualizar(imagen,t,t,estado,sesion,Mock())
            self.assertEqual(v.ensayo.sesion.palabra,'árbol')
            self.assertTrue(app.aplicar_reintento_qr(v,dict(estado,qr_reintentar_ns=t)))
            self.assertIsNone(v.ensayo.sesion.palabra)
            with patch.object(app.time,'monotonic_ns',return_value=t+1_500_000_000):
                v.actualizar(imagen,t+1_500_000_000,t+1_500_000_000,estado,sesion,Mock())
            self.assertEqual(v.ensayo.sesion.palabra,'árbol')
            self.assertEqual(v.ensayo.ultima_marca,t+1_500_000_000)
            self.assertEqual(leer.call_count,2)

    def test_ventana_espera_camara_y_cierra_sin_iniciar(self):
        sistema = SistemaFacial(Mock(), Galeria([[1,0]], ['Persona A']), Criterio(.5,.1))
        video = NS(proceso=Mock(), fotograma=Mock(return_value=(None,'Esperando camara')), cerrar=Mock())
        video.proceso.poll.return_value = None
        puente = Mock()
        puente.ultimo_estado.return_value = dict(version=1,fase='ESPERAR',estado='Esperando Buscar',a=None)
        with patch.object(app,'cargar_sistema',return_value=sistema), \
                patch.object(app,'preparar_camara',return_value=['camara']), \
                patch.object(app,'preparar_mision',return_value=['control']) as preparar, \
                patch.object(app,'cargar_transporte',return_value=lambda p:video), \
                patch.object(app,'PuenteMision',return_value=puente), \
                patch.object(app.subprocess,'Popen'), patch.dict(app.os.environ,{'DISPLAY':':simulado'}), \
                patch.object(app.cv2,'namedWindow'), patch.object(app.cv2,'resizeWindow'), \
                patch.object(app.cv2,'setMouseCallback'), patch.object(app.cv2,'imshow') as mostrar, \
                patch.object(app.cv2,'waitKeyEx',return_value=27), \
                patch.object(app.cv2,'destroyAllWindows'), patch('sys.stdout',new_callable=io.StringIO):
            self.assertEqual(app.main(['--ip','10.87.179.100','--dominio','5']),0)
        self.assertFalse(preparar.call_args.args[3])
        self.assertEqual(mostrar.call_args.args[1].shape,(708,960,3))
        self.assertEqual(puente.actualizar.call_args.args[0],'ESPERAR')
        video.cerrar.assert_called_once()
        puente.cerrar.assert_called_once()

    def test_ventana_reinicia_sin_recargar_camara_y_separa_intentos(self):
        sistema=SistemaFacial(Mock(),Galeria([[1,0],[0,1]],['Persona A','Persona B']),Criterio(.5,.1))
        video=NS(proceso=Mock(),fotograma=Mock(return_value=(None,'Esperando camara')),cerrar=Mock())
        video.proceso.poll.return_value=None
        puente=app.PuenteMision.__new__(app.PuenteMision)
        puente.cerrojo=threading.Lock()
        puente.datos={}
        puente.comprobar=Mock()
        puente.cerrar=Mock()
        callbacks=[]
        ticks=[]
        sesion_anterior=[]
        def estado(ahora):
            token=puente.datos.get('reinicio_ns',0)
            fase=('REINICIAR' if len(ticks)==2 else 'ESPERAR' if len(ticks)==3 else 'IDA') if token else ('IDA' if ticks else 'ESPERAR')
            return dict(version=1,fase=fase,estado='Simulado',reinicio_ns=token,
                        a=dict(x=1) if fase=='IDA' else None)
        puente.ultimo_estado=estado
        def tecla(_):
            n=len(ticks)
            ticks.append(n)
            e=callbacks[-1].__self__
            if n==0:
                e.confirmar()
                sesion_anterior.append(e.sesion)
            elif n==1:
                e.raton(app.cv2.EVENT_LBUTTONDOWN,700,685)
            elif n==2:
                e.texto='Persona B'
                e.tecla(13)  # Bloqueado mientras ROS no confirma el reposo.
                self.assertEqual(e.sesion.nombre,'Persona A')
            elif n==3:
                self.assertIsNone(e.sesion.inicio_ns)
                self.assertEqual(e.texto,'')
                e.texto='Persona B'
                e.confirmar()
                self.assertEqual(e.sesion.nombre,'Persona B')
                self.assertIsNot(e.sesion,sesion_anterior[0])
            else:
                return 27
            return -1
        with patch.object(app,'cargar_sistema',return_value=sistema) as cargar, \
                patch.object(app,'preparar_camara',return_value=['camara']) as camara, \
                patch.object(app,'preparar_mision',return_value=['control']), \
                patch.object(app,'cargar_transporte',return_value=lambda p:video), \
                patch.object(app,'PuenteMision',return_value=puente) as crear_puente, \
                patch.object(app.subprocess,'Popen'), patch.dict(app.os.environ,{'DISPLAY':':simulado'}), \
                patch.object(app.cv2,'namedWindow') as ventana, patch.object(app.cv2,'resizeWindow'), \
                patch.object(app.cv2,'setMouseCallback',side_effect=lambda v,c:callbacks.append(c)), \
                patch.object(app.cv2,'imshow'), patch.object(app.cv2,'waitKeyEx',side_effect=tecla), \
                patch.object(app.cv2,'getWindowProperty',return_value=1), \
                patch.object(app.cv2,'destroyAllWindows'), patch.object(app,'guardar_estado') as guardar, \
                patch('sys.stdout',new_callable=io.StringIO):
            self.assertEqual(app.main(['--ip','10.87.179.100','--dominio','5','--nombre','Persona A']),0)
        cargar.assert_called_once()
        camara.assert_called_once()
        crear_puente.assert_called_once()
        ventana.assert_called_once()
        video.cerrar.assert_called_once()
        puente.cerrar.assert_called_once()
        cancelados=[c for c in guardar.call_args_list if c.kwargs.get('interrumpida_por_reinicio')]
        self.assertEqual(len(cancelados),1)
        archivos={str(c.args[0]) for c in guardar.call_args_list}
        self.assertEqual(len(archivos),2)


class NodoTests(unittest.TestCase):
    def test_nodo_real_con_dobles_no_publica_por_defecto_y_publica_solo_una_salida(self):
        base = cargar_ros_simulado()
        def modulo(**valores):
            m = ModuleType('simulado'); m.__dict__.update(valores); return m
        reemplazos = {'rclpy':base.rclpy,
            'busqueda_lidar_ros':base,
            'seguridad_lidar':modulo(EvasionLidarNode=base.EvasionLidarNode),
            'nav_msgs.msg':modulo(Odometry=object),
            'rclpy.executors':modulo(ExternalShutdownException=RuntimeError),
            'rclpy.qos':modulo(QoSProfile=lambda **kw:None,ReliabilityPolicy=NS(BEST_EFFORT=1)),
            'tf2_ros':modulo(TransformException=LookupError),
            'rclpy.signals':modulo(SignalHandlerOptions=NS(NO=0))}
        with patch.dict(sys.modules,reemplazos):
            spec = importlib.util.spec_from_file_location('mision_ros_simulada',app.TAREA/'mision_ros.py')
            ros = importlib.util.module_from_spec(spec); spec.loader.exec_module(ros)
        original,cfg_s = cargar_ensayo(config=app.CONFIG_SENALES_MISION)
        for mover in (False,True):
            c = ControlMision(cfg_s,clase_giros=original.ControlGiro)
            entrada = NS(fin=threading.Event(),linea=None)
            def tomar():
                linea,entrada.linea=entrada.linea,None
                return linea
            entrada.tomar=tomar
            nodo = ros.MisionNode(mover,ConfigEvasion(),entrada,c)
            self.assertEqual(len(nodo.publicadores),int(mover))
            for i in range(3):
                ahora=20_000_000_000+i*300_000_000
                entrada.linea=json.dumps(dict(version=1,secuencia=i,orden='INICIAR',actividad_ns=ahora,
                    camara_ns=ahora,meta=None,senal=None,qr_ns=None,qr_votos=0))
                nodo.pose=dict(frame_id='odom',child_frame_id='base_link',x=0.,y=0.,yaw=0.)
                nodo.pose_ns=ahora
                scan=NS(header=NS(frame_id='base_link'),ranges=[1.8]*360,angle_min=-pi,
                    angle_increment=2*pi/360,range_min=.05,range_max=12.)
                with patch.object(ros.time,'monotonic_ns',return_value=ahora), patch('sys.stdout',new_callable=io.StringIO) as out:
                    nodo._scan_cb(scan); nodo._ciclo()
                estado=json.loads(out.getvalue())
                self.assertEqual(estado['disparo_m'],.30)
                self.assertEqual(estado['mover'],mover)
                self.assertEqual(estado['edad_camara_s'],0.)
                self.assertTrue(estado['inicio_solicitado'])
                self.assertEqual(estado['orden_control'],'INICIAR')
                self.assertIn('motivo_meta',estado)
                self.assertEqual(estado['giro']['color'],'azul')
                self.assertIsNone(estado['giro']['pendiente'])
                self.assertIsNone(estado['giro']['conflicto'])
            # El sensor está 10 cm detrás: rango frontal 0.304 -> centro 0.204.
            nodo.tf_buffer=NS(lookup_transform=lambda *a:NS(transform=NS(
                translation=NS(x=-.10,y=0.,z=.2),rotation=NS(x=0.,y=0.,z=0.,w=1.))))
            for j,desplazamiento in enumerate((-.10,0.,0.,0.,0.,0.,0.),start=3):
                ahora=20_000_000_000+j*300_000_000
                nodo.tf_buffer.lookup_transform=lambda *a,tx=desplazamiento:NS(transform=NS(
                    translation=NS(x=tx,y=0.,z=.2),rotation=NS(x=0.,y=0.,z=0.,w=1.)))
                entrada.linea=json.dumps(dict(version=1,secuencia=j,orden='OBSERVAR',actividad_ns=ahora,
                    camara_ns=ahora,meta=None,senal=dict(id=1,direccion='IZQUIERDA',color='azul',marca_ns=ahora),
                    qr_ns=None,qr_votos=0))
                nodo.pose_ns=ahora
                scan.header.frame_id='laser';scan.ranges[180]=.304
                if j==9:
                    for i in list(range(45))+list(range(315,360)):scan.ranges[i]=float('inf')
                with patch.object(ros.time,'monotonic_ns',return_value=ahora),patch('sys.stdout',new_callable=io.StringIO) as out:
                    nodo._scan_cb(scan);nodo._ciclo()
                estado=json.loads(out.getvalue())
                self.assertEqual(estado['vx'],0.)
                self.assertEqual(estado['wz'],.4 if j in (7,8) else 0.)
                if j==3:self.assertAlmostEqual(estado['giro']['distancia_centro_m'],.204)
                if j==9:self.assertIn('detras',estado['giro']['bloqueo_giro'])
            if mover:
                cmd=nodo.pub_cmd.mensajes[-1].twist
                self.assertEqual((cmd.linear.x,cmd.angular.z),(0.,0.))
                self.assertEqual(nodo.pub_cmd.topico,'/cmd_vel')
            else:
                self.assertIsNone(nodo.pub_cmd)
            token=ahora+300_000_000
            for j in range(5):
                ahora=token+j*200_000_000
                entrada.linea=json.dumps(dict(version=1,secuencia=10+j,orden='REINICIAR',actividad_ns=ahora,
                    camara_ns=None,meta=None,senal=None,qr_ns=None,qr_votos=0,reinicio_ns=token))
                nodo.pose_ns=ahora
                with patch.object(ros.time,'monotonic_ns',return_value=ahora), patch('sys.stdout',new_callable=io.StringIO) as out:
                    nodo._ciclo()
                estado=json.loads(out.getvalue())
                self.assertEqual((estado['vx'],estado['wz']),(0.,0.))
                self.assertEqual(estado['reinicio_ns'],token)
            self.assertEqual(c.fase,'ESPERAR')
            self.assertEqual(len(nodo.publicadores),int(mover))
            self.assertIsNone(c.a)

    def test_telemetria_no_bloquea_lazo_de_control_si_salida_bloquea(self):
        import os
        base = cargar_ros_simulado()
        def modulo(**valores):
            m = ModuleType('simulado'); m.__dict__.update(valores); return m
        reemplazos = {'rclpy':base.rclpy,
            'busqueda_lidar_ros':base,
            'seguridad_lidar':modulo(EvasionLidarNode=base.EvasionLidarNode),
            'nav_msgs.msg':modulo(Odometry=object),
            'rclpy.executors':modulo(ExternalShutdownException=RuntimeError),
            'rclpy.qos':modulo(QoSProfile=lambda **kw:None,ReliabilityPolicy=NS(BEST_EFFORT=1)),
            'tf2_ros':modulo(TransformException=LookupError),
            'rclpy.signals':modulo(SignalHandlerOptions=NS(NO=0))}
        with patch.dict(sys.modules,reemplazos):
            spec = importlib.util.spec_from_file_location('mision_ros_simulada_nb',app.TAREA/'mision_ros.py')
            ros = importlib.util.module_from_spec(spec); spec.loader.exec_module(ros)
        original,cfg_s = cargar_ensayo(config=app.CONFIG_SENALES_MISION)
        c = ControlMision(cfg_s,clase_giros=original.ControlGiro)
        entrada = NS(fin=threading.Event(),tomar=lambda:None)
        nodo = ros.MisionNode(True,ConfigEvasion(),entrada,c)
        ahora=20_000_000_000
        c.recibir(json.dumps(dict(version=1,secuencia=0,orden='INICIAR',actividad_ns=ahora,
            camara_ns=ahora,meta=None,senal=None,qr_ns=None,qr_votos=0)),ahora)
        nodo.pose=dict(frame_id='odom',child_frame_id='base_link',x=0.,y=0.,yaw=0.)
        nodo.pose_ns=ahora
        # El primer ciclo registra A y ordena cero; simularlo antes de evaluar
        # telemetría bloqueada mientras existe una propuesta de avance.
        c.decidir(ahora,nodo.pose,ahora,None,ConfigEvasion())
        scan=NS(header=NS(frame_id='base_link'),ranges=[1.8]*360,angle_min=-pi,
            angle_increment=2*pi/360,range_min=.05,range_max=12.)
        class SalidaBloqueada:
            def write(self, _): raise BlockingIOError('pipe lleno')
            def flush(self): pass
        with patch.object(ros.time,'monotonic_ns',return_value=ahora), patch('sys.stdout',SalidaBloqueada()):
            nodo._scan_cb(scan); nodo._ciclo()
            self.assertEqual((nodo.pub_cmd.mensajes[-1].twist.linear.x,
                              nodo.pub_cmd.mensajes[-1].twist.angular.z),(.1,0.))
            nodo._terminar=True
            nodo._scan_cb(scan); nodo._ciclo()
            self.assertEqual((nodo.pub_cmd.mensajes[-1].twist.linear.x,
                              nodo.pub_cmd.mensajes[-1].twist.angular.z),(0.,0.))
        if os.name == 'posix':
            r_fd, w_fd = os.pipe()
            try:
                os.set_blocking(w_fd, False)
                while True:
                    try:
                        os.write(w_fd, b'x'*4096)
                    except BlockingIOError:
                        break
                nodo2 = ros.MisionNode(True,ConfigEvasion(),entrada,
                                      ControlMision(cfg_s,clase_giros=original.ControlGiro))
                nodo2.pose, nodo2.pose_ns = nodo.pose, ahora
                with os.fdopen(w_fd, 'w', closefd=False) as w_file, \
                        patch.object(ros.time,'monotonic_ns',return_value=ahora+300_000_000), \
                        patch('sys.stdout', w_file):
                    nodo2._terminar = True
                    nodo2._scan_cb(scan); nodo2._ciclo()
                self.assertEqual((nodo2.pub_cmd.mensajes[-1].twist.linear.x,
                                  nodo2.pub_cmd.mensajes[-1].twist.angular.z),(0.,0.))
            finally:
                os.close(r_fd); os.close(w_fd)


class LanzadorMisionTests(unittest.TestCase):
    setUp = lanzador.LanzadorEvasionTests.setUp
    ejecutar = lanzador.LanzadorEvasionTests.ejecutar

    def test_nuevo_modo_exclusivo_sin_contactar_robot(self):
        (self.tarea/'mision_guiada_robot.py').write_text('import json,sys\nprint(json.dumps(sys.argv[1:]))\n')
        for mover in (False,True):
            r = self.ejecutar('--mision-guiada', *(['--mover'] if mover else []))
            self.assertEqual(r.returncode,0,r.stderr)
            self.assertEqual('--mover' in json.loads(r.stdout),mover)
            self.assertFalse(self.registro.exists())
        for extra in ('--solo-evasion','--prueba-meta-b','--evasion-reconocimiento','--senales','--avanzar-hasta-meta'):
            self.assertEqual(self.ejecutar('--mision-guiada',extra).returncode,2)
