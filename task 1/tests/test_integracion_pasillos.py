"""Regresiones de integración con sensores simulados; sin publicar ni mover hardware."""
import json
from dataclasses import replace
from pathlib import Path
import unittest

from evasion_lidar import cargar_config
from holgura_giro import HolguraGiro, distancia_libre_atras
import tests.test_mision_guiada as base


class IntegracionPasillosTests(unittest.TestCase):
    def setUp(self):
        self.s = base.MisionTests()
        self.s.setUp()
        self.s.cfg = cargar_config(Path(__file__).parents[1]/'config/evasion_lidar.json')
        self.s.iniciar()

    def sin_video(self, sensor=None, holgura=None, **pose):
        s = self.s
        s.t += 200_000_000
        s.pose.update(pose)
        s.c.recibir(json.dumps(dict(version=1,secuencia=s.seq,orden='OBSERVAR',
            actividad_ns=s.t,camara_ns=None,meta=None,senal=None,qr_ns=None,qr_votos=0)),s.t)
        s.seq += 1
        sensor = sensor or base.lidar()
        if holgura is None:
            holgura = HolguraGiro(True,min(sensor.dist_frente,sensor.dist_izquierda,sensor.dist_derecha),'Simulado',0.)
        return s.c.decidir(s.t,s.pose,s.t,sensor,s.cfg,holgura=holgura)

    def test_sin_video_real_none_avanza_recto_en_ida_y_vuelta_sin_inventar_meta(self):
        for fase in ('IDA','VUELTA'):
            self.s.c.fase = fase
            self.s.c.b = dict(self.s.pose,x=2.)
            for _ in range(3):
                self.assertEqual(self.sin_video()[:2],(.1,0.))
            self.assertEqual(self.s.c.meta_nueva.votos,0)

    def test_muro_sin_video_espera_imagen_posterior_a_la_parada_antes_de_evadir(self):
        self.assertEqual(self.sin_video(sensor=base.lidar(.41))[:2],(0.,0.))
        marca = self.s.c.espera_camara_muro_ns
        self.assertEqual(self.s.paso(sensor=base.lidar(.35),marca=marca)[:2],(0.,0.))
        self.assertEqual(self.s.c.espera_camara_muro_ns,marca)
        for _ in range(3):
            self.assertEqual(self.sin_video(sensor=base.lidar(.35))[:2],(0.,0.))
        r = self.s.paso(sensor=base.lidar(.35))
        self.assertEqual(r[0],0.)
        self.assertNotEqual(r[1],0.)
        self.assertIsNone(self.s.c.espera_camara_muro_ns)

    def test_muro_sin_video_al_recuperar_flecha_respeta_su_lado_y_umbral(self):
        self.sin_video(sensor=base.lidar(.41))
        ev = dict(id=1,direccion='DERECHA',color='azul',marca_ns=self.s.t+200_000_000)
        r = self.s.paso(sensor=base.lidar(.35),senal=ev)
        self.assertEqual(r[:2],(.05,0.))
        self.assertEqual(self.s.c.giros.pendiente['direccion'],'DERECHA')
        for _ in range(7):r=self.s.paso(sensor=base.lidar(.30))
        self.assertEqual(r[0],0.)
        self.assertLess(r[1],0.)

    def test_bandera_posterior_a_espera_se_acepta_solo_con_guardas_de_llegada(self):
        for i in range(12):self.s.paso(x=min(1.,i*.1))
        self.assertEqual(self.sin_video(sensor=base.lidar(.4))[:2],(0.,0.))
        self.assertEqual(self.s.c.fase,'IDA')
        self.s.paso(meta=base.META,sensor=base.lidar(.4))
        self.assertEqual(self.s.c.fase,'FRENAR_B')

    def test_perdida_video_no_omite_gui_odom_lidar_tf_ni_frescura(self):
        self.assertEqual(self.sin_video(sensor=base.lidar(valido=False))[:2],(0.,0.))
        for h in (HolguraGiro(False,None,'Sin TF',0.),HolguraGiro(True,2.,'Viejo',.7)):
            self.assertEqual(self.sin_video(holgura=h)[:2],(0.,0.))
        self.s.t += 700_000_000
        self.assertEqual(self.s.c.decidir(self.s.t,self.s.pose,self.s.t,base.lidar(),self.s.cfg)[:2],(0.,0.))

    def test_no_gira_por_inversion_ni_correccion_con_obstaculo_a_19cm(self):
        h = HolguraGiro(True,.19,'Ocupado',0.,puntos_base=((.22,0.),(-1.,0.),(0.,.19)))
        self.s.c.evasion=dict(sentido=1.,inicio_ns=self.s.t,yaw=self.s.c.yaw_total)
        for _ in range(10):
            r = self.s.paso(sensor=base.lidar(.22,.5,.5),holgura=h)
            self.assertEqual(r[1],0.)  # Puede escapar hacia atrás, jamás girar dentro de la huella.
        self.s.c.evasion = None
        self.s.c.escape_atras = None
        r = self.s.paso(sensor=base.lidar(.8,.19,.6),holgura=h)
        self.assertEqual(r[1],0.)

    def test_escape_se_frena_por_obstaculo_trasero_y_exige_reposo(self):
        h = HolguraGiro(True,.19,'Esquina',0.,puntos_base=((.19,0.),(-1.,0.),(0.,.5)))
        r = self.s.paso(sensor=base.lidar(.19,.5,.5),holgura=h)
        self.assertEqual(r[:2],(-.06,0.))
        cerrado = replace(h,puntos_base=((.19,0.),(-.26,0.),(0.,.5)))
        self.assertEqual(self.s.paso(sensor=base.lidar(.19,.5,.5),holgura=cerrado)[:2],(0.,0.))
        # Reaparecer espacio no salta el reposo ni publica giro en ese ciclo.
        libre = HolguraGiro(True,.8,'Libre',0.,puntos_base=((.8,0.),(-1.,0.),(0.,.8)))
        self.assertEqual(self.s.paso(holgura=libre)[:2],(0.,0.))
        for _ in range(5):self.s.paso(holgura=libre)
        self.assertIsNone(self.s.c.escape_atras)

    def test_escape_acotado_por_distancia_y_tiempo_sin_reiniciar_plazo(self):
        h = HolguraGiro(True,.19,'Esquina',0.,puntos_base=((.19,0.),(-1.,0.),(0.,.5)))
        self.s.paso(sensor=base.lidar(.19,.5,.5),holgura=h)
        self.assertEqual(self.s.paso(sensor=base.lidar(.19,.5,.5),holgura=h,x=-.12)[:2],(0.,0.))
        self.setUp()
        self.s.paso(sensor=base.lidar(.19,.5,.5),holgura=h)
        for _ in range(11):r=self.s.paso(sensor=base.lidar(.19,.5,.5),holgura=h)
        self.assertEqual(r[:2],(0.,0.))

    def test_escape_rechaza_trasero_desconocido_o_caducado_y_respeta_margen_por_edad(self):
        for h in (HolguraGiro(False,None,'Sin TF',0.),HolguraGiro(True,.19,'Viejo',.7),
                  HolguraGiro(True,.19,'Sin puntos',0.),
                  HolguraGiro(True,.19,'Margen insuficiente',.6,puntos_base=((.19,0.),(-.30,0.)))):
            self.assertEqual(self.s.paso(sensor=base.lidar(.19,.5,.5),holgura=h)[:2],(0.,0.))
        self.assertIsNone(distancia_libre_atras(None,.2,.24))

    def test_semiancho_pasillo_comprobado_se_conserva_en_20cm_y_giro_en_24cm(self):
        h = HolguraGiro(True,.21,'Pasillo',0.,puntos_base=((1.,0.),(-1.,0.),(0.,.21),(0.,-.21)))
        self.assertEqual(self.s.paso(sensor=base.lidar(1.,.21,.21),holgura=h)[:2],(.1,0.))
        self.assertIsNotNone(self.s.c.bloqueo_giro(base.lidar(1.,.21,.21),self.s.cfg))

    def test_cambio_marco_o_salto_no_reancla_A_y_no_reanuda_aunque_vuelvan_datos(self):
        for cambio in (dict(frame_id='otro_odom'),dict(x=3.),dict(yaw=.6)):
            self.setUp()
            a = dict(self.s.c.a)
            self.assertEqual(self.s.paso(**cambio)[:2],(0.,0.))
            self.assertEqual(self.s.c.fase,'FALLO')
            self.assertEqual(self.s.c.a,a)
            for _ in range(6):self.assertEqual(self.s.paso()[:2],(0.,0.))

    def test_avance_normal_tambien_exige_margen_longitudinal_en_el_borde_lateral(self):
        h = HolguraGiro(True,.2006,'Diagonal al borde del perfil',.05,
                       puntos_base=((1.,0.),(-1.,0.),(.025,.199),(0.,-.6)))
        r = self.s.paso(sensor=base.lidar(1.,.2006,.6),holgura=h)
        self.assertEqual(r[:2],(0.,0.))

    def test_corte_breve_coherente_recupera_reposo_sin_perder_A_ni_ruta(self):
        a = dict(self.s.c.a)
        ruta = list(self.s.c.ruta_ida)
        self.assertEqual(self.s.paso(dt=.8)[:2],(0.,0.))
        for _ in range(3):self.assertEqual(self.s.paso()[:2],(0.,0.))
        self.assertTrue(self.s.c.recuperando)
        for _ in range(2):self.s.paso()
        self.assertFalse(self.s.c.recuperando)
        self.assertEqual(self.s.c.a,a)
        self.assertEqual(self.s.c.ruta_ida,ruta)

    def test_corte_largo_durante_ruta_de_regreso_enclava_y_conserva_QR(self):
        self.s.c.fase='VUELTA';self.s.c.qr_confirmado=True
        self.s.c.b=dict(self.s.pose,x=2.)
        self.s.c._girando_ruta=dict(inicio_ns=self.s.t,sentido=1.)
        self.assertEqual(self.s.paso(dt=2.1)[:2],(0.,0.))
        self.assertEqual(self.s.c.fase,'FALLO')
        self.assertTrue(self.s.c.qr_confirmado)
