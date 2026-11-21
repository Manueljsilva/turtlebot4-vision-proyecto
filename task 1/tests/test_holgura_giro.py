"""Geometría de contorno sintética; no certifica medidas ni frenado del robot."""

from math import pi
from types import SimpleNamespace as NS
import unittest

from evasion_lidar import ConfigEvasion
from holgura_giro import analizar_holgura, transformacion_plana


def scan(r=1.):
    return NS(ranges=[r]*360,angle_min=-pi,angle_increment=pi/180,
              range_min=.05,range_max=12.)


class HolguraTests(unittest.TestCase):
    def setUp(self):
        self.cfg=ConfigEvasion()

    def analizar(self,s,tf=(0.,0.,0.),edad=.05):
        return analizar_holgura(s,tf,edad,self.cfg)

    def test_rangos_31cm_no_son_bloqueo_32cm_y_tf_si_cambia_la_distancia_real(self):
        s=scan(.31)
        h=self.analizar(s)
        self.assertTrue(h.valida)
        self.assertAlmostEqual(h.distancia_minima_m,.31)
        trasladada=self.analizar(s,(.10,0.,pi/2))
        self.assertTrue(trasladada.valida)
        self.assertAlmostEqual(trasladada.distancia_minima_m,.21)

    def test_flecha_a_30cm_del_sensor_puede_estar_dentro_del_radio_desde_el_centro(self):
        s=scan(1.8)
        s.ranges[180]=.30
        h=self.analizar(s,(-.10,0.,0.))
        self.assertTrue(h.valida)
        self.assertAlmostEqual(h.distancia_minima_m,.20)

    def test_obstaculo_detras_se_incluye_en_la_huella(self):
        s=scan(1.8)
        s.ranges[0]=.20
        self.assertAlmostEqual(self.analizar(s).distancia_minima_m,.20)

    def test_contorno_sin_cobertura_trasera_no_se_declara_libre(self):
        s=scan(1.8)
        for i in list(range(45))+list(range(315,360)):s.ranges[i]=float('inf')
        h=self.analizar(s)
        self.assertFalse(h.valida)
        self.assertIn('detras',h.motivo)
        self.assertIsNone(h.distancia_minima_m)

    def test_nan_mas_del_diez_por_ciento_y_proximidad_bloquean(self):
        for r in (float('-inf'),0.,.01):
            with self.subTest(r=r):
                s=scan();s.ranges[0]=r
                self.assertFalse(self.analizar(s).valida)
        s=scan()
        for i in range(15):s.ranges[i]=float('nan')
        self.assertFalse(self.analizar(s).valida)

    def test_cobertura_parcial_y_geometria_imposible_se_rechazan(self):
        for cambios in (dict(ranges=[1.]*180),dict(angle_increment=0.),
                        dict(angle_min=float('nan')),dict(range_min=13.)):
            s=scan();s.__dict__.update(cambios)
            with self.subTest(cambios=cambios):self.assertFalse(self.analizar(s).valida)

    def test_scan_invertido_completo_tambien_se_admite(self):
        s=scan();s.angle_min=pi;s.angle_increment=-pi/180
        self.assertTrue(self.analizar(s).valida)

    def test_frescura_y_tf_ausente_no_tienen_suposicion_por_defecto(self):
        for edad in (None,-.1,.61,float('nan')):
            self.assertFalse(self.analizar(scan(),edad=edad).valida)
        for tf in (None,(0.,0.,float('nan'))):
            self.assertFalse(self.analizar(scan(),tf=tf).valida)

    def test_tf_normaliza_cuaternion_y_rechaza_inclinacion_o_traslacion_invalida(self):
        t=NS(x=.08,y=-.03,z=.2)
        self.assertEqual(transformacion_plana(t,NS(x=0.,y=0.,z=0.,w=2.)),(.08,-.03,0.))
        for q in (NS(x=0.,y=0.,z=0.,w=0.),NS(x=.3,y=0.,z=0.,w=1.)):
            self.assertIsNone(transformacion_plana(t,q))
        t.x=float('inf')
        self.assertIsNone(transformacion_plana(t,NS(x=0.,y=0.,z=0.,w=1.)))

    def test_distancia_avance_cubre_diagonales_y_omite_solo_trasero(self):
        s=scan(1.8)
        s.ranges[0]=.20
        h=self.analizar(s)
        self.assertTrue(h.valida)
        self.assertAlmostEqual(h.distancia_minima_m,.20)
        self.assertAlmostEqual(h.distancia_avance_m,1.8)
        s2=NS(ranges=[2.5]*720,angle_min=-pi,angle_increment=2*pi/720,range_min=.05,range_max=12.)
        for i in range(720):
            deg=(-pi+i*s2.angle_increment)*180/pi
            if 38<=abs(deg)<=44:s2.ranges[i]=.219
        h2=self.analizar(s2)
        self.assertTrue(h2.valida)
        self.assertAlmostEqual(h2.distancia_minima_m,.219)
        self.assertAlmostEqual(h2.distancia_avance_m,.219)

    def test_recorrido_recto_no_infla_el_robot_hacia_una_pared_lateral(self):
        from holgura_giro import distancia_libre_recta
        s=scan(2.5)
        s.ranges[90]=s.ranges[270]=.275
        s.ranges[180]=.435
        h=self.analizar(s)
        self.assertAlmostEqual(h.distancia_minima_m,.275)
        self.assertAlmostEqual(distancia_libre_recta(h,.25),.185)

    def test_recorrido_recto_protege_diagonales_y_obstaculo_dentro_de_la_huella(self):
        from holgura_giro import distancia_libre_recta
        s=scan(2.5)
        s.ranges[190]=.275  # A 10 grados: invade el siguiente tramo recto.
        self.assertLess(distancia_libre_recta(self.analizar(s),.25),.026)
        s.ranges[270]=.249
        self.assertEqual(distancia_libre_recta(self.analizar(s),.25),0.)

    def test_recorrido_recto_respeta_tf_radio_y_no_inventa_cobertura(self):
        from holgura_giro import distancia_libre_recta, HolguraGiro
        s=scan(2.5);s.ranges[180]=.435
        h=self.analizar(s,(-.04,0.,0.))
        self.assertAlmostEqual(distancia_libre_recta(h,.25),.145)
        self.assertAlmostEqual(distancia_libre_recta(h,.27),.125)
        self.assertIsNone(distancia_libre_recta(HolguraGiro(True,.4,'Sin puntos',.1),.25))
        s.ranges[0]=float('-inf')
        self.assertIsNone(distancia_libre_recta(self.analizar(s),.25))
