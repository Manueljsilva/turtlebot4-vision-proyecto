"""Regresiones de evasión; no necesitan ROS, Docker ni robot."""

from dataclasses import asdict
import math
from types import SimpleNamespace
import unittest

from evasion_lidar import (
    ConfigEvasion, analizar_lidar, calcular_velocidad_evasion, decidir_scan, resumir_cobertura,
)


class EvasionLidarTests(unittest.TestCase):
    def setUp(self):
        self.cfg = ConfigEvasion()
        self.scan = SimpleNamespace(ranges=[2.5] * 360, angle_min=-math.pi,
                                    angle_increment=math.tau / 360,
                                    range_min=0.12, range_max=12.0)

    def analizar(self, offset=0.0):
        s = self.scan
        return analizar_lidar(s.ranges, s.angle_min, s.angle_increment,
                              s.range_min, s.range_max, offset, self.cfg)

    def sector(self, minimo, maximo, valor):
        for i in range(len(self.scan.ranges)):
            ang = math.degrees(self.scan.angle_min + i * self.scan.angle_increment)
            if minimo <= ang <= maximo:
                self.scan.ranges[i] = valor

    def test_camino_observado_libre_avanza(self):
        self.assertEqual(calcular_velocidad_evasion(self.analizar(), self.cfg), (0.1, 0.0))

    def test_obstaculo_frontal_gira_hacia_lado_mas_libre(self):
        self.sector(-20, 20, 0.28)
        self.sector(-90, -40, 0.40)
        vx, wz = calcular_velocidad_evasion(self.analizar(), self.cfg)
        self.assertEqual(vx, 0)
        self.assertGreater(wz, 0)
        self.sector(40, 90, 0.20)
        self.assertLess(calcular_velocidad_evasion(self.analizar(), self.cfg)[1], 0)

    def test_precaucion_reduce_avance(self):
        self.sector(-15, 15, 0.4)
        vx, wz = calcular_velocidad_evasion(self.analizar(), self.cfg)
        self.assertGreater(vx, 0)
        self.assertLess(vx, self.cfg.velocidad_avance)
        self.assertNotEqual(wz, 0)

    def test_regresion_una_lectura_trasera_no_autoriza_avanzar(self):
        self.scan.ranges = [float('nan')] * 360
        self.scan.ranges[0] = 2.0
        estado = self.analizar()
        self.assertFalse(estado.observacion_valida)
        self.assertEqual(calcular_velocidad_evasion(estado, self.cfg), (0, 0))

    def test_frente_desconocido_no_se_considera_libre(self):
        for valor in (float('nan'), float('inf'), -1.0, 15.0):
            with self.subTest(valor=valor):
                self.scan.ranges = [2.5] * 360
                self.sector(-36, 36, valor)
                self.assertEqual(calcular_velocidad_evasion(self.analizar(), self.cfg), (0, 0))

    def test_lado_desconocido_no_autoriza_giro(self):
        self.sector(-20, 20, 0.25)
        self.sector(20, 95, float('nan'))
        self.assertEqual(calcular_velocidad_evasion(self.analizar(), self.cfg), (0, 0))

    def test_cobertura_insuficiente_detiene(self):
        self.sector(-36, 36, float('nan'))
        self.scan.ranges[180] = 2.0
        self.assertEqual(calcular_velocidad_evasion(self.analizar(), self.cfg), (0, 0))

    def test_retorno_inferior_al_minimo_no_se_omite(self):
        self.scan.ranges[180] = 0.05
        self.assertEqual(calcular_velocidad_evasion(self.analizar(), self.cfg)[0], 0)

    def test_geometria_invalida_no_produce_movimiento(self):
        for campo, valor in (('angle_increment', 0), ('angle_min', float('inf')),
                             ('range_min', -1), ('range_max', float('nan'))):
            with self.subTest(campo=campo):
                anterior = getattr(self.scan, campo)
                setattr(self.scan, campo, valor)
                self.assertEqual(calcular_velocidad_evasion(self.analizar(), self.cfg), (0, 0))
                setattr(self.scan, campo, anterior)

    def test_sin_scan_o_sin_tf_detiene(self):
        for scan, yaw in ((None, 0), (self.scan, None), (self.scan, float('nan'))):
            with self.subTest(scan=scan is not None, yaw=yaw):
                self.assertEqual(decidir_scan(scan, yaw, 0.1, self.cfg)[:2], (0, 0))

    def test_scan_caducado_o_edad_invalida_detiene(self):
        for edad in (None, 0.6001, -0.1, float('nan'), float('inf')):
            with self.subTest(edad=edad):
                self.assertEqual(decidir_scan(self.scan, 0, edad, self.cfg)[:2], (0, 0))
        self.assertGreater(decidir_scan(self.scan, 0, 0.2, self.cfg)[0], 0)

    def test_offset_lidar_ubica_obstaculo_al_frente_del_robot(self):
        self.sector(-105, -75, 0.25)
        self.assertGreater(calcular_velocidad_evasion(self.analizar(), self.cfg)[0], 0)
        self.assertEqual(calcular_velocidad_evasion(self.analizar(math.pi / 2), self.cfg)[0], 0)

    def test_configuracion_no_finita_o_fuera_de_limites_se_rechaza(self):
        for nombre in asdict(self.cfg):
            for valor in (float('nan'), float('inf'), True, '0.1'):
                with self.subTest(nombre=nombre, valor=valor):
                    with self.assertRaises(ValueError):
                        ConfigEvasion(**{nombre: valor})
        for cambios in ({'velocidad_avance': 0.307}, {'velocidad_giro': 1.91},
                        {'velocidad_avance': -0.1}, {'edad_maxima_s': 2},
                        {'distancia_parada_m': 0.7}, {'distancia_lateral_m': 0.4},
                        {'cobertura_minima': 0.29}, {'cobertura_minima': 1.01},
                        {'fraccion_errores_maxima': 0.11}, {'fraccion_errores_maxima': -0.01}):
            with self.subTest(cambios=cambios), self.assertRaises(ValueError):
                ConfigEvasion(**cambios)

    def test_informe_identifica_sector_e_inf_nan_y_fuera_de_rango(self):
        self.sector(-36, 36, float('inf'))
        self.scan.ranges[180] = 1.2
        self.scan.ranges[181] = float('nan')
        self.scan.ranges[182] = -1.0
        estado = self.analizar()
        frente = estado.sectores[0]
        self.assertEqual(frente.nombre, 'frente')
        self.assertEqual(frente.utiles, 1)
        self.assertEqual(frente.nan, 1)
        self.assertEqual(frente.fuera_rango, 1)
        self.assertEqual(frente.sin_retorno, frente.total - 3)
        self.assertFalse(frente.suficiente)
        self.assertEqual(frente.total, sum((frente.utiles, frente.sin_retorno,
                                         frente.nan, frente.fuera_rango)))
        resumen = resumir_cobertura(estado)
        self.assertIn('frente=1/', resumen)
        self.assertIn('INSUFICIENTE', resumen)
        self.assertIn('NaN=1', resumen)
        self.assertEqual(calcular_velocidad_evasion(estado, self.cfg), (0, 0))

    def test_conteo_global_no_duplica_lecturas_entre_sectores_solapados(self):
        estado = self.analizar()
        self.assertEqual(estado.lecturas_validas, 360)
        self.assertTrue(all(s.suficiente for s in estado.sectores))

    def test_umbrales_precaucion_y_parada_son_distintos(self):
        for distancia, avanzar, girar in ((0.46, True, False), (0.44, True, True),
                                         (0.31, False, True)):
            with self.subTest(distancia=distancia):
                self.scan.ranges = [2.5] * 360
                self.sector(-20, 20, distancia)
                vx, wz = calcular_velocidad_evasion(self.analizar(), self.cfg)
                self.assertEqual(vx > 0, avanzar)
                self.assertEqual(wz != 0, girar)

    def scan_sin_retornos_parciales(self, valor=1.0):
        # Patrón sintético de 40 % finito y 60 % +inf distribuido angularmente,
        # parecido a los porcentajes del registro; no reconstruye el scan real.
        self.scan.ranges = [valor if i % 5 < 2 else float('inf') for i in range(720)]
        self.scan.angle_increment = math.tau / 720

    def test_retornos_finitos_40_por_ciento_con_inf_no_bloquean_el_avance(self):
        self.scan_sin_retornos_parciales()
        estado = self.analizar()
        self.assertTrue(estado.observacion_valida)
        self.assertTrue(all(0.35 <= s.utiles / s.total <= 0.45 for s in estado.sectores))
        self.assertEqual(calcular_velocidad_evasion(estado, self.cfg), (0.1, 0))
        self.assertIn('mín=30%', resumir_cobertura(estado))

    def test_mismo_patron_con_umbral_50_reproduce_bloqueo_anterior(self):
        self.scan_sin_retornos_parciales()
        s = self.scan
        cfg = ConfigEvasion(cobertura_minima=0.50)
        estado = analizar_lidar(s.ranges, s.angle_min, s.angle_increment,
                               s.range_min, s.range_max, cfg=cfg)
        self.assertFalse(estado.observacion_valida)
        self.assertEqual(calcular_velocidad_evasion(estado, cfg), (0, 0))

    def test_sin_retornos_totales_y_cobertura_menor_30_siguen_detenidos(self):
        for paso in (None, 5):
            with self.subTest(paso=paso):
                self.scan.ranges = [1.0 if paso and i % paso == 0 else float('inf')
                                    for i in range(720)]
                self.scan.angle_increment = math.tau / 720
                estado = self.analizar()
                self.assertFalse(estado.observacion_valida)
                self.assertEqual(calcular_velocidad_evasion(estado, self.cfg), (0, 0))

    def test_40_por_ciento_finito_y_resto_nan_sigue_detenido(self):
        self.scan_sin_retornos_parciales()
        self.scan.ranges = [float('nan') if v == float('inf') else v for v in self.scan.ranges]
        self.assertEqual(calcular_velocidad_evasion(self.analizar(), self.cfg), (0, 0))

    def test_obstaculo_cercano_no_se_oculta_entre_inf(self):
        self.scan_sin_retornos_parciales()
        self.scan.ranges[360] = 0.25
        vx, wz = calcular_velocidad_evasion(self.analizar(), self.cfg)
        self.assertEqual(vx, 0)
        self.assertNotEqual(wz, 0)

    def test_inf_negativo_obliga_a_detener_aunque_la_cobertura_sea_buena(self):
        for i in (180, 130):
            with self.subTest(indice=i):
                self.scan.ranges = [2.5] * 360
                self.scan.ranges[i] = float('-inf')
                estado = self.analizar()
                self.assertTrue(estado.proximidad_no_cuantificable)
                self.assertEqual(calcular_velocidad_evasion(estado, self.cfg), (0, 0))

    def test_correccion_lateral_empieza_a_distancia_configurada(self):
        for distancia, corregir in ((0.29, False), (0.27, True)):
            with self.subTest(distancia=distancia):
                self.scan.ranges = [2.5] * 360
                self.sector(50, 80, distancia)
                vx, wz = calcular_velocidad_evasion(self.analizar(), self.cfg)
                self.assertGreater(vx, 0)
                self.assertEqual(wz < 0, corregir)


if __name__ == '__main__':
    unittest.main()
