import math
import unittest
import numpy as np

from exploracion import (
    MAX_VEL_ANGULAR,
    MAX_VEL_LINEAL,
    analizar_lidar,
    calcular_velocidad_evasion,
    detectar_fronteras,
    dilatar_ocupacion,
)


class ExploracionFronterasTests(unittest.TestCase):
    def test_detecta_frontera_libre_y_evita_paredes_y_cajas(self):
        # Grilla 30x30: desconocido (-1) a la derecha, libre (0) a la izquierda,
        # y una caja ocupada (100) en medio de parte de la frontera.
        grilla = np.full((30, 30), -1, dtype=np.int16)
        grilla[2:28, 2:16] = 0
        # Paredes exteriores arriba y abajo
        grilla[2, 2:16] = 100
        grilla[27, 2:16] = 100
        # Caja en la mitad inferior de la frontera
        grilla[16:22, 13:16] = 100

        fronteras = detectar_fronteras(
            grilla=grilla,
            resolucion=0.05,
            origen_x=0.0,
            origen_y=0.0,
            robot_x=0.25,
            robot_y=0.50,
            robot_yaw=0.0,
            radio_seguridad_celdas=3,
            min_tamano_cluster=4,
        )

        self.assertGreaterEqual(len(fronteras), 1)
        # Ninguna frontera debe caer cerca de la caja ni de las paredes
        for f in fronteras:
            cx = int(f.x / 0.05)
            cy = int(f.y / 0.05)
            self.assertEqual(grilla[cy, cx], 0)
            # Debe respetar el margen de seguridad respecto a celdas ocupadas
            ocupados = grilla >= 50
            bloqueado = dilatar_ocupacion(ocupados, 3)
            self.assertFalse(bloqueado[cy, cx])

    def test_lista_negra_excluye_frontera_inalcanzable(self):
        grilla = np.full((24, 24), -1, dtype=np.int16)
        grilla[4:20, 4:16] = 0
        # Paredes arriba, abajo e izquierda para dejar una única frontera a la derecha
        grilla[4, 4:16] = 100
        grilla[19, 4:16] = 100
        grilla[4:20, 4] = 100

        primera = detectar_fronteras(
            grilla=grilla,
            resolucion=0.05,
            origen_x=0.0,
            origen_y=0.0,
            robot_x=0.35,
            robot_y=0.60,
            radio_seguridad_celdas=2,
            min_tamano_cluster=4,
        )
        self.assertEqual(len(primera), 1)
        objetivo = primera[0]

        segunda = detectar_fronteras(
            grilla=grilla,
            resolucion=0.05,
            origen_x=0.0,
            origen_y=0.0,
            robot_x=0.35,
            robot_y=0.60,
            radio_seguridad_celdas=2,
            min_tamano_cluster=4,
            lista_negra=[(objetivo.x, objetivo.y)],
            radio_exclusion_m=0.40,
        )
        self.assertEqual(segunda, [])

    def test_descarta_ruido_menor_al_cluster_minimo(self):
        grilla = np.full((20, 20), 0, dtype=np.int16)
        # Dos celdas aisladas desconocidas
        grilla[10, 10] = -1
        grilla[10, 11] = -1
        fronteras = detectar_fronteras(
            grilla=grilla,
            resolucion=0.05,
            origen_x=0.0,
            origen_y=0.0,
            robot_x=0.1,
            robot_y=0.1,
            radio_seguridad_celdas=1,
            min_tamano_cluster=8,
        )
        self.assertEqual(fronteras, [])


class EvasionLidarTests(unittest.TestCase):
    def _scan_uniforme(self, valor: float, n: int = 360) -> tuple[list[float], float, float]:
        angle_min = -math.pi
        angle_increment = (2.0 * math.pi) / n
        return [valor] * n, angle_min, angle_increment

    def test_camino_libre_avanza_recto_sin_exceder_limites(self):
        ranges, amin, ainc = self._scan_uniforme(2.5)
        estado = analizar_lidar(ranges, amin, ainc)
        self.assertFalse(estado.peligro_parada)
        self.assertFalse(estado.precaucion)
        vx, vw = calcular_velocidad_evasion(estado)
        self.assertGreater(vx, 0.0)
        self.assertEqual(vw, 0.0)
        self.assertLessEqual(vx, MAX_VEL_LINEAL)
        self.assertLessEqual(abs(vw), MAX_VEL_ANGULAR)

    def test_caja_al_frente_y_izquierda_libre_gira_a_la_izquierda(self):
        ranges, amin, ainc = self._scan_uniforme(2.0)
        for i in range(len(ranges)):
            ang = amin + i * ainc
            # Caja frente al robot (|ang| < 20 deg)
            if abs(ang) < math.radians(20.0):
                ranges[i] = 0.28
            # Caja también por el lado derecho (-70 a -30 deg)
            elif math.radians(-70.0) <= ang <= math.radians(-30.0):
                ranges[i] = 0.40

        estado = analizar_lidar(ranges, amin, ainc, distancia_parada_m=0.35)
        self.assertTrue(estado.peligro_parada)
        vx, vw = calcular_velocidad_evasion(estado)
        self.assertEqual(vx, 0.0)
        self.assertGreater(vw, 0.0)  # Giro positivo = izquierda (más libre)

    def test_sin_lecturas_validas_detiene_robot(self):
        estado = analizar_lidar([float("nan")] * 20, -math.pi, 0.1)
        self.assertTrue(estado.peligro_parada)
        self.assertEqual(calcular_velocidad_evasion(estado), (0.0, 0.0))

    def test_offset_rotacion_lidar_en_base_link(self):
        # Si el LiDAR está montado rotado +90 deg (+pi/2), un obstáculo en -pi/2
        # del LiDAR corresponde al frente (0 rad) del robot en base_link.
        ranges, amin, ainc = self._scan_uniforme(2.5)
        for i in range(len(ranges)):
            ang = amin + i * ainc
            if abs(ang - (-math.pi / 2.0)) < math.radians(15.0):
                ranges[i] = 0.25

        sin_offset = analizar_lidar(ranges, amin, ainc, angle_offset_rad=0.0)
        self.assertFalse(sin_offset.peligro_parada)

        con_offset = analizar_lidar(ranges, amin, ainc, angle_offset_rad=math.pi / 2.0)
        self.assertTrue(con_offset.peligro_parada)


if __name__ == "__main__":
    unittest.main()
