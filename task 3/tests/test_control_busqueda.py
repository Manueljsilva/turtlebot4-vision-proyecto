"""Permisos, A y confirmación facial sin red ni hardware."""

import json
from types import SimpleNamespace as NS
import unittest

from control_busqueda import PuertaBusqueda, SesionBusqueda, pose_odometria


AHORA = 5_000_000_000
POSE = dict(frame_id='odom', child_frame_id='base_link', x=2., y=-1., yaw=0.3)


def mensaje(orden='BUSCAR', secuencia=0, actividad=AHORA, camara=AHORA):
    return json.dumps(dict(version=1, secuencia=secuencia, orden=orden,
                           actividad_ns=actividad, camara_ns=camara))


class PuertaBusquedaTests(unittest.TestCase):
    def setUp(self):
        self.p = PuertaBusqueda()

    def test_abrir_programa_no_permita_mover_ni_inventa_a(self):
        self.assertFalse(self.p.permiso(AHORA, POSE, AHORA)[0])
        self.p.recibir(mensaje('ESPERAR'), AHORA)
        self.assertFalse(self.p.permiso(AHORA, POSE, AHORA)[0])
        self.assertIsNone(self.p.punto_a)

    def test_inicio_exige_pose_nueva_y_conserva_a_sin_sustituir_por_pose_actual(self):
        self.p.recibir(mensaje(), AHORA)
        self.assertFalse(self.p.permiso(AHORA, None, None)[0])
        self.assertFalse(self.p.permiso(AHORA, POSE, AHORA-1)[0])
        self.assertIsNone(self.p.punto_a)
        self.assertTrue(self.p.permiso(AHORA, POSE, AHORA)[0])
        a = dict(self.p.punto_a)
        self.assertTrue(self.p.permiso(AHORA+10, dict(POSE, x=5), AHORA+10)[0])
        self.assertEqual(self.p.punto_a, a)

    def test_falta_camara_registra_a_pero_no_autoriza_movimiento(self):
        self.p.recibir(mensaje(camara=None), AHORA)
        self.assertFalse(self.p.permiso(AHORA, POSE, AHORA)[0])
        self.assertEqual(self.p.punto_a['x'], 2)

    def test_latido_no_puede_refrescar_actividad_o_camara_vieja(self):
        for actividad, camara in ((AHORA-500_000_001, AHORA),
                                  (AHORA, AHORA-1_000_000_001), (AHORA, AHORA+1)):
            with self.subTest(actividad=actividad,camara=camara):
                p = PuertaBusqueda()
                p.recibir(mensaje(actividad=actividad,camara=camara), AHORA)
                self.assertFalse(p.permiso(AHORA, POSE, AHORA)[0])

    def test_recepcion_y_odometria_caducan_durante_busqueda(self):
        self.p.recibir(mensaje(), AHORA)
        self.assertTrue(self.p.permiso(AHORA, POSE, AHORA)[0])
        self.assertFalse(self.p.permiso(AHORA+500_000_001, POSE, AHORA+500_000_001)[0])
        self.p.recibir(mensaje(secuencia=1,actividad=AHORA+1_000_000_000,
                              camara=AHORA+1_000_000_000), AHORA+1_000_000_000)
        self.assertFalse(self.p.permiso(AHORA+1_000_000_000, POSE, AHORA)[0])

    def test_datos_invalidos_repetidos_futuros_no_permiten_mover(self):
        for linea in ('no json', '[]', '{}', 'x'*2049, mensaje(secuencia=-1),
                      mensaje(actividad=True), mensaje(actividad=AHORA+1), mensaje(camara='1')):
            with self.subTest(linea=linea):
                self.p.recibir(linea, AHORA)
                self.assertFalse(self.p.permiso(AHORA, POSE, AHORA)[0])
        p = PuertaBusqueda()
        p.recibir(mensaje(), AHORA)
        self.assertTrue(p.permiso(AHORA, POSE, AHORA)[0])
        p.recibir(mensaje(), AHORA)
        self.assertFalse(p.permiso(AHORA, POSE, AHORA)[0])

    def test_encontrado_se_detiene_y_nunca_reanuda_con_otro_buscar(self):
        self.p.recibir(mensaje(), AHORA)
        self.p.permiso(AHORA, POSE, AHORA)
        self.p.recibir(mensaje('VERIFICAR',1), AHORA)
        self.assertFalse(self.p.permiso(AHORA, POSE, AHORA)[0])
        self.p.recibir(mensaje('ENCONTRADO',2), AHORA)
        self.assertTrue(self.p.finalizado)
        self.p.recibir(mensaje('BUSCAR',3), AHORA)
        self.assertFalse(self.p.permiso(AHORA, POSE, AHORA)[0])

    def test_cambio_de_marco_bloquea_aunque_se_restauren_nombres_originales(self):
        self.p.recibir(mensaje(), AHORA)
        self.p.permiso(AHORA, POSE, AHORA)
        self.assertFalse(self.p.permiso(AHORA, dict(POSE,frame_id='map'), AHORA)[0])
        self.assertFalse(self.p.permiso(AHORA, POSE, AHORA)[0])

    def test_pose_valida_con_origen_real_cero_y_rechazo_sin_marco_o_cuaternion(self):
        p = NS(x=0.,y=0.,z=0.)
        q = NS(x=0.,y=0.,z=0.,w=1.)
        msg = NS(header=NS(frame_id='odom'), child_frame_id='base_link',
                 pose=NS(pose=NS(position=p,orientation=q)))
        self.assertEqual(pose_odometria(msg)['x'], 0)
        q.w = 0
        self.assertIsNone(pose_odometria(msg))
        q.w, p.x = 1, float('nan')
        self.assertIsNone(pose_odometria(msg))
        p.x, msg.header.frame_id = 0, ''
        self.assertIsNone(pose_odometria(msg))


class ConfirmacionBusquedaTests(unittest.TestCase):
    def setUp(self):
        self.s = SesionBusqueda()
        self.objetivo = dict(aceptado=True,nombre='Persona A')

    def test_iniciar_fija_nombre_y_no_admite_cambio_en_curso(self):
        self.s.iniciar('Persona A', AHORA)
        with self.assertRaises(ValueError): self.s.iniciar('Persona B', AHORA)
        self.assertEqual(self.s.nombre,'Persona A')

    def test_primer_frame_nuevo_finaliza_y_no_cambia_resultado_despues(self):
        self.s.iniciar('Persona A', AHORA)
        for i in (1,2,3):
            self.s.actualizar(self.objetivo,AHORA+i,AHORA+i,True)
            self.assertEqual(self.s.confirmaciones,1)
            self.assertEqual(self.s.orden,'ENCONTRADO')
        self.s.actualizar(dict(aceptado=False,nombre='NO_CONCLUYENTE'),None,AHORA+4,True)
        self.assertEqual(self.s.orden,'ENCONTRADO')

    def test_fotograma_repetido_previo_a_inicio_o_sin_a_no_confirma(self):
        self.s.actualizar(self.objetivo, AHORA, AHORA, True)
        self.assertEqual(self.s.orden,'ESPERAR')
        self.s.iniciar('Persona A',AHORA)
        for marca,a in ((AHORA,True),(AHORA+1,False)):
            self.s.actualizar(self.objetivo,marca,AHORA+1,a)
            self.assertEqual(self.s.confirmaciones,0)
        for _ in range(10): self.s.actualizar(self.objetivo,AHORA+2,AHORA+2,True)
        self.assertEqual(self.s.confirmaciones,1)

    def test_otra_persona_rechazo_y_camara_caducada_no_se_aceptan(self):
        for fallo,marca in ((dict(aceptado=True,nombre='Persona B'),AHORA+2),
                            (dict(aceptado=False,nombre='NO_CONCLUYENTE'),AHORA+2),
                            (self.objetivo,None)):
            s = SesionBusqueda()
            s.iniciar('Persona A',AHORA)
            s.actualizar(fallo,marca,AHORA+2,True)
            self.assertEqual(s.confirmaciones,0)
            self.assertEqual(s.orden,'BUSCAR')
            self.assertFalse(s.encontrado)


if __name__ == '__main__':
    unittest.main()
