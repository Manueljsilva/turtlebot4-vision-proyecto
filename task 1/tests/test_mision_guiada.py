"""Ensayo integrado con sensores/visión simulados, sin red ni robot físico."""

import importlib.util
import io
import json
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
        self.mod, self.cfg_s = cargar_ensayo()
        self.c = ControlMision(self.cfg_s, clase_giros=self.mod.ControlGiro)
        self.cfg = ConfigEvasion()
        self.t, self.seq = 2_000_000_000, 0
        self.pose = dict(frame_id='odom', child_frame_id='base_link', x=0., y=0., yaw=0.)

    def paso(self, orden='OBSERVAR', dt=.2, meta=None, sensor=None, marca=None, qr_ns=None, votos=0,
             senal=None, **pose):
        self.t += round(dt*1e9)
        self.pose.update(pose)
        if marca is None:
            marca = self.t
        self.c.recibir(json.dumps(dict(version=1, secuencia=self.seq, orden=orden,
            actividad_ns=self.t, camara_ns=marca, meta=meta, senal=senal,
            qr_ns=qr_ns, qr_votos=votos)), self.t)
        self.seq += 1
        return self.c.decidir(self.t, self.pose, self.t, sensor or lidar(), self.cfg)

    def iniciar(self):
        self.paso('INICIAR')
        self.assertEqual(self.c.fase, 'IDA')

    def llegar_b(self):
        self.iniciar()
        for i in range(12):
            self.paso(x=min(1., i*.1))
        self.paso(meta=META, sensor=lidar(.55))
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
        self.paso(meta=META, sensor=lidar(.55))
        self.assertEqual(self.c.fase, 'FRENAR_A')
        for _ in range(5):
            self.paso()
        self.assertEqual(self.c.fase, 'FIN')
        self.assertEqual(self.paso('INICIAR')[:2], (0., 0.))
        self.assertEqual(self.c.fase, 'FIN')

    def test_meta_inicial_visible_no_cuenta_como_b(self):
        self.iniciar()
        for i in range(12):
            self.paso(meta=META, sensor=lidar(.55), x=min(1., i*.1))
        self.assertEqual(self.c.fase, 'IDA')
        self.assertEqual(self.c.meta_nueva.votos, 0)
        self.assertIn('falta ausencia previa',self.c.diagnostico_meta)

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
        self.assertIn('frente 1.80m > 0.60m',self.c.diagnostico_meta)
        for _ in range(4):
            self.paso(meta=[.05, .2, .1, .4], sensor=lidar(.55))
        self.assertEqual(self.c.fase, 'IDA')
        self.assertIn('centrarla',self.c.diagnostico_meta)

    def test_repetir_paquete_inadmisible_no_declara_llegada_y_nuevo_valido_si(self):
        self.iniciar()
        for i in range(12):
            self.paso(x=min(1., i*.1))
        self.paso(meta=META, sensor=lidar(1.8))
        marca = self.t
        for _ in range(3):
            self.paso(meta=META, sensor=lidar(.55), marca=marca)
        self.assertEqual(self.c.meta_nueva.votos, 0)
        self.assertEqual(self.c.fase,'IDA')
        self.paso(meta=META, marca=0)
        self.assertEqual(self.c.meta_nueva.votos, 0)
        self.paso(meta=META,sensor=lidar(.55))
        self.assertEqual(self.c.fase,'FRENAR_B')
        self.assertEqual(self.c.meta_nueva.votos,1)

    def test_b_no_puede_convertirse_en_a_durante_la_salida(self):
        self.qr_y_salida()
        for _ in range(12):
            self.paso()  # Ausencia visual, pero sigue en la posición B.
        for _ in range(4):
            self.paso(meta=META, sensor=lidar(.55))
        self.assertEqual(self.c.fase, 'VUELTA')
        self.assertEqual(self.c.meta_nueva.votos, 0)

    def test_meta_lejos_de_a_no_finaliza_retorno(self):
        self.qr_y_salida()
        for i in range(12):
            self.paso(y=(i+1)*.1)
        for _ in range(4):
            self.paso(meta=META, sensor=lidar(.55))
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
        vx,wz,_=self.paso(senal=v.senal,sensor=lidar(.44,izquierda=.8,derecha=1.8))
        self.assertEqual(vx,0.)
        self.assertGreater(wz,0.)

    def test_giro_azul_espera_48cm_y_detiene_antes_de_invertir_por_evasion(self):
        self.iniciar()
        ev = lambda: dict(id=1, direccion='IZQUIERDA', color='azul', marca_ns=self.t+200_000_000)
        self.assertEqual(self.paso(senal=ev(), sensor=lidar(.5))[:2], (.1, 0.))
        self.assertEqual(self.paso(senal=ev(), sensor=lidar(.5, izquierda=.2))[:2], (0., 0.))
        vx, wz, _ = self.paso(senal=ev(), sensor=lidar(.48))
        self.assertEqual((vx, wz), (0., .4))
        self.assertEqual(self.paso(sensor=lidar(.48, derecha=.32))[:2], (0., 0.))

    def test_caducar_flecha_detiene_mision_sin_evasion_automatica(self):
        self.iniciar()
        self.paso(senal=dict(id=1, direccion='DERECHA', color='azul', marca_ns=self.t+200_000_000))
        for _ in range(33):
            self.paso()
        self.assertEqual(self.c.fase, 'FALLO')
        self.assertIn('caducada', self.c.motivo)

    def test_revalidacion_por_imagen_nueva_conserva_flecha_visible_y_limita_espera(self):
        self.iniciar()
        for _ in range(40):
            ev = dict(id=1, direccion='DERECHA', color='azul', marca_ns=self.t+200_000_000)
            self.paso(senal=ev)
        self.assertEqual(self.c.fase, 'IDA')
        self.assertIsNotNone(self.c.giros.pendiente)  # Más de seis segundos, imágenes nuevas.
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
        self.paso(senal=ev, sensor=lidar(.48))
        self.assertIsNone(self.c.giros.activo)
        ev.update(id=2, color='rojo', marca_ns=self.t+200_000_000)
        self.assertEqual(self.paso(senal=ev, sensor=lidar(.48))[:2], (0., -.4))

    def test_salida_bloqueada_conserva_qr_y_giro_contrario_enclava(self):
        self.qr_y_salida(solo_orientar=True)
        self.assertEqual(self.paso(sensor=lidar(.3))[:2], (0., 0.))
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


class PreparacionTests(unittest.TestCase):
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
            self.assertTrue(any(str(CONFIG) == x[-2] for x in copias))
            self.assertTrue(any(str(CONTROL) == x[-2] for x in copias))
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
            'rclpy.signals':modulo(SignalHandlerOptions=NS(NO=0))}
        with patch.dict(sys.modules,reemplazos):
            spec = importlib.util.spec_from_file_location('mision_ros_simulada',app.TAREA/'mision_ros.py')
            ros = importlib.util.module_from_spec(spec); spec.loader.exec_module(ros)
        original,cfg_s = cargar_ensayo()
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
                self.assertEqual(estado['disparo_m'],.48)
                self.assertEqual(estado['mover'],mover)
                self.assertEqual(estado['edad_camara_s'],0.)
                self.assertTrue(estado['inicio_solicitado'])
                self.assertEqual(estado['orden_control'],'INICIAR')
                self.assertIn('motivo_meta',estado)
            if mover:
                cmd=nodo.pub_cmd.mensajes[-1].twist
                self.assertEqual((cmd.linear.x,cmd.angular.z),(.1,0.))
                self.assertEqual(nodo.pub_cmd.topico,'/cmd_vel')
            else:
                self.assertIsNone(nodo.pub_cmd)


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
