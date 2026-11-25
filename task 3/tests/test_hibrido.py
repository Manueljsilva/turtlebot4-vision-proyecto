"""Regresiones de entrega de control y misión completa; no usan webcam ni ROS reales."""
import io
import json
from math import pi
from pathlib import Path
import tempfile
import threading
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

import cv2
import numpy as np
from control_hibrido import ControlHibrido, adquirir_bloqueos
from camara_gestos import VideoGestos
from ensayo_giros_companero import cargar_ensayo
from evasion_lidar import cargar_config
from holgura_giro import HolguraGiro
from interfaz_hibrida import InterfazHibrida
import mision_guiada_robot as app
from tests.test_mision_guiada import lidar, META


class HibridoTests(unittest.TestCase):
    def setUp(self):
        mod, cfg = cargar_ensayo(config=app.CONFIG_SENALES_MISION)
        self.c = ControlHibrido(cfg, clase_giros=mod.ControlGiro)
        self.cfg = cargar_config(Path(__file__).parents[1]/'config/evasion_lidar.json')
        self.t, self.seq, self.token = 10_000_000_000, 0, 0
        self.modo = 'AUTONOMO'
        self.pose = dict(frame_id='odom', child_frame_id='base_link', x=0.,y=0.,yaw=0.)
        self.marca_gesto = None

    def paso(self, orden='OBSERVAR', gesto='DETENER', meta=None, sensor=None, h=None, dt=.2,
             fresco=True, **pose):
        self.t += round(dt*1e9)
        self.pose.update(pose)
        if fresco:
            self.marca_gesto = self.t
        d = dict(version=1,secuencia=self.seq,orden=orden,actividad_ns=self.t,camara_ns=self.t,
                 meta=meta,senal=None,qr_ns=self.t if orden=='QR_LISTO' else None,
                 qr_votos=int(orden=='QR_LISTO'),reinicio_ns=self.c.reinicio_ns,
                 modo=self.modo,modo_ns=self.token,gesto=gesto,gesto_ns=self.marca_gesto)
        self.seq += 1
        self.c.recibir(json.dumps(d),self.t)
        sensor = sensor or lidar()
        h = h or HolguraGiro(sensor.observacion_valida,
            min(sensor.dist_frente,sensor.dist_izquierda,sensor.dist_derecha),'Simulado',0.)
        return self.c.decidir(self.t,self.pose,self.t,sensor,self.cfg,holgura=h)

    def iniciar(self):
        self.paso('INICIAR')
        self.assertEqual(self.c.fase,'IDA')

    def cambiar(self, modo):
        self.modo, self.token = modo,self.t+200_000_000
        for _ in range(8):
            self.assertEqual(self.paso(gesto='AVANZAR')[:2],(0.,0.))
            if self.c.modo == modo and not self.c.cambio_modo:
                break
        self.assertEqual(self.c.modo,modo)
        self.assertFalse(self.c.cambio_modo)

    def test_alternancia_preserva_a_b_qr_y_fase(self):
        self.iniciar()
        a = dict(self.c.a)
        self.c.b, self.c.qr_confirmado = dict(self.pose,x=2.),True
        self.c.qr_pendiente = self.t
        for modo in ('GESTOS','AUTONOMO','GESTOS'):
            self.c.giros.pendiente={'id':1}
            self.c.evasion={'sentido':1.}
            self.c.escape_atras={'inicio_ns':self.t}
            self.c.senal_por_procesar={'id':1}
            self.c.limpiar_intenciones()
            self.cambiar(modo)
            self.assertEqual(self.c.a,a)
            self.assertEqual(self.c.b['x'],2.)
            self.assertTrue(self.c.qr_confirmado)
            self.assertEqual(self.c.fase,'IDA')
            self.assertIsNone(self.c.giros.pendiente)
            self.assertIsNone(self.c.escape_atras)

    def test_handover_no_cuenta_poses_repetidas_ni_mientras_se_mueve(self):
        self.iniciar()
        self.modo,self.token='GESTOS',self.t+200_000_000
        self.paso()
        marca=self.t
        for _ in range(3):
            self.t+=200_000_000
            self.c.actividad_ns=self.c.camara_ns=self.t
            self.assertEqual(self.c.decidir(self.t,self.pose,marca,lidar(),self.cfg)[:2],(0.,0.))
        self.assertTrue(self.c.cambio_modo)
        for i in range(4):self.paso(x=.02*i)
        self.assertTrue(self.c.cambio_modo)
        for _ in range(12):self.paso()
        self.assertFalse(self.c.cambio_modo)

    def test_gestos_usan_perfil_task3_y_giran_sobre_sitio(self):
        self.iniciar();self.cambiar('GESTOS')
        self.assertEqual(self.paso(gesto='AVANZAR')[:2],(.1,0.))
        self.assertEqual(self.paso(gesto='IZQUIERDA')[:2],(0.,.4))
        self.assertEqual(self.paso(gesto='DERECHA')[:2],(0.,-.4))
        self.assertEqual(self.paso(gesto='DETENER')[:2],(0.,0.))

    def test_latido_no_rejuvenece_mano_y_parar_es_inmediato(self):
        self.iniciar();self.cambiar('GESTOS')
        self.paso(gesto='AVANZAR')
        for _ in range(4):r=self.paso(gesto='AVANZAR',fresco=False)
        self.assertEqual(r[:2],(0.,0.))
        self.assertEqual(self.paso(gesto='AVANZAR')[:2],(.1,0.))
        self.assertEqual(self.paso(gesto='DETENER')[:2],(0.,0.))

    def test_modo_manual_no_evade_solo_ni_inicia_escape(self):
        self.iniciar();self.cambiar('GESTOS')
        r=self.paso(gesto='DETENER',sensor=lidar(.29,.5,.5))
        self.assertEqual(r[:2],(0.,0.))
        self.assertIsNone(self.c.evasion);self.assertIsNone(self.c.escape_atras)
        self.assertEqual(self.paso(gesto='AVANZAR',sensor=lidar(.2))[:2],(0.,0.))

    def test_holgura_angular_escaneo_y_tf_siguen_bloqueando(self):
        self.iniciar();self.cambiar('GESTOS')
        for h in (HolguraGiro(False,None,'Sin TF',0.),HolguraGiro(True,.23,'Ocupado',0.),
                  HolguraGiro(True,2.,'Viejo',.7)):
            self.assertEqual(self.paso(gesto='DERECHA',h=h)[:2],(0.,0.))
        self.assertEqual(self.paso(gesto='AVANZAR',sensor=lidar(valido=False))[:2],(0.,0.))

    def test_mensaje_malformado_no_entrega_control_y_legacy_no_autoriza_mano(self):
        self.iniciar();self.cambiar('GESTOS')
        for cambio in ({'modo':'MAL'},{'modo_ns':self.t+10**10},{'gesto_ns':self.t+10**10},
                       {'modo_ns':-1},{'gesto':'RETROCEDER'}):
            d=dict(version=1,secuencia=self.seq,orden='OBSERVAR',actividad_ns=self.t,camara_ns=self.t,
                   meta=None,senal=None,qr_ns=None,qr_votos=0,modo='GESTOS',modo_ns=self.token,
                   gesto='AVANZAR',gesto_ns=self.t)
            d.update(cambio);self.seq+=1
            self.c.recibir(json.dumps(d),self.t)
            self.assertIsNone(self.c.actividad_ns)
            self.assertEqual(self.c.modo,'GESTOS')
        self.paso(gesto='AVANZAR')
        self.seq+=1
        self.c.recibir(json.dumps(dict(version=1,secuencia=self.seq,orden='ESPERAR',
            actividad_ns=self.t,camara_ns=None)),self.t)
        self.assertEqual(self.c.gesto,'DETENER');self.assertEqual(self.c.modo,'GESTOS')

    def test_corte_largo_durante_giro_manual_enclava_sin_reanclar(self):
        self.iniciar();self.cambiar('GESTOS')
        a=dict(self.c.a)
        self.paso(gesto='DERECHA')
        self.paso(dt=2.1)
        self.assertEqual(self.c.fase,'FALLO');self.assertEqual(self.c.a,a)

    def test_mision_completa_gestos_meta_rostro_qr_regreso_a(self):
        self.iniciar();self.cambiar('GESTOS')
        for i in range(12):self.paso(gesto='AVANZAR',x=min(1.,i*.1))
        self.paso(meta=META,sensor=lidar(.4),gesto='AVANZAR')
        self.assertEqual(self.c.fase,'FRENAR_B')
        for _ in range(5):self.paso()
        self.assertEqual(self.c.fase,'BUSCAR_B')
        self.assertEqual(self.paso(gesto='AVANZAR')[:2],(0.,0.))
        self.assertEqual(self.paso(gesto='DERECHA')[:2],(0.,-.25))
        self.assertEqual(self.paso('VERIFICAR',gesto='DERECHA')[:2],(0.,0.))
        self.paso('QR_LISTO');self.assertEqual(self.c.fase,'FRENAR_QR')
        for _ in range(5):self.paso()
        self.assertEqual(self.c.fase,'ORIENTAR_SALIDA');self.assertTrue(self.c.qr_confirmado)
        for i in range(1,17):self.paso(gesto='IZQUIERDA',yaw=min(pi,i*.2))
        for _ in range(6):self.paso()
        self.assertEqual(self.c.fase,'VUELTA')
        for i in range(12):self.paso(gesto='AVANZAR',x=max(0.,1.-i*.1))
        self.paso(meta=META,sensor=lidar(.4))
        self.assertEqual(self.c.fase,'FRENAR_A')
        for _ in range(5):self.paso()
        self.assertEqual(self.c.fase,'FIN');self.assertTrue(self.c.qr_confirmado)
        self.assertEqual(self.paso(gesto='AVANZAR')[:2],(0.,0.))

    def test_cambiar_durante_orientacion_recalcula_desde_entrada_b(self):
        self.iniciar();self.cambiar('GESTOS')
        self.c.fase='ORIENTAR_SALIDA';self.c.yaw_entrada_b=0.
        self.c.yaw_salida=pi;self.c.yaw_total=.8;self.pose['yaw']=.8
        self.c.pose=dict(self.pose)
        self.cambiar('AUTONOMO')
        self.assertAlmostEqual(self.c.yaw_salida,pi)
        self.assertEqual(self.c.yaw_inicio_salida,.8)

    def test_reinicio_borra_modo_y_no_restaura_gestos(self):
        self.iniciar();self.cambiar('GESTOS')
        self.c.qr_confirmado=True
        self.t+=200_000_000
        self.c.recibir(json.dumps(dict(version=1,secuencia=self.seq,orden='REINICIAR',
            actividad_ns=self.t,camara_ns=None,meta=None,senal=None,qr_ns=None,qr_votos=0,
            reinicio_ns=self.t,modo='AUTONOMO',modo_ns=0,gesto='DETENER',gesto_ns=None)),self.t)
        self.assertEqual(self.c.fase,'REINICIAR');self.assertEqual(self.c.modo,'AUTONOMO')
        self.assertIsNone(self.c.a);self.assertFalse(self.c.qr_confirmado);self.assertIsNone(self.c.gesto_ns)

    def test_flecha_durante_gestos_no_reaparece_al_volver_a_autonomo(self):
        self.iniciar();self.cambiar('GESTOS')
        self.seq+=1
        d=dict(version=1,secuencia=self.seq,orden='OBSERVAR',actividad_ns=self.t,camara_ns=self.t,
            meta=None,senal=dict(id=42,direccion='DERECHA',color='azul',marca_ns=self.t),
            qr_ns=None,qr_votos=0,modo=self.modo,modo_ns=self.token,gesto='DETENER',gesto_ns=self.t)
        self.c.recibir(json.dumps(d),self.t)
        self.assertIsNone(self.c.senal_por_procesar)
        self.cambiar('AUTONOMO')
        self.assertIsNone(self.c.giros.pendiente);self.assertIsNone(self.c.senal_por_procesar)

    def test_secuencia_repetida_y_captura_con_orden_distinta_no_habilitan_movimiento(self):
        self.iniciar();self.cambiar('GESTOS');self.paso(gesto='AVANZAR')
        d=dict(version=1,secuencia=self.seq,orden='OBSERVAR',actividad_ns=self.t,camara_ns=self.t,
            meta=None,senal=None,qr_ns=None,qr_votos=0,modo=self.modo,modo_ns=self.token,
            gesto='DERECHA',gesto_ns=self.marca_gesto)
        self.c.recibir(json.dumps(d),self.t)
        self.assertIsNone(self.c.actividad_ns)
        self.paso(gesto='AVANZAR')
        d.update(secuencia=self.c.secuencia,gesto='AVANZAR',gesto_ns=self.t)
        self.c.recibir(json.dumps(d),self.t)
        self.assertIsNone(self.c.actividad_ns)

    def test_misma_solicitud_no_reinicia_reposo_y_solicitud_anterior_no_cambia_modo(self):
        self.iniciar();self.cambiar('GESTOS')
        self.paso(gesto='AVANZAR')
        self.assertFalse(self.c.cambio_modo)
        self.seq+=1
        d=dict(version=1,secuencia=self.seq,orden='OBSERVAR',actividad_ns=self.t,camara_ns=self.t,
            meta=None,senal=None,qr_ns=None,qr_votos=0,modo='AUTONOMO',modo_ns=0,
            gesto='DETENER',gesto_ns=None)
        self.c.recibir(json.dumps(d),self.t)
        self.assertEqual(self.c.modo,'GESTOS');self.assertIsNone(self.c.actividad_ns)


class TransporteInterfazTests(unittest.TestCase):
    def test_bloqueos_excluyen_ambos_controles_y_rollback(self):
        with tempfile.TemporaryDirectory() as tmp:
            rutas=[str(Path(tmp)/v) for v in ('auto','gestos')]
            ocupados=adquirir_bloqueos([rutas[1]])
            with self.assertRaises(BlockingIOError):adquirir_bloqueos(rutas)
            # El primer lock del intento fallido se liberó.
            libres=adquirir_bloqueos([rutas[0]])
            for v in libres+ocupados:v.close()
            ambos=adquirir_bloqueos(rutas)
            for ruta in rutas:
                with self.assertRaises(BlockingIOError):adquirir_bloqueos([ruta])
            for v in ambos:v.close()
            for v in adquirir_bloqueos(rutas):v.close()

    def video(self):
        v=VideoGestos.__new__(VideoGestos)
        v.cerrojo=threading.Lock();v.ultima=None;v.error='Esperando'
        return v

    def test_receptor_no_renueva_captura_ni_acepta_repeticiones(self):
        v=self.video();d=dict(secuencia=1,captura_ns=10**9,orden='AVANZAR',motivo='Indice',jpg='')
        v.aceptar(json.dumps(d),10**9)
        self.assertEqual(v.muestra(1_100_000_000)[:2],('AVANZAR',10**9))
        self.assertEqual(v.muestra(1_600_000_000)[:2],('DETENER',None))
        with self.assertRaises(ValueError):v.aceptar(json.dumps(d),10**9)
        with self.assertRaises(ValueError):v.aceptar(json.dumps(dict(d,captura_ns=10**10)),10**9)

    def test_panel_alternar_reinicio_y_traduccion_de_raton(self):
        puente=Mock();h=InterfazHibrida(puente);h.entrada=Mock(reinicio_ns=None)
        with patch('interfaz_hibrida.VideoGestos') as video:
            h.raton(cv2.EVENT_LBUTTONDOWN,25,25)
            self.assertEqual(h.modo,'GESTOS');video.assert_called_once()
            h.raton(cv2.EVENT_LBUTTONDOWN,1180,737)
            h.entrada.raton.assert_called_once_with(cv2.EVENT_LBUTTONDOWN,700,685,None,None)
            dibujo=h.dibujar(np.zeros((708,960,3),dtype=np.uint8),dict(modo='GESTOS',cambio_modo=True))
            self.assertEqual(dibujo.shape,(760,1440,3))
            h.reiniciar();video.return_value.cerrar.assert_called_once()
            self.assertEqual(h.modo,'AUTONOMO')
            puente.cambiar_modo.assert_called_with('AUTONOMO',0)

    def test_puente_latido_conserva_marca_mano_y_no_publica_otro_ros(self):
        p=app.PuenteMision.__new__(app.PuenteMision);p.cerrojo=threading.Lock();p.datos={}
        p.cambiar_modo('GESTOS',100)
        p.actualizar_gesto('AVANZAR',120)
        for t in (130,150,800):
            p.actualizar('OBSERVAR',t,t)
            self.assertEqual(p.datos['gesto_ns'],120)
            self.assertEqual(p.datos['modo_ns'],100)
        p.cambiar_modo('AUTONOMO',900)
        self.assertEqual(p.datos['gesto'],'DETENER');self.assertIsNone(p.datos['senal'])
