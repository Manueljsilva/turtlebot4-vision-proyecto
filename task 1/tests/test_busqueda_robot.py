"""Transporte real local y nodo con dobles de ROS; nunca conecta a hardware."""

import importlib.util
import io
import json
from math import pi
from pathlib import Path
import sys
import threading
import time
from types import ModuleType, SimpleNamespace as NS
import unittest
from unittest.mock import patch

import busqueda_robot as app
from control_busqueda import SesionBusqueda
from control_giros import ControlGiro
from evasion_lidar import ConfigEvasion, decidir_scan
from prueba_facial_robot import ConsultaEnVivo, SistemaFacial
from reconocimiento_facial import Criterio, Galeria


class PreparacionBusquedaTests(unittest.TestCase):
    def setUp(self):
        self.cmd = ['sudo','-n','docker','exec','-i','-e','ROS_DOMAIN_ID=5','proyecto2-sim',
                    'bash','--noprofile','--norc','-c','script','camara-facial',
                    '/tmp/proyecto2-facial.X/camara_robot_ros.py', app.TOPICO]

    def test_solo_copia_control_sin_galeria_y_publica_solo_con_mover(self):
        for mover in (False,True):
            with patch.object(app,'ejecutar_comando',return_value='/cmd_vel [geometry_msgs/msg/TwistStamped]') as cmd:
                orden=app.preparar_movimiento(self.cmd,'proyecto2-sim',app.TAREA/'config/evasion_lidar.json',mover)
            self.assertEqual('--mover' in orden,mover)
            self.assertIn('-i',orden)
            copias=[c.args[0] for c in cmd.call_args_list if 'cp' in c.args[0]]
            self.assertEqual(len(copias),7)
            self.assertEqual({Path(c[-2]).name for c in copias},
                             {'evasion_lidar.py','seguridad_lidar.py','control_busqueda.py',
                              'busqueda_lidar_ros.py','evasion_lidar.json','control_giros.py','config_senales.py'})

    def test_no_arranca_control_con_cmd_vel_ausente_o_tipo_incorrecto(self):
        for respuesta in ('','/cmd_vel [geometry_msgs/msg/Twist]',
                           '/cmd_vel [geometry_msgs/msg/TwistStamped]\n/odom [std_msgs/msg/String]'):
            with patch.object(app,'ejecutar_comando',return_value=respuesta) as cmd:
                with self.assertRaises(ValueError):
                    app.preparar_movimiento(self.cmd,'proyecto2-sim',Path('config.json'),True)
                self.assertFalse(any('cp' in c.args[0] for c in cmd.call_args_list))

    def test_senales_copia_solo_config_y_control_puro_al_docker(self):
        with patch.object(app, 'ejecutar_comando', return_value='/cmd_vel [geometry_msgs/msg/TwistStamped]') as cmd:
            orden = app.preparar_movimiento(self.cmd, 'proyecto2-sim', app.TAREA/'config/evasion_lidar.json',
                False, app.TAREA/'config/senales_giro.json', 'vuelta')
        self.assertIn('--senales', orden)
        self.assertIn('vuelta', orden)
        self.assertNotIn('--mover', orden)
        copias = [c.args[0][-2] for c in cmd.call_args_list if 'cp' in c.args[0]]
        self.assertIn(str(app.TAREA/'control_giros.py'), copias)
        self.assertNotIn(str(app.TAREA/'geometria_senales.py'), copias)
        self.assertNotIn(str(app.TAREA/'senales_giro.py'), copias)

    def test_objetivo_se_fija_solo_con_buscar_y_se_bloquea_durante_busqueda(self):
        sistema=SistemaFacial(None,Galeria([[1,0],[0,1]],['Persona A','Persona B']),Criterio(.5,.1))
        consulta=ConsultaEnVivo(sistema,'Persona A')
        sesion=SesionBusqueda()
        entrada=app.EntradaBusqueda(consulta,sesion)
        self.assertIsNone(sesion.inicio_ns)  # --nombre precarga, no arranca.
        entrada.confirmar()
        self.assertEqual(sesion.nombre,'Persona A')
        entrada.texto='Persona B'
        entrada.confirmar()
        self.assertEqual(consulta.nombre,'Persona A')
        self.assertEqual(sesion.nombre,'Persona A')
        self.assertTrue(entrada.tecla(27))

    def test_latido_real_no_mantiene_permiso_si_la_gui_deja_de_actualizar(self):
        worker = '''import json,sys,time
sys.path.insert(0,sys.argv[1])
from control_busqueda import PuertaBusqueda
p=PuertaBusqueda()
pose=dict(frame_id='odom',child_frame_id='base_link',x=0.,y=0.,yaw=0.)
for linea in sys.stdin:
    ahora=time.monotonic_ns()
    p.recibir(linea,ahora)
    permitido,motivo=p.permiso(ahora,pose,ahora)
    print(json.dumps(dict(version=1,recibido_ns=ahora,a=p.punto_a,estado=motivo,vx=.1 if permitido else 0)),flush=True)
'''
        puente=app.PuenteBusqueda([sys.executable,'-u','-c',worker,str(app.TAREA)])
        try:
            ahora=time.monotonic_ns()
            puente.actualizar('BUSCAR',ahora,ahora)
            limite=time.monotonic()+2
            visto=False
            while time.monotonic()<limite:
                estado=puente.ultimo_estado(time.monotonic_ns())
                if estado and estado['vx']>0:
                    visto=True
                    break
                time.sleep(.01)
            self.assertTrue(visto,'No se habilitó la búsqueda simulada')
            while time.monotonic()<limite:
                estado=puente.ultimo_estado(time.monotonic_ns())
                if estado and estado['vx']==0 and 'actividad' in estado['estado']:
                    break
                time.sleep(.01)
            self.assertEqual(estado['vx'],0)
            self.assertIn('actividad',estado['estado'])
            self.assertTrue(puente.emisor.is_alive())
        finally:
            puente.cerrar()
        self.assertEqual(puente.proceso.returncode,0)
        self.assertFalse(puente.emisor.is_alive())


def cargar_ros_simulado():
    """Cargar los dos nodos reales con primitivas de ROS mínimas y sin red."""
    class Nodo:
        def __init__(self,*args,**kwargs): self.publicadores=[]; self.logs=[]
        def create_subscription(self,*args): return NS()
        def create_timer(self,*args,**kwargs): return NS()
        def create_publisher(self,tipo,topico,profundidad):
            pub=NS(topico=topico,mensajes=[])
            pub.publish=pub.mensajes.append
            self.publicadores.append(pub)
            return pub
        def get_logger(self): return NS(info=self.logs.append)
        def get_clock(self): return NS(now=lambda:NS(to_msg=lambda:None))
    def twist():
        return NS(header=NS(),twist=NS(linear=NS(x=0.),angular=NS(z=0.)))
    def modulo(**valores):
        m=ModuleType('simulado')
        m.__dict__.update(valores)
        return m
    reemplazos={
        'rclpy':modulo(time=NS(Time=lambda:None)),
        'rclpy.clock':modulo(Clock=lambda **k:None,ClockType=NS(STEADY_TIME=1)),
        'rclpy.executors':modulo(ExternalShutdownException=RuntimeError),
        'rclpy.node':modulo(Node=Nodo),
        'rclpy.parameter':modulo(Parameter=lambda *a,**k:None),
        'rclpy.qos':modulo(QoSProfile=lambda **k:None,ReliabilityPolicy=NS(BEST_EFFORT=1),
                           HistoryPolicy=NS(KEEP_LAST=1),qos_profile_sensor_data=None),
        'rclpy.signals':modulo(SignalHandlerOptions=NS(NO=0)),
        'geometry_msgs':modulo(),'geometry_msgs.msg':modulo(TwistStamped=twist),
        'sensor_msgs':modulo(),'sensor_msgs.msg':modulo(LaserScan=object),
        'nav_msgs':modulo(),'nav_msgs.msg':modulo(Odometry=object),
        'tf2_ros':modulo(Buffer=lambda:None,TransformException=LookupError,TransformListener=lambda *a:None),
    }
    with patch.dict(sys.modules,reemplazos):
        for nombre in ('seguridad_lidar','busqueda_lidar_ros'):
            spec=importlib.util.spec_from_file_location(nombre,app.TAREA/(nombre+'.py'))
            m=importlib.util.module_from_spec(spec)
            sys.modules[nombre]=m
            spec.loader.exec_module(m)
        return m


class NodoIntegradoTests(unittest.TestCase):
    def test_evasion_publica_giro_sin_avance_y_luego_reanuda_recto(self):
        mod = cargar_ros_simulado()
        entrada = NS(fin=threading.Event(), linea=None)
        def tomar():
            linea, entrada.linea = entrada.linea, None
            return linea
        entrada.tomar = tomar
        nodo = mod.BusquedaLidarNode(True, ConfigEvasion(), entrada)
        pose = dict(frame_id='odom', child_frame_id='base_link', x=0., y=0., yaw=0.)
        for i, frente in enumerate((1., .40, .40, 1.)):
            ahora = 8_000_000_000+i*100_000_000
            nodo.pose, nodo.pose_ns = pose, ahora
            entrada.linea = json.dumps(dict(version=1, secuencia=i, orden='BUSCAR',
                actividad_ns=ahora, camara_ns=ahora))
            scan = NS(header=NS(frame_id='base_link'), angle_min=-pi, angle_increment=2*pi/360,
                      range_min=.05, range_max=12., ranges=[frente if abs(-pi+j*2*pi/360)<.15 else 1.
                                                         for j in range(360)])
            with patch.object(mod.time, 'monotonic_ns', return_value=ahora), patch('sys.stdout', new_callable=io.StringIO):
                nodo._scan_cb(scan)
                nodo._ciclo()
            cmd = nodo.pub_cmd.mensajes[-1].twist
            if frente == 1.:
                self.assertEqual((cmd.linear.x, cmd.angular.z), (.1, 0.))
            else:
                self.assertEqual(cmd.linear.x, 0.)
                self.assertGreater(cmd.angular.z, 0.)
        self.assertEqual(len(nodo.publicadores), 1)

    def test_flecha_prevalece_sobre_evasion_de_muro_y_no_avanza_antes_de_90(self):
        mod = cargar_ros_simulado()
        entrada = NS(fin=threading.Event(), linea=None)
        def tomar():
            linea, entrada.linea = entrada.linea, None
            return linea
        entrada.tomar = tomar
        cfg = ConfigEvasion()
        nodo = mod.BusquedaLidarNode(True, cfg, entrada, ControlGiro('ida'))
        ahora = 5_000_000_000
        scan = NS(header=NS(frame_id='base_link'),
                  ranges=[.4 if abs(-pi+i*2*pi/360)<.15 else 1. for i in range(360)],
                  angle_min=-pi, angle_increment=2*pi/360, range_min=.05, range_max=12.)
        self.assertGreater(decidir_scan(scan, 0., 0., cfg)[1], 0.)  # La evasión elegiría izquierda.
        nodo.pose = dict(frame_id='odom', child_frame_id='base_link', x=1., y=2., yaw=0.)
        nodo.pose_ns = ahora
        def ciclo(secuencia):
            entrada.linea = json.dumps(dict(version=1, secuencia=secuencia, orden='BUSCAR',
                actividad_ns=ahora, camara_ns=ahora,
                senal=dict(id=1, direccion='DERECHA', color='azul', marca_ns=ahora)))
            with patch.object(mod.time, 'monotonic_ns', return_value=ahora), patch('sys.stdout', new_callable=io.StringIO):
                nodo._scan_cb(scan)
                nodo._ciclo()
            t = nodo.pub_cmd.mensajes[-1].twist
            return t.linear.x, t.angular.z
        self.assertEqual(ciclo(0), (0., -.4))
        scan.ranges[180] = .25
        self.assertEqual(ciclo(1), (0., 0.))  # La seguridad detiene; no impone izquierda.
        scan.ranges[180] = .4
        nodo.pose['yaw'] = -85*pi/180
        vx, wz = ciclo(2)
        self.assertEqual(vx, 0.)
        self.assertLess(wz, 0.)
        nodo.pose['yaw'] = -pi/2
        self.assertEqual(ciclo(3), (0., 0.))
        self.assertEqual(ciclo(4)[0], 0.)  # El scan aún muestra pared: evasión sin arcos.
        scan.ranges = [1.]*360  # Frente libre después del giro.
        self.assertEqual(ciclo(5), (.1, 0.))
        self.assertEqual(len(nodo.publicadores), 1)

    def test_giro_en_salida_unica_lidar_caducado_para_y_el_rostro_sigue_teniendo_prioridad(self):
        mod = cargar_ros_simulado()
        entrada = NS(fin=threading.Event(), linea=None)
        def tomar():
            linea, entrada.linea = entrada.linea, None
            return linea
        entrada.tomar = tomar
        nodo = mod.BusquedaLidarNode(True, ConfigEvasion(), entrada, ControlGiro('ida'))
        self.assertEqual(len(nodo.publicadores), 1)
        ahora = 5_000_000_000
        scan = NS(header=NS(frame_id='base_link'), ranges=[1.]*360, angle_min=-pi,
                  angle_increment=2*pi/360, range_min=.05, range_max=12.)
        nodo.pose = dict(frame_id='odom', child_frame_id='base_link', x=1., y=2., yaw=0.)
        def ciclo(secuencia, orden='BUSCAR', refrescar=True):
            nodo.pose_ns = ahora
            entrada.linea = json.dumps(dict(version=1, secuencia=secuencia, orden=orden,
                actividad_ns=ahora, camara_ns=ahora,
                senal=dict(id=1, direccion='DERECHA', color='azul', marca_ns=ahora)))
            with patch.object(mod.time, 'monotonic_ns', return_value=ahora), patch('sys.stdout', new_callable=io.StringIO):
                if refrescar:
                    nodo._scan_cb(scan)
                nodo._ciclo()
            t = nodo.pub_cmd.mensajes[-1].twist
            return t.linear.x, t.angular.z
        self.assertEqual(ciclo(0), (0., -.4))
        ahora += 700_000_000
        self.assertEqual(ciclo(1, refrescar=False), (0., 0.))
        self.assertEqual(ciclo(2, orden='VERIFICAR'), (0., 0.))
        nodo.pose['yaw'] = -pi/2
        self.assertEqual(ciclo(3), (0., 0.))
        self.assertEqual(ciclo(4), (.1, 0.))  # No vuelve a girar con la misma señal.
        self.assertEqual(ciclo(5, orden='ENCONTRADO'), (0., 0.))
        self.assertEqual(ciclo(6), (0., 0.))

    def test_senales_activadas_exigen_su_protocolo_y_sin_mover_no_publican(self):
        mod = cargar_ros_simulado()
        entrada = NS(fin=threading.Event(), tomar=lambda: None)
        nodo = mod.BusquedaLidarNode(False, ConfigEvasion(), entrada, ControlGiro('ida'))
        self.assertEqual(nodo.publicadores, [])
        with patch('sys.stdout', new_callable=io.StringIO):
            self.assertEqual(nodo.publicar_velocidad(.1, .4), (0., 0.))
        self.assertIn('detector de senales', nodo.motivo_control)

    def test_una_salida_gobernada_por_lidar_camara_y_parada_final(self):
        mod=cargar_ros_simulado()
        ahora=5_000_000_000
        entrada=NS(fin=threading.Event(),linea=None)
        def tomar():
            linea,entrada.linea=entrada.linea,None
            return linea
        entrada.tomar=tomar
        n=mod.BusquedaLidarNode(True,ConfigEvasion(),entrada)
        self.assertEqual([p.topico for p in n.publicadores],['/cmd_vel'])
        scan=NS(header=NS(frame_id='base_link'),ranges=[1.]*360,angle_min=-pi,
                angle_increment=2*pi/360,range_min=.05,range_max=12.)
        pose=NS(header=NS(frame_id='odom'),child_frame_id='base_link',pose=NS(pose=NS(
            position=NS(x=1.,y=2.,z=0.),orientation=NS(x=0.,y=0.,z=0.,w=1.))))
        def orden(tipo,secuencia,camara=ahora):
            entrada.linea=json.dumps(dict(version=1,secuencia=secuencia,orden=tipo,
                                          actividad_ns=ahora,camara_ns=camara))
        with patch.object(mod.time,'monotonic_ns',return_value=ahora),patch('sys.stdout',new_callable=io.StringIO):
            n._scan_cb(scan)
            n.odometria(pose)
            n._ciclo()
            self.assertEqual(n.pub_cmd.mensajes[-1].twist.linear.x,0)
            orden('BUSCAR',0)
            n._ciclo()
            self.assertEqual(n.pub_cmd.mensajes[-1].twist.linear.x,.1)
            orden('BUSCAR',1,None)
            n._ciclo()
            self.assertEqual(n.pub_cmd.mensajes[-1].twist.linear.x,0)
            orden('BUSCAR',2)
            n._scan=None
            n._ciclo()
            self.assertEqual(n.pub_cmd.mensajes[-1].twist.linear.x,0)
            n._scan=scan
            orden('VERIFICAR',3)
            n._ciclo()
            self.assertEqual(n.pub_cmd.mensajes[-1].twist.angular.z,0)
            orden('ENCONTRADO',4)
            n._ciclo()
            orden('BUSCAR',5)
            n._ciclo()
            self.assertEqual(n.pub_cmd.mensajes[-1].twist.linear.x,0)
            self.assertTrue(n.puerta.finalizado)
            self.assertEqual(n.puerta.punto_a['x'],1)

    def test_sin_mover_no_crea_publicador_y_eof_anula_toda_salida(self):
        mod=cargar_ros_simulado()
        entrada=NS(fin=threading.Event(),tomar=lambda:None)
        n=mod.BusquedaLidarNode(False,ConfigEvasion(),entrada)
        self.assertEqual(n.publicadores,[])
        entrada.fin.set()
        with patch('sys.stdout',new_callable=io.StringIO):
            self.assertEqual(n.publicar_velocidad(.1,.4),(0.,0.))


if __name__=='__main__':
    unittest.main()
