"""Ensayo aislado del snapshot: transporte y control, sin red ni robot."""

import io
import json
from dataclasses import asdict
from math import pi
from pathlib import Path
import shutil
import subprocess
import tempfile
import threading
import time
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

import busqueda_robot as app
from ensayo_giros_companero import CARPETA, CONFIG, CONTROL, cargar_ensayo, diagnostico_senales
from evasion_lidar import cargar_config, analizar_lidar
from prueba_facial_robot import SistemaFacial
from reconocimiento_facial import Criterio, Galeria
from tests.test_busqueda_robot import cargar_ros_simulado
from tests import test_lanzador_evasion as lanzador


AHORA = 20_000_000_000


def evento(ahora=AHORA):
    return dict(id=1, direccion='DERECHA', color='azul', marca_ns=ahora)


def scan(frente=1.8):
    return NS(header=NS(frame_id='base_link'), angle_min=-pi, angle_increment=2*pi/360,
              range_min=.05, range_max=12.,
              ranges=[frente if abs(-pi+i*2*pi/360) < .25 else 1.8 for i in range(360)])


class SnapshotTests(unittest.TestCase):
    def test_config_una_deteccion_funciona_con_control_original_en_copia_ros(self):
        # Reproduce el directorio de módulos que se copia a Docker, sin importar ROS.
        with tempfile.TemporaryDirectory() as tmp:
            destino=Path(tmp)
            for nombre in ('config_senales.py','control_busqueda.py','control_meta.py',
                           'control_mision.py','evasion_lidar.py'):
                shutil.copyfile(app.TAREA/nombre,destino/nombre)
            shutil.copyfile(CONTROL,destino/'control_giros.py')
            shutil.copyfile(CONFIG,destino/'senales.json')
            r=subprocess.run([sys.executable,'-c',
                'from config_senales import cargar_config_senales; '
                'from control_mision import ControlMision; '
                'cfg=cargar_config_senales("senales.json"); '
                'c=ControlMision(cfg); '
                'assert cfg.confirmaciones==1 and cfg.esperar_pared_m==0.48; '
                'assert c.giros.cfg is cfg and c.giros.pendiente is None'],
                cwd=tmp,capture_output=True,text=True,timeout=10)
            self.assertEqual(r.returncode,0,r.stderr)

    def test_copias_verificadas_y_cambio_rechazado(self):
        mod, cfg = cargar_ensayo()
        self.assertEqual(cfg.esperar_pared_m, .48)
        original = mod.cargar_config_senales(CARPETA/'senales_giro.json')
        self.assertEqual(original.esperar_pared_m, .52)
        self.assertEqual({k for k, v in asdict(cfg).items() if v != asdict(original)[k]},
                         {'esperar_pared_m','confirmaciones'})
        self.assertEqual(cfg.confirmaciones,1)
        self.assertEqual(original.confirmaciones,3)
        self.assertEqual(cfg.tiempo_espera_pared_s, 6.)
        self.assertEqual(Path(mod.__file__).resolve(), CONTROL)
        with tempfile.TemporaryDirectory() as tmp:
            for nombre in ('control_giros.py', 'senales_giro.json'):
                shutil.copyfile(CARPETA/nombre, Path(tmp)/nombre)
            with (Path(tmp)/'control_giros.py').open('a') as f:
                f.write('\n# modificado\n')
            with self.assertRaisesRegex(ValueError, 'no coincide'):
                cargar_ensayo(tmp)

    def test_lejos_memoriza_cerca_gira_y_exige_90_grados(self):
        mod, cfg_s = cargar_ensayo()
        cfg = cargar_config(app.TAREA/'config/evasion_lidar.json')
        self.assertEqual((cfg.velocidad_avance, cfg.velocidad_giro), (.1, .4))
        ctrl = mod.ControlGiro('ida', cfg_s)
        pose = dict(x=0., y=0., yaw=0.)
        def lidar(dist):
            s = scan(dist)
            return analizar_lidar(s.ranges, s.angle_min, s.angle_increment,
                                  s.range_min, s.range_max, cfg=cfg)
        self.assertIsNone(ctrl.decidir(evento(), pose, AHORA, True, lidar(1.8), cfg))
        self.assertEqual(ctrl.pendiente['direccion'], 'DERECHA')
        self.assertIsNone(ctrl.decidir(None, pose, AHORA+50_000_000, True, lidar(.50), cfg))
        self.assertIsNone(ctrl.activo)
        giro = ctrl.decidir(None, pose, AHORA+100_000_000, True, lidar(.48), cfg)
        self.assertEqual(giro[:2], (0., -.4))
        pose['yaw'] = -85*pi/180
        self.assertLess(ctrl.decidir(None, pose, AHORA+200_000_000, True, lidar(.48), cfg)[1], 0)
        pose['yaw'] = -pi/2
        fin = ctrl.decidir(None, pose, AHORA+300_000_000, True, lidar(.48), cfg)
        self.assertEqual(fin[:2], (0., 0.))
        self.assertIn('completado', fin[2])

    def test_documenta_caducidad_original_aun_con_mismo_cartel_visible(self):
        # Caracterización del snapshot conservado; NO es el comportamiento deseado final.
        mod, cfg_s = cargar_ensayo()
        cfg = cargar_config(app.TAREA/'config/evasion_lidar.json')
        ctrl = mod.ControlGiro('ida', cfg_s)
        pose = dict(x=0., y=0., yaw=0.)
        for segundo in range(8):
            ahora = AHORA+segundo*1_000_000_000
            s = scan(.48 if segundo == 7 else 1.8)
            lidar = analizar_lidar(s.ranges, s.angle_min, s.angle_increment,
                                   s.range_min, s.range_max, cfg=cfg)
            salida = ctrl.decidir(evento(ahora), pose, ahora, True, lidar, cfg)
        self.assertIsNone(salida)
        self.assertIsNone(ctrl.pendiente)
        self.assertEqual(ctrl.ultimo_id, 1)

    def test_preparacion_copia_control_experimental_sin_cambiar_el_habitual(self):
        cmd = ['sudo','-n','docker','exec','-i','proyecto2-sim','bash','-c','script',
               'camara-facial','/tmp/proyecto2-facial.X/camara_robot_ros.py',app.TOPICO]
        original = (app.TAREA/'control_giros.py').read_bytes()
        for mover in (False, True):
            with patch.object(app, 'ejecutar_comando', return_value='/cmd_vel [geometry_msgs/msg/TwistStamped]') as ejecutar:
                orden = app.preparar_movimiento(cmd, 'proyecto2-sim', app.TAREA/'config/evasion_lidar.json',
                    mover, CONFIG, 'ida', control_giros_origen=CONTROL)
            self.assertEqual('--mover' in orden, mover)
            copias = [c.args[0] for c in ejecutar.call_args_list if 'cp' in c.args[0]]
            control = [c for c in copias if c[-1].endswith('/control_giros.py')]
            self.assertEqual(len(control), 1)
            self.assertEqual(control[0][-2], str(CONTROL))
            config = [c for c in copias if c[-1].endswith('/senales.json')]
            self.assertEqual(len(config), 1)
            self.assertEqual(config[0][-2], str(CONFIG))
            self.assertNotEqual(CONFIG, CARPETA/'senales_giro.json')
            self.assertFalse(any('rostros' in c[-2] or 'galeria' in c[-2] or 'mision_meta' in c[-2] for c in copias))
        self.assertEqual((app.TAREA/'control_giros.py').read_bytes(), original)

    def test_argumentos_incompatibles_no_contactan_robot(self):
        for opciones in (['--ensayo-giro-companero'],
                         ['--ensayo-giro-companero','--senales','--config-senales','otra.json']):
            with patch.object(app, 'preparar_camara') as camara, patch('sys.stderr', new_callable=io.StringIO):
                with self.assertRaises(SystemExit) as fin:
                    app.main(['--ip','10.87.179.100','--dominio','5',*opciones])
            self.assertEqual(fin.exception.code, 2)
            camara.assert_not_called()

    def test_interfaz_selecciona_snapshot_espera_camara_y_cierra_sin_arrancar(self):
        sistema = SistemaFacial(Mock(), Galeria([[1,0],[0,1]], ['Persona A','Persona B']), Criterio(.5,.1))
        video = NS(proceso=Mock(), fotograma=Mock(return_value=(None,'Esperando camara')), cerrar=Mock())
        video.proceso.poll.return_value = None
        puente = Mock()
        puente.ultimo_envio_senal.return_value = None
        puente.ultimo_estado.return_value = dict(a=None, estado='Esperando Buscar',
            giro_ensayo=dict(fase='SIN_FLECHA_PENDIENTE', frente_m=None))
        with tempfile.TemporaryDirectory() as tmp:
            temporal = Path(tmp)
            (temporal/'datos').mkdir()
            with patch.object(app, 'TAREA', temporal), \
                    patch.object(app, 'cargar_config', return_value=cargar_config(Path(__file__).resolve().parents[1]/'config/evasion_lidar.json')), \
                    patch.object(app, 'cargar_config_senales', side_effect=AssertionError('No cargar config habitual')), \
                    patch.object(app, 'cargar_sistema', return_value=sistema), \
                    patch.object(app, 'preparar_camara', return_value=['camara']), \
                    patch.object(app, 'preparar_movimiento', return_value=['control']) as preparar, \
                    patch.object(app, 'cargar_transporte', return_value=lambda p:video), \
                    patch.object(app, 'PuenteBusqueda', return_value=puente), \
                    patch.object(app.subprocess, 'Popen'), patch.dict(app.os.environ, {'DISPLAY':':simulado'}), \
                    patch.object(app.cv2, 'namedWindow'), patch.object(app.cv2, 'resizeWindow'), \
                    patch.object(app.cv2, 'setMouseCallback'), patch.object(app.cv2, 'imshow') as mostrar, \
                    patch.object(app.cv2, 'waitKeyEx', return_value=27), \
                    patch.object(app.cv2, 'getWindowProperty', return_value=1), \
                    patch.object(app.cv2, 'destroyAllWindows'), patch('sys.stdout', new_callable=io.StringIO) as salida:
                fin = app.main(['--ip','10.87.179.100','--dominio','5','--senales','--ensayo-giro-companero'])
            self.assertEqual(fin, 0)
            self.assertEqual(preparar.call_args.args[4], CONFIG)
            self.assertEqual(preparar.call_args.kwargs['control_giros_origen'], CONTROL)
            self.assertIn('[ENSAYO GIROS]', salida.getvalue())
            self.assertEqual(mostrar.call_args.args[1].shape, (708,960,3))
            self.assertEqual(puente.actualizar.call_args.args[0], 'ESPERAR')
            self.assertFalse(list((temporal/'datos/sesiones').glob('*.json')))
            video.cerrar.assert_called_once()
            puente.cerrar.assert_called_once()

    def test_diagnostico_distingue_confirmacion_previa_y_posterior_a_buscar(self):
        ev = evento()
        resultado = dict(estado='DERECHA azul: 1/1 confirmada', confirmaciones=1,
                         evento=ev, detecciones=[])
        previo = diagnostico_senales(resultado, None, AHORA+100_000_000)
        self.assertFalse(previo['iniciada'])
        self.assertFalse(previo['evento_posterior_a_buscar'])
        self.assertEqual(previo['evento_id'], 1)
        antiguo = diagnostico_senales(resultado, AHORA+1, AHORA+100_000_000)
        self.assertFalse(antiguo['evento_posterior_a_buscar'])
        posterior = diagnostico_senales(resultado, AHORA-1, AHORA+100_000_000, ev)
        self.assertTrue(posterior['evento_posterior_a_buscar'])
        self.assertEqual(posterior['ultimo_id_enviado'], 1)
        self.assertAlmostEqual(posterior['edad_evento_s'], .1)
        self.assertEqual(ev['marca_ns'], AHORA)  # Diagnóstico no renueva frescura.


class IntegracionEnsayoTests(unittest.TestCase):
    def test_evento_confirmado_llega_por_tuberia_real_y_se_consume(self):
        worker = '''import json,sys,time
sys.path.insert(0,sys.argv[1])
from control_busqueda import PuertaBusqueda
from ensayo_giros_companero import cargar_ensayo
from evasion_lidar import ConfigEvasion, analizar_lidar
from math import pi
mod,cfg_s=cargar_ensayo()
ctrl=mod.ControlGiro('ida',cfg_s)
cfg=ConfigEvasion()
p=PuertaBusqueda()
pose=dict(frame_id='odom',child_frame_id='base_link',x=0.,y=0.,yaw=0.)
lidar=analizar_lidar([1.8]*360,-pi,2*pi/360,.05,12.,cfg=cfg)
for linea in sys.stdin:
    ahora=time.monotonic_ns()
    p.recibir(linea,ahora)
    ok,motivo=p.permiso(ahora,pose,ahora)
    ctrl.decidir(p.senal,pose,ahora,ok,lidar,cfg)
    print(json.dumps(dict(version=1,recibido_ns=ahora,a=p.punto_a,estado=motivo,
                         id_recibido=p.senal['id'] if p.senal else None,
                         id_consumido=ctrl.ultimo_id)),flush=True)
'''
        puente = app.PuenteBusqueda([sys.executable,'-u','-c',worker,str(app.TAREA)], senales=True)
        try:
            limite = time.monotonic()+2
            estado = None
            while time.monotonic() < limite:
                ahora = time.monotonic_ns()
                puente.actualizar('BUSCAR', ahora, ahora, evento(ahora))
                estado = puente.ultimo_estado(ahora)
                if estado and estado['id_consumido'] == 1:
                    break
                time.sleep(.02)
            self.assertIsNotNone(estado)
            self.assertEqual(estado['id_recibido'], 1)
            self.assertEqual(estado['id_consumido'], 1)
            self.assertEqual(puente.ultimo_envio_senal()['id'], 1)
        finally:
            puente.cerrar()

    def test_nodo_sin_publicador_y_salida_unica_con_sensores_frescos(self):
        mod = cargar_ros_simulado()
        original, cfg_s = cargar_ensayo()
        cfg = cargar_config(app.TAREA/'config/evasion_lidar.json')
        for mover in (False, True):
            entrada = NS(fin=threading.Event(), linea=None)
            def tomar():
                linea, entrada.linea = entrada.linea, None
                return linea
            entrada.tomar = tomar
            nodo = mod.BusquedaLidarNode(mover, cfg, entrada, original.ControlGiro('ida', cfg_s))
            self.assertEqual(len(nodo.publicadores), int(mover))
            nodo.pose = dict(frame_id='odom', child_frame_id='base_link', x=0., y=0., yaw=0.)
            def ciclo(segundo, frente, fresco=True):
                ahora = AHORA+int(segundo*1e9)
                entrada.linea = json.dumps(dict(version=1, secuencia=int(segundo*10), orden='BUSCAR',
                    actividad_ns=ahora, camara_ns=ahora if fresco else AHORA,
                    senal=evento(ahora)))
                nodo.pose_ns = ahora
                out = io.StringIO()
                with patch.object(mod.time, 'monotonic_ns', return_value=ahora), patch('sys.stdout', out):
                    nodo._scan_cb(scan(frente))
                    nodo._ciclo()
                return json.loads(out.getvalue())
            lejos = ciclo(0, 1.8)
            self.assertEqual(lejos['giro_ensayo']['fase'], 'ESPERA_PARED')
            self.assertEqual((lejos['vx'], lejos['wz']), (.1, 0.))
            aproximando = ciclo(.3, .5)
            self.assertEqual(aproximando['giro_ensayo']['fase'], 'ESPERA_PARED')
            self.assertEqual(aproximando['giro_ensayo']['disparo_m'], .48)
            self.assertEqual((aproximando['vx'], aproximando['wz']), (.1, 0.))
            cerca = ciclo(.6, .48)
            self.assertEqual(cerca['giro_ensayo']['fase'], 'GIRANDO')
            self.assertEqual((cerca['vx'], cerca['wz']), (0., -.4))
            bloqueo = ciclo(.9, .32)
            self.assertEqual((bloqueo['vx'], bloqueo['wz']), (0., 0.))
            mas_cerca = ciclo(1.2, .30)
            self.assertEqual((mas_cerca['vx'], mas_cerca['wz']), (0., 0.))
            sin_camara = ciclo(1.5, .48, fresco=False)
            self.assertEqual((sin_camara['vx'], sin_camara['wz']), (0., 0.))
            self.assertFalse(sin_camara['giro_ensayo']['permitido'])
            if mover:
                self.assertEqual(nodo.pub_cmd.topico, '/cmd_vel')
            else:
                self.assertIsNone(nodo.pub_cmd)


class LanzadorEnsayoTests(unittest.TestCase):
    setUp = lanzador.LanzadorEvasionTests.setUp
    ejecutar = lanzador.LanzadorEvasionTests.ejecutar

    def test_opcion_explicita_y_exclusiva_de_la_prueba_combinada(self):
        (self.tarea/'busqueda_robot.py').write_text('import json,sys\nprint(json.dumps(sys.argv[1:]))\n')
        for mover in (False, True):
            r = self.ejecutar('--evasion-reconocimiento','--senales','--ensayo-giro-companero',
                              *(['--mover'] if mover else []))
            self.assertEqual(r.returncode, 0, r.stderr)
            args = json.loads(r.stdout)
            self.assertIn('--ensayo-giro-companero', args)
            self.assertEqual('--mover' in args, mover)
            self.assertFalse(self.registro.exists())
        for opciones in (('--solo-evasion',), ('--solo-senales',), ('--prueba-meta-b',),
                         ('--evasion-reconocimiento',)):
            r = self.ejecutar(*opciones, '--ensayo-giro-companero')
            self.assertEqual(r.returncode, 2, r.stderr)
            self.assertFalse(self.registro.exists())
