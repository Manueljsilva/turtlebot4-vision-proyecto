"""Contrato del lanzador con dobles de Docker, sudo y webcam; sin hardware."""

import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest


RAIZ = Path(__file__).resolve().parents[1]


class LanzadorRobotTests(unittest.TestCase):
    def setUp(self):
        self.temporal = tempfile.TemporaryDirectory(prefix='proyecto2-lanzador-')
        self.addCleanup(self.temporal.cleanup)
        proyecto = Path(self.temporal.name) / 'Proyecto con espacios'
        self.tarea = proyecto / 'task 2'
        self.tarea.mkdir(parents=True)
        for nombre in ['ejecutar_gestos_robot.sh', 'gestos.py', 'control_gestos.py',
                       'puente_gestos_ros.py']:
            shutil.copyfile(RAIZ / nombre, self.tarea / nombre)
        (self.tarea / 'modelos').mkdir()
        (self.tarea / 'modelos/hand_landmarker.task').touch()
        self.registro = proyecto / 'docker.jsonl'
        self.camara = proyecto / 'camara_iniciada'
        binarios = proyecto / 'bin'
        binarios.mkdir()
        (proyecto / '.venv/bin').mkdir(parents=True)

        def ejecutable(ruta, contenido):
            ruta.write_text(f'#!{sys.executable}\n' + contenido)
            ruta.chmod(0o755)

        ejecutable(proyecto / '.venv/bin/python', '''import json, os, sys, time
if len(sys.argv) > 2 and sys.argv[1] == '-u':
    open(os.environ['PRUEBA_CAMARA'], 'w').close()
    print(json.dumps(dict(version=1, orden='DETENER', captura_ns=time.monotonic_ns(), secuencia=0)))
else:
    os.execv(sys.executable, [sys.executable, *sys.argv[1:]])
''')
        ejecutable(binarios / 'sudo', '''import os, sys
if sys.argv[1:] == ['-v']:
    sys.exit(0)
os.execvp(sys.argv[1], sys.argv[1:])
''')
        ejecutable(binarios / 'ping', 'import sys\nsys.exit(0)\n')
        ejecutable(binarios / 'docker', '''import json, os, sys
args = sys.argv[1:]
with open(os.environ['PRUEBA_REGISTRO'], 'a') as archivo:
    archivo.write(json.dumps(args) + '\\n')
if args[0] == 'inspect':
    print(os.environ.get('PRUEBA_ESTADO', 'true host'))
elif args[0] == 'exec' and '-i' in args:
    for linea in sys.stdin:
        assert json.loads(linea)['orden'] == 'DETENER'
elif args[0] == 'exec' and any('ros2 topic type /cmd_vel' in a for a in args):
    print(os.environ.get('PRUEBA_TIPO', 'geometry_msgs/msg/TwistStamped'))
''')
        self.env = dict(os.environ, PATH=str(binarios) + os.pathsep + os.environ['PATH'],
                        PRUEBA_REGISTRO=str(self.registro), PRUEBA_CAMARA=str(self.camara),
                        TB4_IP='', TB4_DOMINIO='', TB4_CONTENEDOR='proyecto2-sim')

    def ejecutar(self, *extras):
        return subprocess.run(['bash', str(self.tarea / 'ejecutar_gestos_robot.sh'),
                               '--ip', '10.87.179.100', '--dominio', '5', *extras],
                              cwd='/tmp', env=self.env, text=True, capture_output=True, timeout=10)

    def entrada_puente(self):
        llamadas = [json.loads(linea) for linea in self.registro.read_text().splitlines()]
        return next(args for args in llamadas if args[0] == 'exec' and '-i' in args)

    def test_por_defecto_solo_comprueba_y_pasa_red_y_modo_real(self):
        resultado = self.ejecutar()
        self.assertEqual(resultado.returncode, 0, resultado.stderr)
        args = self.entrada_puente()
        for esperado in ['ROS_DOMAIN_ID=5', 'ROS_STATIC_PEERS=10.87.179.100',
                         'RMW_IMPLEMENTATION=rmw_fastrtps_cpp',
                         'ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET', '--robot-real', '--norc']:
            self.assertIn(esperado, args)
        self.assertNotIn('--publicar', args)
        self.assertIn('COMPROBACION', resultado.stderr)

    def test_mover_activa_publicacion_explicitamente(self):
        resultado = self.ejecutar('--mover', '--camara', '1')
        self.assertEqual(resultado.returncode, 0, resultado.stderr)
        self.assertIn('--publicar', self.entrada_puente())
        self.assertTrue(self.camara.exists())

    def test_tipo_incorrecto_no_abre_camara(self):
        self.env['PRUEBA_TIPO'] = 'geometry_msgs/msg/Twist'
        resultado = self.ejecutar('--mover')
        self.assertNotEqual(resultado.returncode, 0)
        self.assertIn('Tipo de /cmd_vel inesperado', resultado.stderr)
        self.assertFalse(self.camara.exists())

    def test_red_bridge_no_abre_camara(self):
        self.env['PRUEBA_ESTADO'] = 'true bridge'
        resultado = self.ejecutar('--mover')
        self.assertNotEqual(resultado.returncode, 0)
        self.assertIn('red host', resultado.stderr)
        self.assertFalse(self.camara.exists())

    def test_argumentos_invalidos_no_invocan_docker(self):
        for extras in [('--dominio', '102'), ('--dominio', '-1'), ('--ip', '999.1.1.1')]:
            with self.subTest(extras=extras):
                resultado = self.ejecutar(*extras)
                self.assertEqual(resultado.returncode, 2, resultado.stderr)
                self.assertFalse(self.registro.exists())
                self.assertFalse(self.camara.exists())
