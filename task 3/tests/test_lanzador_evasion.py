"""Lanzador de hardware con Docker/sudo/ping simulados; nunca mueve un robot."""

import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest


TAREA = Path(__file__).resolve().parents[1]


class LanzadorEvasionTests(unittest.TestCase):
    def setUp(self):
        temporal = tempfile.TemporaryDirectory(prefix='prueba-evasion-')
        self.addCleanup(temporal.cleanup)
        raiz = Path(temporal.name) / 'Proyecto con espacios'
        self.tarea = raiz / 'task 1'
        (self.tarea / 'config').mkdir(parents=True)
        for nombre in ('ejecutar_autonomo_robot.sh', 'evasion_lidar.py', 'seguridad_lidar.py',
                       'config/evasion_lidar.json'):
            shutil.copyfile(TAREA / nombre, self.tarea / nombre)
        (raiz / '.venv/bin').mkdir(parents=True)
        (raiz / '.venv/bin/python').symlink_to(sys.executable)
        binarios = raiz / 'bin'
        binarios.mkdir()
        self.registro = raiz / 'docker.jsonl'
        for nombre, cuerpo in {
            'sudo': "import os, sys\nif sys.argv[1:] == ['-v']: sys.exit(0)\nos.execvp(sys.argv[1], sys.argv[1:])\n",
            'ping': 'pass\n',
            'docker': '''import json, os, sys
args = sys.argv[1:]
with open(os.environ['EVASION_REGISTRO'], 'a') as f:
    f.write(json.dumps(args) + '\\n')
if args[0] == 'inspect':
    print(os.environ.get('EVASION_RED', 'true host'))
elif any('ros2 topic type' in a for a in args):
    print(os.environ.get('EVASION_TIPO', 'sensor_msgs/msg/LaserScan' if args[-1] == '/scan' else 'geometry_msgs/msg/TwistStamped'))
elif 'mktemp' in args:
    print('/tmp/proyecto2-evasion.ABC12345')
''',
        }.items():
            p = binarios / nombre
            p.write_text(f'#!{sys.executable}\n' + cuerpo)
            p.chmod(0o755)
        self.env = dict(os.environ, PATH=str(binarios) + os.pathsep + os.environ['PATH'],
                        TB4_IP='10.87.179.100', TB4_DOMINIO='5', TB4_CONTENEDOR='proyecto2-sim',
                        EVASION_REGISTRO=str(self.registro))

    def ejecutar(self, *args):
        return subprocess.run(['bash', str(self.tarea / 'ejecutar_autonomo_robot.sh'), *args],
                              cwd='/tmp', env=self.env, capture_output=True, text=True, timeout=10)

    def llamadas(self):
        return [json.loads(s) for s in self.registro.read_text().splitlines()]

    def test_observar_por_defecto_y_red_correcta_sin_galeria(self):
        resultado = self.ejecutar('--solo-evasion')
        self.assertEqual(resultado.returncode, 0, resultado.stderr)
        final = self.llamadas()[-1]
        for valor in ('ROS_DOMAIN_ID=5', 'ROS_STATIC_PEERS=10.87.179.100',
                      'RMW_IMPLEMENTATION=rmw_fastrtps_cpp', '--norc'):
            self.assertIn(valor, final)
        self.assertNotIn('--mover', final)
        self.assertIn('SIN MOVIMIENTO', resultado.stderr)
        self.assertFalse(any('rm' in c for c in self.llamadas()))

    def test_mover_solo_con_opcion_explicita(self):
        resultado = self.ejecutar('--solo-evasion', '--mover', '--dominio', '005')
        self.assertEqual(resultado.returncode, 0, resultado.stderr)
        self.assertIn('--mover', self.llamadas()[-1])
        self.assertIn('ROS_DOMAIN_ID=5', self.llamadas()[-1])

    def test_rechaza_mision_no_integrada_y_argumentos_invalidos_antes_de_docker(self):
        for args in ((), ('--mover',), ('--solo-evasion', '--ip', '999.1.1.1'),
                     ('--solo-evasion', '--dominio', '102'), ('--solo-evasion', '--slam-nav2')):
            with self.subTest(args=args):
                resultado = self.ejecutar(*args)
                self.assertEqual(resultado.returncode, 2, resultado.stderr)
                self.assertFalse(self.registro.exists())

    def test_tipo_incorrecto_o_red_bridge_no_lanza_receptor(self):
        for variable, valor in (('EVASION_TIPO', 'geometry_msgs/msg/Twist'),
                                ('EVASION_RED', 'true bridge')):
            with self.subTest(variable=variable):
                self.env[variable] = valor
                resultado = self.ejecutar('--solo-evasion', '--mover')
                self.assertNotEqual(resultado.returncode, 0)
                self.assertFalse(any('-it' in c for c in self.llamadas()))
                self.env.pop(variable)
                self.registro.unlink()

    def test_configuracion_invalida_impide_contactar_docker(self):
        (self.tarea / 'config/evasion_lidar.json').write_text('{"velocidad_avance": 3}')
        resultado = self.ejecutar('--solo-evasion', '--mover')
        self.assertNotEqual(resultado.returncode, 0)
        self.assertFalse(self.registro.exists())

    def test_reconocimiento_delega_al_host_y_rechaza_movimiento(self):
        (self.tarea / 'prueba_facial_robot.py').write_text(
            'import json, sys\nprint(json.dumps(sys.argv[1:]))\n')
        resultado = self.ejecutar('--solo-reconocimiento', '--nombre', 'Persona A')
        self.assertEqual(resultado.returncode, 0, resultado.stderr)
        args = json.loads(resultado.stdout)
        self.assertIn('Persona A', args)
        self.assertIn('10.87.179.100', args)
        self.assertIn('5', args)
        self.assertFalse(self.registro.exists())
        for opciones in (('--solo-reconocimiento', '--mover'),
                         ('--solo-reconocimiento', '--solo-evasion')):
            with self.subTest(opciones=opciones):
                resultado = self.ejecutar(*opciones)
                self.assertEqual(resultado.returncode, 2, resultado.stderr)
                self.assertFalse(self.registro.exists())

    def test_integrada_delega_al_host_y_solo_pasa_mover_si_es_explicito(self):
        (self.tarea/'busqueda_robot.py').write_text('import json, sys\nprint(json.dumps(sys.argv[1:]))\n')
        for mover in (False,True):
            resultado = self.ejecutar('--evasion-reconocimiento', *(['--mover'] if mover else []))
            self.assertEqual(resultado.returncode,0,resultado.stderr)
            args=json.loads(resultado.stdout)
            self.assertEqual('--mover' in args,mover)
            self.assertIn('--config',args)
            self.assertFalse(self.registro.exists())
        for extra in ('--solo-evasion','--solo-reconocimiento','--sin-ventana'):
            resultado=self.ejecutar('--evasion-reconocimiento',extra)
            self.assertEqual(resultado.returncode,2,resultado.stderr)

    def test_senales_solas_sin_mover_y_combinadas_solo_con_opcion_explicita(self):
        for archivo in ('prueba_senales_robot.py', 'busqueda_robot.py'):
            (self.tarea/archivo).write_text('import json,sys\nprint(json.dumps(sys.argv[1:]))\n')
        r = self.ejecutar('--solo-senales', '--tramo', 'vuelta')
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn('vuelta', json.loads(r.stdout))
        self.assertFalse(self.registro.exists())
        r = self.ejecutar('--evasion-reconocimiento', '--senales', '--tramo', 'ida', '--mover')
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn('--senales', json.loads(r.stdout))
        self.assertIn('--mover', json.loads(r.stdout))
        for opciones in (('--solo-senales', '--mover'), ('--solo-evasion', '--senales'),
                         ('--solo-reconocimiento', '--tramo', 'ida'),
                         ('--solo-senales', '--nombre', 'Persona A'),
                         ('--evasion-reconocimiento', '--tramo', 'vuelta'),
                         ('--solo-senales', '--evasion-reconocimiento')):
            r = self.ejecutar(*opciones)
            self.assertEqual(r.returncode, 2, r.stderr)
        self.assertFalse(self.registro.exists())


if __name__ == '__main__':
    unittest.main()
