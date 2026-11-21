"""Evasión y reconocimiento con giros visuales opcionales; A registrado, sin retorno aún."""

import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import threading
import time
import uuid

import cv2

from control_busqueda import CONFIRMACIONES_VISUALES, EDAD_CAMARA_NS, SesionBusqueda, reciente
from control_giros import cargar_config_senales
from evasion_lidar import cargar_config
from interfaz_facial import BUSCAR, SALIR, EntradaObjetivo, dibujar_interfaz, dentro, escribir
from prueba_facial_robot import (TAREA, TOPICO, ConsultaEnVivo, cargar_sistema, cargar_transporte,
                                ejecutar_comando, interpretar_topicos, preparar_camara)
from senales_giro import ConsultaSenales, dibujar_detecciones


def preparar_movimiento(comando_camara, contenedor, config, mover, config_senales=None, tramo='ida',
                        control_giros_origen=None):
    """Usa la misma sesión Docker/red de cámara; nunca copia datos faciales."""
    remoto = str(Path(comando_camara[-2]).parent)
    base = comando_camara[:comando_camara.index(contenedor) + 1]
    base = [v for v in base if v != '-i']
    shell = [*base, 'bash', '--noprofile', '--norc', '-c']
    texto = ejecutar_comando([*shell,
                             'unset ROS_LOCALHOST_ONLY; source /opt/ros/jazzy/setup.bash && '
                             'ros2 topic list -t --no-daemon --spin-time 5'])
    topicos = interpretar_topicos(texto)
    esperados = {'/cmd_vel': 'geometry_msgs/msg/TwistStamped',
                 '/scan': 'sensor_msgs/msg/LaserScan', '/odom': 'nav_msgs/msg/Odometry'}
    for topico, tipo in esperados.items():
        tipos = topicos.get(topico, [])
        if tipos and tipos != [tipo]:
            raise ValueError(f'Tipo incompatible en {topico}: {tipos}')
    if topicos.get('/cmd_vel') != [esperados['/cmd_vel']]:
        raise ValueError('No se descubrió /cmd_vel stamped; comprobar conexión/controladores')
    ejecutar_comando([*shell, 'source /opt/ros/jazzy/setup.bash && '
                     'python3 -c "import rclpy, tf2_ros, nav_msgs.msg, sensor_msgs.msg, geometry_msgs.msg"'])
    for archivo in ('evasion_lidar.py', 'seguridad_lidar.py', 'control_giros.py', 'config_senales.py',
                    'control_busqueda.py', 'busqueda_lidar_ros.py', 'telemetria_ros.py'):
        origen = control_giros_origen if archivo == 'control_giros.py' and control_giros_origen else TAREA / archivo
        ejecutar_comando(['sudo', '-n', 'docker', 'cp', str(origen), f'{contenedor}:{remoto}/{archivo}'])
    ejecutar_comando(['sudo', '-n', 'docker', 'cp', str(config), f'{contenedor}:{remoto}/config.json'])
    opciones_senales = []
    if config_senales is not None:
        ejecutar_comando(['sudo', '-n', 'docker', 'cp', str(config_senales),
                         f'{contenedor}:{remoto}/senales.json'])
        opciones_senales = ['--senales', '--tramo', tramo, '--config-senales', f'{remoto}/senales.json']
    return [*comando_camara[:comando_camara.index(contenedor) + 1],
            'bash', '--noprofile', '--norc', '-c',
            'unset ROS_LOCALHOST_ONLY; source /opt/ros/jazzy/setup.bash && exec python3 -u "$@"',
            'busqueda-lidar', f'{remoto}/busqueda_lidar_ros.py', '--config', f'{remoto}/config.json',
            *(['--mover'] if mover else []), *opciones_senales]


class PuenteBusqueda:
    """Latido separado de GUI; no renueva la edad de una inferencia bloqueada."""

    def __init__(self, comando, senales=False):
        self.proceso = subprocess.Popen(comando, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                        text=True, bufsize=1)
        self.fin = threading.Event()
        self.cerrojo = threading.Lock()
        self.datos = dict(orden='ESPERAR', actividad_ns=0, camara_ns=None)
        self.senales = senales
        self.ultima_senal_enviada = None
        if senales:
            self.datos['senal'] = None
        self.estado = None
        self.fallo = None
        self.emisor = threading.Thread(target=self.enviar, daemon=True)
        self.lector = threading.Thread(target=self.recibir, daemon=True)
        self.emisor.start()
        self.lector.start()

    def actualizar(self, orden, actividad_ns, camara_ns, senal=None):
        with self.cerrojo:
            self.datos = dict(orden=orden, actividad_ns=actividad_ns, camara_ns=camara_ns)
            if self.senales:
                self.datos['senal'] = senal

    def enviar(self):
        secuencia = 0
        try:
            while not self.fin.is_set():
                with self.cerrojo:
                    datos = dict(self.datos)
                self.proceso.stdin.write(json.dumps(dict(version=1, secuencia=secuencia, **datos)) + '\n')
                self.proceso.stdin.flush()
                if datos.get('senal') is not None:
                    with self.cerrojo:
                        self.ultima_senal_enviada = dict(datos['senal'])
                secuencia += 1
                self.fin.wait(0.1)
        except (OSError, ValueError) as exc:
            self.fallo = f'Se cerró el control de movimiento: {exc}'
        finally:
            try:
                self.proceso.stdin.write(json.dumps(dict(version=1, secuencia=secuencia,
                    orden='PARAR', actividad_ns=time.monotonic_ns(), camara_ns=None)) + '\n')
                self.proceso.stdin.flush()
                self.proceso.stdin.close()
            except (OSError, ValueError):
                pass

    def recibir(self):
        try:
            while not self.fin.is_set():
                linea = self.proceso.stdout.readline(8193)
                if not linea:
                    if not self.fin.is_set():
                        self.fallo = 'El nodo de movimiento cerró su salida'
                    break
                if len(linea) > 8192:
                    raise ValueError('Telemetría demasiado larga')
                d = json.loads(linea)
                if not isinstance(d, dict) or d.get('version') != 1:
                    raise ValueError('Telemetría no válida')
                with self.cerrojo:
                    self.estado = d
        except (OSError, ValueError) as exc:
            self.fallo = f'Error del nodo de movimiento: {exc}'

    def ultimo_estado(self, ahora_ns):
        with self.cerrojo:
            estado = self.estado
        if estado and reciente(estado.get('recibido_ns'), ahora_ns, EDAD_CAMARA_NS):
            return estado
        return None

    def ultimo_envio_senal(self):
        with self.cerrojo:
            return dict(self.ultima_senal_enviada) if self.ultima_senal_enviada else None

    def comprobar(self):
        if self.fallo or self.proceso.poll() is not None:
            raise RuntimeError(self.fallo or 'Terminó el nodo de movimiento')

    def cerrar(self):
        self.fin.set()
        self.emisor.join(timeout=1)
        try:
            self.proceso.wait(timeout=2)
        except subprocess.TimeoutExpired:
            self.proceso.terminate()
            try:
                self.proceso.wait(timeout=1)
            except subprocess.TimeoutExpired:
                self.proceso.kill()
                self.proceso.wait(timeout=1)
        self.lector.join(timeout=0.5)
        if not self.lector.is_alive():
            self.proceso.stdout.close()


class EntradaBusqueda(EntradaObjetivo):
    def __init__(self, consulta, sesion):
        super().__init__(consulta)
        self.sesion = sesion
        self.aviso = 'Buscar registra A e inicia la evasion cuando los sensores esten listos.'

    def confirmar(self):
        if self.sesion.inicio_ns is not None:
            return
        super().confirmar()
        if not self.error:
            self.sesion.iniciar(self.consulta.nombre, time.monotonic_ns())
            self.aviso = 'Nombre fijado para esta busqueda. Esc o Salir detienen la prueba.'

    def raton(self, evento, x, y, banderas=None, parametro=None):
        if self.sesion.inicio_ns is None or dentro(SALIR, x, y):
            super().raton(evento, x, y, banderas, parametro)

    def tecla(self, codigo):
        if self.sesion.inicio_ns is None or codigo in (27, 65307, ord('q'), ord('Q')):
            return super().tecla(codigo)
        return self.cerrar


def dibujar_busqueda(imagen, resultado, estado_camara, entrada, estado_ros, mover,
                     senales=None, tramo='ida'):
    panel = dibujar_interfaz(imagen, resultado, estado_camara, entrada)
    if entrada.sesion.inicio_ns is not None:
        cv2.rectangle(panel, (18, 101), (943, 178), (27, 27, 27), -1)
        estado = estado_ros['estado'] if estado_ros else 'Esperando respuesta del control: detenido'
        if entrada.sesion.encontrado:
            estado = 'Persona encontrada. Robot detenido. Regreso a A pendiente.'
        escribir(panel, estado, 18, 123, (100, 225, 100) if entrada.sesion.encontrado else (70, 190, 240))
        a = estado_ros.get('a') if estado_ros else None
        texto_a = (f"A guardado: x={a['x']:.2f} m, y={a['y']:.2f} m ({a['frame_id']})"
                   if a else 'A pendiente: esperando odometria valida; robot detenido')
        escribir(panel, texto_a, 18, 148)
        escribir(panel, f'Rostro aceptado: {entrada.sesion.confirmaciones}/{CONFIRMACIONES_VISUALES} imagen nueva valida', 18, 172)
        cv2.rectangle(panel, BUSCAR[:2], BUSCAR[2:], (65, 65, 65), -1)
        escribir(panel, 'En curso', 775, 76)
    cv2.rectangle(panel, (0, 669), (840, 707), (27, 27, 27), -1)
    pie = 'MOVIMIENTO ACTIVADO' if mover else 'SIN MOVIMIENTO'
    pie += (f" | {tramo.upper()} | {senales['estado']}" if senales is not None
            else ' | Evasion + reconocimiento | Esc: detener y salir')
    if senales is not None and entrada.sesion.inicio_ns is None:
        pie = 'VISTA PREVIA; pulsar Buscar | ' + pie
    diagnostico = estado_ros.get('giro_ensayo') if estado_ros else None
    if diagnostico is not None:
        frente = diagnostico['frente_m']
        distancia = '-' if frente is None else f'{frente:.2f} m'
        pie = f"ENSAYO COMPANERO | {diagnostico['fase']} | frente {distancia} | {pie}"
    escribir(panel, pie, 18, 693, limite=96)
    return panel


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ip', required=True)
    parser.add_argument('--dominio', type=int, required=True)
    parser.add_argument('--contenedor', default='proyecto2-sim')
    parser.add_argument('--config', type=Path, default=TAREA/'config/evasion_lidar.json')
    parser.add_argument('--topico-camara', default=TOPICO)
    parser.add_argument('--nombre')
    parser.add_argument('--mover', action='store_true')
    parser.add_argument('--senales', action='store_true', help='Confirmar señales y ejecutar giros con odometría')
    parser.add_argument('--tramo', choices=('ida', 'vuelta'), default='ida')
    parser.add_argument('--config-senales', type=Path, default=TAREA/'config/senales_giro.json')
    parser.add_argument('--ensayo-giro-companero', action='store_true',
                        help='Probar espera de pared del snapshot e78c3b5, sin fusionar la rama')
    args = parser.parse_args(argv)
    if args.ensayo_giro_companero and not args.senales:
        parser.error('--ensayo-giro-companero requiere --senales')
    if args.ensayo_giro_companero and args.config_senales != TAREA/'config/senales_giro.json':
        parser.error('El ensayo usa config/senales_giro_ensayo.json; no combinar con --config-senales')
    video, puente = None, None
    ventana = 'Task 1 - evasion y busqueda de persona'
    abierta = False
    try:
        if not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
            raise ValueError('Esta prueba requiere una terminal de Ubuntu con interfaz gráfica')
        cfg_evasion = cargar_config(args.config)
        control_origen = None
        if args.ensayo_giro_companero:
            from ensayo_giros_companero import CONFIG, CONTROL, SNAPSHOT, cargar_ensayo
            _, cfg_senales = cargar_ensayo()
            args.config_senales, control_origen = CONFIG, CONTROL
            print(f'[ENSAYO GIROS] Compañero {SNAPSHOT[:7]}: pared {cfg_senales.esperar_pared_m:.2f} m, '
                  f'memoria {cfg_senales.tiempo_espera_pared_s:.1f} s; '
                  f'avance {cfg_evasion.velocidad_avance:.2f} m/s, giro {cfg_evasion.velocidad_giro:.2f} rad/s. '
                  'Controlador original con ajustes locales de señales y LiDAR; no es la misión completa.', flush=True)
        else:
            cfg_senales = cargar_config_senales(args.config_senales) if args.senales else None
        consulta_senales = ConsultaSenales(args.tramo, cfg_senales) if args.senales else None
        inicio_senales = None
        cv2.setNumThreads(1)
        consulta = ConsultaEnVivo(cargar_sistema(), args.nombre)
        sesion = SesionBusqueda()
        entrada = EntradaBusqueda(consulta, sesion)
        comando_camara = preparar_camara(args.ip, args.dominio, args.contenedor, args.topico_camara)
        comando_control = preparar_movimiento(comando_camara, args.contenedor, args.config, args.mover,
            args.config_senales if args.senales else None, args.tramo,
            control_giros_origen=control_origen)
        VideoRobot = cargar_transporte()
        video = VideoRobot(subprocess.Popen(comando_camara, stdin=subprocess.PIPE, stdout=subprocess.PIPE))
        puente = PuenteBusqueda(comando_control, senales=args.senales)
        carpeta = TAREA/'datos/sesiones'
        carpeta.mkdir(exist_ok=True)
        archivo_a = carpeta/f'punto_a_{uuid.uuid4().hex}.json'
        guardado = False
        cv2.namedWindow(ventana, cv2.WINDOW_NORMAL | cv2.WINDOW_GUI_NORMAL)
        abierta = True
        cv2.resizeWindow(ventana, 960, 708)
        cv2.setMouseCallback(ventana, entrada.raton)
        print('Evasión + reconocimiento. Buscar fija el nombre y registra A antes de moverse. '
              'Cerrar otros controles. El retorno a A aún no está implementado.', flush=True)
        ultimo_log = 0
        while True:
            puente.comprobar()
            if video.proceso.poll() is not None:
                raise RuntimeError('Terminó el receptor de cámara')
            imagen, estado_camara = video.fotograma()
            marca = video._decodificado['recibido_ns'] if imagen is not None else None
            resultado = consulta.actualizar(imagen, marca)
            resultado_senales = None
            if consulta_senales is not None:
                if sesion.inicio_ns is not None and inicio_senales != sesion.inicio_ns:
                    consulta_senales = ConsultaSenales(args.tramo, cfg_senales)
                    inicio_senales = sesion.inicio_ns
                posterior = sesion.inicio_ns is None or (marca is not None and marca > sesion.inicio_ns)
                resultado_senales = consulta_senales.actualizar(
                    imagen if posterior else None, marca if posterior else None, time.monotonic_ns())
            ahora = time.monotonic_ns()
            if not reciente(marca, ahora, EDAD_CAMARA_NS):
                imagen, marca = None, None
                resultado = consulta.actualizar(None, None)
                if consulta_senales is not None:
                    resultado_senales = consulta_senales.actualizar(None, None, ahora)
            estado_ros = puente.ultimo_estado(ahora)
            a = estado_ros.get('a') if estado_ros else None
            if a and not guardado:
                archivo_a.write_text(json.dumps(dict(version=1, punto_a=a,
                    alcance='odometria de esta sesion; retorno no implementado'), indent=2)+'\n')
                guardado = True
                print(f'Punto A registrado en {archivo_a}', flush=True)
            sesion.actualizar(resultado, marca, ahora, bool(a))
            # La GUI/inferencia debe renovar su actividad; el hilo de latido no la inventa.
            evento_senal = resultado_senales['evento'] if resultado_senales else None
            if evento_senal and (sesion.inicio_ns is None or evento_senal['marca_ns'] <= sesion.inicio_ns):
                evento_senal = None  # La vista previa no inicia ni anticipa una maniobra.
            puente.actualizar(sesion.orden, ahora, marca, evento_senal)
            if ahora - ultimo_log >= 1_000_000_000:
                print(f"[BUSQUEDA] {sesion.orden} | {resultado['nombre']} | "
                      f"{estado_ros['estado'] if estado_ros else 'Esperando nodo ROS'}", flush=True)
                if args.ensayo_giro_companero:
                    from ensayo_giros_companero import diagnostico_senales
                    print('[SENALES ENSAYO] ' + json.dumps(diagnostico_senales(
                        resultado_senales, sesion.inicio_ns, ahora, puente.ultimo_envio_senal()),
                        ensure_ascii=False), flush=True)
                    print('[GIRO ENSAYO] ' + json.dumps(
                        estado_ros.get('giro_ensayo') if estado_ros else None, ensure_ascii=False), flush=True)
                ultimo_log = ahora
            anotada = dibujar_detecciones(imagen, resultado_senales, args.tramo) if (
                imagen is not None and resultado_senales is not None) else imagen
            cv2.imshow(ventana, dibujar_busqueda(anotada, resultado, estado_camara, entrada,
                estado_ros, args.mover, resultado_senales, args.tramo))
            if entrada.tecla(cv2.waitKeyEx(20)) or cv2.getWindowProperty(ventana, cv2.WND_PROP_VISIBLE) < 1:
                break
    except KeyboardInterrupt:
        return 0
    except (OSError, ValueError, TypeError, KeyError, RuntimeError, subprocess.CalledProcessError, cv2.error) as exc:
        print(f'Error de búsqueda integrada: {exc}', file=sys.stderr)
        return 1
    finally:
        try:
            if puente is not None:
                puente.cerrar()
        finally:
            try:
                if video is not None:
                    video.cerrar()
            finally:
                if abierta:
                    cv2.destroyAllWindows()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
