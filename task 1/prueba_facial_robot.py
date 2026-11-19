"""Prueba facial con la cámara ROS del TurtleBot y visión en la laptop, sin mover.

Reutiliza el transporte de cámara existente de Task 2 sin arrancar sus gestos.
No guarda imágenes, modifica la galería, recalibra ni publica velocidades.
"""

import argparse
from dataclasses import dataclass
import ipaddress
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import time
import unicodedata

import cv2
import numpy as np

from modelos_rostros import MODELOS, preparar_modelos, sha256_archivo
from reconocimiento_facial import Criterio, ExtractorFacial, Galeria, decidir


TAREA = Path(__file__).resolve().parent
TOPICO = '/oakd/rgb/preview/image_raw/compressed'
EDAD_MAXIMA_NS = 1_000_000_000


@dataclass
class SistemaFacial:
    extractor: ExtractorFacial
    galeria: Galeria
    criterio: Criterio


def cargar_sistema(carpeta=TAREA):
    carpeta = Path(carpeta)
    datos = json.loads((carpeta / 'datos/criterio.json').read_text(encoding='utf-8'))
    if datos.get('version') != 1:
        raise ValueError('Versión de criterio no compatible')
    esperados = {nombre: d['sha256'] for nombre, d in MODELOS.items()}
    if datos.get('modelos_sha256') != esperados:
        raise ValueError('El criterio no corresponde a estos modelos; revisar el notebook')
    if sha256_archivo(carpeta / 'datos/particion_rostros.json') != datos.get('particion_sha256'):
        raise ValueError('El manifiesto y el criterio no coinciden; revisar el notebook')
    with np.load(carpeta / 'datos/galeria.npz', allow_pickle=False) as archivo:
        galeria = Galeria(archivo['vectores'], archivo['alumnos'], archivo['origenes'])
    if galeria.vectores.shape[1:] != (128,) or len(set(galeria.alumnos.tolist())) < 2:
        raise ValueError('Galería incompatible o con identidades insuficientes')
    criterio = Criterio(**datos['criterio'])
    rutas = preparar_modelos(carpeta / 'modelos', permitir_descarga=False)
    extractor = ExtractorFacial(rutas, **datos['extractor'])
    return SistemaFacial(extractor, galeria, criterio)


def resolver_nombre(nombre, galeria):
    if nombre is None:
        return None
    clave = unicodedata.normalize('NFC', nombre.strip()).casefold()
    opciones = [n for n in sorted(set(galeria.alumnos.tolist()))
                if unicodedata.normalize('NFC', n.strip()).casefold() == clave]
    if len(opciones) != 1:
        raise ValueError('Nombre ausente o ambiguo; consultar --listar-nombres')
    return opciones[0]


def mensaje_rechazo(motivo):
    return dict(nombre='NO_CONCLUYENTE', aceptado=False, objetivo_confirmado=False,
                candidato=None, similitud=None, margen=None, motivo=motivo, caja=None,
                afinidad_objetivo=None)


class ConsultaEnVivo:
    """Una decisión por paquete de imagen; no acumula votos de frames repetidos."""

    def __init__(self, sistema, nombre=None):
        self.sistema = sistema
        self.nombre = resolver_nombre(nombre, sistema.galeria)
        self.ultima_marca = None
        self.ultimo_resultado = None

    def seleccionar_nombre(self, nombre):
        """Cambiar solo el objetivo; conservar la identidad observada y su caducidad."""
        seleccionado = resolver_nombre(nombre, self.sistema.galeria)
        if seleccionado is None:
            raise ValueError('Escribe el nombre de una persona de la galería')
        self.nombre = seleccionado
        if self.ultimo_resultado is not None:
            self.ultimo_resultado = dict(
                self.ultimo_resultado,
                objetivo_confirmado=bool(self.ultimo_resultado['aceptado']
                                        and self.ultimo_resultado['nombre'] == seleccionado))

    def actualizar(self, imagen, recibido_ns, ahora_ns=None):
        ahora_ns = time.monotonic_ns() if ahora_ns is None else ahora_ns
        if (imagen is None or type(recibido_ns) is not int
                or not 0 <= ahora_ns - recibido_ns <= EDAD_MAXIMA_NS):
            self.ultimo_resultado = None
            return mensaje_rechazo('SIN_IMAGEN_RECIENTE')
        if recibido_ns == self.ultima_marca:
            return self.ultimo_resultado or mensaje_rechazo('SIN_IMAGEN_RECIENTE')
        self.ultima_marca = recibido_ns
        analisis = self.sistema.extractor.analizar_imagen(imagen)
        if analisis.estado != 'OK':
            resultado = mensaje_rechazo(analisis.estado)
        else:
            ranking = self.sistema.galeria.puntuar(analisis.descriptor)
            resultado = decidir(ranking, self.sistema.criterio)
            resultado['objetivo_confirmado'] = bool(
                resultado['aceptado'] and self.nombre is not None
                and resultado['nombre'] == self.nombre)
            resultado['caja'] = analisis.caja
            resultado['afinidad_objetivo'] = (
                dict(ranking).get(self.nombre) if self.nombre is not None else None
            )
        self.ultimo_resultado = resultado
        return resultado


def cargar_transporte():
    ruta = TAREA.parent / 'task 2'
    if not (ruta / 'camara_robot.py').is_file():
        raise FileNotFoundError('Falta el transporte compartido task 2/camara_robot.py')
    sys.path.insert(0, str(ruta))
    from camara_robot import VideoRobot
    return VideoRobot


def ejecutar_comando(comando):
    try:
        resultado = subprocess.run(comando, check=False, text=True, stdout=subprocess.PIPE,
                                   stderr=subprocess.PIPE, timeout=25)
    except subprocess.TimeoutExpired as exc:
        raise RuntimeError('Una comprobación de conexión agotó 25 s; revisar Docker y enlace ROS') from exc
    if resultado.returncode:
        detalle = (resultado.stderr or resultado.stdout or 'El comando no proporcionó detalles').strip()
        raise RuntimeError(f'La comprobación falló (código {resultado.returncode}): {detalle[-2000:]}')
    return resultado.stdout.strip()


def interpretar_topicos(texto):
    """Formato oficial de ros2 topic list -t; no ejecuta el contenido recibido."""
    salida = {}
    for linea in texto.splitlines():
        coincidencia = re.fullmatch(r'\s*(/\S+)\s+\[([^\]]+)\]\s*', linea)
        if coincidencia:
            salida[coincidencia[1]] = [tipo.strip() for tipo in coincidencia[2].split(',')]
    return salida


def preparar_camara(ip, dominio, contenedor, topico, fps=None):
    ipaddress.IPv4Address(ip)
    if not 0 <= dominio <= 101:
        raise ValueError('Dominio fuera de 0-101')
    if not re.fullmatch(r'[a-zA-Z0-9][a-zA-Z0-9_.-]*', contenedor):
        raise ValueError('Contenedor inválido')
    if not re.fullmatch(r'/[a-zA-Z0-9_/]+', topico):
        raise ValueError('Tópico inválido')
    for nombre in ('docker', 'sudo', 'ping'):
        if not shutil.which(nombre):
            raise ValueError(f'No se encuentra {nombre}')
    fuentes = TAREA.parent / 'task 2'
    archivos = ('camara_robot.py', 'camara_robot_ros.py')
    if not all((fuentes / a).is_file() for a in archivos):
        raise FileNotFoundError('Faltan los módulos compartidos de cámara de Task 2')
    ejecutar_comando(['ping', '-c', '2', '-W', '2', ip])
    subprocess.run(['sudo', '-v'], check=True)
    estado = ejecutar_comando(['sudo', '-n', 'docker', 'inspect', '--format',
                              '{{.State.Running}} {{.HostConfig.NetworkMode}}', contenedor])
    if estado != 'true host':
        raise ValueError('Docker debe estar encendido en red host; ver COMANDOS_LAB.txt')
    entorno = ['-e', f'ROS_DOMAIN_ID={dominio}', '-e', 'RMW_IMPLEMENTATION=rmw_fastrtps_cpp',
               '-e', 'ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET', '-e', f'ROS_STATIC_PEERS={ip}',
               '-e', 'RCUTILS_LOGGING_USE_STDOUT=0']
    if fps is not None:
        entorno.extend(['-e', f'TB4_VIDEO_FPS={float(fps):.1f}'])
    base = ['sudo', '-n', 'docker', 'exec', *entorno, contenedor, 'bash', '--noprofile', '--norc', '-c']
    texto = ejecutar_comando([*base,
                             'unset ROS_LOCALHOST_ONLY; source /opt/ros/jazzy/setup.bash && '
                             'ros2 topic list -t --no-daemon --spin-time 10'])
    topicos = interpretar_topicos(texto)
    tipos = topicos.get(topico, [])
    permitidos = ('sensor_msgs/msg/Image', 'sensor_msgs/msg/CompressedImage')
    imagenes = [f'{nombre} [{", ".join(t)}]' for nombre, t in sorted(topicos.items())
                if any(tipo in permitidos for tipo in t)]
    print('[CAMARA] Tópicos de imagen visibles: ' + ('; '.join(imagenes) or 'ninguno'), flush=True)
    if tipos and (len(tipos) != 1 or tipos[0] not in permitidos):
        raise ValueError(f'Tipo de cámara inesperado en {topico}: {", ".join(tipos)}')
    if not tipos:
        print(f'[CAMARA] Aún no aparece {topico} en dominio {dominio}. '
              'La ventana permanecerá abierta esperando la cámara. q o Ctrl+C para terminar.',
              flush=True)
    remoto = ejecutar_comando(['sudo', '-n', 'docker', 'exec', contenedor,
                              'mktemp', '-d', '/tmp/proyecto2-facial.XXXXXXXX'])
    if not re.fullmatch(r'/tmp/proyecto2-facial\.[a-zA-Z0-9]+', remoto):
        raise ValueError('Ruta temporal inesperada')
    for archivo in archivos:
        ejecutar_comando(['sudo', '-n', 'docker', 'cp', str(fuentes / archivo),
                         f'{contenedor}:{remoto}/{archivo}'])
    return ['sudo', '-n', 'docker', 'exec', '-i', *entorno, contenedor,
            'bash', '--noprofile', '--norc', '-c',
            'unset ROS_LOCALHOST_ONLY; source /opt/ros/jazzy/setup.bash && '
            'exec python3 -u "$1" --topico "$2"', 'camara-facial',
            f'{remoto}/camara_robot_ros.py', topico]


def main(argv=None):
    from interfaz_facial import EntradaObjetivo, dibujar_interfaz

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ip')
    parser.add_argument('--dominio', type=int)
    parser.add_argument('--contenedor', default='proyecto2-sim')
    parser.add_argument('--topico-camara', default=TOPICO)
    parser.add_argument('--nombre', help='Nombre de la galería que queremos confirmar (opcional)')
    parser.add_argument('--listar-nombres', action='store_true', help='Muestra etiquetas locales y sale')
    parser.add_argument('--sin-ventana', action='store_true')
    args = parser.parse_args(argv)
    video = None
    ventana_creada = False
    ventana = 'Task 1 - reconocimiento facial del robot'
    try:
        if (not args.sin_ventana and not args.listar_nombres
                and not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY'))):
            raise ValueError('Ejecutar en Ubuntu gráfico o añadir --sin-ventana')
        sistema = cargar_sistema()
        consulta = ConsultaEnVivo(sistema, args.nombre)
        entrada = EntradaObjetivo(consulta)
        if args.listar_nombres:
            print('\n'.join(sorted(set(sistema.galeria.alumnos.tolist()))))
            return 0
        if args.ip is None or args.dominio is None:
            parser.error('Indicar --ip y --dominio')
        comando = preparar_camara(args.ip, args.dominio, args.contenedor, args.topico_camara)
        VideoRobot = cargar_transporte()
        proceso = subprocess.Popen(comando, stdin=subprocess.PIPE, stdout=subprocess.PIPE)
        video = VideoRobot(proceso)
        print(f'Galería local: {len(set(sistema.galeria.alumnos.tolist()))} personas. '
              f'Criterio: umbral={sistema.criterio.umbral:.4f}, margen={sistema.criterio.margen_minimo:.4f}.',
              flush=True)
        print('Robot quieto. Colocar UNA fotografía del dataset frente a su cámara. Esc o Ctrl+C para terminar.',
              flush=True)
        if not args.sin_ventana:
            cv2.namedWindow(ventana, cv2.WINDOW_NORMAL | cv2.WINDOW_GUI_NORMAL)
            ventana_creada = True
            cv2.resizeWindow(ventana, 960, 708)
            cv2.setMouseCallback(ventana, entrada.raton)
        ultimo_log = 0.0
        while True:
            imagen, estado = video.fotograma()
            # VideoRobot decodifica en este hilo: la marca corresponde a esta imagen,
            # no a un paquete más reciente que llegue mientras se ejecuta la inferencia.
            paquete = video._decodificado if imagen is not None else None
            marca = paquete['recibido_ns'] if paquete is not None else None
            resultado = consulta.actualizar(imagen, marca)
            if marca is not None and time.monotonic_ns() - marca > EDAD_MAXIMA_NS:
                imagen = None
                resultado = consulta.actualizar(None, None)
                estado = 'Sin imagen reciente'
            ahora = time.monotonic()
            if ahora - ultimo_log >= 1.0:
                objetivo = (f" | objetivo_confirmado={resultado['objetivo_confirmado']}"
                            if consulta.nombre is not None else '')
                if imagen is None:
                    print(f'[CAMARA] {estado} | reconocimiento en espera', flush=True)
                else:
                    print(f"[FACIAL] {resultado['nombre']} | {resultado['motivo']}"
                          f"{objetivo}", flush=True)
                ultimo_log = ahora
            if not args.sin_ventana:
                cv2.imshow(ventana, dibujar_interfaz(imagen, resultado, estado, entrada))
                if entrada.tecla(cv2.waitKeyEx(20)) or cv2.getWindowProperty(ventana, cv2.WND_PROP_VISIBLE) < 1:
                    break
            else:
                time.sleep(0.05)
            if proceso.poll() is not None:
                raise RuntimeError(f'El receptor de cámara terminó ({proceso.returncode})')
    except KeyboardInterrupt:
        return 0
    except (OSError, ValueError, TypeError, KeyError, RuntimeError, subprocess.CalledProcessError, cv2.error) as exc:
        print(f'Error de prueba facial: {exc}', file=sys.stderr)
        return 1
    finally:
        if video is not None:
            video.cerrar()
        if ventana_creada:
            cv2.destroyAllWindows()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
