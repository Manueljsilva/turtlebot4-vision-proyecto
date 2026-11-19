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

from control_busqueda import EDAD_CAMARA_NS, SesionBusqueda, reciente
from control_giros import cargar_config_senales
from evasion_lidar import cargar_config
from interfaz_facial import BUSCAR, SALIR, EntradaObjetivo, dibujar_interfaz, dentro, escribir
from meta_bandera import ConsultaMetaBandera, decodificar_qr_asociado
from mision_meta import MisionMeta
from prueba_facial_robot import (TAREA, TOPICO, ConsultaEnVivo, cargar_sistema, cargar_transporte,
                                ejecutar_comando, interpretar_topicos, preparar_camara)
from senales_giro import ConsultaSenales, dibujar_detecciones


def preparar_movimiento(comando_camara, contenedor, config, mover, config_senales=None, tramo='ida'):
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
    for archivo in ('evasion_lidar.py', 'seguridad_lidar.py', 'control_giros.py',
                    'control_busqueda.py', 'busqueda_lidar_ros.py'):
        ejecutar_comando(['sudo', '-n', 'docker', 'cp', str(TAREA / archivo), f'{contenedor}:{remoto}/{archivo}'])
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
        if senales:
            self.datos['senal'] = None
        self.estado = None
        self.fallo = None
        self.emisor = threading.Thread(target=self.enviar, daemon=True)
        self.lector = threading.Thread(target=self.recibir, daemon=True)
        self.emisor.start()
        self.lector.start()

    def actualizar(self, orden, actividad_ns, camara_ns, senal=None, tramo=None):
        with self.cerrojo:
            self.datos = dict(orden=orden, actividad_ns=actividad_ns, camara_ns=camara_ns)
            if self.senales:
                self.datos['senal'] = senal
            if tramo is not None:
                self.datos['tramo'] = tramo

    def enviar(self):
        secuencia = 0
        try:
            while not self.fin.is_set():
                with self.cerrojo:
                    datos = dict(self.datos)
                self.proceso.stdin.write(json.dumps(dict(version=1, secuencia=secuencia, **datos)) + '\n')
                self.proceso.stdin.flush()
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
                     senales=None, tramo='ida', mision=None, meta_res=None):
    panel = dibujar_interfaz(imagen, resultado, estado_camara, entrada)
    if entrada.sesion.inicio_ns is not None:
        cv2.rectangle(panel, (18, 101), (943, 178), (27, 27, 27), -1)
        estado = estado_ros['estado'] if estado_ros else 'Esperando respuesta del control: detenido'
        if mision is not None:
            if mision.etapa == 'FINALIZADO':
                ganador = (mision.mejor_resultado.get('candidato') or mision.mejor_resultado.get('nombre')
                           if mision.mejor_resultado else 'SIN_ROSTRO')
                qr_txt = mision.palabra_qr or 'sin QR'
                af_txt = f'{mision.mejor_afinidad:.2f}' if mision.mejor_afinidad is not None else '-'
                estado = f'MISION COMPLETADA en A | Ganador: {ganador} (sim={af_txt}) | QR: {qr_txt}'
            else:
                estado = f'Etapa {mision.etapa} ({mision.tramo.upper()}) | {estado}'
        elif entrada.sesion.encontrado:
            estado = 'Persona encontrada. Robot detenido. Regreso a A pendiente.'
        color_estado = (100, 225, 100) if (
            entrada.sesion.encontrado or (mision is not None and mision.etapa == 'FINALIZADO')
        ) else (70, 190, 240)
        escribir(panel, estado, 18, 123, color_estado)
        a = estado_ros.get('a') if estado_ros else None
        texto_a = (f"A guardado: x={a['x']:.2f} m, y={a['y']:.2f} m ({a['frame_id']})"
                   if a else 'A pendiente: esperando odometria valida; robot detenido')
        escribir(panel, texto_a, 18, 148)
        if mision is not None:
            af = f'{mision.mejor_afinidad:.3f}' if mision.mejor_afinidad is not None else '-'
            qr_info = mision.palabra_qr or '-'
            meta_txt = f"Meta:{meta_res['cuenta']}/3" if meta_res else 'Meta:0/3'
            escribir(panel, f'Vistas B:{mision.vistas_distintas}/4 | Mejor afinidad:{af} | QR:{qr_info} | {meta_txt}', 18, 172)
        else:
            escribir(panel, f'Confirmacion del rostro: {entrada.sesion.confirmaciones}/3 imagenes nuevas', 18, 172)
        cv2.rectangle(panel, BUSCAR[:2], BUSCAR[2:], (65, 65, 65), -1)
        escribir(panel, 'En curso', 775, 76)
    cv2.rectangle(panel, (0, 669), (840, 707), (27, 27, 27), -1)
    pie = 'MOVIMIENTO ACTIVADO' if mover else 'SIN MOVIMIENTO'
    pie += (f" | {tramo.upper()} | {senales['estado']}" if senales is not None
            else ' | Evasion + reconocimiento | Esc: detener y salir')
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
    parser.add_argument('--mision', action='store_true', help='Misión completa: ida azul -> meta B -> barrido 4 fotos -> vuelta roja -> meta A')
    parser.add_argument('--fps', type=float, default=None, help='FPS de cámara hacia el host (por defecto 5; 12 en --mision)')
    args = parser.parse_args(argv)
    if args.mision:
        args.senales = True
        if args.fps is None:
            args.fps = 12.0
    video, puente = None, None
    ventana = 'Task 1 - evasion y busqueda de persona'
    abierta = False
    try:
        if not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
            raise ValueError('Esta prueba requiere una terminal de Ubuntu con interfaz gráfica')
        cargar_config(args.config)
        cfg_senales = cargar_config_senales(args.config_senales) if args.senales else None
        consulta_senales = ConsultaSenales(args.tramo, cfg_senales) if args.senales else None
        mision = MisionMeta() if args.mision else None
        consulta_meta = ConsultaMetaBandera() if args.mision else None
        tramo_activo = args.tramo
        inicio_senales = None
        cv2.setNumThreads(1)
        consulta = ConsultaEnVivo(cargar_sistema(), args.nombre)
        sesion = SesionBusqueda()
        entrada = EntradaBusqueda(consulta, sesion)
        comando_camara = preparar_camara(args.ip, args.dominio, args.contenedor, args.topico_camara, fps=args.fps)
        comando_control = preparar_movimiento(comando_camara, args.contenedor, args.config, args.mover,
            args.config_senales if args.senales else None, args.tramo)
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
        print('Misión autónoma lista. Buscar fija el nombre objetivo y registra A antes de moverse.',
              flush=True)
        ultimo_log = 0
        etapa_previa = None
        while True:
            puente.comprobar()
            if video.proceso.poll() is not None:
                raise RuntimeError('Terminó el receptor de cámara')
            imagen, estado_camara = video.fotograma()
            marca = video._decodificado['recibido_ns'] if imagen is not None else None
            ahora = time.monotonic_ns()
            if not reciente(marca, ahora, EDAD_CAMARA_NS):
                imagen_valida, marca_valida = None, None
            else:
                imagen_valida, marca_valida = imagen, marca

            estado_ros = puente.ultimo_estado(ahora)
            a = estado_ros.get('a') if estado_ros else None
            pose_actual = estado_ros.get('pose') if estado_ros else None
            if a and not guardado:
                archivo_a.write_text(json.dumps(dict(version=1, punto_a=a,
                    alcance='odometria de esta sesion'), indent=2)+'\n')
                guardado = True
                print(f'Punto A registrado en {archivo_a}', flush=True)

            resultado_senales = None
            meta_res = None

            if mision is not None:
                if sesion.inicio_ns is not None and mision.etapa == 'ESPERANDO':
                    mision.iniciar(pose_actual or a)
                    consulta_senales = ConsultaSenales('ida', cfg_senales)
                    consulta_meta.reiniciar()
                    tramo_activo = 'ida'

                if mision.etapa in ('IDA', 'VUELTA'):
                    if mision.tramo != tramo_activo:
                        tramo_activo = mision.tramo
                        consulta_senales = ConsultaSenales(tramo_activo, cfg_senales)
                        consulta_meta.reiniciar()
                    posterior = sesion.inicio_ns is not None and marca_valida is not None and marca_valida > sesion.inicio_ns
                    resultado_senales = consulta_senales.actualizar(
                        imagen_valida if posterior else None, marca_valida if posterior else None, ahora)
                    meta_res = consulta_meta.actualizar(
                        imagen_valida if posterior else None, marca_valida if posterior else None, ahora)
                    resultado = consulta.actualizar(None, None, ahora)
                    qr_texto = None
                elif mision.etapa == 'BARRIDO_B':
                    resultado = consulta.actualizar(imagen_valida, marca_valida, ahora)
                    qr_texto = None
                    if (imagen_valida is not None and resultado.get('caja') is not None
                            and resultado.get('afinidad_objetivo') is not None):
                        qr_texto = decodificar_qr_asociado(imagen_valida, resultado['caja'])
                else:
                    resultado = consulta.actualizar(None, None, ahora)
                    qr_texto = None

                orden_mision = mision.actualizar(pose_actual, meta_res, resultado, qr_texto=qr_texto)
                if mision.etapa != etapa_previa:
                    print(f'[MISION] Cambio a etapa: {mision.etapa} (tramo={mision.tramo})', flush=True)
                    if mision.etapa == 'GIRO_RETORNO_B':
                        af_txt = f'{mision.mejor_afinidad:.3f}' if mision.mejor_afinidad is not None else 'N/A'
                        cand = (mision.mejor_resultado.get('candidato') or mision.mejor_resultado.get('nombre')
                                if mision.mejor_resultado else 'NINGUNO')
                        print(f'[META B] Ganador más afín a {consulta.nombre}: {cand} (afinidad={af_txt}) | QR={mision.palabra_qr}', flush=True)
                    elif mision.etapa == 'FINALIZADO':
                        print(f'[EXITO EN A] Objetivo: {consulta.nombre} | QR: {mision.palabra_qr} | Mejor afinidad: {mision.mejor_afinidad}', flush=True)
                    etapa_previa = mision.etapa

                puente.actualizar(
                    orden_mision, ahora, marca_valida,
                    resultado_senales['evento'] if resultado_senales else None,
                    tramo=mision.tramo,
                )
            else:
                resultado = consulta.actualizar(imagen_valida, marca_valida, ahora)
                if consulta_senales is not None:
                    if sesion.inicio_ns is not None and inicio_senales != sesion.inicio_ns:
                        consulta_senales = ConsultaSenales(args.tramo, cfg_senales)
                        inicio_senales = sesion.inicio_ns
                    posterior = sesion.inicio_ns is None or (marca_valida is not None and marca_valida > sesion.inicio_ns)
                    resultado_senales = consulta_senales.actualizar(
                        imagen_valida if posterior else None, marca_valida if posterior else None, ahora)
                sesion.actualizar(resultado, marca_valida, ahora, bool(a))
                # La GUI/inferencia debe renovar su actividad; el hilo de latido no la inventa.
                puente.actualizar(sesion.orden, ahora, marca_valida,
                                  resultado_senales['evento'] if resultado_senales else None)

            if ahora - ultimo_log >= 1_000_000_000:
                etiqueta = mision.etapa if mision is not None else sesion.orden
                print(f"[BUSQUEDA] {etiqueta} | {resultado['nombre']} | "
                      f"{estado_ros['estado'] if estado_ros else 'Esperando nodo ROS'}", flush=True)
                ultimo_log = ahora
            anotada = dibujar_detecciones(imagen, resultado_senales, tramo_activo) if (
                imagen is not None and resultado_senales is not None) else imagen
            if anotada is not None and meta_res and meta_res.get('caja') is not None:
                anotada = anotada.copy()
                mx, my, mw, mh = meta_res['caja']
                cv2.rectangle(anotada, (mx, my), (mx + mw, my + mh), (0, 255, 0), 2)
                cv2.putText(anotada, f"META {meta_res['cuenta']}/3", (mx, max(20, my - 8)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
            cv2.imshow(ventana, dibujar_busqueda(anotada, resultado, estado_camara, entrada,
                estado_ros, args.mover, resultado_senales, tramo_activo, mision=mision, meta_res=meta_res))
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
