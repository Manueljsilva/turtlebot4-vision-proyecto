"""Ensayo integrado con flechas, meta B, rostro/QR y regreso guiado por rojo.

No reproduce los pasos de ida ni usa mapa. Sin --mover no crea publicador.
"""

import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import uuid

import cv2

from busqueda_robot import EntradaBusqueda, PuenteBusqueda, preparar_movimiento
from control_busqueda import EDAD_CAMARA_NS, reciente
from control_mision import cargar_config_mision
from ensayo_giros_companero import CONFIG, CONTROL, cargar_ensayo
from evasion_lidar import cargar_config
from interfaz_facial import BUSCAR, dibujar_interfaz, escribir
from percepcion_meta_qr import DetectorMetaQR
from prueba_facial_robot import (TAREA, TOPICO, ConsultaEnVivo, cargar_sistema,
    cargar_transporte, ejecutar_comando, mensaje_rechazo, preparar_camara)
from prueba_meta_robot import EnsayoB
from senales_giro import ConsultaSenales, dibujar_detecciones


class SesionInterfaz:
    def __init__(self):
        self.nombre = self.inicio_ns = None

    def iniciar(self, nombre, ahora):
        if self.inicio_ns is not None or not isinstance(nombre, str) or not nombre.strip():
            raise ValueError('Nombre ausente o mision ya iniciada')
        self.nombre, self.inicio_ns = nombre, ahora


class PuenteMision(PuenteBusqueda):
    def actualizar(self, orden, actividad_ns, camara_ns, meta=None, senal=None, qr_ns=None, qr_votos=0):
        with self.cerrojo:
            self.datos = dict(orden=orden, actividad_ns=actividad_ns, camara_ns=camara_ns,
                              meta=meta, senal=senal, qr_ns=qr_ns, qr_votos=qr_votos)


def preparar_mision(comando_camara, contenedor, config, mover):
    cargar_ensayo()  # Verificar originales antes de copiarlos.
    comando = preparar_movimiento(comando_camara, contenedor, config, mover,
                                  CONFIG, 'ida', control_giros_origen=CONTROL)
    indice = next(i for i, v in enumerate(comando) if v.endswith('/busqueda_lidar_ros.py'))
    remoto = str(Path(comando[indice]).parent)
    for archivo in ('control_meta.py', 'control_mision.py', 'mision_ros.py'):
        ejecutar_comando(['sudo', '-n', 'docker', 'cp', str(TAREA/archivo), f'{contenedor}:{remoto}/{archivo}'])
    ejecutar_comando(['sudo', '-n', 'docker', 'cp', str(TAREA/'config/mision_guiada.json'),
                     f'{contenedor}:{remoto}/mision.json'])
    comando[indice] = f'{remoto}/mision_ros.py'
    comando.extend(['--config-mision', f'{remoto}/mision.json'])
    return comando


def guardar_estado(archivo, sesion, estado, palabra):
    datos = dict(version=1, ensayo='mision_guiada_azul_meta_rostro_qr_rojo',
        nombre_solicitado=sesion.nombre, identidad_confirmada=sesion.nombre if palabra is not None else None,
        palabra_qr=palabra, a_odometria=estado.get('a'), b_odometria=estado.get('b'),
        fase=estado['fase'], motivo=estado['estado'], regreso_a_confirmado=estado['fase'] == 'FIN',
        ruta_reproducida=False)
    archivo.parent.mkdir(parents=True, exist_ok=True)
    temporal = archivo.with_suffix('.tmp')
    temporal.write_text(json.dumps(datos, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    temporal.replace(archivo)


class VisionMision:
    def __init__(self, sistema, cfg):
        self.ensayo = EnsayoB(sistema)
        self.detector_meta = DetectorMetaQR()
        self.cfg = cfg
        self.tramo = self.consulta = None
        self.ultima = None
        self.meta = self.senal = self.resultado_senal = None
        self.deteccion_meta = None

    def actualizar(self, imagen, marca, ahora, estado, sesion, pausar):
        self.meta = self.senal = None
        if (imagen is None or not reciente(marca, ahora, EDAD_CAMARA_NS) or estado is None
                or sesion.inicio_ns is None or marca <= sesion.inicio_ns):
            self.resultado_senal = None
            self.deteccion_meta = None
            return imagen
        fase = estado['fase']
        if fase == 'BUSCAR_B':
            self.deteccion_meta = None
            b = self.ensayo.sesion
            if b.inicio_ns is None:
                b.iniciar(sesion.nombre, sesion.inicio_ns)
                b.meta_confirmada, b.orden = True, 'BARRER'
                b.habilitar_busqueda(estado['busqueda_desde_ns'])
            return self.ensayo.actualizar(imagen, marca, ahora, pausar)
        if fase not in ('IDA', 'VUELTA'):
            self.resultado_senal = None
            self.deteccion_meta = None
            return imagen
        tramo = 'ida' if fase == 'IDA' else 'vuelta'
        if tramo != self.tramo:
            self.tramo, self.consulta = tramo, ConsultaSenales(tramo, self.cfg)
            self.ultima = self.resultado_senal = self._meta_anterior = None
            self.deteccion_meta = None
        if marca <= estado['fase_desde_ns']:
            return imagen
        if self.ultima is None or marca > self.ultima:
            self.ultima = marca
            p = self.detector_meta.detectar(imagen, leer_qr=False, buscar_meta=True)
            self.deteccion_meta = p.get('meta')
            caja = p['meta']['caja'] if p['meta'] else None
            if caja is not None:
                x, y, w, h = caja
                alto, ancho = imagen.shape[:2]
                caja = [x/ancho, y/alto, w/ancho, h/alto]
                self.meta = caja if (min(caja) >= 0 and caja[0]+caja[2] <= 1 and caja[1]+caja[3] <= 1) else None
            self.resultado_senal = self.consulta.actualizar(imagen, marca, time.monotonic_ns())
            self._meta_anterior = self.meta
        else:
            self.meta = getattr(self, '_meta_anterior', None)
        ev = self.resultado_senal.get('evento') if self.resultado_senal else None
        self.senal = ev if ev and reciente(ev['marca_ns'], time.monotonic_ns(), EDAD_CAMARA_NS) else None
        anotada = dibujar_detecciones(imagen, self.resultado_senal, tramo) if self.resultado_senal else imagen.copy()
        if self.deteccion_meta is not None:
            x, y, w, h = [round(v) for v in self.deteccion_meta['caja']]
            cv2.rectangle(anotada, (x,y), (x+w,y+h), (80,220,220), 2)
            escribir(anotada, 'META VISTA', x, max(18,y-7), (80,220,220), limite=30)
        return anotada


def dibujar(imagen, estado_camara, entrada, vision, estado, mover):
    if imagen is not None and estado and estado['fase'] == 'BUSCAR_B':
        imagen = imagen.copy()
        for c in vision.ensayo.caras:
            x, y, w, h = (round(v) for v in c['caja'])
            color = (100, 225, 100) if c['aceptado'] and c['nombre'] == entrada.sesion.nombre else (70, 190, 240)
            cv2.rectangle(imagen, (x,y), (x+w,y+h), color, 2)
            escribir(imagen, c['nombre'], x, max(18,y-7), color, limite=45)
        for i, q in enumerate(vision.ensayo.percepcion['codigos']):
            asociado = vision.ensayo.qr_asociado
            etiqueta = 'QR OBJETIVO' if asociado and asociado['qr'] == i else 'QR SIN CONFIRMAR'
            x, y, w, h = (round(v) for v in q['caja'])
            cv2.rectangle(imagen, (x,y), (x+w,y+h), (200,180,60), 2)
            escribir(imagen, etiqueta, x, max(18,y-7), (200,180,60), limite=35)
    panel = dibujar_interfaz(imagen, mensaje_rechazo('Rostros solo tras meta B'), estado_camara, entrada)
    if entrada.sesion.inicio_ns is not None:
        cv2.rectangle(panel, (0, 99), (959, 245), (27, 27, 27), -1)
        cv2.rectangle(panel, BUSCAR[:2], BUSCAR[2:], (65, 65, 65), -1)
        escribir(panel, 'En curso', 775, 76)
        fase = estado['fase'] if estado else 'SIN TELEMETRIA: DETENIDO'
        escribir(panel, f'FASE: {fase}', 18, 121, (70, 190, 240))
        escribir(panel, estado['estado'] if estado else 'Esperando nodo ROS', 18, 145)
        senal = (vision.ensayo.sesion.motivo if fase == 'BUSCAR_B' else
                 vision.resultado_senal['estado'] if vision.resultado_senal else 'Flechas solo durante ida/vuelta')
        escribir(panel, senal, 18, 169)
        observados = ', '.join(c['nombre'] for c in vision.ensayo.caras) if fase == 'BUSCAR_B' else 'en espera'
        if fase in ('IDA', 'VUELTA'):
            escribir(panel, 'META: '+estado.get('motivo_meta','Esperando diagnostico'), 18, 193, (80,220,220))
        else:
            escribir(panel, f'Objetivo: {entrada.sesion.nombre} | Observados: {observados or "sin caras"}', 18, 193)
        if estado and fase == 'FIN':
            escribir(panel, 'MENSAJE DEL QR: '+str(vision.ensayo.sesion.palabra), 18, 219, (100, 225, 100))
        else:
            escribir(panel, 'QR guardado para mostrar en A' if vision.ensayo.sesion.palabra is not None else
                     vision.ensayo.resumen_qr() if fase == 'BUSCAR_B' else
                     'Rostros y QR habilitados solo tras meta B y reposo', 18, 219)
        edad = estado.get('edad_camara_s') if estado else None
        dato = f'{edad:.2f}s desde recepcion ROS' if isinstance(edad, (int,float)) else 'sin marca reciente'
        escribir(panel, f'Camara: {estado_camara} | {dato}', 18, 240)
    cv2.rectangle(panel, (0, 669), (840, 707), (27, 27, 27), -1)
    escribir(panel, ('MOVIMIENTO ACTIVADO' if mover else 'SIN MOVIMIENTO')+' | Ensayo A-B-A | Esc: salir', 18, 693)
    return panel


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--ip', required=True)
    p.add_argument('--dominio', type=int, required=True)
    p.add_argument('--contenedor', default='proyecto2-sim')
    p.add_argument('--config', type=Path, default=TAREA/'config/evasion_lidar.json')
    p.add_argument('--nombre')
    p.add_argument('--topico-camara', default=TOPICO)
    p.add_argument('--mover', action='store_true')
    args = p.parse_args(argv)
    video = puente = None
    abierta = False
    ventana = 'Task 1 - ensayo guiado A-B-A'
    try:
        if not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
            raise ValueError('Abrir en una terminal de Ubuntu con interfaz grafica')
        cargar_config(args.config)
        cargar_config_mision(TAREA/'config/mision_guiada.json')
        _, cfg_s = cargar_ensayo()
        sistema = cargar_sistema()
        sesion = SesionInterfaz()
        entrada = EntradaBusqueda(ConsultaEnVivo(sistema, args.nombre), sesion)
        entrada.aviso = 'Ensayo integrado: bandera B al fondo; Buscar fija nombre y registra A.'
        vision = VisionMision(sistema, cfg_s)
        cv2.setNumThreads(1)
        comando = preparar_camara(args.ip, args.dominio, args.contenedor, args.topico_camara)
        control = preparar_mision(comando, args.contenedor, args.config, args.mover)
        video = cargar_transporte()(subprocess.Popen(comando, stdin=subprocess.PIPE, stdout=subprocess.PIPE))
        puente = PuenteMision(control)
        cv2.namedWindow(ventana, cv2.WINDOW_NORMAL | cv2.WINDOW_GUI_NORMAL)
        abierta = True
        cv2.resizeWindow(ventana, 960, 708)
        cv2.setMouseCallback(ventana, entrada.raton)
        archivo = TAREA/'datos/sesiones'/f'mision_guiada_{uuid.uuid4().hex}.json'
        guardado = None
        ultimo_log = 0
        print('ENSAYO INTEGRADO: azul > meta B > rostro/QR > orientacion entrada+180 > rojo > meta A. '
              'No reproduce ruta ni usa mapa. Sin --mover no publica. Cerrar otros controles.', flush=True)
        while True:
            puente.comprobar()
            if video.proceso.poll() is not None:
                raise RuntimeError('Termino el receptor de camara')
            imagen, estado_camara = video.fotograma()
            marca = video._decodificado['recibido_ns'] if imagen is not None else None
            ahora = time.monotonic_ns()
            estado = puente.ultimo_estado(ahora)
            imagen = vision.actualizar(imagen, marca, ahora, estado, sesion,
                lambda: puente.actualizar('VERIFICAR', time.monotonic_ns(), marca))
            ahora = time.monotonic_ns()
            if not reciente(marca, ahora, EDAD_CAMARA_NS):
                imagen, marca = None, None
                if estado_camara == 'Camara del robot':
                    estado_camara = 'Sin imagen reciente'
            b = vision.ensayo.sesion
            orden = 'ESPERAR' if sesion.inicio_ns is None or estado is None else (
                'INICIAR' if estado['fase'] == 'ESPERAR' else 'OBSERVAR')
            if estado and estado['fase'] == 'BUSCAR_B':
                orden = 'QR_LISTO' if b.palabra is not None else ('VERIFICAR' if b.orden == 'VERIFICAR' else 'OBSERVAR')
            if estado and estado.get('a') and sesion.inicio_ns is not None:
                clave = (estado['fase'], b.palabra)
                if clave != guardado:
                    guardar_estado(archivo, sesion, estado, b.palabra)  # Persistir antes de autorizar retorno.
                    guardado = clave
                    if estado['fase'] == 'FIN':
                        print(f'[RESULTADO EN A] {b.palabra}', flush=True)
            puente.actualizar(orden, ahora, marca, vision.meta, vision.senal,
                b.ultima_marca if b.palabra is not None else None, 3 if b.palabra is not None else 0)
            if ahora-ultimo_log >= 1_000_000_000:
                print(f'[MISION] {estado or "Esperando control"} | [QR] {vision.ensayo.resumen_qr()}', flush=True)
                if estado:
                    print(f'[META] {estado.get("motivo_meta", "Esperando diagnostico")}', flush=True)
                paquete = getattr(video, '_decodificado', None)
                edad = (ahora-paquete['recibido_ns'])/1e9 if paquete else None
                detalle = f'ultima imagen decodificada: {edad:.2f}s desde recepcion ROS' if edad is not None else 'sin imagen decodificada'
                print(f'[CAMARA] {estado_camara} | {detalle} | Buscar confirmado: {sesion.inicio_ns is not None}', flush=True)
                ultimo_log = ahora
            cv2.imshow(ventana, dibujar(imagen, estado_camara, entrada, vision, estado, args.mover))
            if entrada.tecla(cv2.waitKeyEx(20)) or cv2.getWindowProperty(ventana, cv2.WND_PROP_VISIBLE) < 1:
                break
    except KeyboardInterrupt:
        return 0
    except (OSError, ValueError, TypeError, KeyError, RuntimeError, subprocess.CalledProcessError, cv2.error) as exc:
        print(f'Error de ensayo integrado: {exc}', file=sys.stderr)
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
