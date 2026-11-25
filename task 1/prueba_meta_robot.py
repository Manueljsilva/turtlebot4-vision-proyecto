"""Meta, frenado, giro, persona y QR; con avance opcional hasta ver la bandera.

--avanzar-hasta-meta añade evasión previa. Sin --mover no publica; no hay retorno.
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
from control_busqueda import CONFIRMACIONES_VISUALES, EDAD_CAMARA_NS, reciente
from control_meta import SesionPruebaB, misma_caja
from evasion_lidar import cargar_config
from interfaz_facial import BUSCAR, dibujar_interfaz, escribir
from percepcion_meta_qr import (DetectorMetaQR, analizar_candidatos, asociar_qr,
                                detectar_regiones_fotos, texto_valido)
from prueba_facial_robot import (TAREA, TOPICO, ConsultaEnVivo, cargar_sistema,
    cargar_transporte, ejecutar_comando, mensaje_rechazo, preparar_camara)


def preparar_barrido(comando_camara, contenedor, config, mover, avanzar=False):
    comando = preparar_movimiento(comando_camara, contenedor, config, mover)
    indice = next(i for i, v in enumerate(comando) if v.endswith('/busqueda_lidar_ros.py'))
    remoto = str(Path(comando[indice]).parent)
    for archivo in ('control_meta.py', 'barrido_ros.py'):
        ejecutar_comando(['sudo', '-n', 'docker', 'cp', str(TAREA/archivo), f'{contenedor}:{remoto}/{archivo}'])
    comando[indice] = f'{remoto}/barrido_ros.py'
    if avanzar:
        comando.append('--avanzar-hasta-meta')
    return comando


class EntradaMeta(EntradaBusqueda):
    def __init__(self, consulta, sesion):
        super().__init__(consulta, sesion)
        self.aviso = ('Buscar fija el nombre e inicia evasion hasta la meta.' if sesion.avanzar else
                      'Ensayo cerca de B. Buscar fija el nombre; espera la meta antes de girar.')


class EnsayoB:
    def __init__(self, sistema, avanzar=False):
        self.sistema = sistema
        self.sesion = SesionPruebaB(avanzar)
        self.detector = DetectorMetaQR()
        self.ultima_marca = None
        self.percepcion = dict(meta=None, codigos=[])
        self.caras = []
        self.fallo_movimiento = None
        self.estado_qr = 'Esperando meta, reposo y persona objetivo'
        self.qr_asociado = None
        self.tamano_imagen = None
        self.inspeccionando = False
        self._pausa_inicio_ns = None
        self._pausa_bloqueada_hasta_ns = None
        self._caja_inspeccionada = None

    def _rostro_centrado(self, cara, margen=0.22):
        if not self.tamano_imagen or self.tamano_imagen[0] <= 0:
            return True
        x, _, w, _ = cara['caja']
        centro_x = (x + w * 0.5) / self.tamano_imagen[0]
        return abs(centro_x - 0.5) <= margen

    def _actualizar_inspeccion(self, ahora, objetivos, pausar):
        if objetivos:
            self.inspeccionando = False
            self._pausa_inicio_ns = None
            self._caja_inspeccionada = None
            if pausar:
                pausar()  # Parar antes de la decodificación QR, que puede tardar.
            return
        # Evitar gastar pausa al entrar por un borde o repetir sobre la misma foto lado a lado.
        margen = 0.28 if self._pausa_inicio_ns is not None else 0.22
        pendientes = [c for c in self.caras if not c['aceptado'] and self._rostro_centrado(c, margen)]
        if self._caja_inspeccionada is not None and self._pausa_inicio_ns is None:
            for c in self.caras:
                if not c['aceptado'] and misma_caja(c['caja'], self._caja_inspeccionada):
                    self._caja_inspeccionada = list(c['caja'])
                    break
            pendientes = [c for c in pendientes if not misma_caja(c['caja'], self._caja_inspeccionada)]
        if not pendientes:
            self.inspeccionando = False
            self._pausa_inicio_ns = None
            return
        if self._pausa_bloqueada_hasta_ns is not None and ahora < self._pausa_bloqueada_hasta_ns:
            self.inspeccionando = False
            self._pausa_inicio_ns = None
            return
        if self._pausa_inicio_ns is None:
            self._pausa_inicio_ns = ahora
        if ahora - self._pausa_inicio_ns < 900_000_000:
            self.inspeccionando = True
            if pausar:
                pausar()
        else:
            self.inspeccionando = False
            self._pausa_inicio_ns = None
            self._caja_inspeccionada = list(pendientes[0]['caja'])
            self._pausa_bloqueada_hasta_ns = ahora + 400_000_000

    def recibir_control(self, estado, ahora):
        if not estado or not reciente(estado.get('recibido_ns'), ahora, EDAD_CAMARA_NS):
            return
        if estado.get('terminado') and self.fallo_movimiento is None and self.sesion.orden != 'QR_LISTO':
            self.fallo_movimiento = estado['estado']
        marca = estado.get('busqueda_desde_ns')
        if type(marca) is int and 0 <= marca <= ahora:
            self.sesion.habilitar_busqueda(marca)

    def actualizar(self, imagen, marca, ahora, pausar=None):
        s = self.sesion
        if not reciente(marca, ahora, EDAD_CAMARA_NS) or imagen is None:
            self.tamano_imagen = None
            self.caras, self.percepcion = [], dict(meta=None, codigos=[])
            self.qr_asociado = None
            self.inspeccionando = False
            self._pausa_inicio_ns = None
            if s.palabra is None:
                self.estado_qr = 'Esperando imagen reciente para leer QR'
            s.actualizar(self.percepcion, [], None, '', None, ahora)
            return None
        self.tamano_imagen = (imagen.shape[1], imagen.shape[0])
        if (s.inicio_ns is None or marca <= s.inicio_ns or marca == self.ultima_marca
                or (self.ultima_marca is not None and marca < self.ultima_marca)
                or s.orden in ('PARAR', 'QR_LISTO')):
            if s.orden == 'QR_LISTO' and marca != self.ultima_marca:
                self.caras, self.percepcion, self.qr_asociado = [], dict(meta=None, codigos=[]), None
            if s.orden in ('PARAR', 'QR_LISTO'):
                self.inspeccionando = False
                self._pausa_inicio_ns = None
            return imagen
        self.ultima_marca = marca
        if s.meta_confirmada and (s.busqueda_desde_ns is None or marca <= s.busqueda_desde_ns):
            self.caras, self.percepcion = [], dict(meta=None, codigos=[])
            self.inspeccionando = False
            self._pausa_inicio_ns = None
            s.motivo = ('Meta confirmada; esperando reposo para habilitar rostros' if s.busqueda_desde_ns is None
                        else 'Esperando imagen nueva posterior al frenado')
            self.estado_qr, self.qr_asociado = 'Esperando reposo e imagen posterior', None
            return imagen
        self.caras = analizar_candidatos(imagen, self.sistema) if s.meta_confirmada else []
        objetivos = [c for c in self.caras if c['aceptado'] and c['nombre'] == s.nombre]
        if s.meta_confirmada:
            self._actualizar_inspeccion(ahora, objetivos, pausar)
        if not s.meta_confirmada:
            self.percepcion = self.detector.detectar(imagen, leer_qr=False, buscar_meta=True)
        elif len(objetivos) == 1:
            self.percepcion = self.detector.detectar(imagen, leer_qr=True, buscar_meta=False,
                                                    caja_objetivo=objetivos[0]['caja'])
        else:
            # Buscar QR sin el objetivo ralentiza la búsqueda y no aporta asociación.
            self.percepcion = dict(meta=None, codigos=[])
        regiones = detectar_regiones_fotos(imagen) if len(objetivos) == 1 and self.percepcion['codigos'] else []
        asociacion, motivo = asociar_qr(self.caras, self.percepcion['codigos'], s.nombre, regiones)
        self.qr_asociado = asociacion
        self.estado_qr = motivo if s.meta_confirmada else 'Esperando meta confirmada y reposo'
        terminado = time.monotonic_ns()
        # Una inferencia lenta no convierte un paquete antiguo en una imagen reciente.
        s.actualizar(self.percepcion, self.caras, asociacion, motivo, marca, terminado)
        if not reciente(marca, terminado, EDAD_CAMARA_NS):
            self.tamano_imagen = None
            self.caras, self.percepcion = [], dict(meta=None, codigos=[])
            self.qr_asociado = None
            self.estado_qr = 'Imagen caducada durante lectura; esperando fotograma reciente'
            return None
        if s.palabra is not None:
            self.estado_qr = 'QR del objetivo confirmado y guardado'
        return imagen

    def reintentar_qr(self):
        self.sesion.reiniciar_lectura_qr()
        self.caras, self.percepcion = [], dict(meta=None, codigos=[])
        self.qr_asociado = None
        self.inspeccionando = False
        self._pausa_inicio_ns = None
        self._pausa_bloqueada_hasta_ns = None
        self._caja_inspeccionada = None
        self.estado_qr = 'Confirmacion QR caducada antes de acuse; esperando imagen nueva'

    def resumen_qr(self):
        if self.sesion.palabra is not None:
            return f'QR del objetivo confirmado y guardado: {self.sesion.palabra} | confirmacion={CONFIRMACIONES_VISUALES}/{CONFIRMACIONES_VISUALES}'
        codigos = self.percepcion['codigos']
        legibles = sum(texto_valido(q['texto']) for q in codigos)
        return (f'{self.estado_qr} | detectados={len(codigos)}, legibles={legibles} | '
                f'confirmacion={self.sesion.confirmaciones}/{CONFIRMACIONES_VISUALES} | {self.detalle_imagen_qr()}')

    def detalle_imagen_qr(self):
        imagen = ('x'.join(str(v) for v in self.tamano_imagen)+'px' if self.tamano_imagen else
                  'sin imagen reciente')
        # Cajas ya expresadas en la imagen original, no en el recorte ampliado.
        tamanos = ','.join(f'{round(q["caja"][2])}x{round(q["caja"][3])}'
                          for q in self.percepcion['codigos'][:3]) or 'ninguno'
        return f'imagen={imagen} | QR_px={tamanos}'


def guardar_resultado(sesion, carpeta):
    if sesion.orden != 'QR_LISTO' or sesion.palabra is None:
        raise ValueError('No hay QR confirmado para guardar')
    carpeta.mkdir(parents=True, exist_ok=True)
    archivo = carpeta/f'prueba_b_{uuid.uuid4().hex}.json'
    with archivo.open('x', encoding='utf-8') as f:
        json.dump(dict(version=1, prueba='meta_b_rostro_qr_sin_retorno',
            nombre_solicitado=sesion.nombre, identidad_confirmada=sesion.nombre,
            palabra_qr=sesion.palabra, regreso_a_confirmado=False), f, ensure_ascii=False, indent=2)
        f.write('\n')
    return archivo


def resumen_odometria(estado):
    if not estado:
        return 'sin telemetria'
    campos = []
    for clave, etiqueta in (('edad_odom_s', 'edad'), ('max_intervalo_odom_s', 'rx_max'),
                            ('ultimo_corte_odom_s', 'corte')):
        valor = estado.get(clave)
        if isinstance(valor, (int, float)):
            campos.append(f'{etiqueta}={valor:.2f}s')
    return ', '.join(campos) or 'sin muestras'


def dibujar_prueba(imagen, estado_camara, entrada, ensayo, estado_ros, mover):
    s = ensayo.sesion
    anotada = imagen.copy() if imagen is not None else None
    if anotada is not None:
        elementos = [(c['caja'], c['nombre'], (100, 225, 100) if c['aceptado'] else (70, 190, 240))
                     for c in ensayo.caras]
        for i, q in enumerate(ensayo.percepcion['codigos']):
            objetivo = ensayo.qr_asociado is not None and ensayo.qr_asociado['qr'] == i
            etiqueta = 'QR OBJETIVO' if objetivo else ('QR LEGIBLE' if texto_valido(q['texto']) else 'QR SIN LECTURA')
            color = (100, 225, 100) if objetivo else ((220, 170, 80) if texto_valido(q['texto']) else (70, 190, 240))
            elementos.append((q['caja'], etiqueta, color))
        if ensayo.qr_asociado and 'foto' in ensayo.qr_asociado:
            elementos.append((ensayo.qr_asociado['foto'], 'FOTO ASOCIADA', (200, 140, 180)))
        if ensayo.percepcion['meta']:
            elementos.append((ensayo.percepcion['meta']['caja'], 'META', (200, 180, 60)))
        for caja, etiqueta, color in elementos:
            x, y, w, h = [round(v) for v in caja]
            cv2.rectangle(anotada, (x, y), (x+w, y+h), color, 2)
            escribir(anotada, etiqueta, x, max(18, y-7), color, limite=50)
    panel = dibujar_interfaz(anotada, mensaje_rechazo('Ensayo B'), estado_camara, entrada)
    if s.inicio_ns is not None:
        cv2.rectangle(panel, (0, 99), (959, 245), (27, 27, 27), -1)
        cv2.rectangle(panel, BUSCAR[:2], BUSCAR[2:], (65, 65, 65), -1)
        escribir(panel, 'En curso', 775, 76)
        escribir(panel, s.motivo, 18, 121, (70, 190, 240), limite=108)
        escribir(panel, estado_ros['estado'] if estado_ros else 'Esperando control: detenido', 18, 145)
        angulo = estado_ros.get('barrido_deg', 0.) if estado_ros else 0.
        fase = 'Evasion hasta meta' if not s.meta_confirmada and s.avanzar else 'Busqueda sobre el sitio'
        if s.meta_confirmada and s.busqueda_desde_ns is None:
            fase = 'Esperando reposo confirmado; rostros aun en espera'
        elif ensayo.fallo_movimiento:
            fase = ('Movimiento detenido; reconocimiento en imagenes nuevas sigue activo' if s.meta_confirmada
                    else 'Movimiento bloqueado; esperando meta para reconocimiento')
        escribir(panel, f'{fase} | Giro: {angulo:.0f}/360 grados', 18, 169)
        observados = ', '.join(c['nombre'] for c in ensayo.caras) if imagen is not None else 'EN ESPERA'
        escribir(panel, f'Objetivo: {s.nombre} | Observados: {observados or "sin caras"}', 18, 193)
        if s.palabra is not None:
            escribir(panel, 'QR GUARDADO (vista de prueba en B): '+s.palabra, 18, 217, (100, 225, 100))
            escribir(panel, 'Texto UTF-8 completo en la terminal y JSON local. Regreso aun pendiente.', 18, 239)
        else:
            codigos = ensayo.percepcion['codigos']
            legibles = sum(texto_valido(q['texto']) for q in codigos)
            escribir(panel, f'QR: {len(codigos)} vistos / {legibles} legibles | {s.confirmaciones}/{CONFIRMACIONES_VISUALES} | '+
                     ensayo.detalle_imagen_qr(), 18, 217, limite=112)
            escribir(panel, ensayo.estado_qr, 18, 239, (70, 190, 240), limite=105)
    cv2.rectangle(panel, (0, 669), (840, 707), (27, 27, 27), -1)
    escribir(panel, (('AVANCE + GIRO ACTIVADOS' if s.avanzar else 'GIRO ACTIVADO') if mover else 'SIN MOVIMIENTO')+
             ' | Meta > rostro > QR | Esc: salir', 18, 693, limite=98)
    return panel


def _preparar_frame_preview(frame, lado=250):
    alto, ancho = frame.shape[:2]
    menor = min(alto, ancho)
    y0, x0 = (alto - menor) // 2, (ancho - menor) // 2
    recorte = frame[y0:y0 + menor, x0:x0 + menor]
    if lado > 0 and menor != lado:
        recorte = cv2.resize(recorte, (lado, lado), interpolation=cv2.INTER_AREA)
    return recorte


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ip')
    parser.add_argument('--dominio', type=int)
    parser.add_argument('--contenedor', default='proyecto2-sim')
    parser.add_argument('--config', type=Path, default=TAREA/'config/evasion_lidar.json')
    parser.add_argument('--topico-camara', default=TOPICO)
    parser.add_argument('--nombre')
    parser.add_argument('--mover', action='store_true')
    parser.add_argument('--avanzar-hasta-meta', action='store_true', help='Evasion previa a la bandera; despues solo giro')
    parser.add_argument('--webcam', action='store_true', help='Prueba local con webcam sin robot ni ROS')
    parser.add_argument('--camara-local', type=int, default=0)
    parser.add_argument('--lado-preview', type=int, default=250)
    parser.add_argument('--sin-meta', action='store_true', help='Con --webcam, habilita de frente rostro y QR')
    args = parser.parse_args(argv)
    if args.webcam:
        if args.mover or args.avanzar_hasta_meta or args.ip is not None or args.dominio is not None:
            parser.error('--webcam es solo prueba local sin robot, --ip, --dominio ni --mover')
    else:
        if args.sin_meta:
            parser.error('--sin-meta solo aplica junto a --webcam')
        if not args.ip or args.dominio is None:
            parser.error('Indica --ip y --dominio, o usa --webcam para prueba local')
    video = puente = captura = None
    abierta = False
    ventana = 'Task 1 - prueba meta B, rostro y QR'
    try:
        if os.name != 'nt' and not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
            raise ValueError('Abrir desde una terminal de Ubuntu con interfaz grafica')
        if not args.webcam:
            cargar_config(args.config)
        cv2.setNumThreads(1)
        sistema = cargar_sistema()
        ensayo = EnsayoB(sistema, args.avanzar_hasta_meta)
        entrada = EntradaMeta(ConsultaEnVivo(sistema, args.nombre), ensayo.sesion)
        if args.webcam:
            captura = cv2.VideoCapture(args.camara_local)
            if not captura.isOpened():
                raise RuntimeError('No se pudo abrir la webcam local')
        else:
            comando = preparar_camara(args.ip, args.dominio, args.contenedor, args.topico_camara)
            control = preparar_barrido(comando, args.contenedor, args.config, args.mover, args.avanzar_hasta_meta)
            video = cargar_transporte()(subprocess.Popen(comando, stdin=subprocess.PIPE, stdout=subprocess.PIPE))
            puente = PuenteBusqueda(control)
        cv2.namedWindow(ventana, cv2.WINDOW_NORMAL | cv2.WINDOW_GUI_NORMAL)
        abierta = True
        cv2.resizeWindow(ventana, 960, 708)
        cv2.setMouseCallback(ventana, entrada.raton)
        if args.webcam:
            print(f'PRUEBA LOCAL WEBCAM ({args.lado_preview}x{args.lado_preview}px): sin robot. '
                  + ('Rostro + QR activos desde Buscar.' if args.sin_meta else 'Espera meta antes de rostro + QR.')
                  + ' Esc detiene y sale.', flush=True)
        else:
            print(('ENSAYO: evasion hasta ver meta, frenar y girar para buscar persona/QR. ' if args.avanzar_hasta_meta else
                   'ENSAYO CERCA DE B: esperar meta, frenar y girar para buscar persona/QR. ')+
                  'Cerrar otros controles. No registra A ni regresa. Esc detiene y sale.', flush=True)
        guardado, ultimo_log, ultimo_qr = False, 0, None
        while True:
            if args.webcam:
                ok, frame = captura.read()
                imagen = _preparar_frame_preview(frame, args.lado_preview) if ok and frame is not None else None
                estado_camara = f'Webcam local {args.lado_preview}x{args.lado_preview}' if imagen is not None else 'Sin fotograma de webcam'
                marca = time.monotonic_ns() if imagen is not None else None
                ahora = time.monotonic_ns()
                estado_ros = dict(recibido_ns=ahora, estado='Webcam local (sin ROS)',
                                  busqueda_desde_ns=ensayo.sesion.inicio_ns)
                if (args.sin_meta and ensayo.sesion.inicio_ns is not None
                        and not ensayo.sesion.meta_confirmada):
                    ensayo.sesion.meta_confirmada, ensayo.sesion.orden = True, 'BARRER'
                    ensayo.sesion.habilitar_busqueda(ensayo.sesion.inicio_ns)
                elif (not args.sin_meta and ensayo.sesion.meta_confirmada
                      and ensayo.sesion.busqueda_desde_ns is None and marca is not None):
                    ensayo.sesion.habilitar_busqueda(marca)
                imagen = ensayo.actualizar(imagen, marca, ahora)
                ahora = time.monotonic_ns()
                if (not reciente(marca, ahora, EDAD_CAMARA_NS) or
                        (ensayo.sesion.inicio_ns is not None and marca <= ensayo.sesion.inicio_ns)):
                    imagen, marca = None, None
            else:
                puente.comprobar()
                if video.proceso.poll() is not None:
                    raise RuntimeError('Termino el receptor de camara')
                imagen, estado_camara = video.fotograma()
                marca = video._decodificado['recibido_ns'] if imagen is not None else None
                ahora = time.monotonic_ns()
                estado_ros = puente.ultimo_estado(ahora)
                ensayo.recibir_control(estado_ros, ahora)
                imagen = ensayo.actualizar(imagen, marca, ahora,
                    lambda: puente.actualizar('VERIFICAR', time.monotonic_ns(), marca))
                ahora = time.monotonic_ns()
                if (not reciente(marca, ahora, EDAD_CAMARA_NS) or
                        (ensayo.sesion.inicio_ns is not None and marca <= ensayo.sesion.inicio_ns)):
                    imagen, marca = None, None
                orden_control = ('VERIFICAR' if ensayo.inspeccionando and ensayo.sesion.orden != 'QR_LISTO'
                                 else ensayo.sesion.orden)
                puente.actualizar(orden_control, ahora, marca)
            if ensayo.sesion.orden == 'QR_LISTO' and not guardado:
                guardado = True
                if args.webcam:
                    print(f'[WEBCAM QR] {ensayo.sesion.nombre}: {ensayo.sesion.palabra}', flush=True)
                else:
                    archivo = guardar_resultado(ensayo.sesion, TAREA/'datos/sesiones')
                    print(f'QR de {ensayo.sesion.nombre}: {ensayo.sesion.palabra}\nGuardado en {archivo}', flush=True)
            if ahora-ultimo_log >= 1_000_000_000:
                control_texto = estado_ros['estado'] if estado_ros else 'Esperando nodo ROS: detenido'
                print(f'[PRUEBA B] {ensayo.sesion.orden}: {ensayo.sesion.motivo} | '
                      f'CONTROL: {control_texto} | ODOM_RX: {resumen_odometria(estado_ros)} | '
                      f'CAMARA: {estado_camara}', flush=True)
                ultimo_log = ahora
                diagnostico_qr = ensayo.resumen_qr()
                if diagnostico_qr != ultimo_qr:
                    print(f'[QR] {diagnostico_qr}', flush=True)
                    ultimo_qr = diagnostico_qr
            cv2.imshow(ventana, dibujar_prueba(imagen, estado_camara, entrada, ensayo, estado_ros, args.mover))
            if entrada.tecla(cv2.waitKeyEx(20)) or cv2.getWindowProperty(ventana, cv2.WND_PROP_VISIBLE) < 1:
                break
    except KeyboardInterrupt:
        return 0
    except (OSError, ValueError, TypeError, KeyError, RuntimeError, subprocess.CalledProcessError, cv2.error) as exc:
        print(f'Error de prueba meta B: {exc}', file=sys.stderr)
        return 1
    finally:
        try:
            if puente is not None:
                puente.cerrar()
        finally:
            try:
                if video is not None:
                    video.cerrar()
                if captura is not None:
                    captura.release()
            finally:
                if abierta:
                    cv2.destroyAllWindows()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
