"""Cámara del robot y giros reconocidos, sin publicar movimiento ni cargar galería."""

import argparse
import os
from pathlib import Path
import subprocess
import sys
import time

import cv2
import numpy as np

from control_giros import COLORES_TRAMO, cargar_config_senales
from interfaz_facial import escribir
from prueba_facial_robot import TAREA, TOPICO, cargar_transporte, preparar_camara
from senales_giro import ConsultaSenales, dibujar_detecciones


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ip', required=True)
    parser.add_argument('--dominio', type=int, required=True)
    parser.add_argument('--contenedor', default='proyecto2-sim')
    parser.add_argument('--topico-camara', default=TOPICO)
    parser.add_argument('--tramo', choices=tuple(COLORES_TRAMO), default='ida')
    parser.add_argument('--config-senales', type=Path, default=TAREA/'config/senales_giro.json')
    args = parser.parse_args(argv)
    video, abierta = None, False
    ventana = 'Task 1 - prueba de senales sin movimiento'
    try:
        if not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
            raise ValueError('Abrir desde una terminal de Ubuntu con interfaz gráfica')
        cv2.setNumThreads(1)
        consulta = ConsultaSenales(args.tramo, cargar_config_senales(args.config_senales))
        comando = preparar_camara(args.ip, args.dominio, args.contenedor, args.topico_camara)
        VideoRobot = cargar_transporte()
        video = VideoRobot(subprocess.Popen(comando, stdin=subprocess.PIPE, stdout=subprocess.PIPE))
        cv2.namedWindow(ventana, cv2.WINDOW_NORMAL | cv2.WINDOW_GUI_NORMAL)
        abierta = True
        ultimo_log = 0
        while True:
            if video.proceso.poll() is not None:
                raise RuntimeError('Terminó el receptor de cámara')
            imagen, estado_camara = video.fotograma()
            marca = video._decodificado['recibido_ns'] if imagen is not None else None
            ahora = time.monotonic_ns()
            resultado = consulta.actualizar(imagen, marca, ahora)
            if marca is None or not 0 <= time.monotonic_ns()-marca <= 1_000_000_000:
                imagen = None
                resultado = consulta.actualizar(None, None, time.monotonic_ns())
            panel = np.full((660, 960, 3), 27, dtype=np.uint8)
            escribir(panel, f'Tramo {args.tramo.upper()}: giros {COLORES_TRAMO[args.tramo]}', 18, 28)
            escribir(panel, resultado['estado'], 18, 58, (70, 190, 240))
            escribir(panel, 'SIN MOVIMIENTO | q/Esc: salir | no lee meta ni QR', 18, 88)
            if imagen is not None:
                anotada = dibujar_detecciones(imagen, resultado, args.tramo)
                escala = min(920/anotada.shape[1], 530/anotada.shape[0])
                anotada = cv2.resize(anotada, None, fx=escala, fy=escala)
                panel[110:110+anotada.shape[0], 20:20+anotada.shape[1]] = anotada
            else:
                escribir(panel, estado_camara, 18, 350, (70, 190, 240))
            cv2.imshow(ventana, panel)
            if ahora-ultimo_log >= 1_000_000_000:
                print(f"[SENALES] {args.tramo}: {resultado['estado']}", flush=True)
                ultimo_log = ahora
            if cv2.waitKey(20) & 0xff in (27, ord('q')) or cv2.getWindowProperty(ventana, cv2.WND_PROP_VISIBLE) < 1:
                break
    except KeyboardInterrupt:
        return 0
    except (OSError, ValueError, TypeError, KeyError, RuntimeError, subprocess.CalledProcessError, cv2.error) as exc:
        print(f'Error de prueba de señales: {exc}', file=sys.stderr)
        return 1
    finally:
        if video is not None:
            video.cerrar()
        if abierta:
            cv2.destroyAllWindows()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
