"""Ver bandera/QR localizados con cámara real; sin galería ni salida de velocidad."""

import argparse
import os
import subprocess
import sys
import time

import cv2
import numpy as np

from control_busqueda import EDAD_CAMARA_NS, reciente
from interfaz_facial import escribir
from percepcion_meta_qr import DetectorMetaQR
from prueba_facial_robot import TOPICO, cargar_transporte, preparar_camara


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--ip', required=True)
    p.add_argument('--dominio', type=int, required=True)
    p.add_argument('--contenedor', default='proyecto2-sim')
    p.add_argument('--topico-camara', default=TOPICO)
    args = p.parse_args(argv)
    video, abierta = None, False
    ventana = 'Task 1 - bandera de meta, sin movimiento'
    try:
        if not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
            raise ValueError('Abrir en una terminal grafica de Ubuntu')
        cv2.setNumThreads(1)
        detector = DetectorMetaQR()
        cmd = preparar_camara(args.ip, args.dominio, args.contenedor, args.topico_camara)
        video = cargar_transporte()(subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE))
        cv2.namedWindow(ventana, cv2.WINDOW_NORMAL | cv2.WINDOW_GUI_NORMAL)
        abierta = True
        ultima, ultimo_log = None, 0
        resultado = dict(meta=None, codigos=[])
        while True:
            if video.proceso.poll() is not None:
                raise RuntimeError('Termino el receptor de camara')
            imagen, estado = video.fotograma()
            marca = video._decodificado['recibido_ns'] if imagen is not None else None
            ahora = time.monotonic_ns()
            if imagen is not None and reciente(marca, ahora, EDAD_CAMARA_NS):
                if ultima is None or marca > ultima:
                    resultado = detector.detectar(imagen, leer_qr=False, buscar_meta=True)
                    ultima = marca
            if imagen is None or not reciente(marca, time.monotonic_ns(), EDAD_CAMARA_NS):
                if imagen is not None:
                    estado = 'Sin imagen reciente tras procesar'
                imagen = None
                resultado = dict(meta=None,codigos=[])
            meta = resultado['meta']
            texto = ('META VISTA: '+meta.get('metodo','geometria')+f'; {meta["esquinas"]} cruces' if meta else
                     'SIN META: mostrar bandera completa y con suficiente tamano' if imagen is not None else estado)
            panel = np.full((660,960,3),27,np.uint8)
            escribir(panel,'PRUEBA VISUAL DE BANDERA | SIN MOVIMIENTO',18,28)
            escribir(panel,texto,18,58,(80,220,220))
            escribir(panel,'No confirma llegada A/B | QR localizados: '+str(len(resultado['codigos']))+' | q/Esc: salir',18,88)
            if imagen is not None:
                vista = imagen.copy()
                if meta:
                    x,y,w,h=[round(v) for v in meta['caja']]
                    cv2.rectangle(vista,(x,y),(x+w,y+h),(80,220,220),2)
                factor=min(920/vista.shape[1],530/vista.shape[0])
                vista=cv2.resize(vista,None,fx=factor,fy=factor)
                panel[110:110+vista.shape[0],20:20+vista.shape[1]]=vista
            cv2.imshow(ventana,panel)
            if ahora-ultimo_log >= 1_000_000_000:
                print('[META VISUAL] '+texto,flush=True)
                paquete = getattr(video, '_decodificado', None)
                if paquete:
                    print(f'[CAMARA] Ultima imagen decodificada: {(time.monotonic_ns()-paquete["recibido_ns"])/1e9:.2f}s desde recepcion ROS; no mide captura', flush=True)
                ultimo_log=ahora
            if cv2.waitKey(20) & 0xff in (27,ord('q')) or cv2.getWindowProperty(ventana,cv2.WND_PROP_VISIBLE) < 1:
                break
    except KeyboardInterrupt:
        return 0
    except (OSError,ValueError,TypeError,KeyError,RuntimeError,subprocess.CalledProcessError,cv2.error) as exc:
        print(f'Error de prueba visual de meta: {exc}',file=sys.stderr)
        return 1
    finally:
        if video is not None:
            video.cerrar()
        if abierta:
            cv2.destroyAllWindows()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
