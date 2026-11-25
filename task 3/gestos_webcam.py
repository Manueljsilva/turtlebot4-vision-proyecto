"""Webcam aislada: reutiliza reglas de Task 2; solo emite muestras, nunca ROS."""
import argparse
import base64
import json
from pathlib import Path
import sys
import time


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--camara', type=int, default=0)
    args = p.parse_args()
    import cv2
    import mediapipe as mp
    import numpy as np
    tarea2 = Path(__file__).resolve().parent.parent/'task 2'
    sys.path.insert(0, str(tarea2))
    from gestos import FiltroOrden, Gesto, Orden, interpretar_mano
    modelo = tarea2/'modelos/hand_landmarker.task'
    if not modelo.is_file():
        raise FileNotFoundError('Falta el modelo local de manos de Task 2')
    opciones = mp.tasks.vision.HandLandmarkerOptions(
        base_options=mp.tasks.BaseOptions(model_asset_path=str(modelo), delegate=mp.tasks.BaseOptions.Delegate.CPU),
        running_mode=mp.tasks.vision.RunningMode.VIDEO, num_hands=2,
        min_hand_detection_confidence=.6, min_hand_presence_confidence=.6, min_tracking_confidence=.6)
    cv2.setNumThreads(1)
    captura = cv2.VideoCapture(args.camara)
    filtro = FiltroOrden(estabilidad_s=.4, max_intervalo_s=.5)
    try:
        if not captura.isOpened():
            raise RuntimeError('No se pudo abrir la webcam; cerrar otras aplicaciones que la usen')
        captura.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        captura.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        secuencia, anterior_ms = 0, -1
        with mp.tasks.vision.HandLandmarker.create_from_options(opciones) as detector:
            while True:
                inicio = time.monotonic_ns()
                ok, cuadro = captura.read()
                if not ok:
                    raise RuntimeError('Lectura de webcam interrumpida')
                h, w = cuadro.shape[:2]
                escala = min(1., 640/w, 480/h)
                if escala < 1:
                    cuadro = cv2.resize(cuadro, (round(w*escala), round(h*escala)))
                h, w = cuadro.shape[:2]
                marca_ms = max(anterior_ms+1, inicio//1_000_000)
                anterior_ms = marca_ms
                resultado = detector.detect_for_video(mp.Image(image_format=mp.ImageFormat.SRGB,
                    data=np.ascontiguousarray(cv2.cvtColor(cuadro, cv2.COLOR_BGR2RGB))), marca_ms)
                manos = resultado.hand_landmarks
                gesto = (interpretar_mano([(v.x,v.y) for v in manos[0]], w, h) if len(manos) == 1
                         else Gesto(Orden.DETENER, 'Sin mano' if not manos else 'Mas de una mano'))
                orden = filtro.actualizar(gesto.orden, inicio/1e9)
                if time.monotonic_ns()-inicio > 500_000_000:
                    orden = filtro.detener()
                    gesto = Gesto(Orden.DETENER, 'Inferencia/lectura lenta')
                ok, jpg = cv2.imencode('.jpg', cv2.resize(cuadro,(440,330)), [cv2.IMWRITE_JPEG_QUALITY,65])
                print(json.dumps(dict(secuencia=secuencia, captura_ns=inicio, orden=orden.value,
                    motivo=gesto.motivo, jpg=base64.b64encode(jpg).decode('ascii') if ok else '')), flush=True)
                secuencia += 1
                restante = .08-(time.monotonic_ns()-inicio)/1e9
                if restante > 0:
                    time.sleep(restante)
    finally:
        captura.release()


if __name__ == '__main__':
    try:
        main()
    except (KeyboardInterrupt, BrokenPipeError):
        pass
    except Exception as exc:
        print('[GESTOS] '+str(exc), file=sys.stderr, flush=True)
        raise SystemExit(1)
