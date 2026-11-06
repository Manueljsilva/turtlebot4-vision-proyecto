"""Task B: misma percepción del notebook, con salida de órdenes opcional.

Sin --emitir solo muestra la webcam. Con --emitir escribe JSON por stdout;
los mensajes para la persona van a stderr y la webcam permanece en el host.
"""

import argparse
from pathlib import Path
import sys
import time

from control_gestos import CADUCIDAD_S, codificar, velocidades
from gestos import FiltroOrden, Gesto, Orden, interpretar_mano


RAIZ = Path(__file__).resolve().parent
CONEXIONES = (
    (0, 1), (1, 2), (2, 3), (3, 4), (0, 5), (5, 6), (6, 7), (7, 8),
    (5, 9), (9, 10), (10, 11), (11, 12), (9, 13), (13, 14), (14, 15),
    (15, 16), (13, 17), (0, 17), (17, 18), (18, 19), (19, 20),
)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--emitir", action="store_true", help="Enviar órdenes por stdout")
    parser.add_argument("--camara", type=int, default=0)
    parser.add_argument("--video", type=Path, help="Video para pruebas de desarrollo")
    parser.add_argument("--sin-ventana", action="store_true", help="Para pruebas con video")
    parser.add_argument("--duracion", type=float, help="Finalizar tras estos segundos")
    args = parser.parse_args()
    if args.duracion is not None and not 0 < args.duracion < float("inf"):
        parser.error("--duracion debe ser positiva y finita")
    if args.sin_ventana and args.video is None:
        parser.error("--sin-ventana requiere --video")

    import cv2
    import mediapipe as mp
    import numpy as np

    modelo = RAIZ / "modelos" / "hand_landmarker.task"
    if not modelo.is_file():
        raise FileNotFoundError(f"Falta {modelo}; prepara el modelo desde Tarea1.ipynb")
    opciones = mp.tasks.vision.HandLandmarkerOptions(
        base_options=mp.tasks.BaseOptions(
            model_asset_path=str(modelo), delegate=mp.tasks.BaseOptions.Delegate.CPU),
        running_mode=mp.tasks.vision.RunningMode.VIDEO,
        num_hands=2,
        min_hand_detection_confidence=0.6,
        min_hand_presence_confidence=0.6,
        min_tracking_confidence=0.6,
    )
    captura = cv2.VideoCapture(str(args.video) if args.video else args.camara)
    filtro = FiltroOrden(estabilidad_s=0.4, max_intervalo_s=CADUCIDAD_S)
    secuencia = 0
    ultimo_ms = -1
    ultima_orden = None
    nombre_ventana = "Task B - webcam y ordenes"

    def enviar(orden, captura_ns):
        nonlocal secuencia
        if args.emitir:
            print(codificar(orden, captura_ns, secuencia), flush=True)
            secuencia += 1

    try:
        if not captura.isOpened():
            raise RuntimeError("No se pudo abrir la webcam/video. Cierra la webcam del notebook.")
        captura.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        captura.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        print("[WEBCAM] Sin espejo: tu derecha = DERECHA del robot; tu izquierda = IZQUIERDA.", file=sys.stderr)
        print("[WEBCAM] Palma/sin mano = DETENER. q o cerrar ventana termina la prueba.", file=sys.stderr)
        inicio = time.monotonic()
        with mp.tasks.vision.HandLandmarker.create_from_options(opciones) as detector:
            while args.duracion is None or time.monotonic() - inicio < args.duracion:
                # Antes de leer e inferir: una lectura/inferencia lenta no renueva una orden vieja.
                captura_ns = time.monotonic_ns()
                ok, cuadro = captura.read()
                if not ok:
                    print("[WEBCAM] Fin de video o lectura fallida. DETENER.", file=sys.stderr)
                    break
                alto, ancho = cuadro.shape[:2]
                escala = min(1.0, 640 / ancho, 480 / alto)
                if escala < 1.0:
                    cuadro = cv2.resize(cuadro, (round(ancho * escala), round(alto * escala)))
                alto, ancho = cuadro.shape[:2]
                # Imagen original: interpretar_mano ya adapta los lados del operador.
                rgb = cv2.cvtColor(cuadro, cv2.COLOR_BGR2RGB)
                marca_ms = max(ultimo_ms + 1, captura_ns // 1_000_000)
                ultimo_ms = marca_ms
                resultado = detector.detect_for_video(
                    mp.Image(image_format=mp.ImageFormat.SRGB, data=np.ascontiguousarray(rgb)),
                    marca_ms,
                )
                cantidad = len(resultado.hand_landmarks)
                if cantidad == 1:
                    mano = resultado.hand_landmarks[0]
                    gesto = interpretar_mano([(p.x, p.y) for p in mano], ancho, alto)
                else:
                    gesto = Gesto(Orden.DETENER, "Sin mano" if cantidad == 0 else "Mas de una mano")
                ahora_ns = time.monotonic_ns()
                if ahora_ns - captura_ns > CADUCIDAD_S * 1_000_000_000:
                    filtro.detener()
                    gesto = Gesto(Orden.DETENER, "Lectura o inferencia demasiado lenta")
                orden = filtro.actualizar(gesto.orden, ahora_ns / 1_000_000_000)
                enviar(orden, captura_ns)
                if orden != ultima_orden:
                    vx, wz = velocidades(orden)
                    print(f"[WEBCAM] {orden.value}: vx={vx:+.2f}, wz={wz:+.2f}; {gesto.motivo}",
                          file=sys.stderr, flush=True)
                    ultima_orden = orden

                if not args.sin_ventana:
                    for mano in resultado.hand_landmarks:
                        puntos = [(int(p.x * ancho), int(p.y * alto)) for p in mano]
                        for a, b in CONEXIONES:
                            cv2.line(cuadro, puntos[a], puntos[b], (0, 220, 0), 2)
                        for punto in puntos:
                            cv2.circle(cuadro, punto, 3, (0, 180, 255), -1)
                    cv2.rectangle(cuadro, (0, 0), (ancho, 125), (25, 25, 25), -1)
                    textos = [
                        f"ORDEN AL ROBOT: {orden.value}",
                        f"Propuesta: {gesto.orden.value}", gesto.motivo,
                        "ENVIANDO AL PUENTE" if args.emitir else "VISTA PREVIA / SIN ENVIO",
                        "Tu derecha = DERECHA | q: terminar prueba",
                    ]
                    for i, texto in enumerate(textos):
                        cv2.putText(cuadro, texto, (10, 23 + 23 * i), cv2.FONT_HERSHEY_SIMPLEX,
                                    0.55, (0, 180, 255) if i == 0 else (255, 255, 255), 1)
                    cv2.imshow(nombre_ventana, cuadro)
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        break
                    if cv2.getWindowProperty(nombre_ventana, cv2.WND_PROP_VISIBLE) < 1:
                        break
    except (KeyboardInterrupt, BrokenPipeError):
        pass
    finally:
        filtro.detener()
        try:
            enviar(Orden.DETENER, time.monotonic_ns())
        except BrokenPipeError:
            pass
        captura.release()
        if not args.sin_ventana:
            cv2.destroyAllWindows()
        print("[WEBCAM] Camara liberada. DETENER.", file=sys.stderr)


if __name__ == "__main__":
    main()
