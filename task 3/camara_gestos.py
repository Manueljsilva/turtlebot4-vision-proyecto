"""Última muestra de manos; lectura separada para no bloquear el control/GUI."""
import base64
import json
from pathlib import Path
import subprocess
import sys
import threading
import time

import cv2
import numpy as np
from control_hibrido import GESTOS


class VideoGestos:
    def __init__(self, camara=0):
        self.cerrojo = threading.Lock()
        self.ultima = None
        self.fin = threading.Event()
        self.error = 'Esperando webcam y confirmacion de mano'
        self.proceso = subprocess.Popen([sys.executable, '-u', str(Path(__file__).with_name('gestos_webcam.py')),
            '--camara', str(camara)], stdout=subprocess.PIPE, text=True)
        self.hilo = threading.Thread(target=self._leer, daemon=True)
        self.hilo.start()

    def aceptar(self, linea, ahora):
        d = json.loads(linea)
        if (len(linea) > 250000 or not isinstance(d,dict)
                or set(d) != {'secuencia','captura_ns','orden','motivo','jpg'}
                or type(d['secuencia']) is not int or d['secuencia'] < 0
                or type(d['captura_ns']) is not int or not 0 <= d['captura_ns'] <= ahora
                or d['orden'] not in GESTOS or not isinstance(d['motivo'],str) or len(d['motivo']) > 160
                or not isinstance(d['jpg'],str) or len(d['jpg']) > 240000):
            raise ValueError('Muestra gestual invalida')
        with self.cerrojo:
            anterior = self.ultima
            if anterior and (d['secuencia'] <= anterior['secuencia'] or d['captura_ns'] <= anterior['captura_ns']):
                raise ValueError('Muestra gestual repetida/retrocedida')
            self.ultima = d

    def _leer(self):
        try:
            while not self.fin.is_set():
                linea = self.proceso.stdout.readline(250001)
                if not linea:
                    break
                self.aceptar(linea, time.monotonic_ns())
        except (ValueError, TypeError, OSError, KeyError):
            self.error = 'Protocolo de webcam invalido; detenido'
        finally:
            with self.cerrojo:
                self.ultima = None
                if self.error.startswith('Esperando'):
                    self.error = 'Webcam detenida; volver a autonomo o reabrir gestos'

    def muestra(self, ahora=None):
        ahora = time.monotonic_ns() if ahora is None else ahora
        with self.cerrojo:
            d = dict(self.ultima) if self.ultima else None
        if d is None or not 0 <= ahora-d['captura_ns'] <= 500_000_000:
            return 'DETENER', None, None, self.error if d is None else 'Mano caducada: detenido'
        imagen = None
        if d['jpg']:
            try:
                datos = base64.b64decode(d['jpg'], validate=True)
                imagen = cv2.imdecode(np.frombuffer(datos,dtype=np.uint8),cv2.IMREAD_COLOR)
            except (ValueError, cv2.error):
                return 'DETENER', None, None, 'Vista de webcam invalida'
        return d['orden'], d['captura_ns'], imagen, d['motivo']

    def cerrar(self):
        self.fin.set()
        if self.proceso.poll() is None:
            self.proceso.terminate()
            try:
                self.proceso.wait(timeout=1)
            except subprocess.TimeoutExpired:
                self.proceso.kill()
                self.proceso.wait(timeout=1)
        self.hilo.join(timeout=1)
        self.proceso.stdout.close()
