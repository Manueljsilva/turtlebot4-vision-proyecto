"""Pruebas con procesos y tubería reales, sin ROS, Docker ni webcam."""

from pathlib import Path
import queue
import subprocess
import sys
import threading
import time
import unittest

from control_gestos import codificar
from gestos import Orden


RAIZ = Path(__file__).resolve().parents[1]


class EnlaceTests(unittest.TestCase):
    def setUp(self):
        self.proceso = subprocess.Popen(
            [sys.executable, "-u", str(RAIZ / "puente_gestos_ros.py"), "--sin-ros"],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            text=True, bufsize=1,
        )
        self.lineas = queue.Queue()
        self.lector = threading.Thread(target=self._leer, daemon=True)
        self.lector.start()
        self.esperar("Sin ROS / sin movimiento")

    def _leer(self):
        for linea in self.proceso.stdout:
            self.lineas.put(linea)

    def esperar(self, texto, plazo_s=3):
        limite = time.monotonic() + plazo_s
        vistas = []
        while time.monotonic() < limite:
            try:
                linea = self.lineas.get(timeout=max(0.001, limite - time.monotonic()))
            except queue.Empty:
                break
            vistas.append(linea)
            if texto in linea:
                return
        self.fail(f"No se recibio {texto!r}; recibido: {vistas!r}")

    def enviar(self, orden, secuencia, captura_ns=None):
        captura_ns = time.monotonic_ns() if captura_ns is None else captura_ns
        self.proceso.stdin.write(codificar(orden, captura_ns, secuencia) + "\n")
        self.proceso.stdin.flush()

    def tearDown(self):
        if not self.proceso.stdin.closed:
            self.proceso.stdin.close()
        try:
            self.proceso.wait(timeout=3)
        except subprocess.TimeoutExpired:
            self.proceso.kill()
            self.proceso.wait(timeout=3)
        self.lector.join(timeout=1)
        self.proceso.stdout.close()
        self.proceso.stderr.close()

    def test_transporte_y_parada_con_emisor_bloqueado(self):
        self.enviar(Orden.AVANZAR, 0)
        self.esperar("AVANZAR (0.306, 0.0)")
        # Dejar stdin abierto SIN más muestras: el receptor debe detener por sí mismo.
        self.esperar("DETENER (0.0, 0.0) | Webcam sin actualizar", plazo_s=2)
        self.enviar(Orden.DERECHA, 1)
        self.esperar("DERECHA (0.0, -1.9)")

    def test_cerrar_el_emisor_detiene_y_termina_el_receptor(self):
        self.enviar(Orden.IZQUIERDA, 0)
        self.esperar("IZQUIERDA (0.0, 1.9)")
        self.proceso.stdin.close()
        self.esperar("DETENER (0.0, 0.0) | Enlace cerrado")
        self.assertEqual(self.proceso.wait(timeout=2), 0)

    def test_captura_vieja_y_mensaje_roto_no_se_convierten_en_movimiento(self):
        self.enviar(Orden.AVANZAR, 0, time.monotonic_ns() - 1_000_000_000)
        self.esperar("DETENER (0.0, 0.0) | Orden recibida fuera de tiempo")
        self.enviar(Orden.AVANZAR, 1)
        self.esperar("AVANZAR (0.306, 0.0)")
        self.proceso.stdin.write("{mensaje roto}\n")
        self.proceso.stdin.flush()
        self.esperar("DETENER (0.0, 0.0) | Mensaje de webcam invalido")


if __name__ == "__main__":
    unittest.main()
