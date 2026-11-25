"""Panel de modos y webcam sobre la interfaz de misión existente."""
import time
import cv2
import numpy as np
from camara_gestos import VideoGestos
from interfaz_facial import escribir

BOTON_MODO = (15, 10, 465, 44)
ORIGEN_MISION = (480, 52)


class InterfazHibrida:
    def __init__(self, puente, camara=0):
        self.puente, self.camara = puente, camara
        self.modo, self.token = 'AUTONOMO', 0
        self.video = None
        self.imagen = None
        self.orden, self.motivo = 'DETENER', 'Webcam apagada en autonomo'
        self.entrada = None
        self.al_cambiar = lambda: None

    def alternar(self):
        self.modo = 'GESTOS' if self.modo == 'AUTONOMO' else 'AUTONOMO'
        self.token = time.monotonic_ns()
        self.puente.cambiar_modo(self.modo, self.token)
        self.al_cambiar()
        self.orden, self.imagen = 'DETENER', None
        if self.video is not None:
            self.video.cerrar()
            self.video = None
        if self.modo == 'GESTOS':
            try:
                self.video = VideoGestos(self.camara)
                self.motivo = 'Abriendo webcam; esperar mano estable'
            except OSError as exc:
                self.motivo = 'Webcam no disponible: '+str(exc)
        else:
            self.motivo = 'Webcam apagada en autonomo'

    def reiniciar(self):
        self.cerrar()
        self.modo, self.token = 'AUTONOMO', 0
        self.orden, self.imagen = 'DETENER', None
        self.motivo = 'Reinicio: autonomo, elegir persona'
        self.puente.cambiar_modo('AUTONOMO', 0)

    def actualizar(self):
        if self.video is not None:
            self.orden, marca, self.imagen, self.motivo = self.video.muestra()
            self.puente.actualizar_gesto(self.orden, marca)

    def raton(self, evento, x, y, banderas=None, parametro=None):
        if evento == cv2.EVENT_LBUTTONDOWN and 15 <= x <= 465 and 10 <= y <= 44:
            if self.entrada is not None and self.entrada.reinicio_ns is None:
                self.alternar()
            return
        if self.entrada is not None and x >= 480 and y >= 52:
            self.entrada.raton(evento,x-480,y-52,banderas,parametro)

    def dibujar(self, panel, estado=None):
        lienzo = np.full((760,1440,3),(26,28,34),dtype=np.uint8)
        lienzo[52:760,480:1440] = panel
        cv2.rectangle(lienzo,(15,10),(465,44),(70,95,130),-1)
        escribir(lienzo,'Cambiar a gestos' if self.modo == 'AUTONOMO' else 'Cambiar a autonomo',32,34,limite=36)
        escribir(lienzo,'Task 3 | A > B > rostro + QR > A',500,34,limite=65)
        escribir(lienzo,'Webcam del operador (sin espejo)',15,82,limite=45)
        if self.imagen is not None:
            lienzo[100:430,20:460] = cv2.resize(self.imagen,(440,330))
        else:
            escribir(lienzo,'Esperando webcam' if self.modo == 'GESTOS' else 'Control autonomo',45,260,limite=40)
        real = estado.get('modo', 'Esperando control') if estado else 'Esperando control'
        cambio = estado.get('cambio_modo',False) if estado else False
        escribir(lienzo,'Modo: '+real+(' | frenando' if cambio else ''),15,463,limite=46)
        escribir(lienzo,'Mano: '+self.orden,15,495,limite=46)
        escribir(lienzo,self.motivo,15,525,limite=52)
        for y, texto in ((575,'Indice arriba: avanzar'),(605,'Senalar a tu lado: girar'),
                         (635,'Palma / sin mano: detener'),(675,'En B: giros para buscar la foto'),
                         (705,'Rostro objetivo: parar y leer QR'),(738,'LiDAR y paradas siguen activos')):
            escribir(lienzo,texto,15,y,limite=48)
        return lienzo

    def cerrar(self):
        if self.video is not None:
            self.video.cerrar()
            self.video = None
