"""Entrada de nombre y panel facial en la ventana OpenCV existente, sin ROS.

El texto en edición no cambia el objetivo hasta pulsar Buscar o Enter.
No requiere paquetes gráficos nuevos; las sugerencias permiten elegir etiquetas
con tildes aunque el teclado de HighGUI no las entregue correctamente.
"""

import unicodedata

import cv2
import numpy as np


ANCHO = 960
CAMPO = (18, 49, 710, 89)
BUSCAR = (728, 49, 942, 89)
SALIR = (844, 674, 942, 700)
Y_SUGERENCIAS = 102
PASO_SUGERENCIAS = 25
ALTO_CABECERA = 246
ALTO_VIDEO = 420


def clave_nombre(texto):
    return ''.join(c for c in unicodedata.normalize('NFKD', texto.strip().casefold())
                   if not unicodedata.combining(c))


def ascii_texto(texto):
    return unicodedata.normalize('NFKD', str(texto)).encode('ascii', 'ignore').decode('ascii')


def dentro(caja, x, y):
    x0, y0, x1, y1 = caja
    return x0 <= x < x1 and y0 <= y < y1


class EntradaObjetivo:
    """Control de edición separado de la identidad reconocida por la cámara."""

    def __init__(self, consulta):
        self.consulta = consulta
        self.nombres = sorted(set(consulta.sistema.galeria.alumnos.tolist()))
        self.texto = consulta.nombre or ''
        self.cursor = len(self.texto)
        self.foco = consulta.nombre is None
        self.aviso = ('Objetivo cargado. Puedes editarlo y confirmar con Buscar.' if consulta.nombre else
                      'Escribe un nombre y pulsa Buscar. Puedes escribir sin tildes.')
        self.error = False
        self.cerrar = False

    def sugerencias(self):
        clave = clave_nombre(self.texto)
        return [n for n in self.nombres if clave in clave_nombre(n)][:3]

    def confirmar(self):
        clave = clave_nombre(self.texto)
        opciones = [n for n in self.nombres if clave and clave_nombre(n) == clave]
        if len(opciones) != 1:
            self.aviso = ('Escribe un nombre de la galeria o elige una sugerencia.'
                          if not opciones else 'Nombre ambiguo. Revisa las etiquetas de la galeria.')
            self.error = True
            return
        try:
            self.consulta.seleccionar_nombre(opciones[0])
        except ValueError as exc:
            self.aviso, self.error = str(exc), True
            return
        self.texto = self.consulta.nombre
        self.cursor = len(self.texto)
        self.foco = False
        self.aviso = 'Nombre guardado. Muestra una fotografia a la camara.'
        self.error = False

    def raton(self, evento, x, y, banderas=None, parametro=None):
        if evento != cv2.EVENT_LBUTTONDOWN:
            return
        if dentro(SALIR, x, y):
            self.cerrar = True
        elif dentro(CAMPO, x, y):
            self.foco = True
            self.cursor = len(self.texto)
        elif dentro(BUSCAR, x, y):
            self.confirmar()
        else:
            for i, nombre in enumerate(self.sugerencias()):
                caja = (18, Y_SUGERENCIAS + i * PASO_SUGERENCIAS,
                        942, Y_SUGERENCIAS + (i + 1) * PASO_SUGERENCIAS)
                if dentro(caja, x, y):
                    self.texto, self.cursor, self.foco = nombre, len(nombre), True
                    self.aviso = 'Pulsa Buscar para confirmar este nombre.'
                    self.error = False
                    return
            self.foco = False

    def tecla(self, codigo):
        if codigo in (27, 65307):
            self.cerrar = True
        elif codigo in (10, 13, 65293, 65421):
            self.confirmar()
        elif self.foco:
            if codigo in (8, 127, 65288):
                self.texto = self.texto[:max(0, self.cursor - 1)] + self.texto[self.cursor:]
                self.cursor = max(0, self.cursor - 1)
            elif codigo in (65535, 3014656):
                self.texto = self.texto[:self.cursor] + self.texto[self.cursor + 1:]
            elif codigo in (65361, 2424832):
                self.cursor = max(0, self.cursor - 1)
                return self.cerrar
            elif codigo in (65363, 2555904):
                self.cursor = min(len(self.texto), self.cursor + 1)
                return self.cerrar
            elif codigo in (65360, 2359296):
                self.cursor = 0
                return self.cerrar
            elif codigo in (65367, 2293760):
                self.cursor = len(self.texto)
                return self.cerrar
            elif (32 <= codigo < 0xFE00 and chr(codigo).isprintable()
                  and len(self.texto) < 120):
                self.texto = self.texto[:self.cursor] + chr(codigo) + self.texto[self.cursor:]
                self.cursor += 1
            else:
                return self.cerrar
            self.aviso = 'Nombre en edicion. Pulsa Buscar para activarlo.'
            self.error = False
        elif codigo in (ord('q'), ord('Q')):
            self.cerrar = True
        return self.cerrar


def mensaje_busqueda(resultado, objetivo, imagen_reciente):
    if not imagen_reciente:
        return 'Esperando imagen reciente de la camara del robot...', (70, 190, 240)
    if objetivo is None:
        return 'Escribe a quien quieres buscar y pulsa Buscar.', (230, 220, 180)
    if not resultado['aceptado']:
        mensajes = {
            'SIN_ROSTRO': 'No se detecta un rostro. Muestra una fotografia a la camara.',
            'VARIOS_ROSTROS': 'Hay varias caras. Muestra una sola para esta prueba.',
            'ROSTRO_NO_UTIL': 'Acerca la fotografia para ver mejor el rostro.',
            'ERROR_EXTRACCION': 'No pude analizar esta imagen. Esperando otra.',
            'IMAGEN_INVALIDA': 'No pude analizar esta imagen. Esperando otra.',
        }
        return mensajes.get(resultado['motivo'],
                            'Aun no puedo identificar a esta persona con confianza.'), (70, 190, 240)
    if resultado['objetivo_confirmado']:
        return 'Reconozco a la persona que buscas.', (100, 225, 100)
    return 'Reconozco a otra persona; no es la que buscas.', (70, 190, 240)


def escribir(lienzo, texto, x, y, color=(230, 230, 230), escala=0.52, limite=108):
    cv2.putText(lienzo, ascii_texto(texto)[:limite], (x, y), cv2.FONT_HERSHEY_SIMPLEX,
                escala, color, 1, cv2.LINE_AA)


def dibujar_interfaz(imagen, resultado, estado, entrada):
    lienzo = np.full((708, ANCHO, 3), 27, dtype=np.uint8)
    escribir(lienzo, 'Persona que quieres buscar', 18, 29, escala=0.65)
    cv2.rectangle(lienzo, CAMPO[:2], CAMPO[2:], (54, 54, 54), -1)
    cv2.rectangle(lienzo, CAMPO[:2], CAMPO[2:], (200, 170, 70) if entrada.foco else (100, 100, 100), 1)
    texto = entrada.texto[:entrada.cursor] + ('|' if entrada.foco else '') + entrada.texto[entrada.cursor:]
    escribir(lienzo, texto[-76:] or 'Escribe el nombre...', 28, 75, limite=76)
    cv2.rectangle(lienzo, BUSCAR[:2], BUSCAR[2:], (105, 90, 30), -1)
    escribir(lienzo, 'Buscar', 793, 76, escala=0.65)
    for i, nombre in enumerate(entrada.sugerencias()):
        y = Y_SUGERENCIAS + i * PASO_SUGERENCIAS
        cv2.rectangle(lienzo, (18, y), (942, y + PASO_SUGERENCIAS - 2), (42, 42, 42), -1)
        escribir(lienzo, 'Elegir: ' + nombre, 28, y + 17)
    escribir(lienzo, entrada.aviso, 18, 193,
             color=(100, 130, 245) if entrada.error else (185, 185, 185))
    objetivo = entrada.consulta.nombre or 'sin seleccionar'
    observado = resultado['nombre'] if imagen is not None else 'EN ESPERA'
    escribir(lienzo, f'Buscando: {objetivo} | Reconocido: {observado}', 18, 216)
    mensaje, color = mensaje_busqueda(resultado, entrada.consulta.nombre, imagen is not None)
    escribir(lienzo, mensaje, 18, 238, color)
    if imagen is None:
        escribir(lienzo, estado, 210, ALTO_CABECERA + 205, color=(70, 190, 240), escala=0.65)
    else:
        vista = imagen.copy()
        caja = resultado.get('caja')
        if caja is not None:
            x, y, w, h = (round(v) for v in caja)
            cv2.rectangle(vista, (x, y), (x + w, y + h), color, 2)
        factor = min(ANCHO / vista.shape[1], ALTO_VIDEO / vista.shape[0])
        vista = cv2.resize(vista, (max(1, round(vista.shape[1] * factor)),
                                 max(1, round(vista.shape[0] * factor))))
        y = ALTO_CABECERA + (ALTO_VIDEO - vista.shape[0]) // 2
        x = (ANCHO - vista.shape[1]) // 2
        lienzo[y:y + vista.shape[0], x:x + vista.shape[1]] = vista
    escribir(lienzo, 'Robot quieto | Una cara a la vez | Esc: terminar', 18, 693)
    cv2.rectangle(lienzo, SALIR[:2], SALIR[2:], (62, 62, 62), -1)
    escribir(lienzo, 'Salir', 872, 692)
    return lienzo
