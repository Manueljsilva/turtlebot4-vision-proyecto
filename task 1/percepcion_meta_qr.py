"""Meta, candidatos faciales y asociación QR; imágenes en memoria, sin ROS."""

import cv2
import numpy as np

from geometria_meta import detect_meta
from reconocimiento_facial import decidir


def caja_puntos(puntos):
    p = np.asarray(puntos, dtype=np.float32).reshape(-1, 2)
    return [float(p[:, 0].min()), float(p[:, 1].min()),
            float(np.ptp(p[:, 0])), float(np.ptp(p[:, 1]))]


def texto_valido(texto):
    # Los QR son datos: nunca ejecutar/abrir su contenido ni dejar controles ANSI.
    return isinstance(texto, str) and bool(texto.strip()) and len(texto) <= 256 and all(c.isprintable() for c in texto)


def marcadores_qr(gray):
    """Cuadrados anidados de los localizadores; también QR no decodificables.

    Es un veto conservador de meta, no un decodificador de QR.
    """
    _, negro = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)
    contornos, jerarquia = cv2.findContours(negro, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if jerarquia is None:
        return []
    cajas = []
    for i, c in enumerate(contornos):
        hijo = jerarquia[0, i, 2]
        nieto = jerarquia[0, hijo, 2] if hijo >= 0 else -1
        if nieto < 0:
            continue
        x, y, w, h = cv2.boundingRect(c)
        if min(w, h) < 10 or not .6 <= w/max(h, 1) <= 1.65:
            continue
        a = cv2.approxPolyDP(c, .04*cv2.arcLength(c, True), True)
        if len(a) != 4 or not cv2.isContourConvex(a):
            continue
        area = abs(cv2.contourArea(c))
        ah = abs(cv2.contourArea(contornos[hijo]))/max(area, 1)
        an = abs(cv2.contourArea(contornos[nieto]))/max(area, 1)
        if .2 <= ah <= .9 and .06 <= an <= .5:
            cajas.append([x, y, w, h])
    return cajas


class DetectorMetaQR:
    def __init__(self):
        self.qr = cv2.QRCodeDetector()
        self.qr_recorte = cv2.QRCodeDetector()

    def _leer_recorte(self, gray, puntos):
        """Perspectiva + margen blanco + ampliación; nunca inventar texto."""
        p = np.asarray(puntos, dtype=np.float32).reshape(4, 2)
        lado = max(np.linalg.norm(p-np.roll(p, -1, axis=0), axis=1))
        if not np.isfinite(p).all() or lado < 12:
            return None
        # Recortar desde el original conserva detalle que se pierde al reducir el frame.
        lado = int(np.clip(round(lado), 32, 512))
        destino = np.array([[0, 0], [lado-1, 0], [lado-1, lado-1], [0, lado-1]], np.float32)
        matriz = cv2.getPerspectiveTransform(p, destino)
        recto = cv2.warpPerspective(gray, matriz, (lado, lado), borderValue=255)
        margen = max(8, round(lado*.18))
        recto = cv2.copyMakeBorder(recto, margen, margen, margen, margen,
                                   cv2.BORDER_CONSTANT, value=255)
        factor = max(1., 256/recto.shape[1])
        recto = cv2.resize(recto, None, fx=factor, fy=factor, interpolation=cv2.INTER_CUBIC)
        texto, _, _ = self.qr_recorte.detectAndDecode(recto)
        if texto_valido(texto):
            return texto.strip()
        _, binario = cv2.threshold(recto, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
        texto, _, _ = self.qr_recorte.detectAndDecode(binario)
        return texto.strip() if texto_valido(texto) else None

    def detectar(self, imagen, leer_qr=False, buscar_meta=True, caja_objetivo=None):
        escala = min(1., (1600 if leer_qr else 960)/imagen.shape[1])
        im = cv2.resize(imagen, None, fx=escala, fy=escala) if escala < 1 else imagen
        gray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
        ok, puntos = self.qr.detectMulti(gray)
        if not ok:
            ok, p = self.qr.detect(gray)
            puntos = p if ok else None
        if puntos is None and leer_qr and max(gray.shape) <= 640:
            ampliada = cv2.resize(gray, None, fx=2, fy=2, interpolation=cv2.INTER_CUBIC)
            ok, p = self.qr.detectMulti(ampliada)
            if not ok:
                ok, p = self.qr.detect(ampliada)
            if ok:
                gray, puntos, escala = ampliada, p, escala*2
        codigos, cajas_qr = [], []
        if puntos is not None:
            puntos = np.asarray(puntos, dtype=np.float32).reshape(-1, 4, 2)
            valores = ['']*len(puntos)
            if leer_qr:
                _, valores_leidos, _ = self.qr.decodeMulti(gray, puntos)
                valores[:len(valores_leidos)] = valores_leidos
            for p, texto in zip(puntos, valores):
                caja = caja_puntos(p/escala)
                cajas_qr.append(caja)
                codigos.append(dict(caja=caja, texto=texto.strip() if texto_valido(texto) else None))
            if leer_qr:
                original = cv2.cvtColor(imagen, cv2.COLOR_BGR2GRAY)
                pendientes = [i for i, q in enumerate(codigos) if q['texto'] is None]
                def prioridad(i):
                    x, y, w, h = codigos[i]['caja']
                    if caja_objetivo is None:
                        return -w*h
                    a, b, u, v = caja_objetivo
                    return abs(x+w/2-a-u/2)+abs(y+h/2-b-v/2)
                # Máximo dos recortes por paquete; primero los próximos al objetivo.
                for i in sorted(pendientes, key=prioridad)[:2]:
                    codigos[i]['texto'] = self._leer_recorte(original, puntos[i]/escala)
        if not buscar_meta:
            return dict(meta=None, codigos=codigos)
        # Se excluyen tanto códigos localizados como finders que no pudo decodificar.
        finders = [[v/escala for v in b] for b in marcadores_qr(gray)]
        s = min(1., 480/gray.shape[1])
        pequeno = cv2.resize(gray, None, fx=s, fy=s) if s < 1 else gray.copy()
        for x, y, w, h in cajas_qr:
            x0, y0, x1, y1 = [int(v*escala*s) for v in (x, y, x+w, y+h)]
            cv2.rectangle(pequeno, (max(0, x0-4), max(0, y0-4)), (x1+4, y1+4), 255, -1)
        meta = None
        # La preview pequeña pierde cruces al muestrear radios enteros. Un único
        # reintento acotado mejora su localización subpíxel; no crea detalle nuevo.
        aumento = min(2., 960/max(pequeno.shape))
        factores = (1., aumento) if aumento >= 1.5 else (1.,)
        for factor in factores:
            busqueda = (pequeno if factor == 1. else cv2.resize(pequeno, None, fx=factor, fy=factor,
                                                              interpolation=cv2.INTER_CUBIC))
            pts, grupo, _, confianza = detect_meta(busqueda)
            if not len(grupo):
                continue
            caja = caja_puntos(pts[grupo]/(s*escala*factor))
            x, y, w, h = caja
            # En el reintento buscar también localizadores QR pequeños que antes
            # podían quedar por debajo de diez píxeles. Conserva el veto, sin leer texto.
            extra = ([[v/(s*escala*factor) for v in b] for b in marcadores_qr(busqueda)]
                     if factor != 1. else [])
            tiene_qr = any(x-.1*w <= a+c/2 <= x+1.1*w and y-.1*h <= b+d/2 <= y+1.1*h
                           for a, b, c, d in cajas_qr+finders+extra)
            if not tiene_qr and w >= 25 and h >= 40:
                meta = dict(caja=caja, confianza=confianza, esquinas=len(grupo),
                            metodo='geometria' if factor == 1. else 'geometria_ampliada')
                break
        return dict(meta=meta, codigos=codigos)


def analizar_candidatos(imagen, sistema):
    """Localiza varias caras y consulta cada recorte con el extractor de una cara.

    No cambia su contrato VARIOS_ROSTROS ni los parámetros/criterio calibrados.
    """
    extractor = sistema.extractor
    alto, ancho = imagen.shape[:2]
    escala = min(1., extractor.lado_maximo/max(alto, ancho))
    pequena = cv2.resize(imagen, None, fx=escala, fy=escala) if escala < 1 else imagen
    extractor.detector.setInputSize((pequena.shape[1], pequena.shape[0]))
    _, caras = extractor.detector.detect(pequena)
    if caras is None:
        return []
    if len(caras) > 12:
        raise ValueError('Demasiados candidatos faciales; mostrar menos fotos simultaneas')
    resultados = []
    for cara in caras:
        if not np.isfinite(cara).all() or min(cara[2:4]) < extractor.cara_minima:
            continue
        x, y, w, h = [float(v/escala) for v in cara[:4]]
        x0, y0 = max(0, int(x-.15*w)), max(0, int(y-.15*h))
        x1, y1 = min(ancho, int(x+1.15*w)), min(alto, int(y+1.15*h))
        if x1 <= x0 or y1 <= y0:
            continue
        analisis = extractor.analizar_imagen(imagen[y0:y1, x0:x1])
        decision = dict(aceptado=False, nombre='NO_CONCLUYENTE', motivo=analisis.estado)
        if analisis.estado == 'OK':
            decision = decidir(sistema.galeria.puntuar(analisis.descriptor), sistema.criterio)
        resultados.append(dict(caja=[x, y, w, h], **decision))
    return resultados


def detectar_regiones_fotos(imagen):
    """Cuadriláteros de fotos/papeles completos; hipótesis visual, no identidad."""
    escala = min(1., 960/imagen.shape[1])
    im = cv2.resize(imagen, None, fx=escala, fy=escala) if escala < 1 else imagen
    gray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
    bordes = cv2.Canny(cv2.GaussianBlur(gray, (3, 3), 0), 60, 160)
    bordes = cv2.morphologyEx(bordes, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    contornos, _ = cv2.findContours(bordes, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    regiones = []
    for c in sorted(contornos, key=cv2.contourArea, reverse=True)[:80]:
        p = cv2.approxPolyDP(c, .025*cv2.arcLength(c, True), True)
        if len(p) != 4 or not cv2.isContourConvex(p) or cv2.contourArea(p) < 1000:
            continue
        x, y, w, h = cv2.boundingRect(p)
        if (min(w, h) < 50 or not .3 <= w/h <= 3 or x < 2 or y < 2
                or x+w >= im.shape[1]-2 or y+h >= im.shape[0]-2
                or cv2.contourArea(p)/(w*h) < .65):
            continue
        puntos = p.reshape(4, 2).astype(np.float32)/escala
        caja = caja_puntos(puntos)
        # Canny puede devolver los dos lados del mismo borde de papel.
        repetida = False
        for r in regiones:
            a, b, u, v = r['caja']
            inter = max(0, min(caja[0]+caja[2], a+u)-max(caja[0], a))*max(0, min(caja[1]+caja[3], b+v)-max(caja[1], b))
            if inter/max(caja[2]*caja[3]+u*v-inter, 1) > .85:
                repetida = True
                break
        if not repetida:
            regiones.append(dict(puntos=puntos.tolist(), caja=caja))
        if len(regiones) >= 24:
            break
    return regiones


def asociar_qr(caras, codigos, objetivo, regiones=None):
    """Misma foto delimitada (también esquinas) o vecindad lateral inequívoca."""
    objetivos = [i for i, c in enumerate(caras) if c['aceptado'] and c['nombre'] == objetivo]
    if len(objetivos) != 1:
        return None, 'Rostro objetivo ausente o varias fotos del objetivo'
    if not codigos:
        return None, 'No se detecta QR; mostrarlo completo junto a la foto'
    def centro(caja):
        x, y, w, h = caja
        return x+w/2, y+h/2
    def misma_foto(i, j):
        x, y, w, h = caras[i]['caja']
        for r in regiones or []:
            p = np.asarray(r['puntos'], np.float32)
            if not 1.8*w*h <= abs(cv2.contourArea(p)) <= 40*w*h:
                continue
            dentro = lambda punto: cv2.pointPolygonTest(p, punto, False) >= 0
            if (not all(dentro(v) for v in ((x, y), (x+w, y), (x+w, y+h), (x, y+h)))
                    or not dentro(centro(codigos[j]['caja']))):
                continue
            # No atribuir una esquina de un panel compartido a una cara concreta.
            if sum(dentro(centro(c['caja'])) for c in caras) != 1:
                continue
            if sum(dentro(centro(q['caja'])) for q in codigos) != 1:
                continue
            return r
        return None
    distancias = {}
    for i, c in enumerate(caras):
        x, y, w, h = c['caja']
        if min(w, h) <= 0:
            continue
        for j, q in enumerate(codigos):
            a, b, u, v = q['caja']
            dy = abs((b+v/2)-(y+h/2))/h
            dx = abs((a+u/2)-(x+w/2))/w
            al_costado = a >= x+.8*w or a+u <= x+.2*w
            region = misma_foto(i, j) if .06 <= u/w <= 4 else None
            if region is not None:
                distancias[i, j] = (0, dx+.7*dy, region)
            elif al_costado and .2 <= dx <= 4 and dy <= 1.2 and .2 <= u/w <= 3:
                distancias[i, j] = (1, dx+.7*dy, None)
    i = objetivos[0]
    opciones = sorted((v[0], v[1], j) for (k, j), v in distancias.items() if k == i)
    if not opciones:
        return None, 'QR sin asociar: mostrar foto completa o QR al costado del rostro'
    nivel, d, j = opciones[0]
    if len(opciones) > 1 and opciones[1][0] == nivel and (nivel == 0 or opciones[1][1]-d < .6):
        return None, 'Varios QR proximos: asociacion ambigua'
    otros = [v for (k, z), v in distancias.items() if z == j and k != i]
    if any(v[0] < nivel or (v[0] == nivel and (nivel == 0 or v[1]-d < .6)) for v in otros):
        return None, 'El QR tambien esta junto a otra cara: separar las fotos'
    texto = codigos[j]['texto']
    if not texto_valido(texto):
        return None, 'QR localizado junto al objetivo, pero aun sin lectura'
    resultado = dict(texto=texto, cara=i, qr=j)
    if nivel == 0:
        resultado['foto'] = distancias[i, j][2]['caja']
    return resultado, ('QR asociado a la misma foto' if nivel == 0 else 'QR asociado por vecindad lateral')
