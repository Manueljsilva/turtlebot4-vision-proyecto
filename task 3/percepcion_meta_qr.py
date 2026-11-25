"""Meta, candidatos faciales y asociación QR; imágenes en memoria, sin ROS."""

import cv2
import numpy as np

from control_meta import misma_caja
from geometria_meta import detect_meta
from reconocimiento_facial import decidir, normalizar


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
        lado = int(np.clip(round(lado*2), 128, 512))
        destino = np.array([[0, 0], [lado-1, 0], [lado-1, lado-1], [0, lado-1]], np.float32)
        matriz = cv2.getPerspectiveTransform(p, destino)
        recto = cv2.warpPerspective(gray, matriz, (lado, lado), flags=cv2.INTER_CUBIC, borderValue=255)
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
            factor_qr = min(3., max(2., 750./max(gray.shape)))
            ampliada = cv2.resize(gray, None, fx=factor_qr, fy=factor_qr, interpolation=cv2.INTER_CUBIC)
            ok, p = self.qr.detectMulti(ampliada)
            if not ok:
                ok, p = self.qr.detect(ampliada)
            if ok:
                gray, puntos, escala = ampliada, p, escala*factor_qr
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
                fuente_recorte = original if escala <= 1. else gray
                escala_recorte = escala if escala <= 1. else 1.
                pendientes = [i for i, q in enumerate(codigos) if q['texto'] is None]
                def prioridad(i):
                    x, y, w, h = codigos[i]['caja']
                    if caja_objetivo is None:
                        return -w*h
                    a, b, u, v = caja_objetivo
                    return abs(x+w/2-a-u/2)+abs(y+h/2-b-v/2)
                # Máximo dos recortes por paquete; primero los próximos al objetivo.
                for i in sorted(pendientes, key=prioridad)[:2]:
                    codigos[i]['texto'] = self._leer_recorte(fuente_recorte, puntos[i]/escala_recorte)
                if max(gray.shape) <= 640 and all(q['texto'] is None for q in codigos):
                    factor_qr = min(3., max(2., 750./max(gray.shape)))
                    ampliada = cv2.resize(gray, None, fx=factor_qr, fy=factor_qr, interpolation=cv2.INTER_CUBIC)
                    ok2, p2 = self.qr.detectMulti(ampliada)
                    if not ok2:
                        ok2, p2 = self.qr.detect(ampliada)
                    if ok2 and p2 is not None:
                        p2 = np.asarray(p2, dtype=np.float32).reshape(-1, 4, 2)
                        _, vals2, _ = self.qr.decodeMulti(ampliada, p2)
                        codigos_2 = []
                        for pt2, txt2 in zip(p2, list(vals2)+['']*len(p2)):
                            codigos_2.append(dict(caja=caja_puntos(pt2/(escala*factor_qr)),
                                                  texto=txt2.strip() if texto_valido(txt2) else None))
                        if any(q['texto'] is not None for q in codigos_2):
                            codigos = codigos_2
                            cajas_qr = [q['caja'] for q in codigos]
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


def _roi_aislado(caras, idx, x0, y0, x1, y1):
    for j, otra in enumerate(caras):
        if j == idx or not np.isfinite(otra).all():
            continue
        ox, oy, ow, oh = [float(v) for v in otra[:4]]
        if ox < x1 and ox+ow > x0 and oy < y1 and oy+oh > y0:
            return False
    return True


def _decidir_por_landmarks(extractor, sistema, imagen, cara):
    alineada = extractor.reconocedor.alignCrop(imagen, cara)
    descriptor = normalizar(extractor.reconocedor.feature(alineada))
    return decidir(sistema.galeria.puntuar(descriptor), sistema.criterio)


def _mejor_decision(a, b):
    clave = lambda d: (int(bool(d['aceptado'])),
                       float(d['similitud']) if d.get('similitud') is not None else -2.)
    return b if a is None or clave(b) > clave(a) else a


def _evaluar_escala(imagen, escena, escala, sistema):
    extractor = sistema.extractor
    ea, ean = escena.shape[:2]
    extractor.detector.setInputSize((ean, ea))
    _, caras = extractor.detector.detect(escena)
    if caras is None:
        return []
    if len(caras) > 12:
        raise ValueError('Demasiados candidatos faciales; mostrar menos fotos simultaneas')
    resultados = []
    tiene_reconocedor = hasattr(extractor, 'reconocedor')
    for idx, cara in enumerate(caras):
        if not np.isfinite(cara).all() or min(cara[2:4]) < extractor.cara_minima:
            continue
        xs, ys, ws, hs = [float(v) for v in cara[:4]]
        x0, y0 = max(0, int(xs-.15*ws)), max(0, int(ys-.15*hs))
        x1, y1 = min(ean, int(xs+1.15*ws)), min(ea, int(ys+1.15*hs))
        if x1 <= x0 or y1 <= y0:
            continue
        decision = None
        aislado = _roi_aislado(caras, idx, x0, y0, x1, y1)
        if tiene_reconocedor and aislado and len(cara) >= 15:
            # Alinear directamente con el contexto completo evita el bajón de landmarks del recorte apretado.
            intentos = [(escena, cara)]
            if escala > 1. and min(ws/escala, hs/escala) >= extractor.cara_minima*.85:
                cara_orig = cara.copy()
                cara_orig[:-1] = cara_orig[:-1]/escala
                intentos.insert(0, (imagen, cara_orig))
            for img_ref, cara_ref in intentos:
                try:
                    decision = _mejor_decision(decision, _decidir_por_landmarks(extractor, sistema, img_ref, cara_ref))
                    if decision['aceptado']:
                        break
                except (cv2.error, ValueError):
                    pass
        if decision is None:
            analisis = extractor.analizar_imagen(escena[y0:y1, x0:x1])
            if analisis.estado == 'OK' and analisis.descriptor is not None:
                decision = decidir(sistema.galeria.puntuar(analisis.descriptor), sistema.criterio)
            else:
                decision = dict(aceptado=False, nombre='NO_CONCLUYENTE', motivo=analisis.estado)
        resultados.append(dict(caja=[xs/escala, ys/escala, ws/escala, hs/escala], **decision))
    return resultados


def analizar_candidatos(imagen, sistema):
    """Localiza varias caras y evalúa cada una sin degradar las nítidas ni perder las pequeñas."""
    extractor = sistema.extractor
    alto, ancho = imagen.shape[:2]
    base = min(1., extractor.lado_maximo/max(alto, ancho))
    if not hasattr(extractor, 'reconocedor') or max(alto, ancho) >= 720:
        escena = cv2.resize(imagen, None, fx=base, fy=base) if base < 1 else imagen
        return _evaluar_escala(imagen, escena, base, sistema)
    # En previews pequeñas (p. ej. 250x250), combinar 1x, 2x y 3x/720 px conserva nitidez
    # en caras grandes y alcanza rostros <28 px incluso habiendo otras fotos en el cuadro.
    escalas = [1., min(2., 720./max(alto, ancho))]
    if max(alto, ancho) < 360:
        escalas.append(min(3., 720./max(alto, ancho)))
    candidatos = []
    for esc in escalas:
        escena = imagen if esc == 1. else cv2.resize(imagen, None, fx=esc, fy=esc, interpolation=cv2.INTER_CUBIC)
        for nuevo in _evaluar_escala(imagen, escena, esc, sistema):
            pos = next((i for i, previo in enumerate(candidatos) if misma_caja(previo['caja'], nuevo['caja'])), None)
            if pos is None:
                candidatos.append(nuevo)
            elif _mejor_decision(candidatos[pos], nuevo) is nuevo:
                candidatos[pos] = nuevo
    if len(candidatos) > 12:
        raise ValueError('Demasiados candidatos faciales; mostrar menos fotos simultaneas')
    return candidatos


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
            debajo = b >= y+.8*h and dx <= .45 and .8 <= dy <= 2.2
            region = misma_foto(i, j) if .06 <= u/w <= 4 else None
            if region is not None:
                distancias[i, j] = (0, dx+.7*dy, region)
            elif al_costado and .2 <= dx <= 4 and dy <= 1.2 and .2 <= u/w <= 3:
                distancias[i, j] = (1, dx+.7*dy, None)
            elif debajo and .2 <= u/w <= 3:
                distancias[i, j] = (1, dy+.7*dx, None)
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
