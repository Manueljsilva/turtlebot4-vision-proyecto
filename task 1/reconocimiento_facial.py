"""Reconocimiento facial local YuNet/SFace, independiente de ROS y de la cámara.

La galería, calibración y evaluación son conjuntos distintos. Una similitud
coseno es una puntuación, no una probabilidad de identidad.
"""

from collections import defaultdict
from dataclasses import asdict, dataclass
import json
from pathlib import Path
import time

import cv2
import numpy as np

from modelos_rostros import sha256_archivo


EXTENSIONES = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}
ROLES = {"galeria", "calibracion", "evaluacion"}


@dataclass(frozen=True)
class Foto:
    ruta: str
    alumno: str
    sha256: str
    rol: str = ""


def inventariar(carpeta: Path) -> list[Foto]:
    """Lee etiquetas de subcarpetas, sin cambiar ni copiar fotografías."""
    carpeta = Path(carpeta).resolve()
    fotos = []
    for alumno in sorted(p for p in carpeta.iterdir() if p.is_dir()):
        for p in sorted(alumno.iterdir()):
            if p.is_file() and p.suffix.lower() in EXTENSIONES:
                fotos.append(Foto(p.relative_to(carpeta).as_posix(), alumno.name, sha256_archivo(p)))
    if not fotos:
        raise ValueError(f"No hay fotos etiquetadas en {carpeta}")
    hashes = [f.sha256 for f in fotos]
    if len(set(hashes)) != len(hashes):
        raise ValueError("Hay fotos duplicadas exactas. Revisar antes de separar los conjuntos.")
    return fotos


def particion_inicial(fotos: list[Foto]) -> list[Foto]:
    """Orden fijo por archivo: últimas dos para calibrar/evaluar si hay >=3.

    Es una partición exploratoria por foto, no una separación por sesión.
    Con dos fotos se reserva una para evaluar; con una solo hay referencia.
    """
    grupos = defaultdict(list)
    for foto in fotos:
        grupos[foto.alumno].append(foto)
    salida = []
    for alumno in sorted(grupos):
        grupo = sorted(grupos[alumno], key=lambda f: f.ruta)
        for i, foto in enumerate(grupo):
            rol = "galeria"
            if len(grupo) >= 3 and i == len(grupo) - 2:
                rol = "calibracion"
            if len(grupo) >= 2 and i == len(grupo) - 1:
                rol = "evaluacion"
            salida.append(Foto(foto.ruta, foto.alumno, foto.sha256, rol))
    return salida


def preparar_particion(fotos: list[Foto], archivo: Path, regenerar=False) -> list[Foto]:
    """Reutiliza un manifiesto auditable; cambios de datos requieren regeneración explícita."""
    archivo = Path(archivo)
    if archivo.exists() and not regenerar:
        datos = json.loads(archivo.read_text(encoding="utf-8"))
        if datos.get("version") != 1:
            raise ValueError("Version de particion no compatible")
        salida = [Foto(**fila) for fila in datos["fotos"]]
    else:
        salida = particion_inicial(fotos)
    actual = {(f.ruta, f.alumno, f.sha256) for f in fotos}
    guardado = {(f.ruta, f.alumno, f.sha256) for f in salida}
    if actual != guardado or len(salida) != len(fotos) or any(f.rol not in ROLES for f in salida):
        raise ValueError("Los datos o la particion cambiaron. Revisar y usar REGENERAR_PARTICION=True.")
    if len({f.sha256 for f in salida}) != len(salida):
        raise ValueError("Duplicados entre los conjuntos: la evaluacion no seria valida")
    if not archivo.exists() or regenerar:
        archivo.parent.mkdir(parents=True, exist_ok=True)
        datos = {"version": 1, "metodo": "orden de archivo; ensayo dentro del dataset, no por sesion",
                 "fotos": [asdict(f) for f in salida]}
        archivo.write_text(json.dumps(datos, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return salida


def normalizar(vector) -> np.ndarray:
    vector = np.asarray(vector, dtype=np.float32).reshape(-1)
    if vector.size == 0 or not np.all(np.isfinite(vector)):
        raise ValueError("Descriptor vacio o no finito")
    norma = float(np.linalg.norm(vector))
    if not np.isfinite(norma) or norma <= 1e-8:
        raise ValueError("Descriptor degenerado")
    return vector / norma


@dataclass
class Analisis:
    foto: Foto
    estado: str
    caras: int
    descriptor: np.ndarray | None
    deteccion: float | None
    nitidez: float | None
    tiempo_ms: float
    caja: tuple[float, float, float, float] | None = None

    def fila(self):
        return {"alumno": self.foto.alumno, "archivo": self.foto.ruta,
                "rol": self.foto.rol, "estado": self.estado, "caras": self.caras,
                "deteccion": self.deteccion, "nitidez": self.nitidez,
                "tiempo_ms": round(self.tiempo_ms, 1)}


class ExtractorFacial:
    """Una imagen a la vez, CPU; rechaza cero/varias caras sin escoger una arbitraria."""

    def __init__(self, rutas: dict[str, Path], lado_maximo=960, deteccion_minima=0.85,
                 cara_minima=32):
        if lado_maximo < 32 or cara_minima <= 0 or not 0 < deteccion_minima < 1:
            raise ValueError("Parametros de deteccion invalidos")
        self.lado_maximo = lado_maximo
        self.cara_minima = cara_minima
        self.detector = cv2.FaceDetectorYN.create(
            model=str(rutas["yunet"]), config="", input_size=(320, 320),
            score_threshold=deteccion_minima, nms_threshold=0.3, top_k=5000,
            backend_id=cv2.dnn.DNN_BACKEND_OPENCV, target_id=cv2.dnn.DNN_TARGET_CPU)
        self.reconocedor = cv2.FaceRecognizerSF.create(
            model=str(rutas["sface"]), config="", backend_id=cv2.dnn.DNN_BACKEND_OPENCV,
            target_id=cv2.dnn.DNN_TARGET_CPU)

    def analizar_imagen(self, imagen: np.ndarray, foto: Foto | None = None) -> Analisis:
        foto = foto or Foto("consulta", "", "")
        inicio = time.perf_counter()
        def resultado(estado, caras=0, descriptor=None, deteccion=None, nitidez=None, caja=None):
            return Analisis(foto, estado, caras, descriptor, deteccion, nitidez,
                            1000 * (time.perf_counter() - inicio), caja)
        if imagen is None or imagen.ndim != 3 or imagen.shape[2] != 3 or imagen.size == 0:
            return resultado("IMAGEN_INVALIDA")
        alto, ancho = imagen.shape[:2]
        escala = min(1.0, self.lado_maximo / max(alto, ancho))
        if escala < 1:
            imagen = cv2.resize(imagen, (max(1, round(ancho * escala)), max(1, round(alto * escala))),
                                interpolation=cv2.INTER_AREA)
        try:
            self.detector.setInputSize((imagen.shape[1], imagen.shape[0]))
            _, caras = self.detector.detect(imagen)
            cantidad = 0 if caras is None else len(caras)
            if cantidad != 1:
                return resultado("SIN_ROSTRO" if cantidad == 0 else "VARIOS_ROSTROS", cantidad)
            cara = caras[0]
            if not np.all(np.isfinite(cara)) or min(cara[2:4]) < self.cara_minima:
                return resultado("ROSTRO_NO_UTIL", 1, deteccion=float(cara[-1]))
            alineada = self.reconocedor.alignCrop(imagen, cara)
            descriptor = normalizar(self.reconocedor.feature(alineada))
            gris = cv2.cvtColor(alineada, cv2.COLOR_BGR2GRAY)
            nitidez = float(cv2.Laplacian(gris, cv2.CV_64F).var())
            caja = tuple(float(v / escala) for v in cara[:4])
            return resultado("OK", 1, descriptor, float(cara[-1]), nitidez, caja)
        except (cv2.error, ValueError):
            return resultado("ERROR_EXTRACCION")

    def analizar_archivo(self, ruta: Path, foto: Foto | None = None) -> Analisis:
        try:
            imagen = cv2.imdecode(np.fromfile(Path(ruta), dtype=np.uint8), cv2.IMREAD_COLOR)
        except (OSError, cv2.error):
            imagen = None
        return self.analizar_imagen(imagen, foto)


class Galeria:
    """Puntuación por alumno: media de sus dos referencias más similares (o una)."""

    def __init__(self, vectores, alumnos, origenes=None):
        if len(vectores) == 0 or len(vectores) != len(alumnos):
            raise ValueError("Galeria vacia o sin etiquetas consistentes")
        self.vectores = np.stack([normalizar(v) for v in vectores])
        self.alumnos = np.asarray(alumnos, dtype=str)
        self.origenes = np.asarray(origenes if origenes is not None else [""] * len(alumnos), dtype=str)
        if len(self.origenes) != len(alumnos):
            raise ValueError("Origenes de galeria inconsistentes")

    @classmethod
    def desde_analisis(cls, analisis: list[Analisis]):
        referencias = [a for a in analisis if a.foto.rol == "galeria" and a.estado == "OK"]
        esperados = {a.foto.alumno for a in analisis if a.foto.rol == "galeria"}
        presentes = {a.foto.alumno for a in referencias}
        if esperados - presentes:
            raise ValueError(f"Alumnos sin referencias validas: {sorted(esperados - presentes)}")
        return cls([a.descriptor for a in referencias], [a.foto.alumno for a in referencias],
                   [a.foto.ruta for a in referencias])

    def puntuar(self, descriptor, excluir: str | None = None) -> list[tuple[str, float]]:
        vector = normalizar(descriptor)
        if vector.shape != self.vectores.shape[1:]:
            raise ValueError("Descriptor de consulta incompatible con la galeria")
        similitudes = np.clip(self.vectores @ vector, -1, 1)
        salida = []
        for alumno in sorted(set(self.alumnos.tolist())):
            if alumno == excluir:
                continue
            valores = np.sort(similitudes[self.alumnos == alumno])
            salida.append((alumno, float(np.mean(valores[-2:]))))
        return sorted(salida, key=lambda par: (-par[1], par[0]))

    def guardar(self, archivo: Path):
        archivo = Path(archivo)
        archivo.parent.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(archivo, vectores=self.vectores, alumnos=self.alumnos, origenes=self.origenes)


@dataclass(frozen=True)
class Criterio:
    umbral: float
    margen_minimo: float

    def __post_init__(self):
        if not np.isfinite(self.umbral) or not -1 <= self.umbral <= 1:
            raise ValueError("Umbral coseno invalido")
        if not np.isfinite(self.margen_minimo) or not 0 <= self.margen_minimo <= 2:
            raise ValueError("Margen invalido")


def decidir(ranking: list[tuple[str, float]], criterio: Criterio) -> dict:
    base = {"nombre": "NO_CONCLUYENTE", "candidato": None, "similitud": None,
            "margen": None, "aceptado": False, "motivo": "REFERENCIAS_INSUFICIENTES"}
    if len(ranking) < 2:
        return base
    nombre, primero = ranking[0]
    segundo = ranking[1][1]
    if not np.isfinite(primero) or not np.isfinite(segundo):
        return dict(base, motivo="PUNTUACION_INVALIDA")
    margen = primero - segundo
    base.update(candidato=nombre, similitud=primero, margen=margen)
    if primero < criterio.umbral:
        return dict(base, motivo="SIMILITUD_INSUFICIENTE")
    if margen <= 1e-8 or margen < criterio.margen_minimo:
        return dict(base, motivo="IDENTIDADES_AMBIGUAS")
    return dict(base, nombre=nombre, aceptado=True, motivo="ACEPTADO")


def casos_de_rol(analisis: list[Analisis], galeria: Galeria, rol: str) -> list[dict]:
    """Incluye fallos de detección; no los elimina del denominador de evaluación."""
    salida = []
    for a in analisis:
        if a.foto.rol != rol:
            continue
        valido = a.estado == "OK"
        salida.append({"archivo": a.foto.ruta, "real": a.foto.alumno, "estado": a.estado,
                       "ranking": galeria.puntuar(a.descriptor) if valido else [],
                       "ranking_sin_identidad": galeria.puntuar(a.descriptor, excluir=a.foto.alumno)
                       if valido else []})
    return salida


def calibrar(casos: list[dict]) -> tuple[Criterio, dict]:
    """Usar únicamente calibración; negativos: retirar identidad real de la galería.

    Prioriza cero aceptaciones erróneas en este ensayo diminuto, luego aciertos.
    Los negativos simulados no sustituyen personas nuevas de otra sesión.
    """
    validos = [c for c in casos if len(c["ranking"]) >= 2 and len(c["ranking_sin_identidad"]) >= 2]
    if not validos:
        raise ValueError("Faltan consultas validas de calibracion y al menos tres identidades de galeria")
    rankings = [r for c in validos for r in (c["ranking"], c["ranking_sin_identidad"])]
    def candidatos(valores, minimo, maximo):
        puntos = sorted(set([minimo, maximo, *valores]))
        return sorted(set([minimo, maximo, *[(a + b) / 2 for a, b in zip(puntos, puntos[1:])]]))
    umbrales = candidatos([r[0][1] for r in rankings], -1.0, 1.0)
    margenes = candidatos([r[0][1] - r[1][1] for r in rankings], 0.0, 2.0)
    mejor = None
    for umbral in umbrales:
        for margen in margenes:
            criterio = Criterio(umbral, margen)
            aciertos = errores = falsas = 0
            for caso in validos:
                normal = decidir(caso["ranking"], criterio)
                negativo = decidir(caso["ranking_sin_identidad"], criterio)
                aciertos += int(normal["aceptado"] and normal["nombre"] == caso["real"])
                errores += int(normal["aceptado"] and normal["nombre"] != caso["real"])
                falsas += int(negativo["aceptado"])
            clave = (errores + falsas, -aciertos, -umbral, -margen)
            if mejor is None or clave < mejor[0]:
                mejor = (clave, criterio, {"consultas_totales": len(casos),
                                          "consultas_validas": len(validos), "aciertos": aciertos,
                                          "nombres_erroneos": errores, "falsas_aceptaciones_simuladas": falsas})
    return mejor[1], mejor[2]


def evaluar(casos: list[dict], criterio: Criterio) -> tuple[list[dict], dict]:
    filas = []
    for caso in casos:
        decision = decidir(caso["ranking"], criterio)
        if caso["estado"] != "OK":
            decision["motivo"] = caso["estado"]
        negativo = decidir(caso["ranking_sin_identidad"], criterio)
        filas.append({"archivo": caso["archivo"], "real": caso["real"], **decision,
                      "correcto": decision["aceptado"] and decision["nombre"] == caso["real"],
                      "negativo_simulado_valido": len(caso["ranking_sin_identidad"]) >= 2,
                      "negativo_simulado_aceptado": negativo["aceptado"]})
    resumen = {
        "consultas": len(filas), "aciertos": sum(f["correcto"] for f in filas),
        "nombres_erroneos": sum(f["aceptado"] and not f["correcto"] for f in filas),
        "rechazos": sum(not f["aceptado"] for f in filas),
        "negativos_simulados_validos": sum(f["negativo_simulado_valido"] for f in filas),
        "falsas_aceptaciones_simuladas": sum(f["negativo_simulado_aceptado"] for f in filas),
    }
    return filas, resumen
