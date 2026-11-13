"""Lógica pura de misión A -> B -> Reconocimiento -> A y carga de galería facial."""

from dataclasses import dataclass
from enum import Enum
import json
from pathlib import Path
import cv2
import numpy as np

from modelos_rostros import preparar_modelos
from reconocimiento_facial import Criterio, ExtractorFacial, Galeria, decidir


class EstadoMision(str, Enum):
    INICIO_A = "INICIO_A"
    EXPLORANDO_HACIA_B = "EXPLORANDO_HACIA_B"
    RECONOCIENDO_B = "RECONOCIENDO_B"
    RETORNANDO_A = "RETORNANDO_A"
    FINALIZADO = "FINALIZADO"
    FALLO = "FALLO"


@dataclass(frozen=True)
class SistemaFacial:
    extractor: ExtractorFacial
    galeria: Galeria
    criterio: Criterio


def cargar_config_mision(ruta_config: Path | None = None) -> dict:
    ruta = Path(ruta_config) if ruta_config else Path(__file__).resolve().parent / "config" / "mision_config.json"
    return json.loads(ruta.read_text(encoding="utf-8"))


def aplicar_clahe(imagen: np.ndarray) -> np.ndarray:
    """Mejora adaptativa de contraste para compensar iluminación del laboratorio."""
    lab = cv2.cvtColor(imagen, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    cl = clahe.apply(l)
    return cv2.cvtColor(cv2.merge((cl, a, b)), cv2.COLOR_LAB2BGR)


def cargar_sistema_facial(dir_datos: Path, dir_modelos: Path) -> SistemaFacial:
    """Carga modelos YuNet/SFace locales, galería (.npz) y criterio calibrado (.json)."""
    dir_datos = Path(dir_datos)
    dir_modelos = Path(dir_modelos)

    ruta_galeria = dir_datos / "galeria.npz"
    ruta_criterio = dir_datos / "criterio.json"
    if not ruta_galeria.exists():
        raise FileNotFoundError(f"Falta la galería compilada: {ruta_galeria}")
    if not ruta_criterio.exists():
        raise FileNotFoundError(f"Falta el criterio calibrado: {ruta_criterio}")

    datos_criterio = json.loads(ruta_criterio.read_text(encoding="utf-8"))
    criterio = Criterio(
        umbral=float(datos_criterio["criterio"]["umbral"]),
        margen_minimo=float(datos_criterio["criterio"]["margen_minimo"]),
    )
    ext_cfg = datos_criterio.get("extractor", {})
    rutas_modelos = preparar_modelos(dir_modelos, permitir_descarga=False)
    extractor = ExtractorFacial(
        rutas=rutas_modelos,
        lado_maximo=int(ext_cfg.get("lado_maximo", 960)),
        deteccion_minima=float(ext_cfg.get("deteccion_minima", 0.75)),
        cara_minima=int(ext_cfg.get("cara_minima", 28)),
    )

    with np.load(ruta_galeria, allow_pickle=False) as npz:
        galeria = Galeria(
            vectores=npz["vectores"],
            alumnos=npz["alumnos"],
            origenes=npz["origenes"] if "origenes" in npz else None,
        )

    return SistemaFacial(extractor=extractor, galeria=galeria, criterio=criterio)


def evaluar_frame_facial(frame: np.ndarray, sistema: SistemaFacial, usar_clahe: bool = True) -> dict:
    """Analiza un frame y retorna la decisión de reconocimiento facial."""
    imagen = aplicar_clahe(frame) if usar_clahe else frame
    consulta = sistema.extractor.analizar_imagen(imagen)
    if consulta.estado != "OK" or consulta.descriptor is None:
        return {
            "aceptado": False,
            "nombre": "NO_CONCLUYENTE",
            "similitud": None,
            "motivo": consulta.estado,
        }
    ranking = sistema.galeria.puntuar(consulta.descriptor)
    return decidir(ranking, sistema.criterio)


def votar_identidad(decisiones: list[dict], min_votos: int = 2) -> dict:
    """Consolida una ráfaga de frames por mayoría y desempate por mayor similitud."""
    aceptadas = [d for d in decisiones if d.get("aceptado") and d.get("nombre")]
    if not aceptadas:
        return {"nombre": "NO_CONCLUYENTE", "votos": 0, "total": len(decisiones)}

    conteo: dict[str, list[float]] = {}
    for d in aceptadas:
        nombre = str(d["nombre"])
        sim = float(d.get("similitud") or 0.0)
        conteo.setdefault(nombre, []).append(sim)

    ganador, sims = max(
        conteo.items(),
        key=lambda par: (len(par[1]), sum(par[1]) / len(par[1])),
    )
    if len(sims) < min_votos:
        return {
            "nombre": "NO_CONCLUYENTE",
            "candidato": ganador,
            "votos": len(sims),
            "total": len(decisiones),
        }
    return {
        "nombre": ganador,
        "votos": len(sims),
        "similitud_media": round(sum(sims) / len(sims), 4),
        "total": len(decisiones),
    }
