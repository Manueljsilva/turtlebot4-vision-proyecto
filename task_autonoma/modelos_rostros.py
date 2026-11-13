"""Descarga reproducible de los modelos oficiales de OpenCV Zoo para OpenCV 4.x."""

import hashlib
from pathlib import Path
import shutil
import tempfile
from urllib.request import Request, urlopen


MODELOS = {
    "yunet": {
        "archivo": "face_detection_yunet_2023mar.onnx",
        "carpeta": "face_detection_yunet",
        "bytes": 232589,
        "sha256": "8f2383e4dd3cfbb4553ea8718107fc0423210dc964f9f4280604804ed2552fa4",
    },
    "sface": {
        "archivo": "face_recognition_sface_2021dec.onnx",
        "carpeta": "face_recognition_sface",
        "bytes": 38696353,
        "sha256": "0ba9fbfa01b5270c96627c4ef784da859931e02f04419c829e83484087c34e79",
    },
}


def sha256_archivo(ruta: Path) -> str:
    digest = hashlib.sha256()
    with Path(ruta).open("rb") as archivo:
        for bloque in iter(lambda: archivo.read(1024 * 1024), b""):
            digest.update(bloque)
    return digest.hexdigest()


def preparar_modelos(carpeta: Path, permitir_descarga: bool = True) -> dict[str, Path]:
    """Valida archivos existentes y descarga solo los ausentes, sin reemplazar otros."""
    carpeta = Path(carpeta)
    carpeta.mkdir(parents=True, exist_ok=True)
    rutas = {}
    for nombre, datos in MODELOS.items():
        destino = carpeta / datos["archivo"]
        if not destino.exists():
            if not permitir_descarga:
                raise FileNotFoundError(f"Falta {destino}. Preparar con internet antes del laboratorio.")
            url = ("https://media.githubusercontent.com/media/opencv/opencv_zoo/main/models/"
                   f"{datos['carpeta']}/{datos['archivo']}")
            temporal = None
            try:
                with tempfile.NamedTemporaryFile(dir=carpeta, suffix=".part", delete=False) as salida:
                    temporal = Path(salida.name)
                    peticion = Request(url, headers={"User-Agent": "Proyecto-TurtleBot-OpenCV"})
                    with urlopen(peticion, timeout=45) as respuesta:
                        shutil.copyfileobj(respuesta, salida)
                if temporal.stat().st_size != datos["bytes"] or sha256_archivo(temporal) != datos["sha256"]:
                    raise RuntimeError(f"Descarga no valida para {datos['archivo']}; no se instalo.")
                temporal.replace(destino)
            finally:
                if temporal is not None:
                    temporal.unlink(missing_ok=True)
        if destino.stat().st_size != datos["bytes"] or sha256_archivo(destino) != datos["sha256"]:
            raise RuntimeError(f"El modelo local {destino} no coincide con el oficial; no se reemplazo.")
        rutas[nombre] = destino
    return rutas
