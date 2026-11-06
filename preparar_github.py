"""Exporta el proyecto respetando .gitignore y sin salidas de notebooks.

Los originales, sus ejecuciones y el entorno virtual se conservan intactos.
La copia se crea fuera del proyecto, en /tmp por defecto.
"""

import argparse
import json
from pathlib import Path
import shutil
import subprocess
import tempfile


RAIZ = Path(__file__).resolve().parent


def preparar(destino=None):
    if destino is None:
        destino = Path(tempfile.mkdtemp(prefix="proyecto2-github-"))
    else:
        destino = Path(destino).expanduser().resolve()
        if destino == RAIZ or RAIZ in destino.parents:
            raise ValueError("El destino debe estar fuera de la carpeta del proyecto")
        destino.mkdir(parents=True, exist_ok=False)

    # Git temporal solo para interpretar .gitignore; no modifica .git del proyecto.
    with tempfile.TemporaryDirectory(prefix="proyecto2-indice-") as temporal:
        subprocess.run(["git", "init", "--quiet", temporal], check=True)
        resultado = subprocess.run([
            "git", f"--git-dir={temporal}/.git", f"--work-tree={RAIZ}",
            "-c", "core.excludesFile=/dev/null", "ls-files", "--others",
            "--exclude-standard", "-z",
        ], check=True, capture_output=True)

    archivos = [Path(nombre.decode()) for nombre in resultado.stdout.split(b"\0") if nombre]
    notebooks = 0
    privados = {".git", ".aws", ".agents", ".codex", ".venv", "venv", "env",
                "rostros", "datos", "capturas", "resultados", "__pycache__",
                ".ipynb_checkpoints"}
    for relativa in archivos:
        origen = RAIZ / relativa
        if (origen.is_symlink() or privados.intersection(relativa.parts)
                or origen.suffix.lower() in {".onnx", ".task", ".log"}):
            raise ValueError(f"Archivo local inesperado en la exportación: {relativa}")
        salida = destino / relativa
        salida.parent.mkdir(parents=True, exist_ok=True)
        if origen.suffix == ".ipynb":
            notebook = json.loads(origen.read_text())
            for celda in notebook["cells"]:
                if celda.get("attachments"):
                    raise ValueError(f"Revisar adjuntos incrustados en {relativa}")
                if celda["cell_type"] == "code":
                    celda["outputs"] = []
                    celda["execution_count"] = None
                    celda.get("metadata", {}).pop("execution", None)
            notebook.get("metadata", {}).pop("widgets", None)
            salida.write_text(json.dumps(notebook, ensure_ascii=False, indent=1) + "\n")
            notebooks += 1
        else:
            shutil.copy2(origen, salida)
    return {"destino": str(destino), "archivos": len(archivos),
            "notebooks_sin_salidas": notebooks}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--destino", help="Carpeta nueva fuera del proyecto; por defecto /tmp")
    args = parser.parse_args()
    print(json.dumps(preparar(args.destino), ensure_ascii=False, indent=2))
