"""Controlador original verificado y ajuste local del ensayo, sin sustituir el habitual."""

import hashlib
import importlib.util
from pathlib import Path
import sys

from config_senales import cargar_config_senales


SNAPSHOT = 'e78c3b52d07be0add80103365bb2ad5fc068e91b'
CARPETA = Path(__file__).resolve().parent / 'experimentos/mejora_competicion'
CONTROL = CARPETA / 'control_giros.py'
CONFIG = Path(__file__).resolve().parent / 'config/senales_giro_ensayo.json'
HASHES = {
    'control_giros.py': '14a9a10e20c9a742a09052a949b85bb7775b7d92863134d1836e27b9c42fdd44',
    'senales_giro.json': '163c604919c932e85b87b984a2a0e734d7f236f79c2ff38767433472955e36e1',
}


def diagnostico_senales(resultado, inicio_ns, ahora_ns, enviado=None):
    """Observabilidad de la confirmación y envío, sin crear ni renovar órdenes."""
    resultado = resultado or {}
    evento = resultado.get('evento')
    return dict(iniciada=inicio_ns is not None, estado=resultado.get('estado', 'Sin resultado'),
        confirmaciones=resultado.get('confirmaciones', 0),
        evento_id=evento['id'] if evento else None,
        evento_color=evento['color'] if evento else None,
        evento_direccion=evento['direccion'] if evento else None,
        evento_posterior_a_buscar=bool(evento and inicio_ns is not None and evento['marca_ns'] > inicio_ns),
        edad_evento_s=(ahora_ns-evento['marca_ns'])/1e9 if evento else None,
        ultimo_id_enviado=enviado['id'] if enviado else None,
        edad_ultimo_evento_enviado_s=(ahora_ns-enviado['marca_ns'])/1e9 if enviado else None)


def cargar_ensayo(carpeta=CARPETA, config=CONFIG):
    """Verifica el snapshot y carga el perfil local usado también en Docker."""
    carpeta = Path(carpeta)
    for nombre, esperado in HASHES.items():
        if hashlib.sha256((carpeta / nombre).read_bytes()).hexdigest() != esperado:
            raise ValueError(f'El ensayo del compañero no coincide con {SNAPSHOT[:7]}: {nombre}')
    nombre_modulo = '_ensayo_giros_companero_e78c3b5'
    spec = importlib.util.spec_from_file_location(nombre_modulo, carpeta / 'control_giros.py')
    modulo = importlib.util.module_from_spec(spec)
    sys.modules[nombre_modulo] = modulo  # dataclass necesita resolver su módulo.
    try:
        spec.loader.exec_module(modulo)
        cfg = cargar_config_senales(config)
    except Exception:
        sys.modules.pop(nombre_modulo, None)
        raise
    return modulo, cfg
