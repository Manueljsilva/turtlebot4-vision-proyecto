"""Configuración compartida de percepción y giros habituales/experimentales."""

from dataclasses import asdict, dataclass
import json
from math import isfinite
from pathlib import Path


@dataclass(frozen=True)
class ConfigSenales:
    ancho: int = 640
    diametro_minimo_px: int = 55
    confirmaciones: int = 1
    ausencia_rearme_s: float = 1.5
    angulo_giro_deg: float = 90.0
    tolerancia_deg: float = 5.0
    tiempo_maximo_giro_s: float = 12.0
    desplazamiento_rearme_m: float = 0.4
    esperar_pared_m: float = 0.0
    tiempo_espera_pared_s: float = 6.0

    def __post_init__(self):
        for k, v in asdict(self).items():
            if type(v) not in (int, float) or not isfinite(v):
                raise ValueError(f'{k} debe ser finito')
        if (type(self.ancho) is not int or not 320 <= self.ancho <= 960
                or type(self.diametro_minimo_px) is not int
                or not 30 <= self.diametro_minimo_px <= self.ancho // 2
                or type(self.confirmaciones) is not int or not 1 <= self.confirmaciones <= 10):
            raise ValueError('Ancho, diámetro o confirmaciones fuera de rango')
        if not (1 <= self.ausencia_rearme_s <= 5 and 45 <= self.angulo_giro_deg <= 120
                and 1 <= self.tolerancia_deg <= 10 and 6 <= self.tiempo_maximo_giro_s <= 20
                and .3 <= self.desplazamiento_rearme_m <= 1):
            raise ValueError('Parámetros de giro/rearme fuera de rango')
        if not (0 <= self.esperar_pared_m <= 1.5 and 1 <= self.tiempo_espera_pared_s <= 20):
            raise ValueError('Parámetros de espera de pared fuera de rango')


def cargar_config_senales(ruta):
    d = json.loads(Path(ruta).read_text(encoding='utf-8'))
    if not isinstance(d, dict):
        raise ValueError('La configuración de señales debe ser un objeto')
    return ConfigSenales(**d)
