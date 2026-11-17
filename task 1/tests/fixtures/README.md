# Referencias de señales

Los dos PNG se recortaron de las flechas superiores de las páginas 1 (azul) y 3
(rojo) del PDF local `señales/señales.pdf`, aportado por el usuario el 17 de
noviembre de 2026. Se usan únicamente para regresión visual, sin entrenar modelos.
El PDF original y `detector_señales.py` se conservan intactos.

Si los recortes no están disponibles en una copia del proyecto, las dos pruebas
que los necesitan se omiten explícitamente. Las pruebas de control y los
negativos generados localmente siguen ejecutándose, sin descargar documentos.

El PDF solo presenta flechas derechas. Las izquierdas de las pruebas se obtienen
por reflexión **solo en el test**, no en la cámara del robot. Las pruebas de
escala, brillo y JPEG no demuestran reconocimiento físico ni perspectivas reales.
