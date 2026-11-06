# Task 1: reconocimiento facial en notebook

Primera implementación local terminada el 28 de octubre de 2026. Usa la `.venv` global, OpenCV en CPU y modelos preentrenados YuNet/SFace. Funciona sin ROS, Gazebo ni robot. Task 2 y su código se mantienen en pausa.

**Abrir y ejecutar.** Desde la raíz de Proyecto2:

```bash
code "task 1/01_reconocimiento_rostros.ipynb"
```

Selecciona el kernel de `.venv` en la raíz y ejecuta las celdas en orden. El notebook admite directorio de trabajo en la raíz o en `task 1`. Las dependencias ya están instaladas y los dos modelos se descargaron y verificaron por SHA-256; las siguientes ejecuciones no necesitan internet. Para otra computadora, consultar `requirements.txt` y la celda de preparación de modelos.

El notebook tiene 22 celdas, diez de código. Explica entorno y rutas, modelos, inventario, partición, extracción, galería, calibración, evaluación, consulta configurable y una prueba sin rostro.

**Prueba con tus fotografías.** Los originales están en `rostros/<alumno>/`. La carpeta disponible al implementar contenía 26 imágenes de cinco alumnos; hay fotos nuevas respecto a las 21 de cuatro alumnos de la planificación inicial. El detector encontró una cara útil en cada una. El nombre viene de la etiqueta de carpeta, no se verifica por otro medio.

La partición inicial usa orden de archivo: las últimas dos fotos de cada alumno son calibración y evaluación; las demás son referencias. En esta colección resultaron 16 referencias, cinco consultas de calibración y cinco de evaluación. Los archivos y sus hashes se conservan en `datos/particion_rostros.json`. Son grupos por fotografía, no por sesiones independientes: varias fotos de una misma toma pueden hacer que la prueba resulte optimista.

**Cómo reconoce.** YuNet detecta la cara, se alinea/recorta y SFace produce un descriptor de 128 valores. Se normaliza y se compara mediante similitud coseno. La puntuación por alumno es la media de hasta dos referencias más similares. Solo se acepta si supera el umbral y se separa del segundo candidato; un empate se rechaza. Una similitud no es una probabilidad.

Los dos parámetros se eligen únicamente con calibración. Para ensayar rechazo se puntúa también retirando la identidad correcta de la galería. Eso es un negativo simulado; no demuestra rechazo general de personas nuevas. Las fotos de evaluación no se usan para escoger umbral o margen. Se contabilizan los fallos del detector como rechazos, sin ocultarlos.

**Resultado inicial observado:** cuatro nombres correctos de cinco consultas reservadas, un rechazo NO_CONCLUYENTE y ningún nombre incorrecto. Los cinco negativos simulados de evaluación fueron rechazados. Las cinco consultas de calibración fijaron el umbral en aproximadamente 0.5726 y el margen mínimo en 0.2691. No se relajaron después de ver el caso rechazado. Es un ensayo pequeño dentro de la colección; falta evaluar otras sesiones, personas no inscritas e impresiones vistas por cámara. No interpretar 4/5 como una precisión garantizada fuera de esta prueba.

**Consultar otra foto.** En la sección 9 cambia `CONSULTA`, por ejemplo:

```python
CONSULTA = TAREA_DIR / 'datos' / 'otra_foto.jpg'
```

La salida muestra el nombre aceptado o NO_CONCLUYENTE, el mejor candidato, las puntuaciones y el motivo. Un candidato de una respuesta rechazada no es una identidad aceptada. `MOSTRAR_FOTO=False` evita incrustar imágenes personales en las salidas. Puedes activarlo para revisar la caja de detección; limpia esa salida antes de compartir el notebook.

**Si llegan más alumnos o fotos.** El manifiesto detecta los cambios y evita mezclar silenciosamente nuevas referencias con una evaluación anterior. Revisa el reparto, usa `REGENERAR_PARTICION=True` y repite desde la partición hasta el final; luego vuelve a `False`. Con una foto solo puede haber referencia; hacen falta más tomas para calibrar/evaluar a esa persona. No reutilizar la foto de consulta como su propia referencia ni ajustar los umbrales para arreglar un error del test.

| Archivo/carpeta | Función |
| --- | --- |
| [01_reconocimiento_rostros.ipynb](01_reconocimiento_rostros.ipynb) | Experimento explicado paso a paso |
| [reconocimiento_facial.py](reconocimiento_facial.py) | Datos, partición, extracción, galería y decisión reutilizables |
| [modelos_rostros.py](modelos_rostros.py) | Preparación y comprobación de modelos |
| [modelos/README.md](modelos/README.md) | Procedencia, hashes y licencias |
| `rostros/` | Fotografías originales, fuera de Git |
| `datos/` | Manifiesto, auditoría, galería numérica y criterio, fuera de Git |
| `resultados/` | Evaluación guardada, fuera de Git |
| [COMANDOS.txt](COMANDOS.txt) | Comandos para abrir y comprobar |
| `tests/` | Pruebas del módulo sin ROS |

**Verificación realizada.** Los modelos cargaron y procesaron las 26 fotos en CPU. Se ejecutaron las diez celdas de código mediante IPython, con las imágenes desactivadas. Pasaron 14 pruebas de separación de datos, consistencia del manifiesto, referencias exclusivas, normalización, ambigüedad, calibración, fallos de detección e imagen negra con YuNet real. El notebook se entrega con salidas limpias y no contiene fotos ni descriptores incrustados.

```bash
.venv/bin/python -m unittest discover -s "task 1/tests" -t "task 1" -v
```

**Siguiente etapa.** Probar una foto impresa reservada con webcam, variando iluminación, distancia e inclinación. La misión completa exige navegación autónoma de A a B, reconocimiento, regreso a A y anuncio del nombre. Para navegación siguen pendientes señalización de A/B, geometría, exploración y localización del retorno; SLAM Toolbox/Nav2 son candidatos. Esta implementación facial no elige rutas ni mueve al robot. La conexión física permanece en pausa.

Referencias técnicas: [OpenCV, detección y reconocimiento](https://docs.opencv.org/4.x/d0/dd4/tutorial_dnn_face.html). Alcance y coordinación: [PLAN_PROYECTO.md](../PLAN_PROYECTO.md), [ESTADO_PROYECTO.md](../ESTADO_PROYECTO.md) y [enunciado](../Computer_vision_proyectos.pdf).
