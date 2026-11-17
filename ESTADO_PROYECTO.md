# Estado del proyecto

Actualizado: **17 de noviembre de 2026**. Resumen técnico compartido; fotografías, modelos, galería y archivos de sesión se preparan localmente y quedan fuera de Git.

## Misión final acordada

1. Escribir el nombre objetivo en A y comenzar con una referencia válida del inicio.
2. Navegar entre obstáculos siguiendo las flechas azules. Ante un muro, una flecha confirmada fija el sentido por encima del lado elegido por la evasión. Mantener parada si falta espacio o sensores; no invertir la señal.
3. Completar 90° por odometría antes de reanudar el avance.
4. Confirmar la señal de meta para declarar B. Solo entonces iniciar un barrido controlado alrededor para buscar a la persona solicitada entre las fotografías.
5. Confirmar su rostro, leer únicamente el QR asociado a esa fotografía y conservar su palabra.
6. Iniciar el regreso con flechas rojas, confirmar la meta de A durante la etapa de vuelta, detenerse y mostrar la palabra.

Interpretar la meta según la etapa: no contar la de A al salir como llegada a B ni la misma de B recién vista como regreso a A. Los criterios de separación/llegada deben concretarse al integrar esta parte.

## Task 1: implementación actual

El usuario confirmó movimiento con evasión y reconocimiento juntos en el robot. No aportó métricas específicas de A, parada, frenado o navegación completa. Los choques diagonales y cortes de recepción anteriores no se consideran resueltos.

`--solo-evasion` y `--solo-reconocimiento` conservan sus pruebas independientes. El modo `--evasion-reconocimiento [--mover]` usa Buscar para fijar el nombre y registra A con odometría válida posterior al inicio, sin inventar el origen ni llamarlo mapa. Su única salida de velocidad está condicionada por actividad, cámara, LiDAR/TF y odometría recientes. Pausa desde el primer acierto facial y confirma con tres imágenes nuevas; luego queda detenido.

**El prototipo todavía reconoce durante la marcha.** La transición por meta B, barrido, asociación rostro-QR, conservación de palabra y retorno automático descritos arriba siguen pendientes. No confundir esta prueba con la misión final.

Se añadieron señales sin modificar Task 2 ni recalibrar la galería:

- `--solo-senales --tramo ida|vuelta`: cámara con detección, siempre sin movimiento ni galería. Ida filtra azul; vuelta, rojo.
- `--evasion-reconocimiento --senales --tramo ida|vuelta [--mover]`: giros opcionales en la misma salida ROS. Se confirma un único cartel en tres imágenes nuevas, reiniciando la cuenta al Buscar. El tramo se elige antes; todavía no cambia automáticamente a vuelta.
- Geometría adaptada del aporte `detector_señales.py` en `task 1/geometria_senales.py`, sin SciPy, QR/meta ni predicciones. El original se conserva como referencia. No se instalan dependencias nuevas.
- `control_giros.py` espera alcanzar los 90° antes de terminar. El margen de 5° limita el exceso; ya no finaliza a 85°. LiDAR/TF insuficientes o distancias <=0.32 m en frente/laterales detienen sin invertir el sentido. Si excede el ángulo permitido o agota 12 s, queda detenido hasta cerrar/revisar.
- Rearme con >=1.5 s de ausencia en imágenes nuevas, ID nuevo y >=0.4 m desde el giro previo. Puede omitir carteles demasiado próximos; ajustar solo tras medir la disposición real. Píxeles de detección no equivalen a metros.

Comandos en bloques 11/12 de `task 1/COMANDOS.txt`. Configuración en `task 1/config/senales_giro.json`. La evasión conserva 0.10 m/s y 0.40 rad/s, precaución 0.45 m, parada del avance 0.32 m y lateral 0.28 m. Los sectores actuales no certifican la huella del robot.

## Evidencia y límites

Pasaron **113 pruebas de Task 1 y 51 de Task 2 (164)**. Cubren referencias de señales, escala/brillo/JPEG, repetición/caducidad, color del tramo, giro por yaw, prioridad de la flecha ante muro, avance bloqueado hasta 90°, obstáculos, timeout, exceso de ángulo, salida única y prioridad de la parada facial. Las diez páginas del PDF producen únicamente los cuatro giros derechos esperados. Izquierda se comprueba con reflejos dentro del test; falta una impresión izquierda real. No se aplica espejo a la cámara.

El asistente no ejecutó ROS/Docker físico ni movimiento. Las señales nuevas siguen pendientes de validar con impresiones, vídeo real y medidas de giro/frenado. La edad de imagen es desde recepción local; no cuantifica un retraso previo del sensor/red. El vigilante no garantiza entregar una parada ante corte de red o muerte del nodo ROS.

La galería local ampliada tiene 74 fotos de 13 personas, 47 referencias válidas, 13 consultas de calibración y 13 de evaluación. Evaluación exploratoria: 13 nombres correctos, cero erróneos/rechazos; 13 negativos simulados rechazados. No demuestra independencia entre tomas ni precisión física general. No se publican fotos, descriptores ni pesos.

## Task 2 e informe

Gestos confirmados en simulación y robot, con vídeo comprimido en panel lateral; conserva retraso sin medir. Esta actualización de señales no modifica sus componentes. Mantener la `.venv` global y el Docker existente; confirmar IP/dominio por visita.

El informe publicado (LaTeX, Markdown y PDF de diez páginas) sigue siendo el borrador del 7 de noviembre. Debe adaptarse al alcance y evidencias posteriores antes de entregar. No se entregó en Canvas.

Siguiente paso: comprobar señales quieto, después giros con movimiento en espacio despejado y demás controles cerrados. Integrar entonces la detección de meta y el supervisor por etapas para habilitar la búsqueda facial únicamente en B, QR asociado y regreso a A. El contrato de misión y pendientes completos están en PLAN_PROYECTO.md.
