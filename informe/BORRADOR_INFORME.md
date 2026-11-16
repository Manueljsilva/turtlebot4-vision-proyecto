# Navegación autónoma y control gestual con reconocimiento facial en TurtleBot 4 Lite

**Borrador de trabajo - 7 de noviembre de 2026**

- Curso: Visión por Computadora.
- Integrantes y códigos: **[POR COMPLETAR: los cuatro integrantes]**.
- Docente: Rensso Victor Hugo Mora Colque.
- Fecha de entrega: **[POR CONFIRMAR: plazo anunciado en clase]**.

> Este borrador contiene metodología implementada y resultados parciales comprobados. Las secciones pendientes deben completarse después de las pruebas correspondientes. La estructura es una propuesta del equipo, no una plantilla obligatoria del profesor.

## 1. Introducción y objetivo

El proyecto propone integrar visión por computadora y control de un TurtleBot 4 Lite para recorrer un circuito, identificar a una persona a partir de una fotografía y regresar al punto de partida. El reto permite estudiar la relación entre percepción, comunicación y ejecución física: una clasificación visual correcta debe convertirse en una orden de movimiento o en una decisión de identidad utilizable por el sistema.

El objetivo general es completar la secuencia A → B → reconocimiento de la fotografía en B → regreso a A → anuncio del nombre. El enunciado exige dos modalidades: navegación autónoma (Task A, carpeta `task 1/`) y navegación mediante gestos visuales (Task B, carpeta `task 2/`). La configuración final del circuito y la fotografía se revelan el día de la competición.

Los objetivos específicos son construir una galería facial etiquetada, reconocer órdenes visuales humanas, comunicar esas órdenes al robot y coordinar las etapas de la misión. La navegación autónoma y la integración completa del reconocimiento con el recorrido están pendientes.

## 2. Requisitos del proyecto

La fuente de los requisitos es el [enunciado del curso](../Computer_vision_proyectos.pdf). El informe y las diapositivas deben entregarse en Canvas antes del plazo indicado en clase. El informe debe explicar arquitectura, algoritmos de visión, navegación, decisiones, limitaciones y mejoras; los fragmentos extensos de código corresponden al apéndice.

| Modalidad | Requisito de navegación | Condición de éxito |
| --- | --- | --- |
| Task A | Navegación autónoma, sin intervención después del inicio | Llegar a B, identificar, regresar a A e indicar el nombre correcto |
| Task B | Navegación guiada únicamente por señales visuales humanas | Completar la misma misión mediante gestos |

El enunciado permite modelos preentrenados y procesamiento externo, a bordo o híbrido. La presentación dura entre 8 y 10 minutos y se requiere un video de la competición que muestre ambas modalidades. La rúbrica asigna 4 puntos al informe/documentación, 4 a la presentación, 3 a las preguntas y 9 a la competición.

**Pendiente de confirmar con el profesor:** fecha exacta, señalización de A/B, condiciones de inspección previa, ubicación/movilidad del operador de gestos y formato del anuncio final. El PDF no establece una extensión ni un formato de archivo obligatorio para el informe.

## 3. Arquitectura y entorno

Se separó la percepción del control para probar los componentes antes de disponer del robot. La laptop ejecuta visión en CPU con Python, OpenCV y MediaPipe. ROS 2 Jazzy y Gazebo se ejecutan en el contenedor existente `proyecto2-sim`. Los notebooks facilitan el análisis y la explicación; los programas de ejecución conectan la percepción con ROS.

| Componente | Función | Estado al elaborar este borrador |
| --- | --- | --- |
| Webcam de la laptop | Capturar los gestos del operador | Implementado |
| MediaPipe y reglas propias | Obtener puntos de la mano y clasificarlos | Implementado y probado localmente |
| Filtro temporal y puente ROS | Confirmar órdenes y publicar velocidades | Simulación confirmada; hubo movimiento físico en una versión previa |
| YuNet y SFace | Detectar, describir y comparar rostros | Implementado y evaluado con fotografías locales |
| Cámara del robot | Aportar la imagen de la fotografía en B | Recepción aislada en Pi observada; panel de laptop pendiente |
| Navegación autónoma | Encontrar B y recuperar A | Pendiente |
| Supervisor de misión | Coordinar llegada, reconocimiento, retorno y anuncio | Pendiente |

El flujo gestual actual es webcam → detección de mano → reglas y filtro → JSON por tubería → puente ROS en Docker → `/cmd_vel`. Se utiliza `geometry_msgs/msg/TwistStamped`. ROS 2 organiza la comunicación mediante publicadores y suscriptores asociados a tópicos y tipos de mensaje. [Documentación de ROS 2](https://github.com/ros2/ros2_documentation/blob/jazzy/source/Concepts/Basic/About-Topics.rst).

El reconocimiento facial funciona por separado: fotografía → detección/alineación → descriptor → comparación con la galería → nombre aceptado o NO_CONCLUYENTE. Su entrada desde la cámara del robot y la conservación del nombre hasta A todavía deben integrarse.

## 4. Metodología de reconocimiento facial

Se eligieron modelos preentrenados para desarrollar una primera solución en CPU sin entrenar una red con la pequeña colección disponible. YuNet localiza el rostro y SFace produce una representación para comparar identidades. La API de OpenCV permite alinear y recortar la cara antes de extraer sus características. [Referencia de OpenCV](https://docs.opencv.org/4.x/d0/dd4/tutorial_dnn_face.html).

### 4.1. Datos y separación

La colección local contiene **26 fotografías de cinco alumnos**. Las etiquetas proceden de las carpetas. Se verificaron duplicados exactos y se conservó un manifiesto con rutas, hashes y asignación de cada foto:

| Uso | Fotografías |
| --- | ---: |
| Galería de referencia | 16 |
| Calibración | 5 |
| Evaluación reservada | 5 |

La separación inicial es por archivo, no por sesiones independientes. Fotografías cercanas de una misma toma pueden producir una evaluación optimista. Las consultas reservadas no se utilizan como referencias ni para ajustar el criterio después de observar sus resultados.

### 4.2. Extracción y decisión

La implementación limita el tamaño de procesamiento a 960 píxeles por lado y requiere exactamente una cara útil. Configura confianza mínima de detección de 0.85 y tamaño mínimo de cara de 32 píxeles. SFace genera un descriptor normalizado de 128 valores.

La puntuación de una identidad es la media de sus dos referencias más parecidas, o de una si solo dispone de una referencia válida. Se acepta una identidad cuando la similitud coseno y el margen frente al segundo candidato superan los límites calibrados. Para este conjunto se obtuvieron **0.5726** de similitud mínima y **0.2691** de margen mínimo. Son puntuaciones, no probabilidades.

Si no hay una cara útil, hay varias caras o la evidencia de identidad es insuficiente, se devuelve NO_CONCLUYENTE. La consulta externa de la sección 11 del notebook permite ensayar otra imagen con ese mismo criterio, sin incorporarla a la galería ni recalibrar.

## 5. Metodología de gestos y control

MediaPipe Hand Landmarker proporciona 21 puntos de la mano. Sobre esos puntos se construyó un clasificador propio; el modelo de MediaPipe no decide directamente las órdenes del robot. [Documentación de MediaPipe](https://developers.google.com/edge/mediapipe/solutions/vision/hand_landmarker).

Las coordenadas se convierten a píxeles para respetar la geometría de imágenes rectangulares. Las reglas comparan ángulos y distancias de los dedos para distinguir extensión y flexión. Se utiliza imagen sin espejo y la siguiente convención:

| Gesto | Orden |
| --- | --- |
| Índice hacia arriba y otros tres dedos largos flexionados | Avanzar |
| Índice hacia la derecha del operador, frente a la webcam | Girar a la derecha del robot |
| Índice hacia la izquierda del operador, frente a la webcam | Girar a la izquierda del robot |
| Palma, ausencia, ambigüedad o más de una mano | Detener |

Los giros son continuos sobre el sitio. Se exige una confirmación de 0.4 s y las órdenes caducan después de 0.5 s. Un cambio de dirección pasa primero por detenerse. El puente mantiene una vigilancia temporal independiente de la llegada de nuevas imágenes; el reloj monotónico es compartido por el host y su contenedor.

Los valores configurados actualmente son 0.306 m/s para avanzar y ±1.90 rad/s para girar. El movimiento gestual se confirmó físicamente antes de los últimos ajustes de velocidad y sentido; esos ajustes requieren nueva comprobación y medición del frenado. La parada por caducidad no garantiza detener el robot si muere el propio proceso ROS o se corta la red. No se implementó una barrera protectora basada en LiDAR.

Los ensayos con nuevos gestos en la rama del compañero se mantienen separados de esta versión.

## 6. Navegación autónoma y coordinación de misión

**PENDIENTE DE IMPLEMENTACIÓN Y VALIDACIÓN.**

La estrategia debe encontrar B en una configuración no conocida y permitir el regreso a A. Se debe definir cómo se detectan ambas zonas, cómo se evitan obstáculos y cómo se conserva la información necesaria para el retorno.

SLAM, exploración y planificación corresponden a funciones distintas; la elección de herramientas debe responder a la geometría y reglas confirmadas. Todavía no existe una estrategia autónoma validada. En la versión final de esta sección se deberá justificar el método elegido, describir sus entradas y salidas y mostrar resultados con distintos circuitos.

Para ambas modalidades falta coordinar el reconocimiento en B, conservar la identidad aceptada y anunciarla al regresar a A. En Task B las órdenes de navegación deben seguir procediendo de señales visuales humanas.

## 7. Resultados parciales y alcance de las pruebas

### 7.1. Reconocimiento facial local

Los archivos locales `task 1/datos/criterio.json` y `task 1/resultados/evaluacion.json` registran:

| Resultado de evaluación | Cantidad |
| --- | ---: |
| Consultas reservadas | 5 |
| Nombres correctos | 4 |
| Rechazos | 1 |
| Nombres incorrectos | 0 |

Se rechazaron además los cinco negativos simulados obtenidos al retirar la identidad real de la galería. Este procedimiento no reemplaza probar personas desconocidas independientes. Los resultados son exploratorios y no permiten estimar una precisión general con impresiones vistas por la cámara del robot.

Pasaron 14 pruebas del módulo facial. La nueva consulta se comprobó con archivo ausente, una fotografía reservada y una imagen negra; esta última produjo NO_CONCLUYENTE/SIN_ROSTRO. Las pruebas de software no equivalen a ensayos de misión.

Los resultados guardados en la rama del compañero usan otra colección de 32 fotos y seis alumnos. Se documentan aparte y no se mezclan con la evaluación local anterior.

### 7.2. Control, simulación y robot físico

| Evidencia disponible | Alcance y límite |
| --- | --- |
| Control por gestos confirmado en Gazebo | Valida la integración en simulación; no sustituye las pruebas físicas |
| Movimiento físico por teclado y por gestos confirmado por el usuario | Comprobación parcial de conexión/control, sin misión completa registrada |
| 51 pruebas locales de Task B superadas durante el desarrollo | Casos de lógica, protocolos, paradas y transporte de video; no son 51 intentos con robot |
| Un ensayo aislado recibió aproximadamente 10 mensajes de imagen/s en la Raspberry Pi | No demuestra video visible ni latencia extremo a extremo en la laptop |
| Ensayo final de cámara informado como fallido | Requiere otra visita para localizar el punto de fallo |

Los registros mostraron fallos de disponibilidad/arranque de OAK-D y un antecedente de baja tensión. No se ha aislado una causa definitiva. Falta comprobar estabilidad con el servicio normal, recepción en Docker y visualización en el panel.

## 8. Limitaciones y plan de validación

Las principales limitaciones actuales son la colección facial pequeña, la separación por fotografías de tomas próximas, la falta de validación con impresiones, el acceso limitado al robot y la integración de misión incompleta. La selección de modelos respondió a disponibilidad y ejecución en CPU; todavía no se realizó una comparación experimental sistemática con alternativas.

Se propone completar las siguientes comprobaciones:

1. Estabilizar la entrada de cámara y medir frecuencia, antigüedad de imagen y pérdidas.
2. Evaluar impresiones reservadas con distintas distancias, inclinaciones y condiciones de iluminación.
3. Medir aciertos y confusiones de gestos con diferentes operadores y comprobar el tiempo real de parada.
4. Implementar y validar la navegación autónoma en configuraciones variadas.
5. Integrar reconocimiento, retorno y anuncio en ambas modalidades.
6. Registrar los intentos completos y conservar también los fallos.

Tabla propuesta para las pruebas futuras; no contiene resultados ya medidos:

| Intento | Modalidad | Circuito/foto | Llegada a B | Identidad | Regreso a A | Tiempo | Colisiones | Intervenciones |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| Por realizar | A | Por definir | Pendiente | Pendiente | Pendiente | Pendiente | Pendiente | Pendiente |
| Por realizar | B | Por definir | Pendiente | Pendiente | Pendiente | Pendiente | Pendiente | Pendiente |

## 9. Conclusiones provisionales

El desarrollo permitió separar y probar los componentes de percepción y control antes de disponer continuamente del robot. Existe un reconocimiento facial local con criterio de rechazo y un control gestual conectado con ROS, comprobado en simulación y parcialmente en hardware.

La principal etapa pendiente es demostrar la misión completa en ambas modalidades. La evaluación final deberá añadir evidencia de navegación autónoma, reconocimiento de la impresión desde el robot, retorno, anuncio correcto y tiempos de ejecución con penalizaciones.

## 10. Referencias y evidencia del proyecto

- Mora Colque, R. V. H. *Attention is not all you need for robots - Final project*, 13 de octubre de 2026. [Enunciado local](../Computer_vision_proyectos.pdf), apartados Deliverables/Submission y Rubric.
- OpenCV. [DNN-based Face Detection and Recognition](https://docs.opencv.org/4.x/d0/dd4/tutorial_dnn_face.html). Referencia de detección, alineación y reconocimiento.
- Google AI Edge. [Hand Landmarker](https://developers.google.com/edge/mediapipe/solutions/vision/hand_landmarker). Referencia de los puntos de la mano.
- ROS 2. [Topics, documentación Jazzy](https://github.com/ros2/ros2_documentation/blob/jazzy/source/Concepts/Basic/About-Topics.rst). Comunicación mediante tópicos.
- Evidencia reproducible del equipo: [notebook facial](<../task 1/01_reconocimiento_rostros.ipynb>), [módulo facial](<../task 1/reconocimiento_facial.py>), [reglas gestuales](<../task 2/gestos.py>) y [puente ROS](<../task 2/puente_gestos_ros.py>). Los archivos de datos/resultados personales se conservan localmente, fuera de Git.

## Apéndice pendiente

Reservar este espacio para versiones finales de dependencias, parámetros de navegación, fragmentos de código estrictamente necesarios y registros completos de experimentos. El cuerpo del informe debe explicar las decisiones técnicas y su evidencia.

**Antes de entregar:** completar autores y fecha; sustituir las secciones pendientes por lo efectivamente implementado; actualizar resultados, conclusiones y referencias; preparar las diapositivas y el video oficial; revisar el formato solicitado en clase.
