# Plan de dos semanas con TurtleBot 4 Lite

Actualizado el 5 de noviembre de 2026. Alcance basado en las cinco páginas de [Computer_vision_proyectos.pdf](Computer_vision_proyectos.pdf), revisadas el 28 de octubre, enunciado fechado el 13 de octubre de 2026. Este documento sustituye la interpretación anterior de una foto inicial y varias candidatas al final. Las propuestas técnicas siguientes se distinguen de las reglas oficiales.


**Organización vigente.** `task 1/` corresponde a Task A (navegación autónoma): hay reconocimiento facial local implementado y una base parcial en `task 1/rostros/`; navegación aún sin implementación. `task 2/` corresponde a Task B (gestos) y contiene código, notebook local, modelo, pruebas, dependencias e instrucciones. `.venv`, `.gitignore`, enunciado y documentos de contexto permanecen en la raíz. El 5 de noviembre el usuario confirmó SSH, LiDAR desde Docker y movimiento real por teclado. El usuario confirmó gestos con el robot real y después pidió velocidad máxima e invertir los giros: ajustes implementados con pruebas locales y pendientes de nueva comprobación física. Los comandos comunes están en [COMANDOS_LAB.txt](COMANDOS_LAB.txt); las pruebas gestuales físicas, en bloques 9/10 de `task 2/COMANDOS.txt`. El detector local sigue disponible mediante `.venv/bin/python "task 2/webcam_gestos.py"`, sin Gazebo ni ROS.
**Avance del 28 de octubre.** El usuario dejó Task 2 de lado y autorizó el primer notebook facial de Task 1. `task 1/01_reconocimiento_rostros.ipynb` implementa el flujo YuNet/SFace, una partición fija, calibración y evaluación separadas y consulta de una foto. La base creció a 26 fotos de cinco alumnos; hay 16 referencias, cinco consultas de calibración y cinco de evaluación. Resultado inicial: cuatro nombres correctos, un rechazo, cero nombres incorrectos. Es una prueba exploratoria dentro de una colección pequeña; faltan otras sesiones, desconocidos reales e impresiones por cámara. Dos modelos oficiales preparados con hash verificado; sin instalar paquetes ni tocar Task 2. La navegación debe avanzar como módulo separado; la detección de B y la estrategia final siguen pendientes. Uso y límites en [task 1/README.md](<task 1/README.md>).

**Prioridad del 2 de noviembre.** Se retomó Task 2 para preparar una visita y ajustar avance/giro mediante `task 2/control_gestos.py`; se documentaron ubicación, reinicio y máximos técnicos. El 5 de noviembre el usuario pidió máximos: ahora son 0.306 m/s y ±1.90 rad/s, compartidos por simulación y hardware.

Después se preparó [la guía de primera conexión](<task 2/PRIMERA_PRUEBA_ROBOT.md>) a partir del repositorio del profesor. El 5 de noviembre se confirmaron robot .100, dominio 5, red host de Docker, mensaje stamped y LiDAR remoto tras corregir hora/reiniciar ROS. Se conservó esa secuencia en los comandos globales. El movimiento por teclado y gestos está confirmado; faltan comprobar los ajustes más recientes, medir parada y realizar el recorrido completo con reconocimiento.

Cuatro integrantes, aproximadamente dos semanas disponibles y acceso limitado al robot. Fecha exacta de entrega pendiente; la fecha de la portada del PDF no es una fecha de entrega. El usuario prefiere Ubuntu y Jupyter local, con unos 13 GB de RAM. Estado en `ESTADO_PROYECTO.md`; conexión real en `COMANDOS_LAB.txt`, simulación en `task 2/COMANDOS.txt`.

**Objetivo oficial compartido.** Salir de A, recorrer un circuito con obstáculos hasta B, analizar con la cámara del robot la fotografía de una persona situada en B, identificarla, volver a A e indicar su nombre. El éxito exige retorno a A e identificación correcta. El enunciado no pide recordar una fotografía en A, seleccionar entre varios carteles en B ni acercarse a una foto elegida entre distractores.

| Modalidad oficial | Navegación durante el intento | Reconocimiento y final |
| --- | --- | --- |
| Task A | Completamente autónoma, evitando obstáculos; sin intervención humana después del inicio | Identificar en B, regresar autónomamente a A e indicar el nombre |
| Task B | Guiada únicamente por gestos y movimientos humanos capturados en video; sin teclado, ratón, joystick ni otros controles manuales | Identificar en B, regresar a A mediante gestos e indicar el nombre |

La configuración final y la fotografía concreta solo se revelarán el día de la competición. Diseñar para configuraciones no vistas; no basar la solución en una trayectoria memorizada. El PDF no define exactamente cómo se señalizan A/B ni las condiciones de observación o inspección previa. No asumir un mapa final disponible antes de competir. Tampoco asegura que el circuito sea un laberinto de cartón: ese material proviene de la explicación previa del usuario.

**Reglas y entregables confirmados.** Se permiten expresamente herramientas, bibliotecas y modelos preentrenados, y procesamiento a bordo, en computadora externa por red o híbrido. En Task A, tocar el robot, modificar el entorno o interactuar con el sistema de ejecución después del inicio termina el intento. En Task B, la interacción durante el intento se restringe a señales visuales; tocar el robot, cambiar el entorno o utilizar controles manuales termina el intento. Las pruebas de desarrollo con teclado siguen siendo útiles, pero no son el control de la competición.

Cada equipo dispone de hasta tres intentos oficiales por tarea; cuenta el mejor. Habrá un tiempo máximo todavía no anunciado. Excederlo da cero puntos de competición para ese intento. Colisiones y errores de identificación tienen penalizaciones; sus valores exactos se anunciarán en laboratorio. La competición aporta hasta 9 puntos, 4.5 por tarea, con puntuación relativa al tiempo incluyendo penalizaciones. Los otros 11 puntos corresponden a informe/documentación (4), presentación (4) y preguntas (3). Registrar fiabilidad y tiempo; completar el objetivo con seguridad es la primera meta.

Se exige informe metodológico y diapositivas en Canvas antes del plazo anunciado en clase; sin entrega no se puede presentar. La exposición dura 8–10 minutos por grupo y debe justificar arquitectura, modelos, visión, navegación, integración de sensores, decisiones, resultados y limitaciones. El código detallado puede ir en un apéndice. También se exige un video de la competición que cubra ambas tareas y se suba al Drive del curso. No se ha recibido la fecha exacta ni se ha abierto ese Drive.

**Preguntas pendientes para el profesor.** Priorizar las que determinan navegación y operación:

1. Cómo se reconocen las zonas A y B y qué tolerancia define llegada y retorno; qué geometría, cruces o bucles puede tener el circuito y si se permite inspección previa.
2. Dónde puede estar la cámara de gestos: robot, laptop u otra ubicación; si la persona puede acompañar al robot y cómo se mantiene visible durante ida y vuelta. Permitir computadora externa no define por sí solo la cámara admitida.
3. Qué interacciones visuales se necesitan para iniciar reconocimiento, continuar el retorno y finalizar Task B; si se admite una parada protectora automática y cuáles son sus límites. No añadir navegación o giros autónomos a esta modalidad.
4. Cómo debe indicarse el nombre al volver a A: pantalla, texto, voz u otro formato. El PDF no obliga expresamente a síntesis de voz.
5. Completar los alumnos que faltan en la base etiquetada y aclarar identidades posibles/condiciones de impresión. Ya hay 26 fotos de cinco alumnos en `task 1/rostros/`; el PDF no detalla esa galería.
6. Fecha límite, tiempo máximo por intento y penalizaciones exactas; protocolo para comenzar cada intento.

**Plataforma y avance que se conserva.** Python y OpenCV para visión, MediaPipe Hand Landmarker para puntos de la mano, ROS 2 Jazzy y Gazebo con el simulador de TurtleBot 4. Propuesta facial previa: YuNet para detectar y SFace para describir rostros. Su primera ejecución y evaluación local ya se realizaron; sigue pendiente validación con otras sesiones y fotografías vistas por cámara. El PDF permite modelos preentrenados y ejecución en laptop, por lo que esas decisiones de arquitectura son compatibles con el alcance escrito.

El host inspeccionado usa Ubuntu 26.04 y la `.venv` existente usa Python 3.14.4. ROS Jazzy está en el contenedor Ubuntu 24.04 `proyecto2-sim`; no instalar paquetes Jazzy para 24.04 directamente en el host ni recrear la `.venv`. El usuario confirmó webcam–ROS–Gazebo y acceso/movimiento por teclado y gestos del robot físico. El lanzador físico ya configura tiempo real y red con IP/dominio explícitos, manteniendo webcam y puente en la misma laptop; el usuario confirmó su movimiento gestual; falta validar los ajustes posteriores de velocidad y dirección.

Mantener la percepción separada de ROS. Los notebooks sirven para experimentar, medir y explicar; el control final necesita supervisar órdenes y tiempos aunque la captura falle. `task 2/webcam_gestos.py` emite JSON por una tubería de `docker exec -i` hacia `task 2/puente_gestos_ros.py`. `ejecutar_gestos_gazebo.sh` usa reloj simulado; `ejecutar_gestos_robot.sh` selecciona reloj real sin `/clock` y configura dominio/Fast DDS/peer. Ambos comprueban sin publicar por defecto; `--mover` activa velocidades. Las órdenes caducan con reloj monotónico compartido por host/contenedor en la misma máquina. Pasaron 38 pruebas locales, con dobles ROS/Docker/red y regresión de los lados del operador; la confirmación de gestos físicos anterior a los ajustes procede del usuario. El frenado no está medido. Ver `task 2/INTEGRACION_GESTOS.md`. No instalar `rclpy` con pip.

**Organización propuesta del sistema.** Un supervisor selecciona Task A o Task B y conserva los estados de misión y la identidad reconocida. Hay una sola salida coordinada de velocidad, con protección frente a órdenes caducadas. El modo autónomo no puede tomar el control de la navegación por gestos.

| Módulo | Responsabilidad | Primera comprobación |
| --- | --- | --- |
| Entrada de imágenes | Webcam/video de gestos y cámara ROS del robot para la foto en B | Imagen, frecuencia y antigüedad observables |
| Gestos de Task B | Clasificar y estabilizar órdenes humanas | Orden correcta y parada al perder la mano |
| Control de Task B | Traducir la orden a velocidad y detener al caducar | Recorrido corto con ida, vuelta y parada en Gazebo |
| Reconocimiento facial compartido | Detectar el rostro de la foto en B y obtener nombre desde una galería | Identificar impresiones no usadas como galería |
| Navegación de Task A | Descubrir el recorrido hacia B, evitar obstáculos y regresar a A | Ida y vuelta autónomas en varias configuraciones |
| Supervisor y resultado | Coordinar etapas, conservar nombre y anunciarlo al regresar | Registro de estados y nombre correcto al finalizar |

**Task B: ampliar el prototipo de gestos.** El usuario indicó que la entrada de gestos será su webcam; esa elección guía la implementación. Se conserva `task 2/Tarea1.ipynb` con su nombre original. El prototipo reconoce avanzar, girar a izquierda/derecha y detenerse. Los giros son continuos sobre el sitio mientras se mantiene la orden. La convención se cambió por petición del usuario el 5 de noviembre: frente a la webcam, señalar a tu derecha ordena DERECHA del robot y a tu izquierda, IZQUIERDA. El clasificador invierte una sola vez el eje horizontal de la imagen sin espejo; filtro y giros continuos se conservan; la variante antigua duplicada se eliminó durante la limpieza autorizada.

La primera versión confirma gestos durante 0.4 s y contempla caducidad de 0.5 s; son parámetros iniciales, no requisitos oficiales. Ante mano ausente, ambigüedad, varias manos o entrada caducada, detener. El publicador debe vigilar la antigüedad de la orden independientemente del proceso de visión. Comprobar cambio de dirección y fallo de comunicación. Los tiempos reales de parada todavía no se han medido en robot.

Orden de integración propuesto: observar la orden recibida por ROS sin mover; comprobar dirección, movimiento y parada en simulación; guiar una ida y vuelta; incorporar reconocimiento en B y conservación del nombre hasta A. Durante Task B toda dirección y avance de navegación deben proceder de señales visuales humanas. Determinar si harán falta gestos adicionales para cambio de etapa sin teclado. Una orden de alto nivel que haga ejecutar por sí sola todo el recorrido no cumple la navegación exclusivamente gestual descrita.

**Reconocimiento facial compartido.** La galería inicial se construyó con 16 referencias de `task 1/rostros/`, reservando otras cinco fotos para calibración y cinco para evaluación; faltan identidades por incorporar. La división por archivo es exploratoria: obtener tomas distintas para evaluar generalización. Hay varias referencias por identidad y se admite respuesta NO_CONCLUYENTE. Al llegar a B, la integración futura deberá detenerse, capturar la fotografía mediante la cámara del robot, detectar/alinear el rostro y compararlo con la galería. Confirmar la identificación entre varias observaciones de calidad, conservar nombre y evidencia de decisión, regresar a A y emitir el resultado en el formato que confirme el profesor. No existe una captura de referencia en A en esta secuencia.

YuNet + SFace siguen como candidatos de la propuesta previa; no son una exigencia del PDF. Se implementaron y se probaron localmente, todavía sin validación de campo. Documentación técnica de referencia: [tutorial OpenCV de detección y reconocimiento](https://docs.opencv.org/4.x/d0/dd4/tutorial_dnn_face.html) y [modelos OpenCV Zoo](https://github.com/opencv/opencv_zoo). Mantener la extracción de descriptores y las puntuaciones separadas del control para ensayar con webcam y grabaciones antes de integrar la cámara ROS.

Separar galería, calibración y evaluación por fotografías o sesiones antes de generar aumentos. Usar imágenes distintas de las de galería para medir identificación por nombre. Ajustar umbrales y margen entre candidatos con validación; no asignar siempre el nombre más parecido si la evidencia es insuficiente. Evaluar impresiones reales a distintos ángulos, distancias, iluminación y tamaños. La detección de personas alrededor no debe confundirse con la fotografía del circuito. Mantener fotos, capturas y datos personales fuera de Git y de las salidas persistidas del notebook.

**Task A: navegación autónoma de ida y vuelta.** Es el mayor riesgo de alcance. Debe funcionar sin intervención y en una configuración final desconocida, encontrar B y luego recuperar A. El punto de partida puede registrarse como referencia si el método de localización lo permite; hace falta comprobar deriva y tolerancia de retorno. Todavía no se ha elegido ni implementado la estrategia.

SLAM estima mapa y posición, pero necesita una política que decida dónde explorar y un criterio que identifique B. Nav2 puede planificar hacia objetivos definidos en una representación utilizable, pero no descubre por sí solo cuál es B. Un seguimiento de pared solo es candidato si la geometría y las condiciones de salida lo justifican; no garantiza resolver cruces o bucles. Seleccionar una estrategia después de conocer señalización, restricciones y configuraciones de prueba. El retorno puede aprovechar información obtenida durante la ida, sujeta a las reglas y a la localización medida. No comprometerse a un mapa previo de la configuración final.

El LiDAR, la odometría y la cámara se integrarán según las necesidades medidas. Probar obstáculos y rutas diferentes en Gazebo, incluyendo recuperación o parada cuando no hay un paso seguro. Si se practica con cartón, comprobar que las paredes alcanzan el plano del LiDAR, son estables y permiten el giro del robot. La secuencia propuesta de Task A es iniciar en A → explorar/navegar hasta B → detener y reconocer → regresar a A → detener e indicar nombre. La de Task B comparte las etapas, pero la ida y vuelta son dirigidas por gestos.

**Reparto propuesto para cuatro integrantes.** Trabajo paralelo entre integrantes del grupo; no implica delegación a agentes. Todos deben entender las decisiones para las preguntas de la exposición.

| Responsable | Trabajo principal | Entregable inicial |
| --- | --- | --- |
| Integrante 1 | Gestos, dirección, estabilidad y operación de Task B | Ida y vuelta por gestos en Gazebo con parada comprobada |
| Integrante 2 | Galería, reconocimiento y conservación del nombre | Identificación por nombre de impresiones reservadas |
| Integrante 3 | Sensores y navegación autónoma de Task A | Ida y vuelta sin intervención en configuraciones variadas |
| Integrante 4 | Entorno, interfaces, supervisor e integración | Modo seleccionado, control coordinado y estados registrados |

Acordar las salidas: orden y antigüedad; identidad y calidad; estado de navegación; resultado de misión. La persona de integración trabaja desde el inicio. Compartir versiones de dependencias y datos de ejemplo no personales. Hacer una sincronización breve diaria y documentar los resultados, incluidos fallos.

**Cronograma orientativo de 14 días.** Se conserva la disponibilidad aproximada indicada por el usuario; ajustar cuando se confirme la fecha. La integración por gestos puede continuar, mientras otro integrante resuelve la navegación autónoma.

| Días | Resultado buscado | Acceso al robot |
| --- | --- | --- |
| 1–2 | Cerrar reglas pendientes, acceder a datos, medir entradas y repartir interfaces | Primera visita: cámara, LiDAR, odometría, conexión y grabaciones |
| 3–4 | Gestos conectados a Gazebo, identificación facial inicial, estrategia autónoma escogida y probada en un tramo | Trabajar en casa con datos reales capturados |
| 5–7 | Primer ensayo de ambas modalidades: A → B → reconocer → A → nombre | Segunda visita: comprobar integración y primer recorrido completo |
| 8–10 | Corregir fallos principales; variar circuito, fotografía y orientación; medir tiempos | Visita breve adicional si hace falta reproducir un fallo |
| 11–12 | Ensayos completos sin teclado para Task B y sin intervención para Task A | Tercera visita: validación final y ensayo de grabación |
| 13–14 | Congelar versión, informe, diapositivas y exposición de 8–10 min | Reservar contingencia; grabar la competición oficial cuando ocurra |

La primera demostración al día 7 es una meta de coordinación, no una garantía. Pedir reservas pronto, sujetas a disponibilidad. No sustituir resultados del robot por simulación en el informe ni un ensayo por el video exigido de la competición.

**Primera sesión de laboratorio.** La [guía del profesor](https://github.com/Tilinpro19/turtlebot67) sirve para conexión y utilidades, no es una dependencia del reconocimiento. Descargar modelos, paquetes y datos antes: la red del laboratorio no tiene internet según la guía. Confirmar ROS, dominio de red, batería, reloj y nombres/tipos reales de tópicos. La guía usa imagen comprimida `/oakd/rgb/preview/image_raw/compressed`, QoS de sensores y `TwistStamped` en `/cmd_vel`; verificarlo en la unidad real.

En una sesión de desarrollo, comprobar movimiento y parada, registrar gestos desde la cámara admitida, fotografías desde la altura del robot, odometría y LiDAR durante un recorrido de ida y vuelta. Grabar selectivamente con `ros2 bag` cámara, `/scan`, `/odom`, `/tf`, `/tf_static` y calibración si se usa geometría, confirmando nombres. Abrir al menos una grabación antes de devolver el robot. Reproducir datos en un entorno aislado del robot: la grabación sirve para repetir percepción, pero no cambia su recorrido cuando se prueba una nueva orden.

**Verificación y material técnico.** Medir gestos por orden y persona, confusiones de dirección, latencia, activaciones involuntarias y respuesta ante datos caducados. Evaluar identificación por nombre con fotos reservadas, incluyendo rechazos y falsas identificaciones. En navegación medir llegada a B, regreso a A, tiempo, colisiones e intervenciones. Repetir ambas misiones con circuitos y fotografías distintos. Cinco ensayos completos por modalidad son una meta interna inicial; una muestra pequeña no demuestra fiabilidad general.

Conservar tablas de todos los intentos, arquitectura, convención de gestos, condiciones de cámara, parámetros, motivos de selección de modelos, estrategia de navegación y limitaciones. Relacionar el trabajo con descriptores, redes preentrenadas y estabilidad temporal del curso. Añadir métodos como Kalman o comparaciones LBP solo si resuelven una necesidad o aportan evidencia, sin desplazar la misión completa. Comenzar en CPU y medir antes de buscar GPU o Colab.
