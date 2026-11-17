# Plan del proyecto con TurtleBot 4 Lite

Actualizado el 17 de noviembre de 2026. Fuente base: las cinco páginas de [Computer_vision_proyectos.pdf](Computer_vision_proyectos.pdf), revisadas completas. Las indicaciones del usuario del 13 y 17 de noviembre **prevalecen para Task 1**. Se distinguen los requisitos confirmados, las decisiones propuestas y los componentes realmente implementados en [ESTADO_PROYECTO.md](ESTADO_PROYECTO.md).

**Task 1: alcance vigente confirmado por el usuario.**

1. Antes del inicio, disponer de un apartado para escribir el nombre de una persona presente en el dataset/galería.
2. Registrar el punto A y comenzar la misión autónoma; después del inicio no se asume intervención humana. Evitar contar su meta inicial como llegada a B.
3. Recorrer el camino A → B entre obstáculos, detectando e interpretando las flechas azules como ayuda para decidir dónde girar. La flecha confirmada fija el sentido frente al lado propuesto por evasión ante un muro; completar 90° antes de avanzar. La evitación puede detener por espacio insuficiente, sin invertir la flecha.
4. Confirmar la meta para declarar llegada a B; solo entonces buscar alrededor con barrido controlado. En B habrá varias fotografías: buscar y reconocer la de la persona objetivo. La identidad aceptada debe coincidir con el nombre solicitado; encontrar cualquier cara no basta.
5. Cada fotografía tiene su propio QR. Leer únicamente el situado junto a la fotografía objetivo, que contiene una palabra, y verificar su asociación con ella.
6. Conservar la palabra y entonces regresar con las señales rojas. Confirmar la meta de A durante la etapa de vuelta, detenerse y mostrar allí la información del QR; no reutilizar la meta de B como llegada a A.

La salida final de Task 1 es ahora **la palabra del QR**, no únicamente el nombre. No hay captura de una fotografía de referencia en A: la entrada inicial es el nombre. El texto del QR, el nombre solicitado y la identidad observada son datos distintos.

**Aclaraciones confirmadas por el usuario.**

**Contrato de misión aclarado el 17 de noviembre.** La flecha confirmada del tramo fija el sentido del giro incluso ante un muro: prevalece sobre la elección de lado de la evasión. El LiDAR conserva la posibilidad de detener por falta de espacio, sin ordenar el sentido opuesto. Se debe completar el giro de 90° medido por odometría antes de reanudar el avance. La misión parte de A; durante la ida sigue azul y espera confirmar la señal de meta para declarar B. **Solo después de esa meta** debe detener la navegación de ida y buscar rostros alrededor mediante un barrido controlado. Confirmar la persona solicitada, leer únicamente el QR asociado a su fotografía, conservar su palabra y entonces comenzar la vuelta con rojo. En la vuelta, confirmar la meta de A, detenerse y mostrar la palabra. El significado A/B de una meta se determina por la etapa: evitar que la meta inicial de A o la misma meta de B recién vista se cuenten como una llegada nueva. El filtro de meta, los criterios de separación/llegada, el barrido, QR y transición de retorno están pendientes de implementar. El prototipo actual todavía reconoce durante la marcha y se detiene al confirmar el rostro; no ejecuta esa secuencia completa.

- Aclaración del 17 de noviembre: flechas azules para A → B y rojas para B → A. Sustituye la suposición anterior de que no había señales de regreso. Integrar la guía visual con evitación/localización; registrar A o girar no implementa por sí solo el retorno.
- B contiene varias fotos, cada una con su propio QR. Buscar la persona seleccionada y leer solo su QR; no elegir el primer rostro o el primer QR visible.
- Los cambios se aplican únicamente a Task 1. Task 2 mantiene su objetivo anterior.

**Detalles todavía por aclarar.**

- Tamaño y ubicación reales de las flechas del PDF, política oficial ante una dirección bloqueada y disposición física de fotografías/QR. El prototipo espera detenido si el giro está bloqueado. El usuario confirmó señales de meta para A/B; interpretación y tolerancias siguen pendientes.
- Señalización de A/B, tolerancias de llegada/retorno, geometría y cruces del circuito, observación previa permitida y formato exacto del contenido QR.
- Fecha límite, tiempo máximo y penalizaciones exactas. La fecha 13 de octubre de la portada del PDF es la fecha del enunciado, no la entrega.

**Objetivo vigente por modalidad.**

| Modalidad | Navegación | Objetivo y salida |
| --- | --- | --- |
| Task 1 / Task A | Autónoma; obstáculos y flechas como ayuda para giros; sin intervención después del inicio | Nombre elegido antes de salir → buscar a esa persona entre varias fotos en B → leer su QR → regresar a A → mostrar palabra QR |
| Task 2 / Task B | Únicamente gestos/movimientos visuales durante el intento | Mantiene el objetivo del PDF: identificar la foto en B, regresar a A y mostrar/indicar su nombre; los cambios de Task 1 no se le aplican |

Los dibujos de señales están disponibles en `señales/señales.pdf`; por ahora se implementan solo giros. La configuración final y la fotografía se revelan al competir según el PDF base. Diseñar para configuraciones no vistas; no depender de una ruta final memorizada ni de coordenadas B fijas. Las opciones de ArUco/color/coordenadas del compañero son herramientas de prueba, no señalizaciones oficiales confirmadas. En Task B no se permiten teclado, ratón, joystick ni navegación autónoma durante el intento. Sigue pendiente la ubicación/movilidad admitida del operador, los cambios de etapa mediante señales visuales y el alcance permitido de una parada protectora automática.

**Base del compañero: adopción aprobada e integración parcial.**

El usuario autorizó usar y mejorar la implementación de Landov311 en [paz_y_rostros](https://github.com/Manueljsilva/turtlebot4-vision-proyecto/tree/cf7a605e67903477759834674bcf0807b3692ac7), snapshot revisado `cf7a605e67903477759834674bcf0807b3692ac7`. Sus 19 archivos de `task_autonoma/` aportan SLAM Toolbox, Nav2, exploración por fronteras, cámara, detección de B, reconocimiento en ráfaga y retorno. No implementan todavía el nuevo contrato de nombre objetivo, flechas y QR. Pasaron 13 pruebas locales, contando la carga facial con nuestra galería/modelos; no hay una misión autónoma validada en ROS/Gazebo/hardware por esta revisión.

Ya se adaptó selectivamente la prueba reactiva LiDAR dentro de `task 1/`: `ejecutar_autonomo_robot.sh --solo-evasion`, primero sin movimiento y luego con `--mover`; comandos en bloques 4/5 de `task 1/COMANDOS.txt`. Se corrigieron sectores desconocidos tratados como libres y la orientación asumida sin TF. La prueba de evasión no exige galería/cámara/Nav2; el lanzador también admite ahora reconocimiento estacionario. SLAM/Nav2 y la misión siguen pendientes. Pasaron 42 pruebas locales de Task 1 (14 faciales, 23 de evasión y cinco del lanzador). El usuario confirmó movimiento y mejoría parcial, pero el requisito del 50 % bloqueaba con 37–47 % de retornos finitos y el resto +inf. Se ajustó a mínimo 30 % y tres retornos por sector, con NaN/fuera <=10 %, manteniendo parada por insuficiencia, TF ausente y recepción >0.6 s. Ajustes de distancia: precaución 0.45 m, parada del avance 0.32 m y lateral 0.28 m. Hay pausas de recepción de hasta 1.66 s sin causa aislada; este ajuste, estabilidad y frenado siguen pendientes de validación física.

Continuar la integración dentro de `task 1/` y reutilizar `reconocimiento_facial.py`/`modelos_rostros.py`, evitando copias divergentes. Conservar el notebook, sus ejecuciones, la galería/calibración y la `.venv` global. Los experimentos de gestos del compañero permanecen en su rama; esta autorización no implica fusionar todo el PR ni publicar en GitHub.

La prueba LiDAR local corrige el avance con sectores inválidos y espera TF real. Antes de integrar el resto de la base, corregir posición A ficticia cuando falta TF, reloj de `BasicNavigator` en Gazebo, votos sobre fotogramas repetidos y verificar dependencias de visión dentro de Docker. CLAHE y el cambio de sensibilidad del detector deben evaluarse sin relajar el criterio usando errores del test. Distinguir el rostro de la fotografía de otras personas visibles.

**Estado real que se conserva.**

- `task 1/`: reconocimiento facial local YuNet/SFace en CPU y prueba reactiva LiDAR con movimiento inicial confirmado por el usuario; parada posterior por cobertura/recepción pendiente de diagnóstico. El 13 de noviembre se amplió explícitamente a 74 fotos de 13 personas: 48 destinadas a referencias (47 válidas), 13 consultas de calibración y 13 de evaluación, preservando roles de fotos previas. Evaluación exploratoria actual: 13 nombres correctos de 13 consultas reservadas, cero rechazos y cero nombres erróneos; criterio recalibrado solo con calibración. Los resultados anteriores del notebook corresponden a la galería inicial de cinco personas. Las tomas no son sesiones independientes; los negativos por retirada de identidad son simulados. Faltan alumnos, desconocidos reales e impresiones por cámara. Galería/criterio no deben regenerarse silenciosamente al incorporar navegación.
- `task 2/`: webcam del operador → MediaPipe/reglas/filtro → JSON por tubería → puente ROS → `/cmd_vel` `TwistStamped`. Gestos confirmados en Gazebo y robot real. El 13 de noviembre el usuario confirmó vídeo comprimido en el panel y que los gestos responden bien; solo el vídeo tiene retraso sin medir. No hay un recorrido completo validado.
- Cámara física: combinación reportada como funcional por el usuario = servicio general detenido, limpieza puntual de procesos y arranque manual `standard.launch.py`; panel con `/oakd/rgb/preview/image_raw/compressed`, IP .100 y dominio 5 en esa sesión. Conservar como observación; no atribuir el fallo a energía ni cambiar perfiles sin verificar lo instalado. Evitar controladores duplicados y no convertir el cierre forzado en rutina. Los comandos de panel están en bloques 11/12 de `task 2/COMANDOS.txt`.
- Informe: `informe/main.tex` y PDF de diez páginas son el borrador de avances del 7 de noviembre; reflejan el objetivo anterior. Actualizarlos posteriormente con flechas/QR y con las confirmaciones físicas recientes, sin inventar métricas. No se entregaron en Canvas; fuentes/PDF y panel se publicaron en main el 16 de noviembre, commit `5e5176f`.

**Arquitectura a completar para Task 1.** La propuesta conserva una sola salida coordinada de velocidad y separa percepción, planificación y estados. SLAM estima mapa/posición; exploración decide dónde observar; Nav2 planifica y controla; las flechas orientan la elección de giros durante el recorrido. Para volver a A se deben incorporar las flechas rojas y verificar localización/llegada, usando mapa o navegación coherente con el circuito. El retorno completo aún no está implementado. Detectar un rostro no sustituye confirmar la llegada a B ni identificar a la persona solicitada.

**Prioridad actual: giros por señales.** El 17 de noviembre el usuario confirmó movimiento más reconocimiento juntos en el robot, sin métricas de pose A/parada ni navegación completa. Se conserva ese modo; se añade `--solo-senales` sin movimiento y la opción `--senales --tramo ida|vuelta` de la prueba combinada. Azul se usa para ida y rojo para vuelta, elegido antes del inicio; no hay transición automática al encontrar al objetivo. Se adapta la geometría de círculos/flechas de la compañera, con confirmación en tres imágenes nuevas, rechazo de ambigüedad/color incorrecto y giro de 90° por odometría. Los giros siguen condicionados por sensores/actividad, una sola salida y parada ante bloqueo; timeout enclavado y rearme impiden giros indefinidos o repetidos. La llegada por meta, QR, retorno, búsqueda entre fotos y corrección de choques diagonales continúan pendientes. Primera evidencia necesaria: señales impresas por cámara sin movimiento, después signos/ángulos/frenado con movimiento. Pruebas locales y confirmaciones físicas se registran en ESTADO_PROYECTO.md.

| Componente | Responsabilidad | Comprobación necesaria |
| --- | --- | --- |
| Entrada de misión | Recibir nombre antes del inicio y comprobar que existe en la galería | Rechazar nombres ausentes/ambiguos antes de arrancar |
| Posición y navegación | Registrar A válido, localizar, evitar obstáculos y planificar A → B → A | TF válido, LiDAR fresco, mapa utilizable, parada ante datos insuficientes |
| Percepción de flechas | Detectar dirección y vincularla al tramo/cruce correspondiente | Flechas reales de distintos tamaños/ángulos; estabilidad y repetición por señal |
| Reconocimiento facial | Evaluar fotos/caras candidatas individualmente y confirmar por galería la identidad solicitada | Identificación y rechazo con impresiones reservadas, otras personas y varias fotos; conservar la regla de una cara por consulta del extractor compartido |
| Asociación y lector QR | Relacionar el QR con la fotografía objetivo y obtener su texto | No mezclar QR de otras fotos; tratar ausencia o lectura no concluyente |
| Supervisor y resultado | Conservar objetivo, identidad, palabra y estado; verificar regreso a A y mostrar QR | No declarar éxito con identidad/QR/retorno sin confirmar |

Secuencia propuesta de estados: preparación/nombre → A válido → navegación con obstáculos/flechas azules y giros completos de 90° → meta B confirmada → barrido/búsqueda y confirmación del objetivo entre fotos → lectura QR asociada y palabra guardada → retorno con flechas rojas → meta A confirmada en etapa de vuelta → parada/mostrar palabra. Concretar los estados de fallo/recuperación para flechas ausentes, ambiguas o que sugieran un giro bloqueado, además de fallos de identidad/QR/localización. El reconocimiento mantiene NO_CONCLUYENTE cuando no hay evidencia suficiente.

**Datos, entorno e integración.**

La estructura vigente es `task 1/` para autonomía y `task 2/` para gestos; `.venv`, `.gitignore`, enunciado, README, informe y contexto son globales. El host inspeccionado usa Ubuntu 26.04/Python 3.14.4; ROS 2 Jazzy/Gazebo están en el contenedor existente `proyecto2-sim` con Ubuntu 24.04. No instalar Jazzy 24.04 en el host ni `rclpy` con pip; no recrear `.venv` ni Docker para esta integración. Comprobar compatibilidad de OpenCV/cv_bridge si se incorpora visión dentro de Docker. Empezar en CPU y medir antes de cambiar a GPU/Colab.

Mantener galería, calibración y evaluación separadas por tomas cuando sea posible. No forzar el nombre elegido si el criterio rechaza la consulta. Incorporar alumnos nuevos exige revisar explícitamente el manifiesto y repetir la calibración/evaluación; el test no ajusta umbrales. Fotos, descriptores, capturas, modelos y salidas personales siguen fuera de Git. La implementación compartida no debe incrustar fotografías de alumnos.

La [guía del profesor](https://github.com/Tilinpro19/turtlebot67) es referencia de conexión/utilidades, no dependencia de MediaPipe. La red del laboratorio no tiene internet: preparar paquetes, modelos y datos antes. Confirmar IP/dominio, red host de Docker, tópicos/tipos, TF y relojes en cada visita. Comandos comunes en [COMANDOS_LAB.txt](COMANDOS_LAB.txt). Los ensayos deben distinguir captación local en Pi, recepción en Docker, percepción y ejecución. Una grabación de sensores permite repetir percepción, pero no cambia retrospectivamente el recorrido.

Task 2 conserva imagen sin espejo, confirmación de 0.4 s y caducidad de 0.5 s; derecha del operador → derecha del robot, izquierda → izquierda. Velocidades configuradas 0.306 m/s y ±1.90 rad/s por petición del usuario; frenado y sentidos requieren medición específica. El vigilante no garantiza parar ante muerte del propio proceso ROS o corte de red. El panel conserva última imagen y limita salida a 5 FPS; eso no mide ni resuelve por sí solo la latencia del sensor/red.

**Hitos de trabajo actualizados.** Fechas y reservas se fijarán al confirmar el plazo; las dos semanas desde el 26 de octubre eran una estimación inicial, no una fecha de entrega.

| Hito | Entregable verificable |
| --- | --- |
| 1. Adoptar la base | Integración selectiva en Task 1, correcciones LiDAR/TF/reloj y pruebas existentes conservadas |
| 2. Objetivo y QR | Nombre inicial validado, reconocimiento de la foto objetivo y asociación/lectura QR en pruebas locales |
| 3. Flechas | Dirección visual estable vinculada al giro correspondiente, coordinada con la evitación de obstáculos; azul en ida, rojo en regreso y llegada confirmada por meta y etapa |
| 4. Integración en simulación | A → obstáculos/flechas → rostro objetivo/QR → A → palabra, sin intervención tras inicio |
| 5. Ensayos físicos | Repetir con circuitos, flechas y fotos distintos; medir retardos, llegadas, colisiones y resultado QR |
| 6. Entrega | Adaptar informe, preparar diapositivas, ensayar exposición y registrar competición según el enunciado |

Reparto propuesto para cuatro integrantes humanos: navegación/localización; flechas y cámara; reconocimiento/QR/datos; supervisor, integración y evidencias. Todos deben entender las decisiones para la exposición. Esto no autoriza delegación a subagentes. Mantener interfaces claras y sincronización breve diaria.

**Evaluación y entregables del PDF base.**

Registrar todos los intentos, incluidos fallidos: nombre solicitado, identidad aceptada/rechazo, QR correcto/incorrecto/no leído, flechas observadas y decisiones, llegada a B, regreso a A, tiempo, colisiones e intervenciones. Probar QR de otra fotografía, caras ajenas, pérdida de cámara/LiDAR/TF y ausencia de flechas. Cinco ensayos completos por modalidad son una meta interna inicial, no evidencia suficiente de fiabilidad general.

Según el PDF revisado, cada equipo tiene hasta tres intentos oficiales por tarea; cuenta el mejor. Tiempo máximo y penalizaciones están pendientes de anuncio; exceder el máximo da cero para ese intento. Rúbrica base: competición 9/20 (4.5 por tarea), informe/documentación 4, presentación 4 y preguntas 3. Si las nuevas instrucciones cambian evaluación o puntuación, confirmarlo antes de tratar estos valores como definitivos.

Informe metodológico y diapositivas en Canvas antes del plazo comunicado en clase; sin entrega no se puede presentar. Exposición de 8–10 minutos con arquitectura, algoritmos, navegación, decisiones, resultados y limitaciones; código detallado en apéndice. Video de competición de ambas tareas al Drive del curso, cuyo enlace aún falta. No reemplazar la competición por una simulación o por un ensayo en el informe. El formato exacto de la visualización final del QR se concretará; no se presupone voz.
