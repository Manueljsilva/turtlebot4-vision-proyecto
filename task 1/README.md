# Task 1: búsqueda facial, evasión y giros por señales

**Prueba actual (21/11):** el usuario confirmó lectura al aumentar el tamaño del
QR. Ejecutar `--mision-guiada --mover` desde A para probar azul/evasión → meta B →
rostro objetivo/QR propio → rojo/evasión → meta A. A también lleva bandera,
mirando al robot cuando regrese. Al confirmar meta A y reposo, aparece un panel
grande con la palabra y el robot queda detenido. El regreso completo aún no está
validado físicamente. Guía vigente: [PRUEBA_MISION_GUIADA.md](PRUEBA_MISION_GUIADA.md),
bloque 18 de [COMANDOS.txt](COMANDOS.txt); 215 tests locales. **Reiniciar**, junto
a Salir, detiene y espera reposo antes de permitir otra persona en la misma
ventana; el nuevo Buscar vuelve a registrar A. Reinicio aún pendiente en hardware.
Las descripciones
históricas de tres confirmaciones de abajo se sustituyen por una imagen nueva válida.

Primera implementación facial local terminada el 28 de octubre de 2026. Usa la `.venv` global, OpenCV en CPU y modelos preentrenados YuNet/SFace; el notebook funciona sin ROS, Gazebo ni robot. El 13 de noviembre se incorporó una prueba separada de evasión LiDAR en hardware, con ROS en el Docker existente. Task 2 se conserva sin cambios.

**Nuevo ensayo integrado del 20 de noviembre:** `--mision-guiada [--mover]` enlaza azul/evasión → meta B → frenado/barrido → rostro objetivo/QR propio → orientación de entrada B +180° → rojo/evasión → meta nueva cerca de A → palabra. Bandera B al fondo del camino, como confirmó el usuario. Guarda A en odometría, separa/rearma metas y conserva una sola salida de velocidad; no reproduce la ruta ni incorpora mapa. Sin --mover no publica. **207 pruebas locales de Task 1 pasan; el recorrido integrado y el retorno todavía no se validaron físicamente.** [Guía de colocación, comandos y límites](PRUEBA_MISION_GUIADA.md), bloque 18 de COMANDOS.txt. Los modos anteriores descritos debajo siguen disponibles y conservan sus límites individuales.

**Comprobar la bandera por separado:** `--solo-meta` (bloque 19) abre la cámara sin galería ni publicador, sin pedir nombre/Buscar. Mostrar la bandera completa y esperar cuadro **META VISTA**. No confirma llegada A/B. Se usan los PNG de `señales/` como referencias en pruebas; PDF conservado, sin entrenamiento ni cambio de resolución de vídeo. La misión añade `[META]` para explicar llegada bloqueada y `[CAMARA]` para distinguir imagen caducada. El primer log del usuario permanece en ESPERAR/a=None y no demuestra fallo del detector durante IDA. Corrección para bandera pequeña y cuadro visual comprobada localmente; pendiente en cámara real.

**Contrato de misión aclarado el 17 de noviembre.** La flecha confirmada del tramo fija el sentido del giro incluso ante un muro: prevalece sobre la elección de lado de la evasión. El LiDAR conserva la posibilidad de detener por falta de espacio, sin ordenar el sentido opuesto. Se debe completar el giro de 90° medido por odometría antes de reanudar el avance. La misión parte de A; durante la ida sigue azul y espera confirmar la señal de meta para declarar B. **Solo después de esa meta** debe detener la navegación de ida y buscar rostros alrededor mediante un barrido controlado. Confirmar la persona solicitada, leer únicamente el QR asociado a su fotografía, conservar su palabra y entonces comenzar la vuelta con rojo. En la vuelta, confirmar la meta de A, detenerse y mostrar la palabra. El significado A/B de una meta se determina por la etapa: evitar que la meta inicial de A o la misma meta de B recién vista se cuenten como una llegada nueva. El modo separado --prueba-meta-b conserva meta/barrido/QR sin retorno. El nuevo --mision-guiada del 20 de noviembre enlaza esas etapas con ida/vuelta guiadas, con validación física pendiente. El modo anterior --evasion-reconocimiento todavía reconoce durante la marcha y se detiene al confirmar el rostro.

**Ensayo de meta, persona y QR, actualizado el 19 de noviembre.** Sin --avanzar-hasta-meta se conserva la prueba colocada cerca de B, con la bandera visible y espacio para girar. No registra este lugar como A. Primero comprobar percepción sin movimiento:

```bash
bash "task 1/ejecutar_autonomo_robot.sh" --ip "$TB4_IP" --dominio "$TB4_DOMINIO" --prueba-meta-b
```

Escribir la persona y pulsar **Buscar**. `--nombre` solo precarga el campo. Tres imágenes nuevas de una bandera estable solicitan frenar. Publica cero y espera 0.7 s de odometría estable antes de girar; la visión exige la confirmación ROS y un paquete posterior para analizar rostros. Sin --mover, mostrar las fotos/QR a la cámara para comprobar percepción. Cerrar y repetir añadiendo `--mover` permite únicamente girar sobre el sitio, con avance cero, hasta una vuelta o 90 s. La velocidad es como máximo 0.25 rad/s, limitada también por la configuración LiDAR. Cámara, interfaz, odometría y LiDAR/TF deben estar recientes, y frente/laterales a más de 0.32 m para girar. Un bloqueo espera detenido; fin de vuelta/plazo o discontinuidad real de odometría enclavan el movimiento. La visión ya habilitada sigue analizando imágenes nuevas estando quieto; cerrar/revisar para repetir el movimiento. No es una certificación de la huella del robot; los choques diagonales conocidos siguen pendientes.

Para añadir el avance pedido por el usuario, cerrar la prueba anterior y usar **--avanzar-hasta-meta --mover**:

```bash
bash "task 1/ejecutar_autonomo_robot.sh" --ip "$TB4_IP" --dominio "$TB4_DOMINIO" \
  --prueba-meta-b --avanzar-hasta-meta --mover
```

Tras escribir el nombre y pulsar Buscar, usa evasión LiDAR hasta **ver y confirmar** la bandera B. Conserva avance 0.10 m/s y giro evasivo 0.40 rad/s con la configuración actual. Después de meta prohíbe reanudar avance: frena, confirma reposo y solo gira para buscar. No sigue flechas ni estima distancia a la bandera, y no registra A ni regresa. Usar un espacio despejado con solo la bandera B, sin la meta inicial A visible; la evasión todavía tiene choques diagonales pendientes. Puede parar al ver la bandera desde lejos: esta es una prueba de la transición, no llegada métrica/localización.

El usuario reportó meta/giro corto y parada. El registro ocultaba el primer motivo bajo «Detenido por interfaz»; ahora se conserva y se imprime tras **CONTROL:**, junto con CAMARA. Se corrigió un falso salto de odometría al retomar vídeo: se sigue la pose mientras la cámara/GUI están pausadas y se mantiene cero hasta recuperar frescura. La odometría con edad >0.6 s sigue deteniendo. Un corte de recepción con pose coherente espera >=3 paquetes nuevos durante 0.7 s de reposo antes de reanudar; saltos >0.25 m / >0.5 rad, marca retrocedida y cambio de marco bloquean el movimiento. Durante barrido, cortes >2 s también bloquean por incertidumbre angular. No se decodifican QR hasta encontrar al objetivo, evitando procesamiento innecesario. Estas correcciones tienen pruebas locales; no aíslan por sí solas la causa física exacta del intento anterior.

Solo tras la meta evalúa las caras visibles por recortes individuales con la galería y criterio existentes. Desde el primer acierto del objetivo pausa el giro. Otra persona conocida conserva su nombre, y un rechazo no se convierte en el objetivo solicitado. Para vecindad lateral, situar cada QR **al costado y a una altura similar al rostro**, separando parejas. También admite el QR **en una esquina de la foto**, como en la prueba del usuario, cuando se ve un contorno completo que contiene solo esa cara y ese QR. Mostrar la foto entera sobre un fondo con contraste; sin contorno fiable no se fuerza esa asociación. Exige vecindad espacial mutua y sin ambigüedad: un QR entre dos fotos, varios QR equivalentes, objetivo duplicado o código ilegible no completan el ensayo. Tres imágenes nuevas con la misma palabra y rostro objetivo consistente confirman el resultado. La asociación geométrica todavía necesita prueba con la disposición real.

Guarda únicamente nombre objetivo, identidad confirmada y palabra en `datos/sesiones/prueba_b_*.json`, ignorado por Git; **sin fotos**. La ventana ofrece una vista de prueba del QR en B, y terminal/JSON conservan el texto completo UTF-8. Los QR son datos, no se ejecutan ni abren. El detector de la compañera marcaba tres páginas de QR como meta; se adaptó la geometría y se excluyen códigos/localizadores anidados. La bandera del PDF da positivo y esas tres páginas ya no activan meta en las pruebas locales.

**Registro posterior del usuario:** apareció «Odometria discontinua» durante NAVEGAR, antes de confirmar la bandera. El movimiento quedó bloqueado y nunca confirmó reposo para habilitar rostros. Se corrigió la confusión entre retraso de recepción y salto espacial; ahora se recupera quieto con paquetes nuevos. Si hay un fallo terminal anterior a la meta, después puede confirmar reposo y habilitar solo visión, manteniendo el movimiento bloqueado. `ODOM_RX` muestra edad, máximo intervalo entre callbacks y último corte; un salto real incluye dt_rx/dxy/dyaw. El diagnóstico no identifica por sí solo una causa de Wi-Fi ni mide captura del sensor. El comando del bloque 15 sigue igual; cerrar y volver a ejecutar carga la revisión en Docker. La evasión está incluida y las flechas permanecen separadas.

**Cómo comprobar la lectura del QR.** La revisión muestra por separado `detectados`, `legibles` y `Aceptacion 0/1…1/1`. `QR SIN LECTURA` indica localización sin texto; `QR LEGIBLE` indica texto disponible, todavía sujeto a asociación; `QR OBJETIVO` indica asociación aceptada. Solo **QR GUARDADO**, la palabra y el JSON local confirman éxito. La terminal imprime `[QR]` cuando cambia ese diagnóstico. Así se distingue cero códigos, fallo de lectura y código ajeno/ambiguo, antes agrupados bajo un mensaje genérico.

El lector reintenta hasta dos recortes sobre el original con corrección de perspectiva, margen blanco, ampliación y umbralización; prioriza proximidad al objetivo y no hace esos reintentos sin él. Un contorno propio no ambiguo prevalece sobre un QR ajeno cercano. Se siguen rechazando paneles compartidos/contornos contradictorios. Son mejoras comprobadas localmente; si el código llega demasiado pequeño o borroso, ampliarlo no recupera detalle: mostrar/imprimir QR mayor, completo, enfocado y con margen blanco. No se aumenta automáticamente resolución/tráfico ni se acerca el robot. Cerrar y relanzar el mismo comando carga la revisión; no hay paquetes nuevos ni cambio de criterio facial.

**Este ensayo termina detenido en B; no regresa.** El siguiente paso es integrar las flechas/localización con esta etapa, orientar la salida de B, cambiar a rojo y comprobar una nueva meta de A sin volver a contar la de B. No equivale a invertir velocidades. La edad local de recepción no mide el retraso de captura/red. Se espera indefinidamente el arranque de cámara; datos caducados e inferencia bloqueada revocan el giro. Cerrar otros controles; Esc/Salir/Ctrl+C intentan ceros y cierre, con la limitación habitual ante caída del nodo/corte de red. Comandos completos en bloques 13/14 (solo giro) y 15 (avance hasta meta) de [COMANDOS.txt](COMANDOS.txt).

**Giros por señales, 17 de noviembre.** El usuario confirmó movimiento y reconocimiento juntos en el robot y pidió añadir señales. Azul corresponde a A → B; rojo, a B → A. Esta aclaración sustituye la suposición anterior de que las flechas no guiaban el retorno. La prueba de señales aún no se ha ejecutado físicamente. Meta y QR tienen ahora el ensayo separado que se describe abajo.

Primero, con el robot quieto, comprobar la cámara y el reconocimiento:

```bash
bash "task 1/ejecutar_autonomo_robot.sh" --ip "$TB4_IP" --dominio "$TB4_DOMINIO" --solo-senales --tramo ida
```

La ventana espera la cámara comprimida, dibuja el color/dirección y acepta la primera imagen nueva válida de un único cartel. Para las rojas cerrar y repetir con `--tramo vuelta`. No carga la galería ni crea publicador de velocidad; no admite `--mover`. Varios carteles del color activo son ambiguos; el otro color no ordena giros. Los discos sin flecha, prohibición, meta y QR quedan fuera de este detector de giros.

Después, cerrar la ventana y añadir las señales a la prueba combinada:

```bash
bash "task 1/ejecutar_autonomo_robot.sh" --ip "$TB4_IP" --dominio "$TB4_DOMINIO" --evasion-reconocimiento --senales --tramo ida
```

Buscar fija el nombre y reinicia la confirmación de señales para no usar votos previos al inicio. El comando anterior comprueba sin publicador. Cerrar y repetir añadiendo `--mover` habilita movimiento. La única salida ROS conserva los permisos de cámara/actividad, LiDAR/TF y odometría; el rostro sigue teniendo prioridad para parar. Una señal confirmada pide un giro sobre el sitio y su final se mide por odometría, aunque el cartel salga de la vista durante el giro. El vídeo y los demás sensores deben seguir recientes.

Los ajustes de [config/senales_giro.json](config/senales_giro.json) son: alcanzar 90° por odometría, sin reanudar a 85°, y margen máximo de exceso de 5°, límite de 12 s, una detección válida, ancho de trabajo máximo 640 px y diámetro mínimo de cartel 55 px. El diámetro es un umbral visual, no una medida de distancia al cartel. La velocidad de giro procede de la configuración LiDAR (máximo configurado 0.40 rad/s); reduce al acercarse al ángulo. Sin cobertura/TF/recepción válidos, o si frente/laterales están a <=0.32 m, espera detenido y no cambia al sentido contrario. Si agota los 12 s queda detenido hasta cerrar y revisar. Los sectores actuales no certifican la huella ni corrigen los choques diagonales conocidos.

Para evitar repetir un cartel hacen falta >=1.5 s de ausencia en imágenes nuevas, un evento nuevo y >=0.4 m de desplazamiento desde el giro anterior. Si dos carteles están muy próximos, ese criterio puede omitir el segundo: se debe comprobar con la disposición real antes de ajustarlo. Cambiar `--tramo vuelta` permite ensayar rojo, pero **no inicia un regreso**: al confirmar el rostro todavía queda detenido. La integración de meta/QR en ese modo combinado, transición de ida a vuelta y regreso completo siguen pendientes. La edad de imagen mide recepción local; el retraso previo del sensor/red sigue sin cuantificar y puede afectar al punto del giro.

**Revisión del aporte de la compañera.** Se conserva intacto `../detector_señales.py`. Su geometría de círculos, segmentos y puntas reconoce las flechas derechas del PDF. El script completo depende de SciPy (ausente en la `.venv` compartida), su receptor ROS espera Image raw con cv_bridge y la salida de señales no identifica el color; no encaja directamente con nuestro transporte comprimido y la misión por tramos. Se adaptó solo su geometría en [geometria_senales.py](geometria_senales.py), sustituyendo pares KDTree por NumPy y acotando segmentos. [senales_giro.py](senales_giro.py) añade color, confirmación y frescura, y [control_giros.py](control_giros.py) integra la maniobra sin ROS/OpenCV en Docker. El detector de giros no ejecuta su cámara, seguimiento predictivo, QR ni meta, ni añade espejo al vídeo del robot. La geometría de meta se adapta por separado en el nuevo ensayo B. No se instaló ningún paquete.

Se comprobaron las diez páginas del PDF: giros derechos azules/rojos reconocidos; discos vacíos, prohibición, meta y QR sin giro. Las izquierdas se prueban reflejando solo las referencias dentro del test; el PDF no trae una impresión izquierda real. Escala/brillo/JPEG y pruebas con ROS simulado son regresiones locales, no precisión física medida. Comandos completos en bloques 11/12 de [COMANDOS.txt](COMANDOS.txt).

**Prueba integrada: evasión y búsqueda facial (13 de noviembre).** Desde Ubuntu normal, con robot y Docker conectados y los demás controles cerrados:

```bash
bash "task 1/ejecutar_autonomo_robot.sh" --ip "$TB4_IP" --dominio "$TB4_DOMINIO" --evasion-reconocimiento
```

Así comprueba sin crear publicador de velocidad. Para el ensayo físico repetir añadiendo `--mover`. La ventana espera a que escribas el nombre y pulses **Buscar**; `--nombre` solo lo precarga. Buscar lo fija para esa ejecución y solicita registrar A con la primera odometría válida recibida después del inicio, antes de permitir movimiento. Si faltan cámara, LiDAR/TF, actividad de visión o posición recientes, mantiene cero. A no se sustituye por (0,0) ni por la posición al final del recorrido. Se guarda en `datos/sesiones/punto_a_*.json`, fuera de Git, sin fotos.

Una sola salida ROS aplica la decisión de evasión existente, limitada por el permiso de búsqueda. Conserva 0.10 m/s y 0.40 rad/s y las distancias/configuración vigentes. La primera coincidencia aceptada en una imagen nueva posterior a Buscar y al registro de A finaliza la búsqueda; imágenes caducadas, otra identidad o rechazo no finalizan. Queda detenido y no reanuda al perder la imagen. El nombre permanece fijo durante esta prueba; para otro objetivo cerrar y volver a abrir. Esc/Salir/cierre/Ctrl+C intentan detener y terminar los receptores.

**Esta versión no regresa automáticamente a A**, no lee meta/QR y no busca entre varias caras. Los giros visuales se habilitan aparte con `--senales`; no activan una misión de retorno. Registrar una pose no constituye una ruta de retorno ni demuestra llegar a B. Se guarda la pose del `child_frame_id` en el marco real indicado por `header.frame_id`, según [Odometry de ROS Jazzy](https://github.com/ros2/common_interfaces/blob/jazzy/nav_msgs/msg/Odometry.msg); no se presenta como coordenada de mapa. La referencia odométrica puede derivar, como explica [REP 105](https://github.com/ros-infrastructure/rep/blob/master/rep-0105.rst), y no debe reutilizarse sin verificar tras reinicios. El retorno con mapa/localización/planificación queda pendiente.

El vigilante revisa la actividad de la GUI/inferencia (máximo 0.5 s), imagen desde recepción local (1 s), odometría y LiDAR (0.6 s). El hilo de latido no renueva las marcas de actividad o imagen por su cuenta. EOF/mensaje inválido/caducidad revocan el permiso; cerrar intenta publicar ceros. Esto no garantiza entregar una parada ante corte de red o muerte del propio nodo ROS. La prueba comparte el bloqueo de movimiento con `--solo-evasion`; cerrar gestos, teclado y Nav2 sigue siendo necesario. Los choques diagonales reportados de la evasión no están corregidos por esta integración: probar inicialmente en un espacio despejado y no considerar validada la navegación.

Pasaron 91 pruebas locales, con transporte por subproceso real local y nodos ROS simulados, sin robot. Cubren salida única, modo sin publicación, inicio/pose válidos, pérdida de sensores/actividad, bloqueo de nombre, confirmación sin votos repetidos, parada final y caducidad aunque siga el latido. Sintaxis Python 3.12/Bash y paneles sintéticos revisados. Modelos, galería, datos, notebooks y Task 2 intactos. Comandos completos en bloques 9/10 de `COMANDOS.txt`; el 17 de noviembre el usuario confirmó movimiento y reconocimiento juntos. Faltan métricas de A, parada y frenado, y validar los giros visuales añadidos después.

**Alcance actualizado por el usuario, 13 de noviembre.** Task 1 debe permitir escribir antes del inicio el nombre de una persona de la galería, salir de A, navegar autónomamente entre obstáculos usando las flechas como ayuda para decidir giros, buscar en B la fotografía de esa persona entre varias fotos, leer únicamente el QR situado a su lado, conservar su palabra, regresar a A y mostrar la información del QR. Cada fotografía tiene su propio QR. El 17 de noviembre aclaró que las azules corresponden a la ida y las rojas al regreso; esa aclaración sustituye la suposición anterior. El retorno completo aún necesita integración. Estos cambios afectan solo a Task 1. La identidad solicitada se confirma con el criterio de reconocimiento; no se fuerza cuando el resultado es NO_CONCLUYENTE.

**Base de navegación aprobada, integración parcial.** Se usará y mejorará la implementación de [paz_y_rostros](https://github.com/Manueljsilva/turtlebot4-vision-proyecto/tree/cf7a605e67903477759834674bcf0807b3692ac7), mediante integración selectiva dentro de `task 1/`. Ya se adaptó su prueba reactiva de LiDAR; SLAM/Nav2 y la misión completa siguen pendientes. El ensayo B evalúa candidatos/recortes individuales y asocia espacialmente el QR; conserva la regla de una cara por consulta del extractor compartido. Falta validarlo físicamente e integrarlo en la misión. No se incorporan los experimentos gestuales del compañero. Detalles de señales, disposición de fotos/QR y llegada a A/B pendientes en [PLAN_PROYECTO.md](../PLAN_PROYECTO.md).

**Probar la evasión con el robot.** Usar los bloques 4 y 5 de [COMANDOS.txt](COMANDOS.txt), desde Ubuntu normal, con el robot y Docker ya conectados. Primero ejecutar sin movimiento:

```bash
bash "task 1/ejecutar_autonomo_robot.sh" --ip "$TB4_IP" --dominio "$TB4_DOMINIO" --solo-evasion
```

Después de comprobar las distancias y terminar con Ctrl+C, repetir añadiendo `--mover`. Las variables deben estar definidas en esa misma terminal. Esta prueba no necesita cámara, galería facial, SLAM ni Nav2. El lanzador admite `--solo-evasion`, `--solo-reconocimiento` o `--evasion-reconocimiento`; omitir el modo produce un error y no inicia una misión. Cerrar otros controles antes de mover. Ctrl+C intenta publicar cero antes de cerrar; Ctrl+Z suspende y no debe usarse.

Los ajustes están en [config/evasion_lidar.json](config/evasion_lidar.json): avance 0.10 m/s y giro 0.40 rad/s, independientes de Task 2; precaución desde 0.45 m, parada 0.32 m y corrección lateral a 0.28 m, distancias medidas por el LiDAR. **Por petición del usuario del 19 de noviembre, los giros evasivos frontales/laterales ordenan avance cero**; ya no describen arcos. Cuando el frente y los lados dejan de requerir giro, reanudan avance recto. La precaución de 0.45 m conserva su velocidad angular anterior y a 0.32 m se conserva el giro de emergencia existente, sin relajar sensores/TF/cobertura. El giro por flecha ya separaba giro/avance y exige 90° por odometría; el ensayo usa ahora el umbral local de 0.48 m y conserva bloqueo a <=0.32 m. La separación de órdenes no mide el frenado real ni garantiza huella libre. La configuración se valida antes de contactar Docker. Los ensayos anteriores mostraron un 37–60 % de retornos finitos frontales y un 40–61 % a la izquierda; el requisito inicial del 50 % impedía continuar incluso con escaneos recientes.

Ahora cada sector exige al menos tres retornos finitos y un `cobertura_minima=0.30`, con `fraccion_errores_maxima=0.10` para NaN/fuera de rango. Los `+inf` no aportan distancia ni se convierten en espacio libre; un sector entero sin retornos sigue detenido. Los límites configurables permiten subir la cobertura desde 30 % y reducir errores desde 10 %, sin desactivar esos mínimos. Un `-inf` frontal/lateral detiene ambas velocidades por proximidad no cuantificable, de acuerdo con [REP-117](https://github.com/ros-infrastructure/rep/blob/master/rep-0117.rst). El [controlador SLAMTEC consultado](https://github.com/Slamtec/sllidar_ros2/blob/main/src/sllidar_node.cpp) también genera `+inf` para retornos de distancia cero, incluidos huecos de compensación angular; esto no demuestra la versión/configuración instalada ni que todas las direcciones sin retorno estén despejadas.

Sin TF válido, con sectores insuficientes o recepción de LiDAR de más de 0.6 s se ordena detener; el nuevo registro también contiene pausas de recepción de hasta 1.66 s, aún sin causa aislada. Los criterios de cobertura y distancias son ajustes de este prototipo, no una garantía de detección de todos los obstáculos. Solo permite una evasión con movimiento por contenedor y usa una carpeta temporal distinta por sesión. No mide la antigüedad en el sensor ni garantiza entregar una parada ante corte de red o muerte del proceso. El usuario confirmó avance/giro iniciales y mejoría parcial, pero estabilidad/frenado y este último ajuste siguen pendientes de validar en hardware.

Cada mensaje muestra `frente/izquierda/derecha=útiles/total`, el porcentaje y `OK/INSUFICIENTE`, junto a `inf`, `NaN` y valores fuera de rango. Las mínimas de distancia pueden existir aunque un sector no alcance cobertura. `edad_rx` mide el tiempo desde el último callback local; `rx` es la frecuencia media de los últimos 30 callbacks, no la frecuencia medida en el sensor. Si aparece una parada, terminar con Ctrl+C y repetir sin `--mover` para recoger estos datos. Volver a ejecutar el lanzador copia la versión y configuración actuales a Docker.

**Probar el reconocimiento con la cámara del robot.** Primero detener la evasión con Ctrl+C y cerrar los otros controles. Mantener encendidos los controladores del robot y Docker; ejecutar desde Ubuntu normal:

```bash
bash "task 1/ejecutar_autonomo_robot.sh" --ip "$TB4_IP" --dominio "$TB4_DOMINIO" --solo-reconocimiento
```

Abre una ventana con un campo de texto, el botón **Buscar**, sugerencias de la galería y vídeo. Escribir el nombre y pulsar Buscar o Enter lo activa como objetivo; hacer clic en una sugerencia rellena el campo, pero todavía hay que confirmar. Se puede escribir sin tildes y sin distinguir mayúsculas; un nombre vacío, inexistente o ambiguo muestra un aviso y conserva el objetivo activo anterior. Editar el texto tampoco lo cambia hasta confirmar. La línea `Buscando` distingue ese objetivo del nombre `Reconocido` por la cámara.

Si reconoce el objetivo muestra «Reconozco a la persona que buscas». Si acepta otra identidad, conserva su nombre y muestra «Reconozco a otra persona; no es la que buscas». Un rechazo sigue siendo `NO_CONCLUYENTE`, nunca se convierte en el nombre solicitado. En este modo individual es reconocimiento estacionario y se puede cambiar de objetivo; el modo combinado fija el nombre al iniciar.

Usa por defecto `/oakd/rgb/preview/image_raw/compressed`; `--topico-camara /otro/topico` cambia el origen. Si la cámara tarda en arrancar, la ventana muestra `EN ESPERA` y el estado de cámara; permanece abierta sin límite de espera por el primer fotograma. El reconocimiento empieza automáticamente al recibir imágenes recientes. Colocar una fotografía de una persona presente en la galería, frontal y bien iluminada, sin otras caras en la imagen. Para evaluar, usar una foto reservada de evaluación y registrar rechazos/errores; una demostración con la misma referencia no mide generalización. **Esc**, el botón **Salir**, cerrar la ventana o Ctrl+C terminan. `q` también termina cuando el campo no tiene el foco; mientras se escribe forma parte del nombre. `--sin-ventana` conserva la prueba por terminal y permite fijar el objetivo con `--nombre`.

La comprobación inicial muestra los tópicos de imagen visibles tras diez segundos de descubrimiento. Que el solicitado aún no aparezca permite abrir el receptor y seguir esperando; no cambia automáticamente a otro tópico. Un tipo incompatible o un error real de Docker/ROS sí se informa y detiene el arranque. No iniciar otro controlador ni reiniciar servicios solo porque la cámara esté estabilizándose.

La visión se ejecuta en la `.venv` global del host. `prueba_facial_robot.py` reutiliza los módulos de transporte de cámara de Task 2 sin modificarlos ni lanzar gestos; copia solo esos dos módulos al Docker, en una carpeta temporal propia. No requiere OpenCV/cv_bridge en Docker. No crea publicadores de velocidad y rechaza `--mover` o mezclar evasión/reconocimiento. Tampoco guarda imágenes ni cambia modelos, galería, partición o criterio; verifica su versión y las huellas de modelos/manifiesto antes de empezar y no descarga en el laboratorio. Se mantiene el mismo preprocesamiento del notebook, sin CLAHE ni relajación de umbrales. La decisión desaparece si la imagen excede 1 s desde recepción local, incluso si caduca durante la inferencia; esa edad no mide latencia del sensor. Un paquete repetido reutiliza su resultado y no acumula votos.

Opcionalmente, `--nombre "Etiqueta exacta"` precarga el objetivo; también puede elegirse con el campo de texto sin añadir esta opción. Otra persona o un rechazo no lo confirma. Consultar las etiquetas locales sin conectarse al robot:

```bash
.venv/bin/python "task 1/prueba_facial_robot.py" --listar-nombres
```

Por ahora exige una sola cara: varias producen `VARIOS_ROSTROS`, conservando el contrato del extractor. La búsqueda individual entre varias fotos y la asociación QR siguen pendientes. Los choques diagonales reportados durante la evasión tampoco se consideran resueltos; probar esta etapa con el robot quieto.

**Abrir el notebook.** Desde la raíz de Proyecto2:

```bash
code "task 1/01_reconocimiento_rostros.ipynb"
```

Selecciona el kernel de `.venv` en la raíz y ejecuta las celdas en orden. El notebook admite directorio de trabajo en la raíz o en `task 1`. Las dependencias ya están instaladas y los dos modelos se descargaron y verificaron por SHA-256; las siguientes ejecuciones no necesitan internet. Para otra computadora, consultar `requirements.txt` y la celda de preparación de modelos.

El notebook tiene 24 celdas, once de código. Explica entorno y rutas, modelos, inventario, partición, extracción, galería, calibración, evaluación, consulta configurable, una prueba sin rostro y una consulta externa opcional.

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

**Prueba externa de la sección 11.** Se incorporó la consulta propuesta por Landov311 en [paz_y_rostros](https://github.com/Manueljsilva/turtlebot4-vision-proyecto/pull/1). Coloca una fotografía en `task 1/datos/rostro_prueba_1.jpeg` y ejecuta la última celda después de preparar la galería y el criterio. También puedes editar `CONSULTA_EXTERNA` para elegir otro archivo. Si no existe, la prueba se omite con un mensaje y el notebook termina normalmente.

Muestra la decisión y el ranking completo usando los modelos y umbrales ya preparados. La foto no se incorpora a las referencias ni recalibra el criterio. `MOSTRAR_FOTO_EXTERNA=False` conserva la visualización desactivada por defecto; se puede activar localmente para inspeccionar la detección.

**Qué significa limpiar antes de compartir.** Un notebook guarda tanto el código como las salidas de cada ejecución: tablas, mensajes y fotografías mostradas. La copia publicada en GitHub se guarda sin esas salidas para evitar incluir imágenes personales. Se conservan el código, las fotos originales de `rostros/` o `datos/` y las ejecuciones del notebook local.

**Galería vigente, ampliada el 13 de noviembre de 2026.** El usuario confirmó que la prueba facial del robot funciona, pero faltaban personas nuevas. La carpeta contiene ahora 74 fotografías de 13 personas; la galería anterior tenía cinco personas. Se conservó el papel de las 21 fotos anteriores que siguen presentes, identificado por SHA-256, y se repartieron las identidades nuevas sin mezclar referencias y consultas reservadas. Hay 48 fotos destinadas a galería, 13 a calibración y 13 a evaluación. Una referencia produjo `SIN_ROSTRO` y quedó excluida; las 13 personas tienen referencias válidas, 47 en total. No se modificaron fotografías, modelos, parámetros del detector ni ejecuciones del notebook.

El criterio se recalibró exclusivamente con las 13 consultas de calibración: umbral coseno 0.528145 y margen mínimo 0.162767. Después se evaluaron las 13 fotos reservadas: 13 nombres correctos, cero nombres erróneos y cero rechazos. También se rechazaron los 13 negativos simulados por retirada de identidad; no sustituyen desconocidos reales. Es una evaluación exploratoria por foto, sin independencia demostrada entre tomas ni validación de las nuevas personas a través de la cámara. Los resultados anteriores del notebook y de este documento son históricos. Los artefactos vigentes están en `datos/` y `resultados/evaluacion.json`; `datos/actualizacion_galeria.json` registra el resumen y la copia local anterior en `datos/historial/`. Todo ello sigue fuera de Git.

Cerrar y volver a ejecutar `--solo-reconocimiento` carga la galería ampliada; la terminal debe indicar `Galería local: 13 personas`. No hace falta reiniciar Docker ni el controlador de cámara. Pasaron las 59 pruebas de Task 1 y la carga desde la raíz y desde esta carpeta. La prueba sigue exigiendo una sola cara por imagen.

**Si llegan más alumnos o fotos.** El manifiesto detecta los cambios y evita mezclar silenciosamente nuevas referencias con una evaluación anterior. Antes de regenerarlo, conservar una copia local de los artefactos y revisar que las fotos reservadas anteriores no pasen a referencias. Usa `REGENERAR_PARTICION=True` y repite desde la partición hasta el final; luego vuelve a `False`. Con una foto solo puede haber referencia; hacen falta más tomas para calibrar/evaluar a esa persona. No reutilizar la foto de consulta como su propia referencia ni ajustar los umbrales para arreglar un error del test. El programa de cámara no actualiza la galería mientras está abierto.

| Archivo/carpeta | Función |
| --- | --- |
| [01_reconocimiento_rostros.ipynb](01_reconocimiento_rostros.ipynb) | Experimento explicado paso a paso |
| [reconocimiento_facial.py](reconocimiento_facial.py) | Datos, partición, extracción, galería y decisión reutilizables |
| [modelos_rostros.py](modelos_rostros.py) | Preparación y comprobación de modelos |
| [ejecutar_autonomo_robot.sh](ejecutar_autonomo_robot.sh) | Pruebas separadas de LiDAR y reconocimiento físico; movimiento solo en evasión con --mover |
| [prueba_facial_robot.py](prueba_facial_robot.py) | Cámara ROS del robot, galería y visión en la laptop, sin publicar velocidades |
| [interfaz_facial.py](interfaz_facial.py) | Campo de nombre, sugerencias, Buscar y estados de coincidencia en la ventana existente |
| [evasion_lidar.py](evasion_lidar.py) | Sectores, validación y decisiones sin dependencia de ROS |
| [busqueda_robot.py](busqueda_robot.py) | Interfaz y visión del host coordinadas con evasión |
| [control_busqueda.py](control_busqueda.py) | Inicio, confirmación, A y permiso de movimiento sin ROS |
| [prueba_senales_robot.py](prueba_senales_robot.py) | Cámara y señales del tramo, siempre sin movimiento |
| [geometria_senales.py](geometria_senales.py) | Geometría de la compañera adaptada, sin SciPy/ROS |
| [senales_giro.py](senales_giro.py) | Color, dirección y confirmación de señales con imágenes nuevas |
| [control_giros.py](control_giros.py) | Maniobra por odometría, bloqueos, timeout y rearme |
| [config/senales_giro.json](config/senales_giro.json) | Ángulo, confirmación, tamaño visual y rearme |
| [busqueda_lidar_ros.py](busqueda_lidar_ros.py) | Nodo de evasión con permiso de visión, odometría y parada final |
| [seguridad_lidar.py](seguridad_lidar.py) | Suscriptor LiDAR/TF y publicador opcional en Docker |
| [config/evasion_lidar.json](config/evasion_lidar.json) | Velocidades, distancias y caducidad de la prueba |
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

**Ensayo del punto de giro del compañero (ajustado el 20 de noviembre).** Opción `--ensayo-giro-companero` junto a `--evasion-reconocimiento --senales`: memoriza flecha y espera retorno frontal a **0.48 m**, conservando 90° por odometría. El perfil activo es [config/senales_giro_ensayo.json](config/senales_giro_ensayo.json), cargado tanto en host como en el nodo ROS. Solo cambia el disparo frente al JSON original de 0.52 m; controlador y JSON originales de e78c3b5 siguen intactos y verificados. Perfil LiDAR local (0.10 m/s y 0.40 rad/s, precaución 0.45 m, bloqueo 0.32 m), sin fusionar toda la rama. Memoria original de seis segundos y problemas de coordinación todavía presentes; confirmar temprano no inicia el giro y la evasión sigue actuando mientras espera. Cerrar/relanzar el mismo comando para copiar el ajuste; pendiente validación física. [Instrucciones y límites](experimentos/mejora_competicion/README.md); comandos sin/con movimiento en bloques 16/17 de COMANDOS.txt. No hay publicador sin --mover. Cerrar y quitar la opción recupera el comportamiento habitual. El ensayo meta/rostro/QR se conserva pendiente.

**Siguiente etapa.** Ensayar ese punto de giro y corregir aproximación/caducidad con evidencia, sin dar por resueltos choques diagonales ni vídeo retrasado. Después retomar QR, integrar meta/llegada, flechas y vuelta a A para mostrar su palabra. El notebook, modos anteriores y galería se conservan; el usuario confirmó movimiento/reconocimiento y meta/giro/rostro/parada del ensayo B, no QR ni misión completa. SLAM/localización/planificación y retorno siguen pendientes de integración selectiva.

**Verificación de la prueba LiDAR (13 de noviembre).** Pasaron 42 pruebas de Task 1: 14 faciales existentes, 23 de evasión y cinco del lanzador con dobles de Docker/sudo/ping. Incluyen frente NaN con un único retorno trasero, laterales desconocidos, caducidad, TF ausente en la decisión, obstáculo cercano, orientación, configuración inválida, publicación desactivada por defecto y recuentos diagnósticos. Un patrón sintético con 40 % finito/60 % `+inf` reproduce el bloqueo anterior al exigir 50 % y permite continuar al exigir 30 %; no reconstruye el escaneo real. Se conserva parada con cobertura inferior, datos erróneos abundantes o `-inf`, y detección de obstáculos próximos entre `+inf`. Se verificaron los nuevos umbrales frontal/lateral y sintaxis Python 3.12. El asistente no ejecutó ROS, Docker real ni hardware; recepción sostenida, frenado y esquive reales siguen pendientes.

**Verificación previa de la prueba facial preparada (histórico).** Pasaron 59 pruebas de Task 1: las 54 anteriores y cinco de descubrimiento tardío, errores y espera prolongada. Se comprobó carga real de galería/criterio/modelos locales e imagen negra, coincidencia de objetivo, rechazos, repetición/caducidad de imágenes, ausencia de comandos de movimiento, red/tipo inválidos y delegación del lanzador al host. La espera sigue activa incluso después de cinco minutos simulados, hasta interrupción del usuario; no es una medición del arranque real. Sintaxis Python 3.12 y Bash correcta, ayudas CLI y panel sintético revisados. No se recibió cámara física ni se ejecutó ROS/Docker real desde el asistente; falta la prueba del usuario con fotografías. Se conservaron los hashes de Task 2, modelos, datos, notebooks y lógica de evasión.

**Entrada de nombre en interfaz (13 de noviembre).** Tras confirmar el usuario la galería ampliada, se añadió el campo y Buscar. Pasaron 71 pruebas locales: las 59 anteriores, once de entrada/decisión y una de conexión de la GUI con apertura/cierre simulados. Cubren confirmación explícita, cambio de objetivo sin repetir inferencia, otra identidad conocida, rechazo, datos caducados, sugerencias, nombre inválido/ambiguo, escritura de q y salida. Cuatro paneles sintéticos revisados sin fotografías personales. La interfaz usa el OpenCV ya instalado, sin paquetes nuevos. Sintaxis Python 3.12 y carga de 13 etiquetas desde raíz/task 1 correctas; galería, criterio, modelos, notebooks, evasión y Task 2 conservaron hashes. Falta comprobar teclado y clics reales con el usuario; no se integró movimiento.

Referencias técnicas: [OpenCV, detección y reconocimiento](https://docs.opencv.org/4.x/d0/dd4/tutorial_dnn_face.html). Alcance y coordinación: [PLAN_PROYECTO.md](../PLAN_PROYECTO.md), [ESTADO_PROYECTO.md](../ESTADO_PROYECTO.md) y [enunciado](../Computer_vision_proyectos.pdf).
