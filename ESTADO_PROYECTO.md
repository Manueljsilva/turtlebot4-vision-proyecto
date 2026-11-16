# Estado del proyecto

Actualizado: 16 de noviembre de 2026. Resumen técnico compartido; los datos y las notas de entorno locales no forman parte del repositorio.

## Task 1

Implementado: reconocimiento local YuNet/SFace, galería etiquetada, calibración separada y rechazo NO_CONCLUYENTE; cámara ROS con espera durante el arranque; campo de nombre, sugerencias y Buscar; prueba LiDAR independiente y modo combinado `--evasion-reconocimiento [--mover]`.

El modo combinado fija el nombre con Buscar, registra A con odometría válida posterior al inicio y aplica una única salida de velocidad condicionada por LiDAR/TF, posición, cámara y actividad recientes. Pausa desde la primera coincidencia y confirma tres imágenes nuevas consecutivas. Al confirmar queda detenido. No modifica la galería durante la prueba ni guarda fotografías. A y los archivos de sesión quedan fuera de Git.

El usuario confirmó por separado movimiento con evasión y funcionamiento del reconocimiento/cámara. **No alcanzó a ejecutar la prueba combinada en el laboratorio**; la captura física de A, el comportamiento conjunto y las paradas siguen pendientes. Los choques diagonales y cortes de recepción reportados en la evasión no se consideran resueltos.

Datos locales actuales: 74 fotos de 13 personas; 48 referencias previstas (47 válidas), 13 consultas de calibración y 13 de evaluación. Evaluación exploratoria: 13 nombres correctos, cero nombres erróneos y cero rechazos; 13 negativos simulados por retirada de identidad rechazados. No demuestra independencia entre tomas, precisión física ni rechazo general de desconocidos. Las fotos, vectores y pesos no se publican.

**No hay retorno automático a A**, navegación SLAM/Nav2 integrada, lectura de flechas/QR ni búsqueda entre varias fotografías. El alcance completo permanece en PLAN_PROYECTO.md; no confundir registrar A con planificar el regreso.

## Task 2

Gestos con webcam, MediaPipe y reglas propios confirmados en simulación y robot real. El usuario confirmó vídeo comprimido en el panel lateral; conserva retraso sin medir. No hay misión completa validada ni reconocimiento facial integrado en este modo.

## Informe

`informe/main.tex`, el borrador Markdown y el PDF compilado corresponden al 7 de noviembre. Continúan como borrador: faltan datos de portada y adaptar el alcance a las flechas/QR y avances posteriores. No se presentan como informe final ni entregado.

## Validación y siguiente paso

El 16 de noviembre pasaron **91 pruebas de Task 1 y 51 de Task 2**. Incluyen modelos locales disponibles, lógica, caducidad, sensores ausentes, salida única, ROS simulado y transporte por subproceso local. No son pruebas físicas de la integración. Las copias publicadas de notebooks no tienen salidas; los originales locales se conservan.

Próxima visita: seguir bloques 9/10 de `task 1/COMANDOS.txt`, comprobar primero sin publicación y después con --mover en un espacio despejado. Verificar A válido, inicio por Buscar, reconocimiento de objetivo/otra persona, pérdida de cámara y parada final. Continuar después con navegación/localización y retorno, corrección de evasión, flechas, asociación rostro-QR y misión completa. La edad de imagen se mide desde recepción local, no como latencia del sensor; el vigilante no garantiza entregar una parada con corte de red o muerte del nodo ROS.
