# Estado del proyecto

Actualizado: **19 de noviembre de 2026**. El usuario salió del laboratorio antes de probar la evasión nueva que gira sobre el sitio. Quedan pendientes esa prueba física y el ajuste de distancia en la próxima visita. Meta/rostro/QR sigue conservado y aplazado. Fotografías, modelos, galería y sesiones permanecen locales.

## Misión final acordada

Nombre fijado en A → obstáculos y flechas azules → confirmar meta B → solo entonces barrer y buscar a la persona objetivo entre fotos → confirmar su rostro y leer únicamente su QR → conservar palabra → volver con flechas rojas → confirmar nueva meta A → detenerse y mostrar palabra.

La flecha fija el sentido ante un muro; seguridad puede detener sin invertirla. Completar 90° por odometría antes de avanzar. Interpretar la misma apariencia de meta según etapa y separación, sin contar A al salir como B ni reutilizar B como regreso a A. La misión completa y el retorno todavía no están integrados.

## Task 1: implementado y evidencia

- `--solo-evasion`: LiDAR/TF y cobertura válidos; +inf no equivale a espacio libre y -inf detiene. Avance 0.10 m/s, giro hasta 0.40 rad/s, precaución 0.45 m, parada 0.32 m y lateral 0.28 m. **Cambio nuevo: las correcciones frontales/laterales ordenan avance cero durante el giro**, reanudando avance recto al despejar. Pendiente probar este cambio físicamente; los choques diagonales no se consideran resueltos.
- `--solo-reconocimiento`: visión en la laptop, galería/criterio existentes, nombre/Buscar. Confirmado por el usuario; otra identidad conocida conserva su nombre, rechazo sigue NO_CONCLUYENTE. No publica movimiento.
- `--evasion-reconocimiento [--mover]`: movimiento/reconocimiento juntos confirmados por el usuario. Buscar fija el nombre y registra A con odometría válida posterior al inicio; no inventa origen ni lo llama mapa. Pausa desde el primer acierto facial y confirma tres imágenes nuevas. Una sola salida ROS y permiso por actividad, cámara, odometría y LiDAR/TF recientes. Todavía reconoce durante la marcha; no equivale a la misión completa.
- `--solo-senales --tramo ida|vuelta`: cámara y color/dirección, sin movimiento ni galería. `--senales` en el modo combinado añade giro de 90° por odometría, con bloqueo, timeout y rearme. El controlador habitual gira al confirmar; no incluye aproximación. Tramo elegido antes; rojo no constituye retorno automático.

## Ensayo aislado de giros del compañero

`--evasion-reconocimiento --senales --tramo ida --ensayo-giro-companero [--mover]` usa solo controlador/JSON de señales del snapshot [e78c3b5](https://github.com/Manueljsilva/turtlebot4-vision-proyecto/tree/e78c3b52d07be0add80103365bb2ad5fc068e91b), verificados por hash, con evasión local. No fusiona misión, cambios de cámara, informe ni presentación de la rama.

Retiene flecha y espera frente <=0.52 m. Frente/laterales <=0.32 m bloquean el giro; conserva memoria original de seis segundos y límite de doce segundos para completar. Se reprodujeron caducidad aun con el mismo cartel visible y conflictos de coordinación con evasión; todavía no están corregidos. La copia original permanece intacta y puede compararse con el controlador habitual quitando la opción.

La prueba con cámara/sensores reales **sin --mover** confirmó DERECHA azul 3/3 después de Buscar y envío/recepción/consumo de IDs. Una primera muestra de maniobra activa tuvo frente ~0.407 m, pero lado derecho ~0.25 m bloqueó todas las propuestas de giro. Publicando=false: no valida rotación física ni ángulo. El fragmento posterior seguía en FALLO por tiempo agotado; mostrar otro cartel no libera ese fallo.

`[SENALES ENSAYO]` distingue vista previa/inicio, confirmaciones, ID y envío. `[GIRO ENSAYO]` muestra recepción/consumo, fase, permiso, distancia y propuesta. **GIRANDO puede estar bloqueado**: comprobar motivo en [BUSQUEDA], wz_propuesto y publicando. `frente_m` es distancia al retorno frontal del LiDAR, no metros exactos a la flecha ni margen de carrocería.

Comandos en bloques **16/17** de [task 1/COMANDOS.txt](<task 1/COMANDOS.txt>); límites y procedencia en [la guía del ensayo](<task 1/experimentos/mejora_competicion/README.md>).

## Ensayo de meta, frenado, rostro y QR: conservado y pendiente

`--prueba-meta-b [--avanzar-hasta-meta] [--mover]`: nombre/Buscar → evasión previa opcional → meta en tres imágenes nuevas → publicar cero y confirmar 0.7 s de reposo odométrico → barrido sobre el sitio → persona objetivo → solo su QR → guardar palabra/parar. No sigue flechas, registra A ni regresa. Sin --mover no crea publicador.

El usuario confirmó avance, meta, barrido, rostro y parada en el robot. QR no confirmado: se veía pequeño y en una esquina. Se añadieron diagnóstico detectado/legible/asociado, reintentos acotados por recorte y asociación mediante contorno de la misma foto con una cara/un QR; esta revisión requiere prueba física. No fuerza el objetivo, acepta QR ajeno ni guarda imágenes. Resultado textual en sesiones locales ignoradas.

Se separaron cortes de recepción /odom de saltos espaciales: corte coherente obliga a cero y >=3 paquetes nuevos durante 0.7 s estables antes de recuperar; salto, cambio de marco o corte >2 s en barrido enclavan movimiento. Conservar primer motivo y ángulo/plazo. Una parada terminal no libera movimiento, aunque pueda continuar visión estacionaria ya habilitada. Barrido limitado a una vuelta/90 s y <=0.25 rad/s.

Comandos en bloques **13–15**. El usuario aplazó este ensayo para priorizar el punto de giro; se retoma después.

## Comprobaciones y próxima visita

Pasaron **171 pruebas de Task 1** con visión sintética, tubería local real y dobles de ROS/GUI; **51 de Task 2** comprobadas y sus fuentes conservadas. La revisión de evasión verifica ambos sentidos, 96 combinaciones sin avance/giro simultáneos, caducidad y reanudación solo al despejar, además de la salida única de velocidad. Sintaxis Python 3.12 y Bash verificadas. El asistente no ejecutó movimiento físico ni ROS/Docker real.

En la próxima visita:

1. Preparar red/robot/Docker existentes y confirmar IP/dominio. Cerrar sesiones anteriores en FALLO y otros controles.
2. Comprobar el ensayo actualizado sin --mover y luego medir giro sobre el sitio con movimiento explícito, frenado, distancia y 90°.
3. Ajustar entonces el disparo `esperar_pared_m` (hoy 0.52 m), coordinándolo con evasión y margen de bloqueo. `frente_m` solo es lectura. No se modifica ahora la distancia ni se reducen barreras.
4. Retomar QR y más adelante integrar meta B, flechas, retorno rojo y llegada A.

Faltan validación de huella/choques diagonales, estabilidad de recepción, impresiones distintas, asociación QR y misión completa. La edad de imagen es desde recepción local y no mide retraso previo del sensor/red. El vigilante no garantiza entregar una parada ante corte de red o muerte del nodo ROS.

## Task 2, datos e informe

Gestos confirmados en simulación y robot con vídeo comprimido lateral; vídeo retrasado sin medir. Se conserva Task 2 y su objetivo anterior: identificación en B, retorno por gestos y nombre al regresar. Mantener .venv global y Docker existentes.

Galería/calibración/modelos son locales; no se actualizan ni recalibran al arrancar. La evaluación exploratoria previa no demuestra precisión general con impresiones o desconocidos reales.

El informe publicado (LaTeX, Markdown y PDF de diez páginas) sigue siendo el borrador del 7 de noviembre. Debe adaptarse al alcance y evidencias posteriores antes de entregar. No se incorporó la presentación del compañero ni se entregó en Canvas. Alcance y pendientes en [PLAN_PROYECTO.md](PLAN_PROYECTO.md).
