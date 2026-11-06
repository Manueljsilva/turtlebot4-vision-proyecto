# Gestos de webcam para Gazebo y robot real: Task B

El programa conserva la percepción del notebook: MediaPipe Hand Landmarker, `gestos.py`, imagen sin espejo y filtro temporal de 0.4 s. El notebook está en esta carpeta y la `.venv` compartida permanece en la raíz de Proyecto2. Todo el código activo de esta modalidad está en `task 2`.

La conexión implementada es:

```text
Webcam del host → webcam_gestos.py (.venv, MediaPipe)
              → órdenes JSON por una tubería de docker exec -i
              → puente_gestos_ros.py (Python de ROS Jazzy, dentro de Docker)
              → /cmd_vel, TwistStamped → TurtleBot simulado o real
```

La webcam permanece en tu computadora. El script copia tres módulos pequeños al contenedor existente `proyecto2-sim`; no reinstala paquetes, no recrea el contenedor y no necesita montar la cámara. El transporte usa la [entrada estándar de Docker exec](https://docs.docker.com/reference/cli/docker/container/exec/). Los relojes monotónicos del protocolo se usan solo entre este host y su contenedor; adaptar antes de comunicar computadoras diferentes.

**Robot real.** Completa primero la conexión de `../COMANDOS_LAB.txt`. Cierra Gazebo, teleop y cualquier captura de webcam. Desde Ubuntu normal, fuera de Docker/SSH y desde la raíz de Proyecto2:

```bash
bash "task 2/ejecutar_gestos_robot.sh" --ip 10.87.179.100 --dominio 5
```

Comprueba las órdenes sin publicar velocidad; cierra la prueba y repite añadiendo `--mover`. Prueba palma/ausencia de mano, avance breve, palma y giros. Los bloques 9 y 10 de `COMANDOS.txt` contienen los comandos completos. IP/dominio corresponden a la unidad comprobada el 5 de noviembre, no son permanentes. El lanzador comprueba alcance de IP, Docker host y `/cmd_vel` stamped; configura dominio, Fast DDS y peer. No inicia servicios ni modifica la configuración del robot.

El modo físico selecciona `--robot-real`: `use_sim_time=False`, estampas con reloj del sistema y sin dependencia de `/clock`. Conserva el vigilante monotónico de 0.5 s, temporizador de 20 Hz, las mismas reglas de gestos, parada al cerrar y velocidades de 0.306 m/s y ±1.90 rad/s. Webcam y puente siguen en la misma laptop. La validación física impide valores fuera de 0.306 m/s y 1.90 rad/s, no finitos o DETENER distinto de cero. Cierra los demás controles antes de activarlo.

**Preparación para Gazebo.** Los pasos siguientes corresponden al simulador. Deja Gazebo y su terminal abiertos. Si todavía no está abierto, usa una vez el bloque 3 de `COMANDOS.txt`. Cierra el control por teclado con Ctrl+C en su terminal. Interrumpe la webcam del notebook o cierra cualquier otra aplicación que la esté usando. Si ya hay una prueba de gestos abierta, ciérrala antes de iniciar otra.

**Primero comprobar sin movimiento.** Los comandos siguientes parten de la raíz de Proyecto2, no de `task 2`. En otra terminal normal de Ubuntu:

```bash
cd "/home/manueljsilva/Documentos/Cursos/Vision-comptacion/Proyecto2"
bash "task 2/ejecutar_gestos_gazebo.sh"
```

Si pide contraseña, es la de `sudo` en tu computadora. Se abre una ventana de webcam y en la terminal aparece `COMPROBACION: no se publica velocidad`. Al hacer gestos, el puente dentro de Docker debe mostrar `AVANZAR`, `IZQUIERDA`, `DERECHA` o `DETENER`. Este modo no crea un publicador de velocidad. Si aparece `Esperando /clock activo de Gazebo`, comprueba que la simulación está abierta y en Play.

La imagen sigue sin espejo. La convención se actualizó por petición del usuario el 5 de noviembre: índice arriba avanza; tu derecha ordena DERECHA del robot y tu izquierda, IZQUIERDA, estando frente a la webcam. El clasificador convierte el eje horizontal de imagen una sola vez. Palma abierta, ausencia de mano o ambigüedad detienen. Mantén una dirección al menos 0.4 s.

**Después probar movimiento.** Cierra primero la ventana de la prueba anterior y ejecuta:

```bash
bash "task 2/ejecutar_gestos_gazebo.sh" --mover
```

Aparece `MOVIMIENTO ACTIVADO`. Ahora el puente publica a 20 Hz con el reloj simulado. Velocidades: avanzar a 0.306 m/s; girar sobre el sitio a +1.90 rad/s para izquierda y -1.90 rad/s para derecha; detener con ambas a cero. El texto de la webcam es la orden enviada; la terminal del puente muestra la salida efectiva y cualquier motivo de parada. No ejecutar a la vez el control por teclado u otro programa que mande velocidades.

Prueba primero palma abierta y mano retirada; luego avance breve, palma, giro izquierdo, palma y giro derecho. Comprueba el sentido desde la orientación del robot. Si no coincide, detén con la palma y revisa la convención antes de continuar. Los controles `q`, cierre de ventana y Ctrl+C sirven para terminar estas pruebas de desarrollo; no forman parte de la interacción admitida durante un intento oficial de Task B.

**Paradas implementadas.** La pérdida de mano, más de una mano y los gestos ambiguos producen `DETENER`. Cambiar de dirección obliga a confirmar la nueva. El puente vigila por separado la antigüedad de la captura: si supera 0.5 s, ordena cero, revisándolo cada 0.05 s aunque la captura esté bloqueada. Los mensajes inválidos también detienen. El final del enlace hace publicar cero y termina el puente. Sin reloj simulado activo o sin suscriptores de velocidad, la salida es cero. Una pausa o reset de la simulación se comprueba mediante el avance de `/clock`.

El vigilante depende de que el proceso ROS continúe ejecutándose; no puede publicar si ese proceso o Docker terminan forzosamente. En modo físico tampoco garantiza entregar la parada si se corta la red hacia el robot. No se ha medido aún el tiempo de frenado físico. El programa no incorpora barrera LiDAR ni reconocimiento facial, y no completa todavía la misión A → B → reconocer → A → nombre.

**Si la webcam no abre.** Comprueba que el notebook dejó de capturar. Para otra cámara:

```bash
bash "task 2/ejecutar_gestos_gazebo.sh" --camara 1
bash "task 2/ejecutar_gestos_gazebo.sh" --mover --camara 1
```

Si el script dice que el contenedor no está en ejecución, abre primero Gazebo con `COMANDOS.txt`. Si dice que ya hay un puente abierto, cierra la prueba anterior; el bloqueo evita iniciar dos copias en ese contenedor. Si falta el modelo, prepáralo desde `Tarea1.ipynb` antes de ejecutar. No hace falta instalar `rclpy` en la `.venv`.

**Archivos y comprobaciones.** `webcam_gestos.py` captura y reconoce; `control_gestos.py` define mensajes, velocidades y caducidad; `puente_gestos_ros.py` recibe y publica; `ejecutar_gestos_gazebo.sh` prepara las copias y une los procesos. `gestos.py` es común al notebook, prueba local y ambos lanzadores; adapta los lados del operador a las órdenes del robot.

```bash
.venv/bin/python -m unittest discover -s "task 2/tests" -t "task 2" -v
```

En esta implementación pasaron 25 pruebas, incluidas tubería con procesos reales, emisor bloqueado, EOF, mensajes inválidos, capturas antiguas, cambio de dirección y pausa/reset del reloj. Se procesaron 45 fotogramas negros con MediaPipe real y una parada final, con JSON válido; también se ejecutó la tubería completa de video a receptor local sin ROS. Se comprobó la sintaxis Bash y la sintaxis Python 3.12 de los módulos. Estas comprobaciones locales no prueban precisión con manos reales ni ejecución del puente en ROS. El asistente no ejecutó el puente en el contenedor, porque `sudo` requiere autenticación interactiva; la confirmación posterior de funcionamiento proviene del usuario.

El usuario confirmó que los gestos con Gazebo funcionan y, el 5 de noviembre, la conexión ROS/LiDAR y el movimiento por teclado del robot real. Ese día se implementó el modo físico gestual: pasaron **37 pruebas locales** en total. Las nuevas cubren el ciclo ROS con dobles (reloj real sin `/clock`, reloj simulado, caducidad, ausencia de suscriptores, cierre y modo sin publicación), límites de velocidad y el lanzador con Docker/red simulados desde una ruta con espacios. También pasaron sintaxis Bash y Python 3.12. El asistente no ejecutó webcam, ROS/Docker ni movimiento físico durante esta adaptación. Después, el usuario confirmó movimiento real con el lanzador de gestos. A continuación pidió máximos de velocidad e invertir derecha/izquierda: se aplicaron a la tabla y al clasificador compartidos y pasaron 38 pruebas locales, incluida la conversión mano → filtro → enlace → giro y caducidad. Los nuevos ajustes requieren comprobación física. Falta medir frenado e integrar reconocimiento y misión completa.

**Prueba local conservada.** Para usar el detector sin simulador ni Docker, desde la raíz de Proyecto2:

```bash
.venv/bin/python "task 2/webcam_gestos.py"
```

El modelo está junto al código, en `task 2/modelos/hand_landmarker.task`. El lanzador calcula por separado la carpeta de la tarea y la raíz que contiene `.venv`; las rutas con espacios se entrecomillan. Las pruebas locales y la integración emplean la misma lógica de gestos.

**Reorganización comprobada el 27 de octubre.** Las 25 pruebas pasaron desde la nueva ubicación; el notebook cargó el modelo y ejecutó sus celdas sin webcam desde la raíz y desde `task 2`. El detector y el transporte local se probaron desde `/tmp` con video negro. Se conservaron la `.venv`, el modelo y las salidas guardadas. Esta revisión verificó rutas sin ejecutar nuevamente Gazebo ni la webcam real; el movimiento simulado corresponde a la confirmación anterior del usuario.
