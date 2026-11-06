# Proyecto de visión con TurtleBot 4 Lite

Proyecto de cuatro integrantes. Según el [enunciado oficial](Computer_vision_proyectos.pdf), ambas modalidades deben ir de A a B, identificar a la persona de la fotografía en B, regresar a A e indicar su nombre.

Para compartir avances: [COMPARTIR_GITHUB.md](COMPARTIR_GITHUB.md). El repositorio se prepara sin fotos de alumnos, modelos descargados, `.venv` ni salidas de notebooks; los originales locales se conservan.

- [Task 1 / Task A](<task 1/README.md>): notebook de reconocimiento facial implementado y probado con 26 fotos de cinco alumnos. Navegación autónoma pendiente.
- [Task 2 / Task B](<task 2/README.md>): gestos con Gazebo y robot real confirmados por el usuario. Se ajustaron velocidad máxima y giros a sus lados; falta comprobar esos ajustes en hardware e integrar reconocimiento y misión completa.

**Conexión al robot real para ambas tareas:** [COMANDOS_LAB.txt](COMANDOS_LAB.txt), en la raíz. Copiar un bloque cada vez: datos de sesión → hora/reinicio del robot una vez por visita → Docker con dominio/peer → comprobación de LiDAR. Incluye abrir más terminales, SSH, teclado opcional y cierre. La recepción de LiDAR desde Docker fue confirmada por el usuario el 5 de noviembre. Los gestos físicos se ejecutan después con los bloques 9 y 10 de [task 2/COMANDOS.txt](<task 2/COMANDOS.txt>).

```text
Proyecto2/
├── .venv/                    Entorno Python compartido
├── .gitignore
├── AGENTS.md                 Instrucciones estables
├── ESTADO_PROYECTO.md         Avance y siguiente paso
├── PLAN_PROYECTO.md           Alcance y organización
├── Computer_vision_proyectos.pdf
├── COMANDOS_LAB.txt           Conexion comun al robot real
├── README.md
├── task 1/
│   ├── 01_reconocimiento_rostros.ipynb
│   ├── reconocimiento_facial.py
│   ├── modelos_rostros.py
│   ├── requirements.txt
│   ├── COMANDOS.txt
│   ├── README.md
│   ├── rostros/              Fotos originales, excluidas de Git
│   ├── modelos/              YuNet y SFace, pesos fuera de Git
│   ├── datos/                Partición y galería, fuera de Git
│   ├── resultados/           Evaluación, fuera de Git
│   └── tests/
└── task 2/
    ├── Tarea1.ipynb           Prueba local conservada, sin ROS
    ├── webcam_gestos.py       Prueba local y fuente para ambos modos ROS
    ├── gestos.py              Clasificador y filtro temporal
    ├── control_gestos.py      Mensajes, velocidades y caducidad
    ├── puente_gestos_ros.py   Receptor/publicador en Docker
    ├── ejecutar_gestos_gazebo.sh
    ├── ejecutar_gestos_robot.sh
    ├── requirements.txt
    ├── COMANDOS.txt
    ├── INTEGRACION_GESTOS.md
    ├── README.md
    ├── modelos/              Modelo de mano, excluido de Git
    └── tests/
```

**Notebook facial de Task 1:** [01_reconocimiento_rostros.ipynb](<task 1/01_reconocimiento_rostros.ipynb>), kernel de la `.venv` global y celdas en orden. Modelos ya preparados; no necesita ROS ni Gazebo. Primera evaluación: cuatro nombres correctos, un rechazo y cero nombres erróneos en cinco consultas reservadas.

Los siguientes comandos se ejecutan desde la raíz de Proyecto2. Las comillas son necesarias por el espacio en `task 2`.

**Probar solo los gestos, sin simulador ni ROS:**

```bash
.venv/bin/python "task 2/webcam_gestos.py"
```

También puedes abrir [el notebook conservado](<task 2/Tarea1.ipynb>) con el kernel de la `.venv` global. Su nombre es histórico: ahora está organizado dentro de Task 2. Después del traslado, reinicia el kernel antes de ejecutar las celdas.

**Conectar los gestos con Gazebo ya abierto:**

```bash
bash "task 2/ejecutar_gestos_gazebo.sh"
```

El modo anterior comprueba órdenes sin publicar movimiento. Cierra esa ventana y usa `--mover` para controlar el robot simulado. Para abrir Gazebo, cerrar la sesión o copiar el resto de comandos, consulta [task 2/COMANDOS.txt](<task 2/COMANDOS.txt>).

**Gestos con el robot real ya conectado:** desde Ubuntu normal, fuera de Docker/SSH, con Gazebo y teclado cerrados:

```bash
bash "task 2/ejecutar_gestos_robot.sh" --ip 10.87.179.100 --dominio 5
```

Comprueba las órdenes sin movimiento, cierra la prueba y repite añadiendo `--mover`. Revisa IP/dominio en cada visita. El modo físico usa reloj real y velocidades de 0.306 m/s y ±1.90 rad/s por petición del usuario. Frente a la webcam, tu derecha ordena DERECHA del robot y tu izquierda, IZQUIERDA. Los gestos físicos están confirmados antes de estos ajustes; los cambios pasaron 38 pruebas locales y requieren nueva comprobación en hardware.

**Ejecutar las pruebas:**

```bash
.venv/bin/python -m unittest discover -s "task 2/tests" -t "task 2" -v
```

**Preparar las dependencias en otra computadora:**

```bash
.venv/bin/python -m pip install -r "task 2/requirements.txt"
```

El entorno actual ya está preparado y se conserva. El notebook puede descargar y verificar el modelo de mano si falta; una vez descargado no requiere internet para reconocer gestos.

La [guía del profesor](https://github.com/Tilinpro19/turtlebot67) sirve para la conexión y utilidades del laboratorio. ROS Jazzy permanece dentro del contenedor `proyecto2-sim` con Ubuntu 24.04; la visión funciona en la `.venv` del host Ubuntu 26.04. Cada modo tiene su lanzador: `ejecutar_gestos_gazebo.sh` y `ejecutar_gestos_robot.sh`.

El contexto vigente se mantiene en [ESTADO_PROYECTO.md](ESTADO_PROYECTO.md), las instrucciones en [AGENTS.md](AGENTS.md) y el alcance en [PLAN_PROYECTO.md](PLAN_PROYECTO.md). Limpiar salidas de webcam antes de compartir notebooks y mantener fotografías personales fuera de Git.
