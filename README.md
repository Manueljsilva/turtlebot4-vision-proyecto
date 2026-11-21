# Proyecto de visión con TurtleBot 4 Lite

Proyecto de cuatro integrantes, con [enunciado base](Computer_vision_proyectos.pdf) y alcance actualizado por el usuario el 13 y 17 de noviembre de 2026.

**Task 1 vigente:** escribir antes del inicio el nombre de una persona de la galería → salir de A → navegar autónomamente entre obstáculos, usando las flechas como ayuda para decidir giros → confirmar la meta de B y solo entonces buscar alrededor y reconocer a esa persona entre varias fotografías → leer únicamente el QR situado junto a su foto y guardar su palabra → volver con rojo, confirmar la meta de A y mostrar la información del QR. Cada fotografía tiene su propio QR. Las flechas azules son para la ida y las rojas para el regreso, según la aclaración del 17 de noviembre; todavía debe verificarse la llegada a A. La flecha confirmada fija el sentido ante un muro y debe completar 90° antes de avanzar; el LiDAR conserva la parada si falta espacio. Está aprobada la adopción y mejora de la base del compañero; ya se adaptó su prueba de evasión LiDAR, con la misión completa aún pendiente. Alcance y detalles pendientes en [PLAN_PROYECTO.md](PLAN_PROYECTO.md). El usuario confirmó que los cambios afectan solo a Task 1: Task 2 mantiene identificación en B, retorno por gestos y anuncio del nombre.

- [Task 1 / Task A](<task 1/README.md>): reconocimiento local, nombre/Buscar, evasión LiDAR y movimiento/reconocimiento combinados confirmados por el usuario. El ensayo separado `--prueba-meta-b [--avanzar-hasta-meta] [--mover]` añade meta → frenado → barrido → rostro objetivo → QR asociado; meta, barrido, rostro y parada se probaron en el robot, pero QR sigue pendiente. Se conserva un ensayo aislado del compañero con `--ensayo-giro-companero`: flecha 3/3 y su recepción ROS confirmadas sin movimiento; giro físico de 90° sin validar. La evasión nueva ordena avance cero mientras gira y tiene **171 pruebas locales**, con prueba física aún pendiente. El 20 de noviembre el usuario autorizó reducir la distancia de giro del ensayo de 0.52 a **0.48 m**; se usa un perfil local, conservando controlador/configuración originales y barreras. Verificar el ajuste al relanzar antes de reducir más. Comandos en [task 1/COMANDOS.txt](<task 1/COMANDOS.txt>), bloques 13–17. El nuevo `--mision-guiada [--mover]` enlaza ida azul, meta B, rostro/QR, orientación de salida y vuelta roja hasta una meta cerca del A odométrico: **195 pruebas locales pasan; retorno e integración sin prueba física todavía**. [Guía del ensayo](<task 1/PRUEBA_MISION_GUIADA.md>), bloque 18. SLAM/Nav2, navegación general, choques diagonales y latencia de vídeo siguen pendientes.
- [Task 2 / Task B](<task 2/README.md>): gestos con Gazebo y robot real confirmados por el usuario, con vídeo comprimido en el panel lateral. El vídeo conserva retraso sin medir; faltan las métricas de frenado, reconocimiento integrado y misión completa.

**Informe en elaboración:** [fuente LaTeX](informe/main.tex) y [PDF del borrador](output/pdf/informe_proyecto_borrador.pdf), compilado y revisado en 10 páginas. Incluye metodología, arquitectura y resultados parciales. Faltan los datos de portada, navegación autónoma, cámara integrada, ensayos de la misión completa y apéndice de código. Se conserva el [borrador previo en Markdown](informe/BORRADOR_INFORME.md).

Ese borrador corresponde al avance del 7 de noviembre: debe actualizarse con el nuevo objetivo de flechas/QR y las confirmaciones físicas posteriores antes de entregar.

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
├── informe/                  Fuente LaTeX y borrador previo
├── output/pdf/               PDF compilado del informe
├── task 1/
│   ├── 01_reconocimiento_rostros.ipynb
│   ├── reconocimiento_facial.py
│   ├── modelos_rostros.py
│   ├── ejecutar_autonomo_robot.sh  Modos separados y prueba combinada
│   ├── prueba_facial_robot.py     Cámara del robot y visión en la laptop
│   ├── interfaz_facial.py         Nombre, sugerencias y Buscar
│   ├── busqueda_robot.py          Visión y evasión coordinadas
│   ├── busqueda_lidar_ros.py      Salida única de velocidad
│   ├── control_busqueda.py        Estados, A y permisos
│   ├── prueba_senales_robot.py    Señales con el robot quieto
│   ├── geometria_senales.py       Geometría aportada por la compañera
│   ├── senales_giro.py            Color y confirmación de giros
│   ├── control_giros.py           Giro por odometría en la salida única
│   ├── config/senales_giro.json
│   ├── evasion_lidar.py
│   ├── seguridad_lidar.py
│   ├── config/evasion_lidar.json
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
    ├── camara_robot.py        Recepción de vídeo y panel lateral en host
    ├── camara_robot_ros.py    Suscriptor de imagen en Docker
    ├── ejecutar_gestos_gazebo.sh
    ├── ejecutar_gestos_robot.sh
    ├── requirements.txt
    ├── COMANDOS.txt
    ├── INTEGRACION_GESTOS.md
    ├── README.md
    ├── modelos/              Modelo de mano, excluido de Git
    └── tests/
```

**Notebook facial de Task 1:** [01_reconocimiento_rostros.ipynb](<task 1/01_reconocimiento_rostros.ipynb>), kernel de la `.venv` global y celdas en orden. Modelos ya preparados; no necesita ROS ni Gazebo. La galería ampliada obtuvo 13 nombres correctos de 13 consultas reservadas; es una evaluación exploratoria por foto, sin independencia demostrada entre tomas. Los resultados iniciales conservados del notebook corresponden a cinco personas. Las fotografías, descriptores, galería y pesos se preparan localmente y no están en GitHub.

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

Añade `--ver-camara` para mostrar la webcam de gestos y la cámara del robot lado a lado. Comandos sin movimiento y con movimiento en los bloques 11/12 de [task 2/COMANDOS.txt](<task 2/COMANDOS.txt>). El usuario confirmó recepción de vídeo comprimido con `--topico-camara /oakd/rgb/preview/image_raw/compressed`; conserva retraso sin medir. El asistente no ejecutó hardware.

**Prueba combinada de Task 1, movimiento/reconocimiento confirmados por el usuario el 17 de noviembre:**

```bash
bash "task 1/ejecutar_autonomo_robot.sh" --ip IP_ACTUAL --dominio DOMINIO_ACTUAL --evasion-reconocimiento
```

Por defecto no crea publicador de velocidad. Escribir el nombre y pulsar Buscar registra A con odometría válida antes de permitir movimiento. Para el ensayo físico cerrar la prueba y repetir con `--mover`, con los demás controles cerrados. Al confirmar el objetivo con tres imágenes nuevas queda detenido. **Aún no regresa a A**; no lee meta/QR ni resuelve varias caras. Los giros visuales son opcionales con `--senales --tramo ida` (azul) o `--tramo vuelta` (rojo); no cambian de tramo automáticamente. Para reconocer señales con el robot quieto usar `--solo-senales --tramo ida`; comandos completos en bloques 11/12 de Task 1. La evasión mantiene los choques diagonales reportados como problema pendiente. Pasos completos en [Task 1](<task 1/README.md>).

**Ensayo cerca de B: meta → giro → rostro → QR.** Con robot/Docker preparados, cerrar los otros controles y ejecutar:

```bash
bash "task 1/ejecutar_autonomo_robot.sh" --ip "$TB4_IP" --dominio "$TB4_DOMINIO" --prueba-meta-b
```

Colocar cerca de la bandera B, escribir el nombre y pulsar Buscar. Primero comprobar quieto; cerrar y añadir `--mover` permite solo giro, como máximo una vuelta/90 s. Tras confirmar la meta busca al objetivo, pausa al reconocerlo y guarda solo su QR lateral no ambiguo en un JSON local. No registra A ni regresa; bloques 13/14 de Task 1. La opción --avanzar-hasta-meta añade evasión previa a la bandera; con --mover activo avanza, frena tras confirmar meta y espera 0.7 s de odometría estable antes del barrido. Bloque 15 y detalles de montaje en su README. El motivo real de parada se conserva en CONTROL; las pausas visuales no se interpretan como cortes de odometría, y el fallo de movimiento no cancela el reconocimiento estacionario ya habilitado.

**Ejecutar las pruebas:**

```bash
.venv/bin/python -m unittest discover -s "task 1/tests" -t "task 1" -v
.venv/bin/python -m unittest discover -s "task 2/tests" -t "task 2" -v
```

**Preparar las dependencias en otra computadora:**

```bash
.venv/bin/python -m pip install -r "task 2/requirements.txt"
```

El entorno actual ya está preparado y se conserva. El notebook puede descargar y verificar el modelo de mano si falta; una vez descargado no requiere internet para reconocer gestos.

La [guía del profesor](https://github.com/Tilinpro19/turtlebot67) sirve para la conexión y utilidades del laboratorio. ROS Jazzy permanece dentro del contenedor `proyecto2-sim` con Ubuntu 24.04; la visión funciona en la `.venv` del host Ubuntu 26.04. Cada modo tiene su lanzador: `ejecutar_gestos_gazebo.sh` y `ejecutar_gestos_robot.sh`.

El contexto vigente se mantiene en [ESTADO_PROYECTO.md](ESTADO_PROYECTO.md), las instrucciones en [AGENTS.md](AGENTS.md) y el alcance en [PLAN_PROYECTO.md](PLAN_PROYECTO.md). Limpiar salidas de webcam antes de compartir notebooks y mantener fotografías personales fuera de Git.
