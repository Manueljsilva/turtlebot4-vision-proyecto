# Task 2 / Task B: control por gestos

Esta carpeta reúne el detector local, la integración con Gazebo y con el robot real, las pruebas, el modelo de mano y las instrucciones de esta modalidad. La `.venv` permanece en la raíz de Proyecto2 y se comparte con Task 1.

El usuario confirmó los gestos con webcam tanto en Gazebo como en el robot real. Después pidió velocidad máxima e invertir los giros; esos ajustes están implementados y pendientes de una nueva comprobación física. Todavía faltan integrar el reconocimiento facial y completar la misión de ida y vuelta.

La conexión común está en [COMANDOS_LAB.txt](../COMANDOS_LAB.txt); los comandos de esta tarea, en [COMANDOS.txt](COMANDOS.txt). [PRIMERA_PRUEBA_ROBOT.md](PRIMERA_PRUEBA_ROBOT.md) explica red, reloj, ROS, sensores y las pruebas físicas.

**Prueba local sin Gazebo.** Desde la raíz de Proyecto2:

```bash
.venv/bin/python "task 2/webcam_gestos.py"
```

Se abre la webcam y solo se muestran las órdenes; no se inicia Docker ni se publica movimiento. Para otra cámara añade `--camara 1`. Pulsa q con la ventana seleccionada o ciérrala para terminar la prueba de desarrollo.

También se conserva [Tarea1.ipynb](Tarea1.ipynb). El nombre original no corresponde a la numeración actual: es el notebook local de gestos de Task 2. Ábrelo con el kernel de la `.venv` global. La primera celda reconoce tanto la raíz de Proyecto2 como `task 2` como directorio de trabajo. Reinicia el kernel después del traslado.

| Gesto | Orden respecto al robot |
| --- | --- |
| Índice arriba, otros dedos largos flexionados | AVANZAR |
| Índice hacia tu izquierda, estando frente a la webcam | IZQUIERDA |
| Índice hacia tu derecha, estando frente a la webcam | DERECHA |
| Palma, mano ausente, más de una mano o ambigüedad | DETENER |

La imagen se procesa sin espejo y las direcciones deben mantenerse 0.4 s. Estando frente a la webcam, señalar hacia tu derecha ordena giro derecho del robot y hacia tu izquierda, giro izquierdo. En la imagen sin espejo tus lados aparecen invertidos; el clasificador hace esa conversión una sola vez. El modelo se busca en `task 2/modelos/hand_landmarker.task`; no depende de dónde se ejecute el script.

**Gestos con el robot real.** Primero conecta el robot con los comandos globales. Cierra Gazebo, el control por teclado y la webcam del notebook. Ejecuta desde Ubuntu normal, fuera de Docker/SSH y desde la raíz de Proyecto2. Los valores siguientes son los confirmados el 5 de noviembre; compruébalos en cada visita:

```bash
bash "task 2/ejecutar_gestos_robot.sh" --ip 10.87.179.100 --dominio 5
```

Comprueba las cuatro órdenes en la terminal, sin movimiento. Cierra con q o Ctrl+C. Después:

```bash
bash "task 2/ejecutar_gestos_robot.sh" --ip 10.87.179.100 --dominio 5 --mover
```

Prueba palma y mano retirada; luego avance breve, palma y cada giro. Usa 0.306 m/s y ±1.90 rad/s, máximos documentados con protecciones activas. El lanzador verifica red host, alcance de IP y tipo `TwistStamped`, configura dominio/Fast DDS/peer y usa reloj real sin exigir `/clock`. Admite `--camara 1`, `--contenedor NOMBRE` y las variables `TB4_IP`, `TB4_DOMINIO`, `TB4_CONTENEDOR`. No reinicia el robot. Estas son pruebas de desarrollo: q/Ctrl+C no son controles del intento oficial.

**Integración con Gazebo.** Usa [COMANDOS.txt](COMANDOS.txt) para abrir la simulación y [INTEGRACION_GESTOS.md](INTEGRACION_GESTOS.md) para entender el enlace. Con Gazebo abierto, cierra primero el teclado y cualquier otra captura de webcam. Desde la raíz de Proyecto2:

```bash
bash "task 2/ejecutar_gestos_gazebo.sh"
```

Ese modo comprueba órdenes sin publicar velocidad. Cierra esa prueba antes de activar movimiento:

```bash
bash "task 2/ejecutar_gestos_gazebo.sh" --mover
```

El lanzador funciona también desde otra carpeta porque calcula sus rutas a partir de su propio archivo. Copia los módulos necesarios a Docker, mantiene MediaPipe en el host y encuentra la `.venv` global. No ejecutarlo a la vez que otro control de velocidad.

**Ajustar la velocidad en simulación y hardware.** Las órdenes están en [control_gestos.py](control_gestos.py), diccionario `VELOCIDADES`. Cada pareja significa `(avance en m/s, giro en rad/s)`. Los valores actuales son 0.306 m/s y ±1.90 rad/s por petición del usuario. Para reducirlos, por ejemplo, edita estas entradas:

```python
    Orden.AVANZAR: (0.20, 0.0),
    Orden.IZQUIERDA: (0.0, 0.80),
    Orden.DERECHA: (0.0, -0.80),
```

Conserva `DETENER: (0.0, 0.0)`. Guarda el archivo, cierra el control de gestos anterior y vuelve a ejecutar el lanzador con `--mover`; no hace falta reiniciar Gazebo. El lanzador copia otra vez el módulo a Docker. El ejemplo anterior reduce los valores máximos; solo se aplica si lo guardas en el módulo. Las pruebas actuales comprueban también los valores predeterminados; si los cambias, esas expectativas deberán actualizarse de manera consciente.

El [manual de TurtleBot 4 Lite](https://turtlebot.github.io/turtlebot4-user-manual/overview/features.html) documenta 1.90 rad/s de giro y unos 0.31 m/s de avance con protecciones. La [base Create 3](https://iroboteducation.github.io/create3_docs/api/safety/#maximum-speed) precisa 0.306 m/s con protecciones y 0.460 m/s al desactivarlas; mantener las protecciones para las pruebas del laboratorio. Son límites técnicos documentados, no una velocidad autorizada por el profesor ni una medición del robot disponible. El simulador/controlador puede aplicar límites propios. El modo físico rechaza velocidades no finitas, superiores a 0.306 m/s o 1.90 rad/s y una orden DETENER distinta de cero. Ambos lanzadores comparten `VELOCIDADES`; un cambio afecta a las siguientes ejecuciones de ambos.

**Pruebas desde la raíz de Proyecto2:**

```bash
.venv/bin/python -m unittest discover -s "task 2/tests" -t "task 2" -v
```

**Dependencias para otra computadora:**

```bash
.venv/bin/python -m pip install -r "task 2/requirements.txt"
```

No recrear el entorno actual. El modelo y el notebook ya están preparados en esta computadora. Las fotos, videos y modelos descargados quedan fuera del control de versiones según el `.gitignore` global. El estado y las decisiones del proyecto permanecen en [ESTADO_PROYECTO.md](../ESTADO_PROYECTO.md) y [AGENTS.md](../AGENTS.md).
