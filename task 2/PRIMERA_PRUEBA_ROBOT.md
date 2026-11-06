# Primera sesión con el TurtleBot 4 Lite

Para repetir la conexión copiando bloques en las terminales, usar [COMANDOS_LAB.txt](../COMANDOS_LAB.txt), en la raíz y compartido por ambas tareas. Esta guía explica el procedimiento con más detalle.

Guía preparada el 2 de noviembre y actualizada el 5 para esta laptop: Ubuntu 26.04 en el host, ROS 2 Jazzy dentro de `proyecto2-sim` y visión en la `.venv` global. Los pasos son un procedimiento; las comprobaciones reales se registran en `../ESTADO_PROYECTO.md`. Hay lanzadores distintos para gestos en Gazebo y en el robot real.

**Avance del 5 de noviembre:** el usuario confirmó robot **10.87.179.100** (MAC D8-3A-DD-7E-5D-DD), laptop **10.87.179.102**, SSH como `ubuntu`, servicio activo, **dominio 5**, **rmw_fastrtps_cpp**, namespace vacío y Jazzy. Docker está en **red host, running**. `/cmd_vel` es **TwistStamped**, con publicador de joystick y suscriptor `create3_repub`; `/scan`, `/odom` y cámara raw aparecen. La hora de Pi se corrigió de junio a noviembre. Ya se recibió LiDAR desde Docker; falta probar movimiento. Datos de esta sesión, sujetos a cambios de unidad/DHCP. Los perfiles DDS de `/etc/turtlebot4/setup.bash` son propios de la Pi; no copiar sus rutas a la laptop.

**Conexión recuperada durante esa sesión:** tras corregir la hora de Pi, conectar Wi-Fi y reiniciar `turtlebot4.service`, el usuario detuvo el daemon de Docker y repitió la lista sin daemon con spin 10 s. Ya descubre todos los tópicos y recibe cabecera de `/scan` desde Docker. Antes solo veía tópicos locales pese a multicast bidireccional; no fue necesario cambiar XML, dominio ni firewall (UFW estaba inactivo). No se aisló la causa exacta. Después confirmó movimiento por teclado. Después el usuario confirmó también movimiento con gestos; los nuevos ajustes de velocidad y dirección aún necesitan comprobarse en el robot.

Referencia revisada: [README del profesor](https://github.com/Tilinpro19/turtlebot67/blob/master/README.md), junto con `tb4_connect.sh`, `tb4_check.sh`, `teleop_wasd.py` y `ver_camara_tb4.py`. Esa guía supone un robot ya configurado. No hace falta reinstalarlo ni aplicar `CONFIGURACION_INICIAL.md` para una sesión normal.

**1. Preparar antes de perder internet — terminal normal de Ubuntu.**

La red del laboratorio no tiene internet según la guía. Para usar los scripts del profesor, guarda una copia separada del código propio. Si ya existe, omite `git clone`:

```bash
cd "/home/manueljsilva/Documentos/Cursos/Vision-comptacion/Proyecto2"
mkdir -p referencias
git clone https://github.com/Tilinpro19/turtlebot67.git referencias/turtlebot67
```

`referencias/turtlebot67/` ya está excluida del Git del proyecto. No instalar ROS Jazzy para Ubuntu 24.04 directamente en este host Ubuntu 26.04; se conserva el contenedor actual y la `.venv`.

**2. Datos al recibir el robot.**

Si además entregan un equipo marcado «AC1200», identificar su marca/modelo y las conexiones actuales antes de configurarlo: esa inscripción por sí sola no determina el modelo. Puede ser el router que crea la red local del robot, pero hay que confirmarlo. Encenderlo con su fuente correspondiente y observar el SSID existente; la etiqueta puede mostrar los datos de fábrica, que podrían haberse cambiado. No resetear ni sustituir la configuración del laboratorio. La topología esperada, si el robot ya conoce esa red, es laptop → Wi-Fi → router ← Wi-Fi ← robot. Como alternativa, la laptop puede ir por Ethernet a un puerto LAN del router; esto no es lo mismo que conectar Ethernet directamente a la Raspberry Pi. La conexión directa se documenta en [configuración inicial, §1.4](https://github.com/Tilinpro19/turtlebot67/blob/master/CONFIGURACION_INICIAL.md#14-acceso-por-cable-plan-b) y requiere comprobar la configuración de IP correspondiente. El 5 de noviembre se confirmó la conexión Wi-Fi de ambos al Archer C50; la alternativa Ethernet no se probó.

Pide al encargado la red Wi-Fi y su clave, la IP actual de la Raspberry Pi, el `ROS_DOMAIN_ID` asignado y el acceso SSH. Confirma que usa Jazzy y que es la unidad reservada a tu grupo. Los valores de ejemplo del repositorio no identifican necesariamente el robot que recibas. Conecta la laptop a la red indicada; que diga «sin internet» es normal.

En una terminal normal de Ubuntu, introduce los dos datos de esta sesión:

```bash
read -r -p "IP actual del robot: " TB4_IP
read -r -p "ROS_DOMAIN_ID asignado: " TB4_DOMINIO
ping -c 3 "$TB4_IP"
ssh "ubuntu@$TB4_IP"
```

`ubuntu` es el usuario de la guía; usa el usuario que indique el encargado si es diferente. El prompt cambia al entrar por SSH: los comandos siguientes se ejecutan en el robot.

```bash
systemctl is-active turtlebot4.service
```

Se espera `active`. Si el servicio está activo, ya está arrancado el sistema del robot: no lances otro bringup. Si falta o está inactivo, revisarlo con el encargado antes de iniciar componentes duplicados. Sal del SSH:

```bash
exit
```

En Ubuntu vuelves a tener las variables `TB4_IP` y `TB4_DOMINIO` de esta terminal. SSH confirma acceso al robot; aún no demuestra comunicación ROS desde la laptop.

**3. Comprobar la hora — terminal normal de Ubuntu.**

La guía recomienda sincronizar la Raspberry Pi con la laptop en esta red sin internet. Comprueba primero que la hora de tu laptop sea correcta y aplica, al inicio de la sesión:

```bash
date -u
ssh -t "ubuntu@$TB4_IP" "sudo date -u -s '$(date -u +%Y-%m-%dT%H:%M:%S)'"
```

Puede pedir contraseña SSH y de sudo del robot. Esto ajusta la Pi; el Create 3 tiene su propio reloj y podría seguir desfasado. Si las marcas de `/odom` siguen mal, revisar con el encargado; no afirmar que el comando anterior sincroniza todos los sensores.

**4. Preparar ROS dentro de Docker — terminal normal de Ubuntu.**

Cierra gestos y Gazebo con Ctrl+C en sus terminales. Después, en la terminal donde introdujiste IP y dominio:

```bash
sudo docker restart proyecto2-sim
sudo docker inspect --format '{{.HostConfig.NetworkMode}}' proyecto2-sim
```

La receta siguiente supone que aparece `host`: permite al contenedor compartir la red de la laptop en Linux ([Docker](https://docs.docker.com/engine/network/drivers/host/)). Si aparece `bridge` u otro modo, falta preparar la red DDS del contenedor; no borrar ni recrear a ciegas el contenedor instalado. Poder hacer ping no garantiza descubrimiento ni entrega ROS. El asistente no pudo comprobar ese modo desde su entorno por una restricción de sudo.

Con red `host`, copia los scripts e ingresa a ROS desde esa misma terminal:

```bash
cd "/home/manueljsilva/Documentos/Cursos/Vision-comptacion/Proyecto2"
sudo docker cp referencias/turtlebot67 proyecto2-sim:/tmp/
sudo docker exec -it -e TB4_IP="$TB4_IP" -e TB4_DOMINIO="$TB4_DOMINIO" proyecto2-sim bash
```

Ahora estás dentro del contenedor, no en el robot. Sin activar `.venv`, ejecuta:

```bash
source /opt/ros/jazzy/setup.bash
unset ROS_LOCALHOST_ONLY
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
source /tmp/turtlebot67/tb4_connect.sh "$TB4_IP" "$TB4_DOMINIO"
```

`source` configura esta terminal; `tb4_connect.sh` establece dominio/Fast DDS y prueba multicast y peers estáticos. Si abres otra terminal de Docker, repite su configuración antes de usar ROS. No cambiar el dominio configurado del robot: la laptop debe adoptar el que le corresponde. [Opciones de descubrimiento en Jazzy](https://github.com/ros2/ros2_documentation/blob/jazzy/source/Tutorials/Advanced/Improved-Dynamic-Discovery.rst).

**5. Verificar comunicación sin mover — dentro de Docker.**

```bash
ros2 topic list -t --no-daemon --spin-time 5
ros2 topic info /cmd_vel --verbose
ros2 topic echo /odom --once --no-arr --qos-reliability best_effort
ros2 topic echo /scan --once --no-arr --qos-reliability best_effort
```

La guía espera `/cmd_vel`, `/odom`, `/scan` y `/tf`; `/cmd_vel` debe usar `geometry_msgs/msg/TwistStamped` en su configuración Jazzy. Si tienen un prefijo de namespace, usa sus nombres reales. Debe haber un suscriptor del robot en `/cmd_vel`. Para los sensores se espera recibir un mensaje; si un `echo` no recibe, Ctrl+C y revisar red/dominio/sensor. La lista de tópicos por sí sola no demuestra que lleguen los datos. Mira también los publicadores de velocidad: cerrar otros controles que puedan enviar órdenes.

Para comprobar mensajes de cámara comprimida sin abrir ventanas:

```bash
ros2 topic echo /oakd/rgb/preview/image_raw/compressed --once --field header --qos-reliability best_effort
```

Esto comprueba llegada de un mensaje, no la imagen decodificada. Si quieres verla, el repositorio incluye `ver_camara_tb4.py`; requiere `cv2`, NumPy y acceso gráfico dentro de Docker. Comprueba sus dependencias antes de ir al laboratorio:

```bash
python3 -c 'import rclpy, cv2, numpy; print("Dependencias del visor disponibles")'
```

Si faltan, prepararlas con internet en el contenedor; no instalar `rclpy` con pip en la `.venv`. Para abrir el visor con la configuración X11 habitual del simulador, sal de Docker con `exit` y, desde la terminal normal de Ubuntu:

```bash
xhost +si:localuser:root
sudo docker exec -it -e DISPLAY="$DISPLAY" proyecto2-sim bash
```

Dentro de esa nueva terminal, repite la configuración de ROS; el script recuerda el último robot en el contenedor:

```bash
source /opt/ros/jazzy/setup.bash
unset ROS_LOCALHOST_ONLY
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
source /tmp/turtlebot67/tb4_connect.sh
python3 /tmp/turtlebot67/ver_camara_tb4.py
```

El visor supone que el contenedor ya tiene acceso al socket X11, como durante las pruebas gráficas previas; `DISPLAY` por sí solo no monta ese socket. Si la ventana no abre, el `echo` anterior permite comprobar recepción sin GUI. El visor usa imagen comprimida y QoS de sensores. Su tecla `q` lo cierra.

**6. Primera prueba breve de movimiento — dentro de Docker.**

Alternativa con el paquete estándar ya usado en simulación, sin descargar scripts del profesor:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
  -p stamped:=true -p frame_id:=base_link \
  -p speed:=0.05 -p turn:=0.30 -p use_sim_time:=false
```

Antes de pulsar movimiento, comprobar que la pantalla indique `speed 0.05` y `turn 0.30`. Los parámetros speed/turn se añadieron en la versión 2.4.1 según [el changelog oficial](https://github.com/ros2/teleop_twist_keyboard/blob/rolling/CHANGELOG.rst); la versión instalada del contenedor aún no se comprobó. Si muestra otros valores, Ctrl+C y revisar antes de mover. En este programa `i` avanza, `j/l` giran y `k` detiene; Ctrl+C detiene y sale. Soltar la tecla no equivale a enviar una parada. Usar el robot fuera del dock, con espacio libre y sin otros controles actuando; el publicador de joystick ya fue observado. Probar parada, avance breve y parada, luego giros. El usuario confirmó movimiento físico por teclado, sin comunicar valores medidos ni tiempo de frenado.

Alternativa con el script del profesor, si está copiado al contenedor:

Una vez confirmada la unidad y la comunicación, prueba en un espacio despejado, con el encargado disponible. Estas son pruebas de desarrollo: el teclado no es el control admitido durante la competición de Task B. Primero, sin publicar:

```bash
python3 /tmp/turtlebot67/teleop_wasd.py --dry
```

Termina con Ctrl+C. Después, si el mensaje y el tópico del robot coinciden con lo comprobado:

```bash
python3 /tmp/turtlebot67/teleop_wasd.py
```

`w` avanza; `a/d` giran; espacio o `x` detienen; Ctrl+C detiene y sale. En este script `q` significa avanzar girando, **no salir**. Arranca a 0.10 m/s y 1.0 rad/s según el código revisado; `k` reduce su velocidad de giro. Comprobar primero parada, avance breve y parada, luego giros. Sal antes de iniciar cualquier otro publicador.

**7. Unir los gestos: funcionamiento físico confirmado por el usuario.**

La conexión implementada es webcam → detector en la `.venv` del host → puente ROS en Docker → robot por Wi-Fi. El lanzador físico verifica IP, Docker host y `/cmd_vel` stamped; configura dominio, Fast DDS y peer. El puente usa reloj del sistema (`use_sim_time=False`) sin exigir `/clock`. Conserva el vigilante monotónico de 0.5 s, pérdida/ambigüedad de mano y parada al cerrar el enlace. Webcam y receptor permanecen en la misma laptop para compartir el reloj del protocolo.

Con la conexión ya comprobada, cierra Gazebo, teclado y otras capturas. En Ubuntu normal, fuera de Docker/SSH, desde la raíz del proyecto:

```bash
bash "task 2/ejecutar_gestos_robot.sh" --ip 10.87.179.100 --dominio 5
```

Comprueba las cuatro órdenes sin publicar. Cierra la prueba antes de activar movimiento:

```bash
bash "task 2/ejecutar_gestos_robot.sh" --ip 10.87.179.100 --dominio 5 --mover
```

Comprueba IP/dominio en cada visita. Prueba palma/ausencia de mano, avance breve, palma y giros. Las velocidades actuales son 0.306 m/s y ±1.90 rad/s, máximos con protecciones activas por petición del usuario. Frente a la webcam, tu derecha ordena DERECHA del robot y tu izquierda, IZQUIERDA. `--camara 1` selecciona otra webcam. Los comandos también están en `COMANDOS.txt`, bloques 9 y 10. El usuario confirmó movimiento con el lanzador físico antes de estos ajustes. Los cambios de velocidad y dirección pasaron 38 pruebas locales, con ROS/red/Docker simulados para el modo físico; falta comprobarlos de nuevo en hardware. No se ha medido frenado físico ni se implementó barrera LiDAR; el vigilante no garantiza entregar una parada si termina el proceso ROS o se corta la red.

**Al terminar.** Cierra primero el emisor de órdenes y verifica que el robot esté parado. Sal del contenedor con `exit`; desde Ubuntu puedes usar `sudo docker stop proyecto2-sim`. Si habilitaste acceso X11, revócalo con `xhost -si:localuser:root`. Apaga/entrega el robot según el procedimiento del encargado.

**Datos útiles para continuar:** IP, dominio, modo de red Docker, salida de `ros2 topic info /cmd_vel --verbose`, recepción de `/scan` y `/odom`, y si el visor mostró una imagen. No guardar contraseñas en el registro del proyecto.
