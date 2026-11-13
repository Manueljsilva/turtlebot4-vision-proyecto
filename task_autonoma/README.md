# Task Autónoma (SLAM + Nav2 + Exploración + Visión)

Arquitectura de navegación autónoma para el concurso con TurtleBot 4 Lite (ROS 2 Jazzy).

## Estructura

- `config/slam_params.yaml`: configuración de `slam_toolbox` (`online_async`, resolución `0.05 m`, cierre de lazos activo para reducir deriva al volver a A).
- `config/nav2_params.yaml`: configuración de Nav2 para pasillos estrechos con cajas (`enable_stamped_cmd_vel: true` obligatorio para `/cmd_vel` `TwistStamped`, `robot_radius: 0.18`, `inflation_radius: 0.30/0.32`).
- `config/mision_config.json`: parámetros editables de cámara, exploración, detección de Punto B, reconocimiento facial y evasión LiDAR.
- `launch/slam_nav2_launch.py`: lanza `slam_toolbox` + `nav2_bringup` (`navigation_launch.py`, sin AMCL).
- `exploracion.py`: algoritmo puro (sin ROS) de detección de fronteras seguras sobre `/map` con lista negra de puntos fallidos y analizador reactivo LiDAR.
- `detector_meta_b.py`: detección visual de Punto B (`rostro`, `aruco`, `color`, `coordenada`) con confirmaciones consecutivas.
- `mision.py`: carga de `task 1/datos/{galeria.npz,criterio.json}` + modelos YuNet/SFace y votación por mayoría en ráfaga.
- `camara_ros.py`: suscriptor resiliente a `/oakd/rgb/preview/image_raw` y `/compressed` con QoS `BEST_EFFORT` y caducidad de frame.
- `seguridad_lidar.py`: prueba independiente de evasión de cajas/paredes vía `/scan -> /cmd_vel` (`TwistStamped`).
- `navegacion_autonoma.py`: orquestador completo `A -> explorar hasta B -> reconocer rostro -> retornar a A -> anunciar nombre`.
- `ejecutar_autonomo_gazebo.sh` y `ejecutar_autonomo_robot.sh`: sincronizan automáticamente código, configs, galería y modelos dentro de `proyecto2-sim`.

## Flujo recomendado para probar mañana con el robot

1. Verificar paquetes instalados en Docker (`slam_toolbox`, `nav2_bringup`) con el bloque 1 de `COMANDOS.txt`.
2. Probar primero `--solo-evasion` (sin mover y luego con `--mover`) para validar `/scan` y `/cmd_vel` `TwistStamped`.
3. Probar `--solo-reconocimiento` para confirmar que la cámara OAK-D del robot entrega frames y reconoce una foto impresa.
4. Lanzar `--slam-nav2` en una terminal y la misión completa en otra (`COMANDOS.txt` bloque 3D).
