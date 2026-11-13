#!/usr/bin/env bash
# Navegación Autónoma (SLAM + Nav2 + Exploración + Visión) en TurtleBot 4 real vía Docker.
set -euo pipefail

TAREA_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROYECTO_DIR="$(cd -- "$TAREA_DIR/.." && pwd)"
IP_ROBOT="${TB4_IP:-}"
DOMINIO_ROBOT="${TB4_DOMINIO:-}"
CONTENEDOR="${TB4_CONTENEDOR:-proyecto2-sim}"
ACCION="mision"
ARGS_EXTRA=()

while [[ $# -gt 0 ]]; do
    case "$1" in
        --ip|--dominio|--contenedor)
            [[ $# -ge 2 ]] || { echo "Falta el valor de $1" >&2; exit 2; }
            case "$1" in
                --ip) IP_ROBOT="$2" ;;
                --dominio) DOMINIO_ROBOT="$2" ;;
                --contenedor) CONTENEDOR="$2" ;;
            esac
            shift 2 ;;
        --slam-nav2) ACCION="slam_nav2"; shift ;;
        --solo-evasion) ACCION="evasion"; shift ;;
        --solo-reconocimiento) ACCION="reconocimiento"; shift ;;
        --help|-h)
            echo 'Uso: bash "task_autonoma/ejecutar_autonomo_robot.sh" --ip IP --dominio ID [OPCION] [ARGS...]'
            echo '  --slam-nav2            Inicia slam_toolbox + Nav2 para robot real (terminal dedicada)'
            echo '  --solo-evasion         Prueba evasión reactiva LiDAR (añadir --mover para mover)'
            echo '  --solo-reconocimiento  Prueba cámara del robot + YuNet/SFace sin mover'
            echo '  (sin opción)           Ejecuta la misión autónoma A -> B -> Reconocimiento -> A'
            echo 'Admite TB4_IP, TB4_DOMINIO y TB4_CONTENEDOR de COMANDOS_LAB.txt.'
            exit 0 ;;
        *) ARGS_EXTRA+=("$1"); shift ;;
    esac
done

[[ "$DOMINIO_ROBOT" =~ ^[0-9]{1,3}$ ]] || { echo 'Falta --dominio (0-101)' >&2; exit 2; }
DOMINIO_ROBOT="$((10#$DOMINIO_ROBOT))"
(( DOMINIO_ROBOT <= 101 )) || { echo 'Dominio fuera de 0-101' >&2; exit 2; }
[[ "$CONTENEDOR" =~ ^[a-zA-Z0-9][a-zA-Z0-9_.-]*$ ]] || { echo 'Contenedor invalido' >&2; exit 2; }

PYTHON="$PROYECTO_DIR/.venv/bin/python"
if [[ ! -x "$PYTHON" ]]; then
    PYTHON="$(command -v python3 || true)"
fi
[[ -n "$PYTHON" ]] || { echo 'Falta .venv/bin/python o python3' >&2; exit 1; }
if ! "$PYTHON" -c 'import ipaddress, sys; ipaddress.IPv4Address(sys.argv[1])' "$IP_ROBOT" 2>/dev/null; then
    echo 'Falta --ip o no es una direccion IPv4 valida' >&2
    exit 2
fi

[[ -f "$PROYECTO_DIR/task 1/datos/galeria.npz" ]] || { echo 'Falta task 1/datos/galeria.npz' >&2; exit 1; }
[[ -f "$PROYECTO_DIR/task 1/datos/criterio.json" ]] || { echo 'Falta task 1/datos/criterio.json' >&2; exit 1; }
[[ -d "$PROYECTO_DIR/task 1/modelos" ]] || { echo 'Falta carpeta task 1/modelos' >&2; exit 1; }

command -v docker >/dev/null || { echo 'No se encuentra Docker' >&2; exit 1; }
command -v ping >/dev/null || { echo 'No se encuentra ping' >&2; exit 1; }
ping -c 2 -W 2 "$IP_ROBOT" >/dev/null || { echo 'El robot no responde: revisar IP y Wi-Fi' >&2; exit 1; }
sudo -v
ESTADO="$(sudo docker inspect --format '{{.State.Running}} {{.HostConfig.NetworkMode}}' "$CONTENEDOR")"
[[ "$ESTADO" == 'true host' ]] || { echo 'Preparar Docker en red host con COMANDOS_LAB.txt' >&2; exit 1; }

ENTORNO_ROS=(
    -e "ROS_DOMAIN_ID=$DOMINIO_ROBOT"
    -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    -e ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
    -e "ROS_STATIC_PEERS=$IP_ROBOT"
)

if ! TIPO="$(sudo docker exec "${ENTORNO_ROS[@]}" "$CONTENEDOR" bash --noprofile --norc -c \
    'unset ROS_LOCALHOST_ONLY; source /opt/ros/jazzy/setup.bash && ros2 topic type /cmd_vel --no-daemon --spin-time 5')"; then
    echo 'No se pudo consultar /cmd_vel; revisar la conexion con COMANDOS_LAB.txt' >&2
    exit 1
fi
[[ "$TIPO" == geometry_msgs/msg/TwistStamped ]] || { echo "Tipo de /cmd_vel inesperado: $TIPO" >&2; exit 1; }

echo "Sincronizando task_autonoma hacia $CONTENEDOR:/tmp/proyecto2-autonomo ..." >&2
sudo docker exec "$CONTENEDOR" rm -rf /tmp/proyecto2-autonomo
sudo docker exec "$CONTENEDOR" mkdir -p /tmp/proyecto2-autonomo/config /tmp/proyecto2-autonomo/launch /tmp/proyecto2-autonomo/datos /tmp/proyecto2-autonomo/modelos

for archivo in camara_ros.py detector_meta_b.py exploracion.py mision.py modelos_rostros.py navegacion_autonoma.py reconocimiento_facial.py seguridad_lidar.py; do
    sudo docker cp "$TAREA_DIR/$archivo" "$CONTENEDOR:/tmp/proyecto2-autonomo/$archivo" >/dev/null
done
sudo docker cp "$TAREA_DIR/config/." "$CONTENEDOR:/tmp/proyecto2-autonomo/config/" >/dev/null
sudo docker cp "$TAREA_DIR/launch/." "$CONTENEDOR:/tmp/proyecto2-autonomo/launch/" >/dev/null
if [[ "$ACCION" == "mision" || "$ACCION" == "reconocimiento" ]]; then
    sudo docker cp "$PROYECTO_DIR/task 1/datos/galeria.npz" "$CONTENEDOR:/tmp/proyecto2-autonomo/datos/galeria.npz" >/dev/null
    sudo docker cp "$PROYECTO_DIR/task 1/datos/criterio.json" "$CONTENEDOR:/tmp/proyecto2-autonomo/datos/criterio.json" >/dev/null
    sudo docker cp "$PROYECTO_DIR/task 1/modelos/." "$CONTENEDOR:/tmp/proyecto2-autonomo/modelos/" >/dev/null
fi
echo "Sincronización completa. Iniciando ($ACCION)..." >&2

echo "ROBOT REAL: IP=$IP_ROBOT dominio=$DOMINIO_ROBOT | accion=$ACCION" >&2

case "$ACCION" in
    slam_nav2)
        exec sudo docker exec -it "${ENTORNO_ROS[@]}" "$CONTENEDOR" bash --noprofile --norc -c \
            'unset ROS_LOCALHOST_ONLY; source /opt/ros/jazzy/setup.bash && exec ros2 launch /tmp/proyecto2-autonomo/launch/slam_nav2_launch.py use_sim_time:=false'
        ;;
    evasion)
        exec sudo docker exec -it "${ENTORNO_ROS[@]}" "$CONTENEDOR" bash --noprofile --norc -c \
            'unset ROS_LOCALHOST_ONLY; source /opt/ros/jazzy/setup.bash && export PYTHONPATH="/tmp/proyecto2-autonomo:${PYTHONPATH:-}" && exec python3 -u /tmp/proyecto2-autonomo/seguridad_lidar.py --modo robot --config /tmp/proyecto2-autonomo/config/mision_config.json "$@"' \
            evasion-robot "${ARGS_EXTRA[@]}"
        ;;
    reconocimiento)
        exec sudo docker exec -it "${ENTORNO_ROS[@]}" "$CONTENEDOR" bash --noprofile --norc -c \
            'unset ROS_LOCALHOST_ONLY; source /opt/ros/jazzy/setup.bash && export PYTHONPATH="/tmp/proyecto2-autonomo:${PYTHONPATH:-}" && exec python3 -u /tmp/proyecto2-autonomo/navegacion_autonoma.py --modo robot --solo-reconocimiento --config /tmp/proyecto2-autonomo/config/mision_config.json --datos-dir /tmp/proyecto2-autonomo/datos --modelos-dir /tmp/proyecto2-autonomo/modelos "$@"' \
            rec-robot "${ARGS_EXTRA[@]}"
        ;;
    mision)
        exec sudo docker exec -it "${ENTORNO_ROS[@]}" "$CONTENEDOR" bash --noprofile --norc -c \
            'unset ROS_LOCALHOST_ONLY; source /opt/ros/jazzy/setup.bash && export PYTHONPATH="/tmp/proyecto2-autonomo:${PYTHONPATH:-}" && exec python3 -u /tmp/proyecto2-autonomo/navegacion_autonoma.py --modo robot --config /tmp/proyecto2-autonomo/config/mision_config.json --datos-dir /tmp/proyecto2-autonomo/datos --modelos-dir /tmp/proyecto2-autonomo/modelos "$@"' \
            mision-robot "${ARGS_EXTRA[@]}"
        ;;
esac
