#!/usr/bin/env bash
# Webcam en la laptop; ROS en su Docker; comandos al robot por la red del laboratorio.
set -euo pipefail

TAREA_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROYECTO_DIR="$(cd -- "$TAREA_DIR/.." && pwd)"
IP_ROBOT="${TB4_IP:-}"
DOMINIO_ROBOT="${TB4_DOMINIO:-}"
CONTENEDOR="${TB4_CONTENEDOR:-proyecto2-sim}"
OPCIONES_PUENTE=(--robot-real)
OPCIONES_WEBCAM=()

while [[ $# -gt 0 ]]; do
    case "$1" in
        --ip|--dominio|--contenedor|--camara)
            [[ $# -ge 2 ]] || { echo "Falta el valor de $1" >&2; exit 2; }
            case "$1" in
                --ip) IP_ROBOT="$2" ;;
                --dominio) DOMINIO_ROBOT="$2" ;;
                --contenedor) CONTENEDOR="$2" ;;
                --camara)
                    [[ "$2" =~ ^[0-9]+$ ]] || { echo 'Camara invalida' >&2; exit 2; }
                    OPCIONES_WEBCAM+=(--camara "$2") ;;
            esac
            shift 2 ;;
        --mover) OPCIONES_PUENTE+=(--publicar); shift ;;
        --help|-h)
            echo 'Uso: bash "task 2/ejecutar_gestos_robot.sh" --ip IP --dominio ID [--mover] [--camara 0]'
            echo 'Tambien admite TB4_IP, TB4_DOMINIO y TB4_CONTENEDOR del bloque 1 de COMANDOS_LAB.txt.'
            echo 'Sin --mover comprueba gestos sin publicar velocidad. No inicia ni reinicia el robot.'
            exit 0 ;;
        *) echo "Opcion desconocida: $1" >&2; exit 2 ;;
    esac
done

[[ "$DOMINIO_ROBOT" =~ ^[0-9]{1,3}$ ]] || { echo 'Falta --dominio (0-101)' >&2; exit 2; }
DOMINIO_ROBOT="$((10#$DOMINIO_ROBOT))"
(( DOMINIO_ROBOT <= 101 )) || { echo 'Dominio fuera de 0-101' >&2; exit 2; }
[[ "$CONTENEDOR" =~ ^[a-zA-Z0-9][a-zA-Z0-9_.-]*$ ]] || { echo 'Contenedor invalido' >&2; exit 2; }
PYTHON="$PROYECTO_DIR/.venv/bin/python"
[[ -x "$PYTHON" ]] || { echo 'Falta .venv/bin/python' >&2; exit 1; }
if ! "$PYTHON" -c 'import ipaddress, sys; ipaddress.IPv4Address(sys.argv[1])' "$IP_ROBOT" 2>/dev/null; then
    echo 'Falta --ip o no es una direccion IPv4 valida' >&2
    exit 2
fi
"$PYTHON" -c 'import sys; sys.path.insert(0, sys.argv[1]); from control_gestos import validar_velocidades_robot; validar_velocidades_robot()' "$TAREA_DIR"
[[ -f "$TAREA_DIR/modelos/hand_landmarker.task" ]] || { echo 'Falta el modelo de manos' >&2; exit 1; }
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
# Introspeccion sin crear un publicador de velocidad ni abrir la webcam.
if ! TIPO="$(sudo docker exec "${ENTORNO_ROS[@]}" "$CONTENEDOR" bash --noprofile --norc -c \
    'unset ROS_LOCALHOST_ONLY; source /opt/ros/jazzy/setup.bash && ros2 topic type /cmd_vel --no-daemon --spin-time 5')"; then
    echo 'No se pudo consultar /cmd_vel; revisar la conexion con COMANDOS_LAB.txt' >&2
    exit 1
fi
[[ "$TIPO" == geometry_msgs/msg/TwistStamped ]] || { echo "Tipo de /cmd_vel inesperado: $TIPO" >&2; exit 1; }

sudo docker exec "$CONTENEDOR" mkdir -p /tmp/proyecto2-gestos
for archivo in gestos.py control_gestos.py puente_gestos_ros.py; do
    sudo docker cp "$TAREA_DIR/$archivo" "$CONTENEDOR:/tmp/proyecto2-gestos/$archivo" >/dev/null
done
echo "ROBOT REAL: IP=$IP_ROBOT dominio=$DOMINIO_ROBOT | velocidades en task 2/control_gestos.py" >&2
echo 'Cerrar el teleop de teclado, otros controles y la webcam del notebook. Empezar con palma/sin mano.' >&2
if [[ " ${OPCIONES_PUENTE[*]} " == *' --publicar '* ]]; then
    echo 'MOVIMIENTO ACTIVADO: los gestos enviaran velocidades al robot real.' >&2
else
    echo 'COMPROBACION: las ordenes se muestran en ROS, sin publicar velocidad.' >&2
fi

"$PYTHON" -u "$TAREA_DIR/webcam_gestos.py" --emitir "${OPCIONES_WEBCAM[@]}" |
    sudo docker exec -i "${ENTORNO_ROS[@]}" "$CONTENEDOR" bash --noprofile --norc -c \
    'unset ROS_LOCALHOST_ONLY; source /opt/ros/jazzy/setup.bash && exec python3 -u /tmp/proyecto2-gestos/puente_gestos_ros.py "$@"' \
    puente-gestos "${OPCIONES_PUENTE[@]}"
