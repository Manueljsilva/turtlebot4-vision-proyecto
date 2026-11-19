#!/usr/bin/env bash
# LiDAR, reconocimiento y señales; ROS en el Docker existente.
set -euo pipefail

TAREA_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROYECTO_DIR="$(cd -- "$TAREA_DIR/.." && pwd)"
IP_ROBOT="${TB4_IP:-}"
DOMINIO_ROBOT="${TB4_DOMINIO:-}"
CONTENEDOR="${TB4_CONTENEDOR:-proyecto2-sim}"
CONFIG="$TAREA_DIR/config/evasion_lidar.json"
EVASION=false
RECONOCIMIENTO=false
INTEGRADA=false
MISION=false
SENALES_SOLO=false
CON_SENALES=false
OPCIONES=()
OPCIONES_FACIALES=()
OPCIONES_SENALES=()

while [[ $# -gt 0 ]]; do
    case "$1" in
        --ip|--dominio|--contenedor|--config|--nombre|--topico-camara|--tramo|--config-senales)
            [[ $# -ge 2 ]] || { echo "Falta el valor de $1" >&2; exit 2; }
            case "$1" in
                --ip) IP_ROBOT="$2" ;;
                --dominio) DOMINIO_ROBOT="$2" ;;
                --contenedor) CONTENEDOR="$2" ;;
                --config) CONFIG="$2" ;;
                --nombre|--topico-camara) OPCIONES_FACIALES+=("$1" "$2") ;;
                --tramo|--config-senales) OPCIONES_SENALES+=("$1" "$2") ;;
            esac
            shift 2 ;;
        --solo-evasion) EVASION=true; shift ;;
        --solo-reconocimiento) RECONOCIMIENTO=true; shift ;;
        --evasion-reconocimiento) INTEGRADA=true; shift ;;
        --mision) MISION=true; shift ;;
        --solo-senales) SENALES_SOLO=true; shift ;;
        --senales) CON_SENALES=true; shift ;;
        --sin-ventana) OPCIONES_FACIALES+=(--sin-ventana); shift ;;
        --mover) OPCIONES=(--mover); shift ;;
        --help|-h)
            echo 'Uso: bash "task 1/ejecutar_autonomo_robot.sh" --ip IP --dominio ID --solo-evasion [--mover]'
            echo 'O bien: ... --solo-reconocimiento [--nombre NOMBRE] [--topico-camara TOPICO]'
            echo 'O bien: ... --evasion-reconocimiento [--mover] [--nombre NOMBRE]'
            echo 'O bien: ... --mision [--mover] [--nombre NOMBRE] (ida azul -> meta B -> barrido 4 fotos -> vuelta roja -> meta A)'
            echo 'O bien: ... --solo-senales [--tramo ida|vuelta] (cámara, siempre sin movimiento)'
            echo 'Añadir --senales [--tramo ida|vuelta] a la prueba combinada para giros por señales.'
            echo 'Ida usa azul; vuelta usa rojo. Giro de 90 grados por odometría, ajustable con --config-senales.'
            echo 'Sin --mover solo lee LiDAR y muestra decisiones; no crea publicador de velocidad.'
            echo 'Admite TB4_IP, TB4_DOMINIO, TB4_CONTENEDOR y --config RUTA.'
            echo 'El reconocimiento usa la cámara comprimida y visión en la laptop, siempre sin movimiento.'
            echo 'La prueba combinada registra A con Buscar, evade y se detiene al reconocer; aún no regresa a A.'
            echo 'Misión SLAM/Nav2/retorno/QR y llegada por meta pendientes.'
            exit 0 ;;
        *) echo "Opción no disponible: $1. Consultar --help." >&2; exit 2 ;;
    esac
done

MODOS=0
for MODO in "$EVASION" "$RECONOCIMIENTO" "$INTEGRADA" "$MISION" "$SENALES_SOLO"; do
    if "$MODO"; then MODOS=$((MODOS+1)); fi
done
[[ "$MODOS" -eq 1 ]] || { echo 'Elegir un solo modo; consultar --help.' >&2; exit 2; }
if ! "$INTEGRADA" && ! "$SENALES_SOLO" && ! "$MISION"; then
    if "$CON_SENALES" || [[ ${#OPCIONES_SENALES[@]} -gt 0 ]]; then
        echo 'Señales requieren --solo-senales, --evasion-reconocimiento --senales o --mision.' >&2; exit 2
    fi
fi
if "$INTEGRADA" && ! "$CON_SENALES" && [[ ${#OPCIONES_SENALES[@]} -gt 0 ]]; then
    echo 'Añadir --senales para configurar giros en la prueba combinada.' >&2; exit 2
fi
if "$SENALES_SOLO"; then
    [[ ${#OPCIONES[@]} -eq 0 ]] || { echo '--solo-senales nunca permite --mover.' >&2; exit 2; }
    for OPCION in "${OPCIONES_FACIALES[@]}"; do
        [[ "$OPCION" != --nombre && "$OPCION" != --sin-ventana ]] || {
            echo 'La prueba de señales no usa nombres ni --sin-ventana.' >&2; exit 2; }
    done
elif "$INTEGRADA" || "$MISION"; then
    if "$EVASION" || "$RECONOCIMIENTO"; then
        echo 'Elegir un solo modo: --mision, --evasion-reconocimiento, --solo-evasion o --solo-reconocimiento.' >&2
        exit 2
    fi
    for OPCION in "${OPCIONES_FACIALES[@]}"; do
        [[ "$OPCION" != --sin-ventana ]] || { echo 'Este modo usa Buscar en la ventana para iniciar.' >&2; exit 2; }
    done
elif "$RECONOCIMIENTO"; then
    if "$EVASION" || [[ ${#OPCIONES[@]} -gt 0 ]]; then
        echo 'El reconocimiento se prueba solo, sin --solo-evasion ni --mover.' >&2
        exit 2
    fi
else
    "$EVASION" || { echo 'Indicar --solo-evasion, --solo-reconocimiento o --evasion-reconocimiento.' >&2; exit 2; }
    [[ ${#OPCIONES_FACIALES[@]} -eq 0 ]] || { echo 'Opciones faciales requieren --solo-reconocimiento.' >&2; exit 2; }
fi
[[ "$DOMINIO_ROBOT" =~ ^[0-9]{1,3}$ ]] || { echo 'Falta --dominio (0-101)' >&2; exit 2; }
DOMINIO_ROBOT="$((10#$DOMINIO_ROBOT))"
(( DOMINIO_ROBOT <= 101 )) || { echo 'Dominio fuera de 0-101' >&2; exit 2; }
[[ "$CONTENEDOR" =~ ^[a-zA-Z0-9][a-zA-Z0-9_.-]*$ ]] || { echo 'Contenedor inválido' >&2; exit 2; }
PYTHON="$PROYECTO_DIR/.venv/bin/python"
[[ -x "$PYTHON" ]] || { echo 'Falta .venv/bin/python global' >&2; exit 1; }
if ! "$PYTHON" -c 'import ipaddress, sys; ipaddress.IPv4Address(sys.argv[1])' "$IP_ROBOT" 2>/dev/null; then
    echo 'Falta --ip o no es una dirección IPv4 válida' >&2
    exit 2
fi
if "$MISION"; then
    exec "$PYTHON" -u "$TAREA_DIR/busqueda_robot.py" --ip "$IP_ROBOT" \
        --dominio "$DOMINIO_ROBOT" --contenedor "$CONTENEDOR" --config "$CONFIG" \
        --mision "${OPCIONES_FACIALES[@]}" "${OPCIONES[@]}" "${OPCIONES_SENALES[@]}"
fi
if "$INTEGRADA"; then
    if "$CON_SENALES"; then OPCIONES_SENALES=(--senales "${OPCIONES_SENALES[@]}"); fi
    exec "$PYTHON" -u "$TAREA_DIR/busqueda_robot.py" --ip "$IP_ROBOT" \
        --dominio "$DOMINIO_ROBOT" --contenedor "$CONTENEDOR" --config "$CONFIG" \
        "${OPCIONES_FACIALES[@]}" "${OPCIONES[@]}" "${OPCIONES_SENALES[@]}"
fi
if "$SENALES_SOLO"; then
    exec "$PYTHON" -u "$TAREA_DIR/prueba_senales_robot.py" --ip "$IP_ROBOT" \
        --dominio "$DOMINIO_ROBOT" --contenedor "$CONTENEDOR" \
        "${OPCIONES_FACIALES[@]}" "${OPCIONES_SENALES[@]}"
fi
if "$RECONOCIMIENTO"; then
    exec "$PYTHON" -u "$TAREA_DIR/prueba_facial_robot.py" --ip "$IP_ROBOT" \
        --dominio "$DOMINIO_ROBOT" --contenedor "$CONTENEDOR" "${OPCIONES_FACIALES[@]}"
fi
"$PYTHON" -c 'import sys; sys.path.insert(0, sys.argv[1]); from evasion_lidar import cargar_config; print(cargar_config(sys.argv[2]))' "$TAREA_DIR" "$CONFIG"
command -v docker >/dev/null || { echo 'No se encuentra Docker' >&2; exit 1; }
command -v ping >/dev/null || { echo 'No se encuentra ping' >&2; exit 1; }
ping -c 2 -W 2 "$IP_ROBOT" >/dev/null || { echo 'El robot no responde: revisar IP y Wi-Fi' >&2; exit 1; }
sudo -v
ESTADO="$(sudo docker inspect --format '{{.State.Running}} {{.HostConfig.NetworkMode}}' "$CONTENEDOR")"
[[ "$ESTADO" == 'true host' ]] || { echo 'Preparar Docker en red host con COMANDOS_LAB.txt' >&2; exit 1; }
ENTORNO_ROS=(-e "ROS_DOMAIN_ID=$DOMINIO_ROBOT" -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp
             -e ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET -e "ROS_STATIC_PEERS=$IP_ROBOT")
for TOPICO in /scan /cmd_vel; do
    TIPO="$(sudo docker exec "${ENTORNO_ROS[@]}" "$CONTENEDOR" bash --noprofile --norc -c \
        'unset ROS_LOCALHOST_ONLY; source /opt/ros/jazzy/setup.bash && ros2 topic type "$1" --no-daemon --spin-time 5' consultar "$TOPICO")"
    ESPERADO=sensor_msgs/msg/LaserScan
    [[ "$TOPICO" == /cmd_vel ]] && ESPERADO=geometry_msgs/msg/TwistStamped
    [[ "$TIPO" == "$ESPERADO" ]] || { echo "Tipo de $TOPICO inesperado: $TIPO" >&2; exit 1; }
done
sudo docker exec "$CONTENEDOR" bash --noprofile --norc -c \
    'source /opt/ros/jazzy/setup.bash && python3 -c "import rclpy, sensor_msgs.msg, geometry_msgs.msg, tf2_ros; from rclpy.signals import SignalHandlerOptions"'
# Cada sesión tiene su copia; no borrar archivos de una prueba que siga abierta.
REMOTO="$(sudo docker exec "$CONTENEDOR" mktemp -d /tmp/proyecto2-evasion.XXXXXXXX)"
[[ "$REMOTO" =~ ^/tmp/proyecto2-evasion\.[a-zA-Z0-9]+$ ]] || { echo 'Ruta temporal inesperada' >&2; exit 1; }
for ARCHIVO in evasion_lidar.py seguridad_lidar.py; do
    sudo docker cp "$TAREA_DIR/$ARCHIVO" "$CONTENEDOR:$REMOTO/$ARCHIVO" >/dev/null
done
sudo docker cp "$CONFIG" "$CONTENEDOR:$REMOTO/config.json" >/dev/null
echo "ROBOT REAL: IP=$IP_ROBOT dominio=$DOMINIO_ROBOT | cerrar gestos, teclado, Nav2 y Gazebo antes de mover." >&2
if [[ ${#OPCIONES[@]} -gt 0 ]]; then
    echo 'MOVIMIENTO ACTIVADO. Evasión LiDAR; Ctrl+C termina e intenta publicar cero.' >&2
else
    echo 'COMPROBACIÓN SIN MOVIMIENTO: solo lectura y decisiones de LiDAR.' >&2
fi
exec sudo docker exec -it "${ENTORNO_ROS[@]}" "$CONTENEDOR" bash --noprofile --norc -c \
    'unset ROS_LOCALHOST_ONLY; source /opt/ros/jazzy/setup.bash && exec python3 -u "$@"' \
    evasion-robot "$REMOTO/seguridad_lidar.py" --config "$REMOTO/config.json" "${OPCIONES[@]}"
