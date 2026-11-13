#!/usr/bin/env bash
# Navegación Autónoma (SLAM + Nav2 + Exploración + Visión) en Gazebo dentro de Docker.
set -euo pipefail

TAREA_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROYECTO_DIR="$(cd -- "$TAREA_DIR/.." && pwd)"
CONTENEDOR="${TB4_CONTENEDOR:-proyecto2-sim}"
ACCION="mision"
ARGS_EXTRA=()

while [[ $# -gt 0 ]]; do
    case "$1" in
        --contenedor)
            [[ $# -ge 2 ]] || { echo "Falta el valor de --contenedor" >&2; exit 2; }
            CONTENEDOR="$2"; shift 2 ;;
        --slam-nav2) ACCION="slam_nav2"; shift ;;
        --solo-evasion) ACCION="evasion"; shift ;;
        --solo-reconocimiento) ACCION="reconocimiento"; shift ;;
        --help|-h)
            echo 'Uso: bash "task_autonoma/ejecutar_autonomo_gazebo.sh" [OPCION] [ARGS...]'
            echo '  --slam-nav2            Inicia slam_toolbox + Nav2 en Gazebo (dejar abierto en otra terminal)'
            echo '  --solo-evasion         Prueba evasión reactiva LiDAR (añadir --mover para mover)'
            echo '  --solo-reconocimiento  Prueba cámara + YuNet/SFace sin mover el robot'
            echo '  (sin opción)           Ejecuta la misión autónoma A -> B -> Reconocimiento -> A'
            exit 0 ;;
        *) ARGS_EXTRA+=("$1"); shift ;;
    esac
done

command -v docker >/dev/null || { echo 'No se encuentra Docker' >&2; exit 1; }
sudo -v
ESTADO_CONTENEDOR="$(sudo docker inspect --format '{{.State.Running}}' "$CONTENEDOR")"
[[ "$ESTADO_CONTENEDOR" == true ]] || {
    echo 'Abre primero Gazebo con el bloque 3 de task 2/COMANDOS.txt' >&2
    exit 1
}

[[ -f "$PROYECTO_DIR/task 1/datos/galeria.npz" ]] || { echo 'Falta task 1/datos/galeria.npz' >&2; exit 1; }
[[ -f "$PROYECTO_DIR/task 1/datos/criterio.json" ]] || { echo 'Falta task 1/datos/criterio.json' >&2; exit 1; }
[[ -d "$PROYECTO_DIR/task 1/modelos" ]] || { echo 'Falta carpeta task 1/modelos' >&2; exit 1; }

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

case "$ACCION" in
    slam_nav2)
        exec sudo docker exec -it "$CONTENEDOR" bash -lc \
            'source /opt/ros/jazzy/setup.bash && exec ros2 launch /tmp/proyecto2-autonomo/launch/slam_nav2_launch.py use_sim_time:=true'
        ;;
    evasion)
        exec sudo docker exec -it "$CONTENEDOR" bash -lc \
            'source /opt/ros/jazzy/setup.bash && export PYTHONPATH="/tmp/proyecto2-autonomo:${PYTHONPATH:-}" && exec python3 -u /tmp/proyecto2-autonomo/seguridad_lidar.py --modo gazebo --config /tmp/proyecto2-autonomo/config/mision_config.json "$@"' \
            evasion-gazebo "${ARGS_EXTRA[@]}"
        ;;
    reconocimiento)
        exec sudo docker exec -it "$CONTENEDOR" bash -lc \
            'source /opt/ros/jazzy/setup.bash && export PYTHONPATH="/tmp/proyecto2-autonomo:${PYTHONPATH:-}" && exec python3 -u /tmp/proyecto2-autonomo/navegacion_autonoma.py --modo gazebo --solo-reconocimiento --config /tmp/proyecto2-autonomo/config/mision_config.json --datos-dir /tmp/proyecto2-autonomo/datos --modelos-dir /tmp/proyecto2-autonomo/modelos "$@"' \
            rec-gazebo "${ARGS_EXTRA[@]}"
        ;;
    mision)
        exec sudo docker exec -it "$CONTENEDOR" bash -lc \
            'source /opt/ros/jazzy/setup.bash && export PYTHONPATH="/tmp/proyecto2-autonomo:${PYTHONPATH:-}" && exec python3 -u /tmp/proyecto2-autonomo/navegacion_autonoma.py --modo gazebo --config /tmp/proyecto2-autonomo/config/mision_config.json --datos-dir /tmp/proyecto2-autonomo/datos --modelos-dir /tmp/proyecto2-autonomo/modelos "$@"' \
            mision-gazebo "${ARGS_EXTRA[@]}"
        ;;
esac
