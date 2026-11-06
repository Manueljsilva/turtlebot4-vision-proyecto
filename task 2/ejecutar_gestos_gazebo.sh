#!/usr/bin/env bash
# Webcam en .venv del host; puente ROS en el contenedor ya instalado.
set -euo pipefail

TAREA_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROYECTO_DIR="$(cd -- "$TAREA_DIR/.." && pwd)"
CONTENEDOR="proyecto2-sim"
OPCIONES_PUENTE=()
OPCIONES_WEBCAM=()

while [[ $# -gt 0 ]]; do
    case "$1" in
        --mover) OPCIONES_PUENTE=(--publicar); shift ;;
        --camara)
            [[ $# -ge 2 ]] || { echo 'Falta el numero de --camara' >&2; exit 2; }
            OPCIONES_WEBCAM+=(--camara "$2"); shift 2 ;;
        --help|-h)
            echo 'Uso desde Proyecto2: bash "task 2/ejecutar_gestos_gazebo.sh" [--mover] [--camara 0]'
            echo 'Sin --mover comprueba webcam y enlace ROS sin publicar velocidad.'
            exit 0 ;;
        *) echo "Opcion desconocida: $1" >&2; exit 2 ;;
    esac
done

[[ -x "$PROYECTO_DIR/.venv/bin/python" ]] || { echo 'Falta .venv/bin/python' >&2; exit 1; }
command -v docker >/dev/null || { echo 'No se encuentra Docker' >&2; exit 1; }
sudo -v
ESTADO_CONTENEDOR="$(sudo docker inspect --format '{{.State.Running}}' "$CONTENEDOR")"
[[ "$ESTADO_CONTENEDOR" == true ]] || { echo 'Abre primero Gazebo con el bloque 3 de task 2/COMANDOS.txt' >&2; exit 1; }

# Copiar solo tres modulos pequeños, sin instalar paquetes ni montar la webcam.
sudo docker exec "$CONTENEDOR" mkdir -p /tmp/proyecto2-gestos
for archivo in gestos.py control_gestos.py puente_gestos_ros.py; do
    sudo docker cp "$TAREA_DIR/$archivo" "$CONTENEDOR:/tmp/proyecto2-gestos/$archivo" >/dev/null
done
echo 'Cierra el control por teclado y la webcam del notebook antes de esta prueba.' >&2
if [[ ${#OPCIONES_PUENTE[@]} -eq 0 ]]; then
    echo 'MODO COMPROBACION: las ordenes se reciben en ROS, sin mover el robot.' >&2
else
    echo 'MODO MOVIMIENTO: los gestos controlaran el robot de Gazebo.' >&2
fi

# -i conserva la entrada del puente. Sin -t: se transportan lineas JSON, no teclas.
# Al cerrar la webcam se cierra el enlace; el puente publica DETENER y termina.
"$PROYECTO_DIR/.venv/bin/python" -u "$TAREA_DIR/webcam_gestos.py" --emitir "${OPCIONES_WEBCAM[@]}" |
    sudo docker exec -i -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST "$CONTENEDOR" bash -lc \
    'source /opt/ros/jazzy/setup.bash && exec python3 -u /tmp/proyecto2-gestos/puente_gestos_ros.py "$@"' \
    puente-gestos "${OPCIONES_PUENTE[@]}"
