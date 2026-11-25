#!/usr/bin/env bash
set -euo pipefail
TAREA_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec "$TAREA_DIR/../.venv/bin/python" -u "$TAREA_DIR/mision_guiada_robot.py" "$@"
