# Pautas del proyecto

Leer README.md, ESTADO_PROYECTO.md y PLAN_PROYECTO.md antes de modificar el proyecto. Comprobar el código real y distinguir implementación, pruebas locales y resultados físicos.

- Explicar en español claro. Mantener Task A en `task 1/` y Task B en `task 2/`; entrecomillar rutas con espacios.
- Conservar una sola `.venv` global y los notebooks locales con sus ejecuciones. Limpiar salidas únicamente en las copias publicadas.
- Mantener fotos, descriptores, galería, modelos, capturas, credenciales y archivos de sesión fuera de Git.
- Reutilizar YuNet/SFace y el criterio calibrado; no forzar la identidad solicitada. Separar referencias, calibración y evaluación; modificar datos requiere actualización explícita, sin ajustar usando errores del test.
- La selección del nombre ocurre antes de iniciar Task A. El alcance completo incluye obstáculos/flechas, persona objetivo entre fotos de B, QR asociado, regreso a A y visualización del texto. No declarar completa esa misión.
- El modo combinado actual registra A después de Buscar y se detiene al confirmar el rostro; aún no regresa. Conservar un solo publicador coordinado y `--mover` explícito, edades locales válidas y parada por datos inválidos/caducados.
- El hilo de latido no debe renovar por sí mismo las marcas de actividad o imagen. Confirmar únicamente con imágenes nuevas, sin acumular votos repetidos. No inventar A con posición/TF ausentes ni tratar odometría como mapa.
- Conservar pruebas y modos individuales. Los choques diagonales siguen pendientes; no afirmar seguridad física por pasar pruebas sintéticas.
- No instalar ROS Jazzy en el host ni rclpy con pip. Usar el contenedor existente según COMANDOS_LAB.txt, confirmando IP/dominio por visita. No guardar contraseñas.
- No trasladar los experimentos gestuales del compañero por rutina ni fusionar toda su rama sin una petición concreta.
- No delegar en subagentes salvo petición expresa.

Validación local desde la raíz:

```bash
.venv/bin/python -m unittest discover -s "task 1/tests" -t "task 1" -v
.venv/bin/python -m unittest discover -s "task 2/tests" -t "task 2" -v
```

Actualizar la documentación después de avances relevantes. Conservar explícitos los resultados físicos pendientes y las limitaciones de parada ante fallos de red/proceso.

## Misión vigente de Task 1 (17 de noviembre)

Nombre fijado en A → ida con flechas azules → meta B confirmada → solo entonces barrido para buscar a la persona objetivo → leer únicamente el QR asociado y conservar la palabra → vuelta con flechas rojas → meta A confirmada durante la vuelta → detenerse y mostrar la palabra. Una meta recién vista no constituye otra llegada; interpretar su función por etapa. El prototipo todavía reconoce en ruta y se para al confirmar el rostro: meta, barrido, asociación QR, transición de vuelta y retorno completo siguen pendientes.

La flecha confirmada fija el sentido ante un muro, por encima de la elección de lado de la evasión; la parada por falta de espacio/sensores conserva prioridad. Completar 90° por odometría antes de reanudar el avance; el margen de 5° limita el exceso y no permite finalizar a 85°. Mantener la salida única y los modos sin movimiento. Señales: `--solo-senales` o `--evasion-reconocimiento --senales --tramo ida|vuelta [--mover]`; el tramo actual se selecciona antes y no cambia automáticamente. No afirmar validación física de estas señales por las pruebas locales. No modificar la cámara/gestos de Task 2 ni la galería facial por esta integración.
