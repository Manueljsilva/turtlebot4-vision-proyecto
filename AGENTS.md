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

Nombre fijado en A → ida con flechas azules → meta B confirmada → solo entonces barrido para buscar a la persona objetivo → leer únicamente el QR asociado y conservar la palabra → vuelta con flechas rojas → meta A confirmada durante la vuelta → detenerse y mostrar la palabra. Una meta recién vista no constituye otra llegada; interpretar su función por etapa. El prototipo combinado todavía reconoce en ruta y se para al confirmar el rostro. Hay un ensayo separado de meta/frenado/barrido/rostro/QR, con avance previo opcional; falta validar QR físicamente y unirlo a flechas y retorno en la misión completa.

La flecha confirmada fija el sentido ante un muro, por encima de la elección de lado de la evasión; la parada por falta de espacio/sensores conserva prioridad. Completar 90° por odometría antes de reanudar el avance; el margen de 5° limita el exceso y no permite finalizar a 85°. Mantener la salida única y los modos sin movimiento. Señales: `--solo-senales` o `--evasion-reconocimiento --senales --tramo ida|vuelta [--mover]`; el tramo actual se selecciona antes y no cambia automáticamente. No afirmar validación física de estas señales por las pruebas locales. No modificar la cámara/gestos de Task 2 ni la galería facial por esta integración.

## Ensayos y pendientes del 19 de noviembre

- `--prueba-meta-b [--avanzar-hasta-meta] [--mover]`: frenar y confirmar 0.7 s de reposo antes del barrido; cámaras/actividad/sensores frescos, máximo una vuelta/90 s. No registra A ni regresa. Conservar primer fallo y permitir visión estacionaria ya habilitada sin liberar un movimiento enclavado.
- Un corte coherente de recepción odométrica exige cero y datos estables para recuperar; salto espacial, cambio de marco o incertidumbre angular excesiva enclavan movimiento. No atribuir automáticamente un corte a red ni reiniciar ángulo/plazo para evadir límites.
- QR pequeño: reintentos acotados por recorte solo con objetivo; esquina únicamente con contorno de una sola foto, una cara y un QR. Sin evidencia fiable conservar rechazo de ambigüedad, frescura y tres imágenes nuevas. No forzar identidad ni guardar cualquier QR legible. Solo resultado textual en sesiones locales ignoradas.
- Ensayo opcional `--evasion-reconocimiento --senales --ensayo-giro-companero [--mover]`: copias originales verificadas de e78c3b5 en `task 1/experimentos/mejora_competicion/`, sin fusionar toda la rama. Usa evasión local, espera de 0.52 m y memoria original de seis segundos. Esta caducidad y su coordinación aún requieren mejoras.
- Por petición del usuario, evasión de Task 1 separa avance y giro: vx=0 en correcciones frontales/laterales; reanuda recto al despejar. Conservar las velocidades/distancias y Task 2. El giro por flecha exige 90°; la evasión sin flecha no tiene ese ángulo fijo.
- El usuario salió del laboratorio antes de probar físicamente esa nueva evasión. Próxima visita: probar el giro sobre el sitio y después ajustar el disparo. `frente_m` es telemetría; el parámetro es `esperar_pared_m`, hoy 0.52 m. No reducir ahora umbral ni barreras ni afirmar frenado/huella validados.
- Diagnóstico: 3/3 visual no prueba movimiento. Distinguir evento enviado/recibido/consumido, fase, propuesta angular, permiso y existencia de publicador. GIRANDO puede estar bloqueado. Un timeout exige cerrar y revisar antes de otro intento.

Las fuentes de fotos, galería/criterio, modelos y sesiones deben prepararse localmente. Los resúmenes compartidos no contienen imágenes personales ni sustituyen los resultados físicos pendientes.
