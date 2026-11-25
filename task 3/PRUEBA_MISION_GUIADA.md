# Prueba completa de Task 3

Usar [README.md](README.md) para el comando y el comportamiento vigente. Hacer un recorrido **A → azul/obstáculos → bandera B → persona objetivo/QR propio → salida → rojo/obstáculos → bandera A → palabra**. No requiere pruebas físicas aisladas.

Durante el mismo recorrido, pulsar Cambiar a gestos y volver a autónomo. Comprobar que frena para entregar control, conserva nombre/QR y no ejecuta una flecha pendiente anterior al cambio. En B, el modo gestos permite buscar girando, con avance bloqueado; al identificar el objetivo, también se bloquea el giro para leer el QR. Tras leer, dirigir el giro de salida o volver a autónomo.

Registrar la terminal `[MISION]` completa y observar `modo`, `modo_solicitado`, `cambio_modo`, fase, estado, velocidades propuestas y diagnósticos de sensores. Son propuestas/recepción local, no velocidades o latencia física medidas. Un QR dibujado sin texto no implica lectura. Solo FIN con palabra confirma el resultado de la misión.

Si se detiene, revisar el motivo: mano ausente/caducada, entrega de control, reposo, LiDAR/TF, trayectoria ocupada, cámara ante muro, pérdida de odometría o FALLO. No desactivar guardas para forzar movimiento. Reiniciar permite otro intento; conservar la evidencia textual local sin fotografías.
