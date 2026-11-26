# Misión completa A → B → A

Actualizado el 26/11/2026. Integra todos los cambios autorizados de
`competicion-pasillos-y-vision` (`91d3b55`) en Task 1 y Task 3.
349 pruebas Task 1 y 367 Task 3 pasan; no certifican frenado ni navegación real.
La siguiente prueba física sigue siendo el recorrido completo. Task 2 no se modifica.

## Ejecutar

Cerrar otros controles del robot. Desde la raíz, indicar la IP y dominio de la
sesión, por ejemplo los últimos usados en el laboratorio:

```bash
bash "task 1/ejecutar_autonomo_robot.sh" \
  --ip 10.87.179.100 --dominio 5 \
  --mision-guiada --mover
```

Cerrar y relanzar una vez copia el código/perfiles nuevos a Docker. `Reiniciar`
permite otro intento sin cerrar la ventana, pero no recarga módulos. Sin `--mover`
no crea el publicador. Conservar `.venv`, galería y modelos locales.

1. Colocar el robot en A, orientado hacia el circuito. Escribir una persona de
   la galería y pulsar **Buscar**. A se registra con odometría válida posterior.
2. Seguir flechas **azules**, evitar obstáculos y aproximarse a B.
3. La bandera de B debe estar al fondo, frente al sentido de entrada. Su imagen
   centrada y nueva se acepta al frontal <=0.40 m, tras separarse >=0.60 m de A y
   haber dejado de ver la meta durante >=1.5 s en imágenes nuevas.
4. Frenar: >=3 poses nuevas durante 0.7 s. Solo entonces buscar rostros. Barrido
   a <=0.14 rad/s, con pausas breves ante caras no concluyentes centradas. Caras
   pequeñas se evalúan a varias escalas; esto no garantiza reconocer fotos sin detalle.
5. Reconocer al objetivo y leer únicamente su QR no ambiguo. Admite misma foto
   delimitada, relación lateral o QR debajo según la geometría de asociación.
   QR localizado sin texto no cuenta. **QR confirmado y guardado** acredita lectura.
6. Guardar palabra, frenar e intentar orientar hacia entrada B +180° para VUELTA.
   La revisión permite pasar directamente a VUELTA por LiDAR/ruta si la orientación
   resulta contraria/incoherente o hay determinados cortes durante esa salida;
   no implica haber completado físicamente los 180°.
   Las flechas **rojas** prevalecen sobre la ruta odométrica guardada, que ayuda
   cuando faltan rojas. No hay mapa, SLAM ni garantía de reproducir exactamente la ruta.
7. La bandera de A debe quedar enfrente al regresar. FIN requiere bandera nueva,
   ausencia/separación de B, cercanía odométrica a A <=0.60 m y reposo. Solo entonces
   aparece la palabra en la tarjeta final. Pulsar **Reiniciar** para otra persona.

## Cámara y muro

Sin vídeo reciente, IDA/VUELTA pueden seguir con LiDAR/TF, pose y control recientes.
No aceptan metas, rostros ni QR usando una imagen antigua. Al entrar en la zona
frontal de maniobra sin vídeo, ordenan cero y muestran:

`Muro cercano: esperar imagen nueva para flecha/meta`

El umbral es el mayor entre llegada a meta y precaución + margen de salida:
con el perfil incorporado **max(0.40, 0.38+0.04) = 0.42 m**. Es retorno frontal
LiDAR, no distancia métrica al cartel. Espera una imagen nueva procesada posterior
a la parada durante **hasta 2 s**. Si no llega, permite continuar con LiDAR sin vídeo:
puede perder una flecha o la bandera durante ese intervalo. Rearma la espera al
recuperar vídeo fresco o despejar frontal>0.47m. Una imagen nueva puede aportar
flecha/bandera, conservando sus guardas; no se aceptan detecciones antiguas.
En otras fases, incluido buscar persona/QR, la pérdida de vídeo sigue deteniendo.
La edad se mide desde recepción ROS, no desde captura. Vigencia de imagen: 1 s.

## Geometría y movimiento

- Perfil del compañero conservado: avance **0.10 m/s**, angular máximo **0.40 rad/s**;
  precaución **0.38 m**, parada general **0.28 m**, lateral **0.20 m**, cono frontal **22°**.
- Semiancho recto normal **0.20 m**, comprobado según el usuario. Tras un escape
  sin avance, permite emergencia con semiancho **0.185 m** a≤0.05m/s, frente
  abierto y trayectoria/margen longitudinal válidos. El nuevo ancho está autorizado,
  pero no tiene validación física independiente. Alcance frontal conservador
  y radio de giro: **0.22+0.02 = 0.24 m**. Poder avanzar por un pasillo no autoriza girar allí.
- Avance protegido por trayectoria y margen longitudinal **0.02 + v×(edad scan+0.1)**,
  también con velocidad normal. Aproximación a señal/meta y salida adaptable <=0.05 m/s.
- Flecha confirmada en una imagen válida: guardar dirección, aproximar hasta
  **0.30+0.005 = 0.305 m**, frenar y girar sobre el sitio hasta salida libre. Con este
  perfil, salida frontal >**0.38+0.04 = 0.42 m**, contorno y trayectoria protegidos.
  No hay objetivo fijo de 90° ni avance obligatorio de 40 cm.
- Todo giro exige contorno 360°/TF reciente y mínimo al centro >**0.24 m**.
  Nunca invierte para eludir una huella ocupada, tampoco en corrección lateral.
- Si no cabe girar, puede escapar recto hacia atrás a **0.06 m/s**, solo con recorrido
  trasero conocido y margen >=max(0.05, 0.02+0.06×(edad scan+0.1)). Cada segmento
  se limita a **0.12 m o 2 s**; vuelve a comprobar espacio en cada ciclo. Termina antes
  si se libera el giro o se pierde holgura, frena y exige reposo antes de otra maniobra.
  En el retroceso usa semiancho min(radio,0.185m) y solo puntos x<-0.02m.
  Añade alineación sobre el sitio a media velocidad angular con huella libre,
  asimetría lateral≥0.006m y recorrido≤0.50m en las condiciones de la rama.
- Memoria de flecha: hasta 30 s, sin renovar por latidos. Se conserva la reevaluación
  de maniobras de la rama: giro de 12 s, aproximación de 30 s; al vencer libera la
  maniobra y vuelve a evaluar. Conflictos visuales se registran y cancelan la pendiente.
  Barrido puede reiniciar su vuelta/90 s; orientación agotada puede entregar el
  retorno al respaldo odométrico. El límite total de misión sigue siendo **900 s**.
  Estos límites son internos; no son tiempos oficiales de competición.

## Cortes y fallos

Control de interfaz >0.5 s, pose o LiDAR >0.6 s y TF/cobertura desconocidos conservan
parada. +inf no se convierte en espacio libre; se exige cobertura por cuadrantes.
Cortes odométricos coherentes recuperan con >=3 poses nuevas/0.7 s en reposo.
Salto >0.25 m, >0.5 rad, tiempo retrocedido o cambio de marco enclavan **FALLO**:
A y la ruta anterior no se reinterpretan en coordenadas nuevas. Un corte coherente
>2s durante giro ahora limpia maniobras y recupera tras reposo; también en gestos
de Task 3. Giro contrario por flecha descarta la maniobra, y ciertos problemas de
orientación tras QR entregan a VUELTA sin completar 180°. El QR se conserva.
Los FALLO restantes requieren revisar y Reiniciar desde A; no se liberan solos.

`[MISION]` y la interfaz indican el motivo de parada. Telemetría adicional:
`espera_camara_muro`, `escape_atras`, `ruta_regreso_puntos`, `ruta_regreso_indice`.
El nodo comprueba otra vez la holgura antes de publicar cualquier giro.
Esc/Salir/Ctrl+C intentan ceros. No usar Ctrl+Z. Una muerte del nodo o un corte de
red pueden impedir entregar la parada; el vigilante no garantiza su recepción.

Se guardan solo datos textuales locales en `datos/sesiones/`, ignorados por Git;
no se almacenan imágenes ni se recalibra la galería. El reconocimiento multiescala
cambia la consulta: faltan métricas físicas de confusión y tiempos con cámara real.
La velocidad máxima final sigue pendiente; estas pruebas conservan 0.10 m/s.
