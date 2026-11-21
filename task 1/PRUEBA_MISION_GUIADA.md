# Ensayo integrado A → B → A

Preparado el 20 de noviembre de 2026. **Implementado y comprobado localmente;
todavía sin validación del recorrido en hardware.** Los modos anteriores se conservan.
Este ensayo sigue azul en la ida y rojo en la vuelta; no reproduce una trayectoria
grabada ni incorpora SLAM/Nav2.

## Colocación acordada

- Iniciar físicamente en A, apuntando hacia el camino. Buscar registra la primera
  pose odométrica válida posterior al inicio. No se inventa un origen de mapa.
- Poner la bandera B **en la pared al fondo, frente al camino**, como confirmó
  el usuario. Debe quedar aproximadamente centrada al llegar.
- Las fotos con sus QR deben poder verse mediante giro sobre el sitio desde B;
  no se implementa acercamiento a una foto. Separar cada pareja foto/QR.
- La vuelta necesita flechas rojas orientadas hacia A y una bandera de meta A.
  Para el primer ensayo, separar claramente A y B y dejar espacio de giro.
  No usar dos metas tan próximas que se solapen los márgenes de este ensayo.
- Conservar Wi-Fi del robot y cerrar controles de teclado, gestos, Nav2 y otras
  evasiones. Docker y controladores del robot deben estar preparados una sola vez.

## Ejecutar

Si la bandera no aparece reconocida, cerrar primero el control anterior con
Ctrl+C y comprobar **solo cámara y detector**, sin movimiento:

```bash
bash "task 1/ejecutar_autonomo_robot.sh" \
  --ip 10.87.179.100 --dominio 5 --solo-meta
```

Mostrar la bandera completa: debe aparecer **META VISTA** y su cuadro. No pide
nombre ni Buscar, no carga galería y no crea publicador de velocidad. No confirma
llegada a B/A. Si muestra «Sin imagen reciente», falta continuidad de cámara;
no se puede comprobar la bandera sobre una imagen caducada. Esc/q/Ctrl+C cierra.
Se conservan los PNG originales en `señales/` y el PDF. El detector no necesita
abrir esos archivos durante la recepción en vivo.

Desde Ubuntu normal, en la raíz del proyecto, con IP/dominio vigentes. Primera
comprobación de cámara, interfaz, nombre y registro A, **sin publicador**:

```bash
bash "task 1/ejecutar_autonomo_robot.sh" \
  --ip 10.87.179.100 --dominio 5 --mision-guiada
```

No completará el recorrido sin desplazamiento real de odometría. Cerrar con
Ctrl+C y luego, con el circuito preparado, activar movimiento explícitamente:

```bash
bash "task 1/ejecutar_autonomo_robot.sh" \
  --ip 10.87.179.100 --dominio 5 --mision-guiada --mover
```

Escribir el nombre y pulsar Buscar. `--nombre` solo precarga el campo.
El tópico predeterminado sigue siendo `/oakd/rgb/preview/image_raw/compressed`;
`--topico-camara` permite especificar otro compatible. Espera la cámara sin un
plazo para el primer fotograma. No combinar con los otros modos, --tramo,
--senales o --ensayo-giro-companero: este modo gestiona los colores y etapas.

## Secuencia y qué observar

1. **IDA:** registra A, navega con evasión y confirma flechas azules en tres
   imágenes nuevas. Disparo local a frente <=0.48 m; giro de 90° por odometría,
   siempre con avance cero. El bloqueo de giro a <=0.32 m frente/laterales sigue
   activo, además de los permisos por sensores/TF.
2. **FRENAR_B:** confirma una meta nueva y frena. Exige >=3 poses nuevas durante
   0.7 s de reposo antes de guardar la orientación de entrada y habilitar rostros.
3. **BUSCAR_B:** barrido sobre el sitio, hasta 0.25 rad/s, una vuelta o 90 s.
   Otra persona no termina la búsqueda. El primer objetivo aceptado pausa el giro.
   Lee solo su QR no ambiguo y exige tres imágenes nuevas con el mismo texto y
   rostro consistente, conservando el criterio facial calibrado.
4. **FRENAR_QR / ORIENTAR_SALIDA:** guarda la palabra en la laptop antes de
   autorizar retorno. Tras reposo se orienta a **orientación de entrada B +180°**.
   El giro necesario depende de dónde quedó mirando la foto; no suma 180° a
   ciegas desde esa foto ni necesita buscar otra vez la bandera B. Giro sin avance,
   <=0.25 rad/s, tolerancia 5° y plazo de 45 s para orientar y frenar.
5. **VUELTA:** después de reposo, reinicia la confirmación visual y usa solamente
   flechas rojas, manteniendo evasión y 90° por odometría.
6. **FRENAR_A / FIN:** confirma una meta nueva cerca del inicio odométrico,
   comprueba reposo y muestra la palabra completa en la terminal
   `[RESULTADO EN A]`. La ventana ofrece una vista del mensaje.

La terminal `[MISION]` muestra fase, motivo, órdenes vx/wz, frente/disparo,
confirmaciones de meta y distancia odométrica a A. `[QR]` distingue detectado,
legible, asociado y confirmado. Las órdenes calculadas no miden velocidad real.

Durante IDA/VUELTA, **META VISTA** significa detección visual. `[META]` explica
si aún falta ausencia previa de la bandera, separación del origen, centrado,
frontal cercano o completar un giro; solo 3/3 admisibles confirman llegada.
Antes de Buscar/registrar A, la misión permanece en ESPERAR y no analiza la
bandera. Para verificarla en ese momento, usar --solo-meta por separado.

El registro del 20/11 permanecía en ESPERAR, a=None y «Sin camara reciente».
Eso no permite concluir un fallo visual de meta durante la ida. `[CAMARA]`
muestra el estado y la edad de la última imagen decodificada desde recepción
ROS; no mide captura ni retraso completo. `inicio_solicitado` distingue una
petición de Buscar de A ya registrado; una petición sin imagen fresca espera
detenida. No cambiar límites de frescura para forzar el arranque.

## Si reconoce el rostro pero no vuelve

El ensayo físico del 21/11 llegó a BUSCAR_B y aceptó al objetivo. Logró una
lectura QR asociada **1/3**, después perdió lectura/rostro y terminó en FALLO
al completar el barrido. Una lectura aislada no guarda la palabra ni habilita
regreso. La confirmación requiere el mismo texto asociado al objetivo en tres
imágenes nuevas consecutivas. La llegada a A aún no se probó en ese intento.

Antes de repetir todo el recorrido, cerrar la misión y probar sin movimiento:

```bash
bash "task 1/ejecutar_autonomo_robot.sh" \
  --ip 10.87.179.100 --dominio 5 --prueba-meta-b
```

Elegir nombre/Buscar, mostrar primero la meta hasta confirmar y esperar reposo.
Después mostrar foto objetivo y su QR completos en la misma imagen. En esta
prueba estacionaria puede acercarse el papel para comprobar legibilidad; no se
ordena acercamiento del robot. Conservar borde blanco del QR, nitidez y parejas
separadas. Esperar **«QR del objetivo confirmado y guardado» / 3/3**. Este modo
guarda el resultado de prueba y permanece quieto; no regresa. No añadir --mover.

Tras conseguir lectura estable, repetir --mision-guiada con el circuito real,
incluidas flechas azules/rojas. Las flechas no se ensayaron en el intento del
21/11; su reconocimiento previo no demuestra los giros de esta integración.
El regreso se inicia después de guardar QR: orientar a entrada B +180° y
avanzar siguiendo rojo, no conducir marcha atrás ni reproducir la ruta de ida.

## Cómo evita confundir las banderas

Valores iniciales de ensayo en [config/mision_guiada.json](config/mision_guiada.json),
sin calibración física todavía:

- Antes de aceptar B: ausencia de meta durante >=1.5 s en imágenes nuevas y
  separación odométrica >=0.6 m de A. La meta debe reaparecer centrada, en tres
  imágenes nuevas, con retorno frontal <=0.6 m y espacio suficiente. No se acepta
  una meta mientras hay un giro por flecha activo.
- Antes de aceptar A: nueva ausencia visual tras comenzar la vuelta, separación
  >=0.6 m de B y distancia odométrica a A <=0.6 m, además de meta centrada,
  frontal <=0.6 m y tres confirmaciones nuevas.
- Las repeticiones del mismo paquete no suman votos ni tiempo de ausencia.
  Cámara/pose caducadas reinician la confirmación de meta.

El retorno frontal es un mínimo de un sector LiDAR: **no demuestra que sea la
distancia al cartel**. La odometría puede derivar y la meta tiene igual apariencia
en A/B. Estos filtros reducen confusiones en este montaje, sin identificar el
lugar de forma global ni garantizar llegada exacta. No forzar la llegada ampliando
radios o reduciendo barreras sin revisar el registro.

## Flechas pendientes y paradas

Se conserva el controlador original verificado del compañero y el perfil local
de 0.48 m. La coordinación añadida **solo en este modo**:

- Mientras hay flecha pendiente, si la evasión propone otro giro, se detiene;
  no invierte el sentido de la flecha ni ignora un obstáculo lateral.
- Una imagen nueva que vuelve a confirmar el mismo cartel/sentido renueva la
  espera. Un latido o paquete repetido no la renueva. Sin reconfirmación durante
  seis segundos, detiene el ensayo; aproximación limitada a 30 s incluso si el
  cartel sigue visible. Cambio de sentido del mismo cartel pendiente también detiene.
- Al bloquearse conserva cero; timeout de giro por flecha sigue siendo 12 s.
  No intenta pasar por una dirección bloqueada ni resuelve automáticamente el atasco.

Actividad de GUI >0.5 s, cámara >1 s, pose/LiDAR >0.6 s, TF/cobertura insuficientes
o espacio reducido detienen según la fase. Corte odométrico coherente exige
>=3 poses nuevas /0.7 s de reposo para recuperar. Salto >0.25 m o >0.5 rad,
marca retrocedida y cambio de marco enclavan el movimiento; corte >2 s durante
rotación también. Límite total de ensayo: 900 s. Son valores internos de ensayo,
no reglas oficiales de competición.

**FALLO:** queda detenido, conserva el QR ya guardado y exige cerrar/revisar para
repetir. Esc/Salir/Ctrl+C intentan ceros y terminan; no usar Ctrl+Z. El vigilante
no garantiza entregar una parada si muere el propio nodo ROS o se corta la red.
El vídeo retrasado, los choques diagonales y la huella de giro siguen pendientes
de medir; esta integración no demuestra que estén resueltos.

Se guarda solo JSON UTF-8 en `datos/sesiones/mision_guiada_*.json`, ignorado por Git:
nombre solicitado, identidad confirmada, palabra, poses/marcos A/B, fase y llegada.
No guarda fotos, no recalibra la galería ni cambia Task 2. Los ensayos individuales
de flechas y meta/rostro/QR siguen disponibles para aislar fallos.
