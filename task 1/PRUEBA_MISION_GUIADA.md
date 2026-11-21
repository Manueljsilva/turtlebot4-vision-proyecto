# Ensayo integrado A → B → A

Actualizado el 21 de noviembre de 2026: el usuario confirma que **el QR funciona tras aumentar su tamaño**. La prueba inmediata pasa a ser el recorrido completo, incluidas ambas banderas y flechas azules/rojas. **Una imagen nueva válida basta para aceptar una detección**. Nuevo botón **Reiniciar** para repetir sin cerrar ventana/cámara. **215 pruebas locales pasan; todavía sin validación del recorrido completo ni del reinicio en hardware.** Los modos anteriores se conservan.
Este ensayo sigue azul en la ida y rojo en la vuelta; no reproduce una trayectoria
grabada ni incorpora SLAM/Nav2.

## Colocación acordada

- Iniciar físicamente en A, apuntando hacia el camino. Buscar registra la primera
  pose odométrica válida posterior al inicio. No se inventa un origen de mapa.
- Poner también una **bandera en A**, de frente al robot cuando regrese. Puede
  quedar detrás del robot al arrancar hacia el circuito. Puede ser el mismo diseño
  que B: el programa interpreta la bandera según la etapa y la posición registrada.
- Poner la bandera B **en la pared al fondo, frente al camino**, como confirmó
  el usuario. Debe quedar aproximadamente centrada al llegar.
- Las fotos con sus QR deben poder verse mediante giro sobre el sitio desde B;
  no se implementa acercamiento a una foto. Separar cada pareja foto/QR. Conservar
  el tamaño de QR que ya funcionó, completo y nítido junto al rostro objetivo.
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

**Comprobar que se abrió el modo correcto:** la terminal debe imprimir
`ENSAYO INTEGRADO`, `mover=True` y estados `[MISION]`. Tras nombre/Buscar y
sensores válidos, registra A y pasa a IDA. Si imprime `ENSAYO CERCA DE B` o
`[PRUEBA B]`, se abrió el ensayo aislado, que no realiza el recorrido; si además
indica `mover=False`, no publica velocidades. Eso ocurrió en el registro del
21/11: esperaba la meta sin movimiento habilitado. Cerrar con Ctrl+C y ejecutar
el comando completo anterior con `--mision-guiada --mover`, sin mezclar modos.
No se activa movimiento automáticamente al pulsar Buscar si falta --mover.

## Repetir con otra persona sin cerrar

1. Pulsar **Reiniciar**, abajo junto a Salir. El control ordena cero y el botón
   pasa a «Reiniciando…». Buscar queda bloqueado hasta confirmar reposo con
   odometría nueva durante al menos 0.7 s; no basta esperar una imagen.
2. Cuando se vacíe el campo y se habilite la selección, colocar el robot en A
   mirando hacia el recorrido. Sigue detenido: el reinicio no inicia otra misión.
3. Escribir/elegir otra persona y pulsar **Buscar**. Ese intento vuelve a registrar
   A con una pose posterior al inicio y usa azul. No hereda QR, flechas, puntos
   A/B, orientación ni plazos del anterior.

Cámara y galería permanecen abiertas; cada intento tiene su propio JSON textual
local y conserva el registro anterior. Puede usarse también tras FIN o FALLO.
Revisar el motivo de un fallo antes de repetir; las protecciones siguen activas.
Si no hay odometría reciente, permanece «Reiniciando» y detenido; Salir/Esc siguen
disponibles. Cerrar y relanzar el comando **una vez** para cargar esta versión;
después ya no hace falta Ctrl+C entre intentos. Este botón está en --mision-guiada.

## Secuencia y qué observar

1. **IDA:** registra A, navega con evasión y acepta flechas azules en la primera
   imagen nueva válida. Disparo local a frente <=0.48 m; giro de 90° por odometría,
   siempre con avance cero. El bloqueo de giro a <=0.32 m frente/laterales sigue
   activo, además de los permisos por sensores/TF.
2. **FRENAR_B:** confirma una meta nueva y frena. Exige >=3 poses nuevas durante
   0.7 s de reposo antes de guardar la orientación de entrada y habilitar rostros.
3. **BUSCAR_B:** barrido sobre el sitio, hasta 0.25 rad/s, una vuelta o 90 s.
   Otra persona no termina la búsqueda. El primer objetivo aceptado pausa el giro.
   Lee solo su QR no ambiguo y acepta una imagen nueva con texto legible y
   rostro objetivo reconocido, conservando el criterio facial calibrado.
4. **FRENAR_QR / ORIENTAR_SALIDA:** guarda la palabra en la laptop antes de
   autorizar retorno. Tras reposo se orienta a **orientación de entrada B +180°**.
   El giro necesario depende de dónde quedó mirando la foto; no suma 180° a
   ciegas desde esa foto ni necesita buscar otra vez la bandera B. Giro sin avance,
   <=0.25 rad/s, tolerancia 5° y plazo de 45 s para orientar y frenar.
5. **VUELTA:** después de reposo, reinicia la confirmación visual y usa solamente
   flechas rojas, manteniendo evasión y 90° por odometría.
6. **FRENAR_A / FIN:** confirma una meta nueva cerca del inicio odométrico,
   comprueba reposo y muestra la palabra completa en la terminal
   `[RESULTADO EN A]` y en un **panel grande «Punto A alcanzado / Mensaje del QR»**.
   El robot queda detenido. El panel conserva tildes/ñ y ajusta texto largo;
   sigue visible aunque después deje de llegar vídeo. Estar cerca de A sin ver
   su bandera no finaliza, y durante VUELTA aún no muestra el resultado final.

La terminal `[MISION]` muestra fase, motivo, órdenes vx/wz, frente/disparo,
confirmaciones de meta y distancia odométrica a A. `[QR]` distingue detectado,
legible, asociado y confirmado. Las órdenes calculadas no miden velocidad real.

Durante IDA/VUELTA, **META VISTA** significa detección visual. `[META]` explica
si aún falta ausencia previa de la bandera, separación del origen, centrado,
frontal cercano o completar un giro; una imagen nueva admisible confirma llegada.
Antes de Buscar/registrar A, la misión permanece en ESPERAR y no analiza la
bandera. Para verificarla en ese momento, usar --solo-meta por separado.

El registro del 20/11 permanecía en ESPERAR, a=None y «Sin camara reciente».
Eso no permite concluir un fallo visual de meta durante la ida. `[CAMARA]`
muestra el estado y la edad de la última imagen decodificada desde recepción
ROS; no mide captura ni retraso completo. `inicio_solicitado` distingue una
petición de Buscar de A ya registrado; una petición sin imagen fresca espera
detenida. No cambiar límites de frescura para forzar el arranque.

## Si reconoce el rostro pero no vuelve

**Historial, ya superado por la confirmación de lectura con QR mayor:** el ensayo físico del 21/11 llegó a BUSCAR_B y aceptó al objetivo. Logró una
lectura QR asociada **1/3**, después perdió lectura/rostro y terminó en FALLO
al completar el barrido. Con el filtro anterior, una lectura aislada no guardaba
la palabra ni habilitaba regreso. La revisión actual acepta el texto asociado al objetivo en una
imagen nueva válida; aquel resultado corresponde al filtro anterior. La llegada a A aún no se probó en ese intento.

Si reaparece el problema, cerrar la misión y aislar la lectura sin movimiento:

```bash
bash "task 1/ejecutar_autonomo_robot.sh" \
  --ip 10.87.179.100 --dominio 5 --prueba-meta-b
```

Elegir nombre/Buscar, mostrar primero la meta hasta confirmar y esperar reposo.
Después mostrar foto objetivo y su QR completos en la misma imagen. En esta
prueba estacionaria puede acercarse el papel para comprobar legibilidad; no se
ordena acercamiento del robot. Conservar borde blanco del QR, nitidez y parejas
separadas. Esperar **«QR del objetivo confirmado y guardado» / 1/1**. Este modo
guarda el resultado de prueba y permanece quieto; no regresa. No añadir --mover.

El usuario ya confirmó lectura con el QR mayor. Repetir ahora --mision-guiada con el circuito real,
incluidas flechas azules/rojas. Las flechas no se ensayaron en el intento del
21/11; su reconocimiento previo no demuestra los giros de esta integración.
El regreso se inicia después de guardar QR: orientar a entrada B +180° y
avanzar siguiendo rojo, no conducir marcha atrás ni reproducir la ruta de ida.

## Aceptación con una imagen nueva

Por petición del usuario, ya no se esperan tres confirmaciones visuales. Una
imagen nueva y reciente basta para flecha del color correcto, meta admisible o
rostro objetivo con su QR legible y asociado sin ambigüedad. La cuenta de la
interfaz es 0/1 o 1/1; repetir un paquete no produce una detección nueva. Antes
de Buscar la vista previa no envía una maniobra al controlador.

La primera flecha válida se memoriza inmediatamente. El giro sigue esperando
frente <=0.48 m y espacio/sensores válidos; completa 90° sobre el sitio. Si no
hay imagen reciente o hay señales ambiguas, no se inventa una dirección. La
nueva aceptación no exige una pausa adicional para acumular votos.

El QR sigue necesitando texto: detectados=1 y legibles=0 significa localizar
sin decodificar. Mantener rostro y QR completos/nítidos, con borde blanco del
QR. El diagnóstico muestra resolución recibida y cajas QR en píxeles nativos,
por ejemplo imagen=320x240px | QR_px=28x31; no es una distancia física ni un
mínimo universal. No se cambia resolución ni velocidad automáticamente.

Las guardas de llegada A/B, frescura y seguridad se conservan. Después de la
meta sigue siendo necesario frenar y confirmar reposo odométrico durante
0.7 s con al menos tres poses nuevas: es comprobación de frenado, distinta de
las confirmaciones visuales eliminadas. La velocidad continúa en 0.10 m/s y
0.40 rad/s para evasión; barrido y orientación, hasta 0.25 rad/s.

## Cómo evita confundir las banderas

Valores iniciales de ensayo en [config/mision_guiada.json](config/mision_guiada.json),
sin calibración física todavía:

- Antes de aceptar B: ausencia de meta durante >=1.5 s en imágenes nuevas y
  separación odométrica >=0.6 m de A. La meta debe reaparecer centrada, en una
  imagen nueva válida, con retorno frontal <=0.6 m y espacio suficiente. No se acepta
  una meta mientras hay un giro por flecha activo.
- Antes de aceptar A: nueva ausencia visual tras comenzar la vuelta, separación
  >=0.6 m de B y distancia odométrica a A <=0.6 m, además de meta centrada,
  frontal <=0.6 m y una imagen nueva válida.
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

**FALLO:** queda detenido y conserva el QR ya guardado en el registro. No reanuda
automáticamente: revisar el motivo y pulsar Reiniciar para otro intento, o cerrar.
Esc/Salir/Ctrl+C intentan ceros y terminan; no usar Ctrl+Z. El vigilante
no garantiza entregar una parada si muere el propio nodo ROS o se corta la red.
El vídeo retrasado, los choques diagonales y la huella de giro siguen pendientes
de medir; esta integración no demuestra que estén resueltos.

Se guarda solo JSON UTF-8 en `datos/sesiones/mision_guiada_*.json`, ignorado por Git:
nombre solicitado, identidad confirmada, palabra, poses/marcos A/B, fase y llegada.
No guarda fotos, no recalibra la galería ni cambia Task 2. Los ensayos individuales
de flechas y meta/rostro/QR siguen disponibles para aislar fallos.

El panel final usa Pillow 13.0.0, ya presente en la .venv compartida y declarado
en requirements. No necesita instalar paquetes de ROS en Ubuntu ni cambiar Docker.
