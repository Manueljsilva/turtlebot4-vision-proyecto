# Ensayo del punto de giro del compañero

Copias **sin modificar** de `task 1/control_giros.py` y
`task 1/config/senales_giro.json` de
[`mejora-competicion`, e78c3b5](https://github.com/Manueljsilva/turtlebot4-vision-proyecto/tree/e78c3b52d07be0add80103365bb2ad5fc068e91b).
El cargador comprueba sus SHA-256 antes de utilizarlas. No reemplazan el controlador
habitual ni se fusiona la rama. No incluyen su misión, informe, presentación ni
cambios de cámara.

La opción **--ensayo-giro-companero** solo se admite junto a
`--evasion-reconocimiento --senales`. Los comandos completos están en los bloques
16 y 17 de [COMANDOS.txt](../../COMANDOS.txt). Usan `.venv` y Docker existentes;
no necesitan descargar nada en el laboratorio.

Este ensayo conserva la configuración **LiDAR local**: avance 0.10 m/s, giro hasta
0.40 rad/s, precaución 0.45 m y parada 0.32 m. La rama original usa 0.22 m/s,
0.90 rad/s, 0.55 m y 0.34 m; esos cambios no se incorporan. Por tanto se prueba su
controlador de espera con nuestro perfil de movimiento, no toda la rama tal cual.
No combinar con `--config-senales`: se usa la copia original de 52 cm/seis segundos.

Cambio local solicitado el 19 de noviembre: la evasión frontal y lateral ahora
ordena **avance cero mientras gira**, y vuelve al avance recto cuando ya no requiere
giro. El controlador/configuración originales de señales siguen intactos; este
cambio pertenece a la evasión local con la que se ensayan. Las velocidades angulares,
distancias y protecciones se conservan. No equivale a verificar frenado o huella en
el robot. La evasión gira al llegar a precaución 0.45 m o al corregir un lateral;
una flecha confirmada pide su giro al llegar a <=0.52 m, sujeto a la barrera de 0.32 m.

Nombre + Buscar funcionan como en la prueba combinada: se registra A antes de
avanzar, la evasión funciona aun sin flecha y el rostro objetivo puede detener la
prueba. Para evaluar giros no mostrar su fotografía. No lee meta/QR ni regresa.
`--tramo ida` confirma azules; `--tramo vuelta` permite ensayar rojas sin retorno.

## Primera comprobación, sin movimiento

Ejecutar bloque 16, esperar cámara/sensores y pulsar Buscar. No existe publicador
sin `--mover`. Mostrar una flecha del color elegido: debe confirmarse en tres
imágenes nuevas. Con el robot quieto se puede acercar un panel con la flecha que
también sea visible a la altura del LiDAR; observar distancias en la terminal.

- Frente >0.52 m: `ESPERA_PARED`, dirección y edad `pendiente_s`.
- Frente <=0.52 m y frente/laterales >0.32 m: `GIRANDO`, propuesta vx=0 y
  wz negativa para derecha o positiva para izquierda.
- Sensor/cámara/actividad caducados, cobertura/TF insuficientes o giro bloqueado:
  propuesta cero; revisar también `[BUSQUEDA]` y `permitido`.
- Al permanecer quieto no se completarán los 90°. Tras 12 s de giro propuesto
  puede aparecer `FALLO`; cerrar y relanzar antes del siguiente intento.

## Ensayo físico después de la comprobación

Terminar la ejecución anterior y los otros controles. Colocar la flecha en una
pared con espacio lateral suficiente. Ejecutar bloque 17 con --mover, esperar
vídeo/sensores y pulsar Buscar. Iniciar aproximadamente a 0.8–1.0 m del retorno
frontal para la primera prueba; comprobar si la cámara ya distingue la flecha.
Esto es una colocación inicial de ensayo, no una distancia final validada.

Observar que avanza tras Buscar, retiene la dirección y gira cerca del umbral,
no al verla de lejos. Debe completar 90° por odometría antes de continuar; luego
**vuelve a la evasión y puede seguir avanzando**. Esc/Salir/Ctrl+C detienen y cierran
intentando ceros; Ctrl+Z suspende y no sirve para terminar. La parada depende de
que el nodo siga activo y pueda entregar comandos. Conservar la conexión al robot.

Registrar las líneas `[GIRO ENSAYO]` y `[BUSQUEDA]`: distancia al iniciar giro,
dirección, cuánto tarda, si completa 90° y si se repite. `vx_propuesto` y
`wz_propuesto` son órdenes calculadas, no velocidades medidas. El vídeo puede
llegar retrasado; la edad local no mide la captura. No se guardan imágenes.

Si gira sin flecha pendiente, comprobar primero sin movimiento. El primer registro
físico aportado mantuvo ID consumido 0 y giró por evasión a unos 44 cm; no demuestra
giro guiado por la flecha. Antes de Buscar la ventana indica **VISTA PREVIA**.
Después del inicio se reinicia la cuenta y se necesitan tres imágenes nuevas.
`[SENALES ENSAYO]` muestra confirmaciones, ID y si la imagen es posterior a Buscar,
además del último ID escrito en la tubería. `[GIRO ENSAYO]` muestra recibido y
consumido. Copiar ambos junto al motivo de `[BUSQUEDA]` para distinguir percepción,
inicio, envío y caducidad. No se redujo el umbral a partir de ese registro.

## Límites que se conservan para comparar

El controlador original descarta la dirección tras seis segundos aunque el mismo
cartel siga visible. `SIN_FLECHA_PENDIENTE` tras esa espera puede ser ese fallo,
no un fallo de cámara. Hacer primero esta comprobación **sin --mover**: mantener el
cartel más de siete segundos con frente >0.52 m y después acercar el panel. Con el
mismo ID no volverá a armar el giro; cerrar/reabrir para repetir.

Mientras espera, la evasión conserva el mando. No hay vinculación entre el retorno
LiDAR y la pared de la flecha; un obstáculo diagonal puede disparar o desviar el
recorrido. Con el perfil original de precaución 0.55 m se reprodujo giro contrario
antes de llegar a 0.52 m. El perfil local evita esa superposición frontal simple,
pero no corrige toda la coordinación ni los choques diagonales existentes.
No interpretar un resultado bueno como validación de navegación completa.

La prueba meta/rostro/QR se mantiene pendiente y disponible. Para volver a nuestro
controlador basta cerrar y ejecutar el comando combinado sin
`--ensayo-giro-companero`; no hay que restaurar archivos ni cambiar de rama.
