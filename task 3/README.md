# Task 3: misión A–B–A con autonomía y gestos

Integra el recorrido de Task 1 y las reglas de manos de Task 2. El botón **Cambiar a gestos / Cambiar a autónomo** permite alternar durante el mismo intento, conservando la persona objetivo, las poses A/B y el QR guardado. Empieza en autónomo; se puede cambiar antes de pulsar Buscar.

Desde la raíz del proyecto, con la sesión del robot y Docker ya preparados:

```bash
bash "task 3/ejecutar_hibrido_robot.sh" \
  --ip 10.87.179.100 --dominio 5 --mover
```

IP/dominio son ejemplos de la última sesión: usar los asignados al robot. `--camara 0` selecciona la webcam local. `--nombre "Nombre existente"` precarga el campo, pero hay que pulsar Buscar. `--topico-camara` conserva por defecto `/oakd/rgb/preview/image_raw/compressed`. Sin `--mover`, la interfaz y los sensores funcionan sin crear publicador de velocidad. También sigue disponible el comando de la copia anterior: `bash "task 3/ejecutar_autonomo_robot.sh" --ip ... --dominio ... --mision-guiada --mover`.

1. Colocar en A mirando hacia el circuito. Escribir el nombre y pulsar **Buscar**: registra A con odometría posterior al inicio.
2. En autónomo, sigue flechas azules y evita obstáculos. En gestos, índice arriba avanza; señalar a tu derecha/izquierda gira el robot hacia ese lado sobre el sitio; palma, ausencia, ambigüedad o varias manos detienen. Webcam sin espejo, confirmación de mano 0.4 s y caducidad 0.5 s.
3. Al confirmar bandera B cercana (frontal ≤0.40 m y guardas de separación/ausencia), frena y confirma reposo. En autónomo barre buscando la persona; en gestos se gira con las manos, sin avance en esta fase. Reconocer la persona objetivo detiene para leer únicamente su QR asociado. Otra identidad no finaliza la búsqueda.
4. Tras guardar el texto, intenta orientar la salida respecto al rumbo de entrada en B +180°. En gestos hay que ordenar el giro hasta alcanzar ese rumbo; se admiten ambos sentidos y solo giro. En autónomo lo hace el controlador. La revisión91d3b55 permite pasar a VUELTA por LiDAR/ruta ante ciertos cortes o problemas de orientación, sin completar necesariamente 180°.
5. Vuelta autónoma por rojo/LiDAR con apoyo odométrico, o vuelta dirigida con gestos. La bandera de A debe verse de frente al regresar; exige cercanía odométrica a A y nueva separación/ausencia respecto a B. Solo tras confirmar llegada y reposo muestra la palabra en la interfaz.
6. **Reiniciar** detiene, espera reposo y permite elegir otra persona/registrar A otra vez sin cerrar el programa. Vuelve a autónomo; no recarga archivos de código. Cerrar/relanzar tras actualizar código.

Cambiar de modo siempre ordena cero y exige ≥3 poses nuevas durante 0.7 s de reposo; después espera una muestra gestual nueva. Se descartan flechas y maniobras pendientes del modo anterior. El cambio no reinicia la misión, el nombre, A/B ni la lectura QR. Un FALLO sigue detenido hasta Reiniciar.

Un único nodo ROS publica velocidades; la webcam solo envía muestras al anfitrión. Task 3 adquiere los bloqueos de Task 1 y Task 2 para impedir sus controles simultáneos dentro del mismo contenedor. Cerrar otros controles, incluidos los de otro equipo/contenedor que esos bloqueos no cubren.

Conserva el perfil de Task 1: avance 0.10 m/s, giro de navegación 0.40 rad/s y aproximación ≤0.05 m/s. Giros manuales en B limitados a 0.25 rad/s. No adopta las velocidades máximas de Task 2. LiDAR/TF, contorno y odometría válidos siguen siendo necesarios. Sin vídeo en ida/vuelta permite pasillo con LiDAR; ante muro≤0.42m espera una imagen posterior durante hasta 2 s y después permite continuar por LiDAR si no llega. Puede pasar por alto una flecha/meta sin vídeo; rearma al recuperar vídeo o despejar frontal>0.47m. En B, vídeo caducado detiene. Caída de la webcam detiene gestos y permite cambiar a autónomo explícitamente. El vigilante no garantiza entregar cero ante muerte del nodo o corte de red.

Usa la `.venv` global, el modelo local de manos en `task 2/modelos/hand_landmarker.task` y **la galería/criterio/modelos canónicos de `task 1`**. No entrena ni descarga al iniciar. Mantener datos, fotos, modelos, sesiones y el notebook copiado fuera de Git. No guardar imágenes de ninguna cámara. Task 1 y Task 2 permanecen independientes e intactos.

**Galería ampliada el 26/11:** Task 1 y Task 3 disponen de 18 personas (99 fotos), con cinco personas nuevas. La misión híbrida sigue leyendo la galería canónica de Task 1; la copia local de Task 3 también quedó sincronizada. Cerrar y volver a ejecutar para cargar los nuevos nombres: Reiniciar no actualiza los descriptores en caliente. No hace falta reiniciar el robot/Docker ni entrenar pesos. Evaluación local reservada17/18, con un rechazo y una falsa aceptación entre 18 negativos simulados; no garantiza reconocimiento físico. Ver [detalle de la galería](<../task 1/README.md>). Fotos y artefactos biométricos siguen fuera de Git.

Integración completa de 91d3b55 autorizada el 26/11: tras escape permite paso de emergencia a≤0.05m/s con semiancho0.185m y ajusta el perfil trasero/alineación; el paso normal0.20m y giro circular0.24m permanecen. Cortes coherentes>2s durante giro manual/automático recuperan con reposo; saltos/marcos/tiempo inválidos siguen fallando. No supone que 18.5cm ni el retorno estén físicamente validados.

**367 pruebas locales Task 3 pasan** con sensores, manos y ROS simulados; falta comprobar esta integración en el robot real. Seguir una flecha o la ruta odométrica no garantiza resolver cualquier circuito ni equivale a SLAM.
