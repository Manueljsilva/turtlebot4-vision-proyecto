# Modelos faciales de OpenCV Zoo

Preparados y verificados el 28 de octubre de 2026 para OpenCV 4.14.0, en CPU.
El notebook utiliza `modelos_rostros.py` para comprobar los archivos y descargar
únicamente los ausentes. Los binarios y las descargas parciales están fuera de Git.

| Modelo | Archivo | Bytes | SHA-256 |
| --- | --- | --- | --- |
| YuNet | face_detection_yunet_2023mar.onnx | 232589 | 8f2383e4dd3cfbb4553ea8718107fc0423210dc964f9f4280604804ed2552fa4 |
| SFace | face_recognition_sface_2021dec.onnx | 38696353 | 0ba9fbfa01b5270c96627c4ef784da859931e02f04419c829e83484087c34e79 |

Los tamaños y hashes se contrastaron con los punteros Git LFS oficiales:
[YuNet 2023mar](https://github.com/opencv/opencv_zoo/blob/main/models/face_detection_yunet/face_detection_yunet_2023mar.onnx)
y [SFace 2021dec](https://github.com/opencv/opencv_zoo/blob/main/models/face_recognition_sface/face_recognition_sface_2021dec.onnx).
La descarga usa los binarios de `media.githubusercontent.com/media/opencv/opencv_zoo/main/models/`.

El modelo YuNet 2026 de entrada dinámica está orientado a OpenCV 5.x; no se
sustituye automáticamente el 2023 compatible con esta instalación 4.x.
Ver [documentación de YuNet](https://github.com/opencv/opencv_zoo/tree/main/models/face_detection_yunet)
y [SFace](https://github.com/opencv/opencv_zoo/tree/main/models/face_recognition_sface).
Según sus directorios de origen, YuNet utiliza licencia MIT y SFace Apache 2.0;
consultar sus archivos LICENSE al redistribuir los modelos.

Con ambos archivos presentes y válidos, el notebook puede ejecutarse sin internet.
No copiar las fotografías del curso a esta carpeta; permanecen en `../rostros/`.
