# Investigaciones y planes no operativos

Estado: índice de documentos de investigación  
Alcance: hipótesis, pruebas, POC y diseños futuros; no son instrucciones de operación del robot

Esta carpeta separa la documentación de análisis y planificación de los
documentos vigentes de funcionamiento. Un documento aquí no modifica por sí
mismo el perfil de ejecución real.

- [Desfase de obstáculos durante giros](desfase-obstaculos-en-giros-plan.md): plan de corrección de deskew, sincronización temporal, EKF y costmaps.
- [IMU BMI088 + ESP32-S3](imu-bmi088-esp32s3-interface.md): especificación para reemplazar la fuente inercial del Pixhawk.
- [Datos de puntos fantasma LiDAR](lidar-puntos-fantasma-datos-proyecto.md): análisis técnico del problema y del estado del proyecto.
- [Plan de corrección de puntos fantasma](plan-correccion-puntos-fantasma.md): experimentos y validación de filtrado de suelo/frenados falsos.
- [Percepción LiDAR 3D V2](lidar-percepcion-v2-plan.md): dirección de arquitectura futura, no plan inmediato.
- [POC de ground segmentation de Autoware](autoware-ground-segmentation-integracion.md): integración experimental en simulación.

El plan operativo vigente de reducción de ruido LiDAR permanece en
[`docs/lidar-noise-reduction-plan.md`](../lidar-noise-reduction-plan.md).
