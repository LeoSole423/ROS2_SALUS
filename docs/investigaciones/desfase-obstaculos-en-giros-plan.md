# Investigación: desfase de obstáculos durante giros

Estado: propuesta de corrección; no aplicada  
Alcance: localización, timestamps, LiDAR y costmaps del perfil `real_global_v2`  
Fuente de verdad: configuración y código del checkout actual; la validación final requiere datos del robot  
Última revisión: 2026-08-07

## Objetivo

Eliminar o reducir el desplazamiento y las "colas" de obstáculos estáticos que
se ven en RViz/costmap cuando el vehículo gira. El objetivo no es cambiar la
localización global por GPS/RTK: el problema afecta principalmente a la pose
local de baja latencia y a cómo se proyectan las mediciones del RS16.

## Diagnóstico resumido

La causa más probable es una combinación de cuatro efectos:

1. **Nube sin deskew.** El RS16 genera una nube a lo largo de una revolución;
   el pipeline la convierte en un único `LaserScan` sin compensar el movimiento
   del vehículo entre puntos. Aunque la nube del driver conserva timestamps por
   punto, `pointcloud_to_laserscan` publica `time_increment = 0` y usa un solo
   timestamp para el scan.
2. **Observaciones retenidas.** El costmap local conserva observaciones por
   0,6 s y el global por 1,0 s. Una marca vieja y una marca nueva, proyectadas
   con yaws distintos, quedan visibles simultáneamente.
3. **Odómetro con tiempo de publicación, no de adquisición.**
   `DriveTelemetry` se estampa al ser publicado a 10 Hz. La velocidad y el
   ángulo de dirección pueden corresponder a una medición anterior, pero el EKF
   la recibe como si fuera actual.
4. **Información correlacionada repetida.** El EKF local fusiona pose, yaw,
   velocidades y yaw-rate derivados del mismo modelo Ackermann. El EKF global
   vuelve a fusionar la IMU mediante `vyaw` de la salida local y mediante la
   IMU directa.

El punto 1 es el candidato principal para un error que crece con la velocidad
angular y con la distancia al obstáculo. A 0,4 rad/s y 0,1 s de scan, el giro
durante una revolución es 0,04 rad (2,3°): a 15 m equivale a unos 0,6 m de
desplazamiento transversal en el peor caso.

## Estado actual relevante

| Componente | Estado actual | Consecuencia durante un giro |
|---|---|---|
| EKF local | Fusiona `x,y,yaw,vx,vy,vyaw` de `/wheel/odometry` y `vyaw` de `/imu/data` | Sobrerrepresenta el modelo de ruedas; una mala dirección o latencia afecta varias variables. |
| EKF global | Fusiona `vx,vy,vyaw` de `/odometry/local_global`, GPS X/Y e IMU `vyaw` | La IMU queda correlacionada con una señal ya filtrada localmente. |
| Telemetría de ruedas | Publicada a 10 Hz con `stamp=now()` | Oculta la edad real del encoder/dirección. |
| RS16 → scan | `scan_time: 0.1`, sin deskew por punto | Deforma obstáculos estáticos durante movimiento. |
| Costmap local | Frame `odom`, persistencia 0,6 s | Genera una cola visible corta. |
| Costmap global | Frame `map`, persistencia 1,0 s | Genera cola mayor y además puede mostrar correcciones `map -> odom`. |

Referencias de implementación:

- [EKF local](../../src/navegacion_gps/config/localization_v2.yaml)
- [EKF global](../../src/navegacion_gps/config/localization_global_v2.yaml)
- [Odometría Ackermann](../../src/navegacion_gps/navegacion_gps/ackermann_odometry.py)
- [Publicación de DriveTelemetry](../../src/controller_server/controller_server/controller_server_node.py)
- [Parámetros LiDAR a LaserScan](../../src/navegacion_gps/config/pointcloud_to_laserscan_real.yaml)
- [Parámetros de costmaps](../../src/navegacion_gps/config/nav2_global_v2_real_rolling_params.yaml)

## Orden de implementación

### Fase 0 — Medir antes de modificar

Grabar una curva sostenida hacia ambos lados, con obstáculos estáticos a 3, 8 y
15 m. Registrar como mínimo:

```bash
ros2 bag record \
  /scan_3d /scan /scan_clean \
  /imu/data /wheel/odometry /odometry/local /odometry/global \
  /tf /tf_static /diagnostics
```

Comprobar tres vistas en RViz:

1. Fixed Frame `odom`: separa la calidad de la localización local de las
   correcciones globales.
2. Fixed Frame `map`: muestra el efecto completo de `map -> odom`.
3. Nube `/scan_3d` frente a `/scan_clean`: permite detectar en qué tramo se
   introduce la deformación.

Criterios de diagnóstico:

- El error crece con distancia y yaw-rate: deskew/latencia temporal.
- El error es un ángulo fijo en recta y curva: extrínseca LiDAR–base incorrecta.
- La cola desaparece tras 0,6 s o 1 s: persistencia del costmap.
- Solo se observa en `map`: corrección global o visualización, no el EKF local.

### Fase 1 — Preservar el tiempo de adquisición

Prioridad alta. La PCB BMI088 + ESP32-S3 y el controlador de ruedas deben
publicar el instante de adquisición, no el instante de envío.

- La IMU debe usar un contador monotónico local y una conversión estable al
  reloj de la Jetson/Raspberry. Publicar `header.stamp` de la muestra física.
- Enviar junto al dato de dirección/velocidad un timestamp del microcontrolador
  o del encoder. `DriveTelemetry.stamp` debe representar esa medición.
- Medir y documentar el desfase IMU–LiDAR–ruedas, incluidos buffers UART,
  Ethernet y ROS 2.
- Mantener las mediciones ordenadas temporalmente; no corregir latencia con
  `transform_time_offset` sin antes medirla.

Resultado esperado: a igual giro, las nubes dejan de presentar un desplazamiento
global por usar una pose retrasada.

### Fase 2 — Implementar deskew del RS16

Prioridad máxima. Debe realizarse antes de convertir la nube a `LaserScan`.

Diseño propuesto:

```text
/scan_3d con timestamp por punto
  + trayectoria base_footprint(t) de IMU + odometría local
  -> deskew a un tiempo de referencia de la nube
  -> filtro de suelo
  -> pointcloud_to_laserscan
  -> /scan_clean
```

Requisitos:

- Usar el timestamp individual que ya entrega el driver RS16; no solo el
  `header.stamp` de la nube.
- Interpolar orientación/yaw entre muestras del EKF local o integrar el gyro
  de alta frecuencia. Para este vehículo 2D, compensar yaw y traslación XY es
  el mínimo necesario.
- Elegir y documentar una referencia: primer punto, punto medio o último
  punto de la revolución. La TF y el `header.stamp` de salida deben coincidir
  con esa elección.
- Conservar el frame de la nube y validar que no se aplica dos veces la
  extrínseca `lidar_link -> base_footprint`.
- Medir tiempo de cómputo y evitar introducir más latencia que la que se
  elimina.

No conviene intentar resolver este problema ajustando solo el EKF: un scan con
`time_increment = 0` no puede representar el movimiento interno de una
revolución.

### Fase 3 — Corregir la selección de mediciones del EKF

Prioridad alta, después de disponer de timestamps confiables.

#### EKF local

Conservar:

- `vx` de ruedas;
- `vy=0` como restricción no holonómica;
- `angular_velocity.z` de la IMU.

Reevaluar la fusión simultánea de `x`, `y`, `yaw` y `vyaw` provenientes de
`/wheel/odometry`: todos derivan de la misma velocidad y dirección. La opción
base a validar es integrar en el EKF desde velocidades, sin volver a medir como
absoluta la pose ya integrada por el nodo Ackermann.

La decisión final debe basarse en bags: comparar innovación, covarianza y error
de obstáculos en curvas, no solo la apariencia de la trayectoria.

#### EKF global

No fusionar dos veces la misma información inercial. Si `vyaw` de
`/odometry/local_global` procede de un EKF que ya usa `/imu/data`, el global
debe elegir una sola ruta para esa información, o modelar explícitamente su
correlación (algo que `robot_localization` no hace).

El futuro heading RTK de doble antena debe entrar como medición absoluta en la
capa global, con gating y covarianza reales. No debe reemplazar el yaw-rate
local de baja latencia necesario para deskew y costmap.

### Fase 4 — Ajustar covarianzas y dinámica

Prioridad media. Ajustar solo con datos grabados:

- Calcular bias y desviación estándar de `gyro_z` con el vehículo inmóvil.
- Comparar `yaw_rate` Ackermann contra gyro en curvas de radio conocido.
- Incrementar la incertidumbre de ruedas cuando haya dirección grande,
  aceleración/frenada o terreno deslizante.
- Verificar que las covarianzas publicadas en `Odometry` e `Imu` sean
  varianzas físicas, no constantes elegidas a ojo.
- Comprobar si la respuesta del yaw del EKF es lenta; recién entonces evaluar
  ruido de proceso de `vyaw`, frecuencia del filtro o uso de control.

No fusionar aceleración lineal del BMI088 como respuesta automática a este
problema. Solo evaluarla si se cuenta con calibración de bias, orientación y
una necesidad demostrada de mejorar la predicción longitudinal.

### Fase 5 — Ajustar visualización y persistencia del costmap

Prioridad media-baja; mitiga la visualización pero no reemplaza deskew.

- Durante diagnóstico local, usar RViz con Fixed Frame `odom`.
- Reducir temporalmente `observation_persistence` para medir cuánto de la cola
  proviene del buffer. No elegir el valor final hasta confirmar que el clearing
  conserva robustez ante pérdida breve de scans.
- Mantener `transform_tolerance` como margen de disponibilidad de TF, no como
  compensación de latencia.
- Revisar la extrínseca LiDAR–base con un objeto recto y estático, a ambos lados
  del vehículo; un error de yaw/pitch o traslación produce arcos aparentes al
  girar.

## Criterios de aceptación

La corrección se considera válida si, para la misma secuencia de curvas y los
mismos obstáculos:

| Métrica | Criterio propuesto |
|---|---|
| Error lateral de obstáculo estático a 10 m durante giro | p95 menor que 0,15 m en `odom` |
| Cola posterior al giro | menor que 2 actualizaciones de scan, salvo oclusión real |
| Diferencia de posición entre `/scan_3d` deskewed y `/scan_clean` | sin sesgo angular sistemático |
| Innovaciones/rechazos del EKF | sin oscilación ni rechazos repetidos en curvas normales |
| Latencia nube recibida → scan usable | medida y documentada; no aumentar respecto del baseline sin deskew |
| Navegación | sin falsos frenados ni pérdida de obstáculos reales en el escenario de prueba |

Los umbrales son metas iniciales; deben ajustarse tras obtener la primera bolsa
del robot y conocer la precisión real de los obstáculos de referencia.

## Fuera de alcance

- Integración del GNSS/RTK de doble antena a la Jetson/Raspberry.
- Cambio de hardware del Pixhawk a BMI088 + ESP32-S3, salvo los requisitos de
  timestamp y gyro que esta investigación impone.
- Reescritura completa del stack de localización.

## Referencias externas

- [Configuración de sensores de robot_localization](https://github.com/cra-ros-pkg/robot_localization/blob/rolling-devel/doc/configuring_robot_localization.rst): advierte no fusionar pose y velocidad derivadas de la misma odometría.
- [Código de pointcloud_to_laserscan en Humble](https://github.com/ros-perception/pointcloud_to_laserscan/blob/humble/src/pointcloud_to_laserscan_node.cpp): publica `time_increment = 0`.
- [Parámetros de obstacle layer de Nav2](https://docs.nav2.org/configuration/packages/costmap-plugins/obstacle.html): define `observation_persistence` como retención de mensajes en el buffer.
