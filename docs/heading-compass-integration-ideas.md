# Ideas Para Integrar Brujula En Heading Global

Estado: propuesta
Alcance: ideas de implementación para usar la brújula/magnetómetro MAVROS como ayuda débil de heading en SALUS
Fuente de verdad: observación en robot con MAVROS, arquitectura `real_global_v2` y configuración actual de localización

## Contexto
El robot real vigente usa MAVROS como backend Pixhawk/GNSS. En una prueba
directa sobre `salus`, MAVROS expuso:

- `/imu/data` (`sensor_msgs/msg/Imu`): orientación estimada por el Pixhawk,
  velocidades angulares y aceleración.
- `/mavros_node/compass_hdg` (`std_msgs/msg/Float64`): heading de brújula en
  grados.
- `/mavros_node/mag` (`sensor_msgs/msg/MagneticField`): vector de campo
  magnético.

La navegación global actual ya tiene una fuente de heading por avance GPS:
`/gps/course_heading`. Esa fuente es preferible cuando el robot se mueve con
RTK sano, porque mide el rumbo real de desplazamiento del vehículo y no depende
directamente del entorno magnético.

## Principio de diseño
No usar el magnetómetro crudo (`/mavros_node/mag`) directamente en el EKF.

El vector magnético crudo es sensible a:

- hard iron y soft iron del chasis;
- corriente de motores, actuadores y cableado;
- montaje físico y calibración del compass;
- inclinación del vehículo;
- estructuras metálicas cercanas;
- diferencias de unidad/escala en la publicación de MAVROS.

Si se incorpora una brújula al EKF, conviene usar un yaw ya resuelto y filtrado,
por ejemplo `/mavros_node/compass_hdg`, convertido a un mensaje yaw-only con
covarianza conservadora.

## Uso recomendado
La brújula debería funcionar como ancla débil de heading, no como autoridad
permanente.

Casos donde sí puede aportar:

- **Arranque:** dar una referencia inicial aproximada para evitar que el heading
  global nazca completamente arbitrario.
- **Reposo largo:** ayudar a que `map -> odom` no quede flotando cuando el robot
  permanece detenido durante mucho tiempo y no hay heading GPS válido.

Casos donde no debería dominar:

- en movimiento con `/gps/course_heading` válido;
- durante giros o maniobras con yaw rate apreciable;
- cuando el heading de brújula salta bruscamente;
- cerca de interferencias magnéticas evidentes;
- si la brújula no está calibrada o ArduPilot reporta valores inconsistentes.

## Nodo propuesto
Agregar un nodo intermedio, por ejemplo `compass_heading_gate`, que lea:

- `/mavros_node/compass_hdg`;
- `/controller/drive_telemetry`;
- `/imu/data` para yaw rate;
- `/gps/course_heading/debug` o estado equivalente;
- opcionalmente `/gps/rtk_status_mavros`.

Y publique una salida yaw-only, por ejemplo:

- `/imu/compass_heading` (`sensor_msgs/msg/Imu`)

El mensaje publicado debería llenar solo la orientación yaw. Roll, pitch,
velocidades y aceleraciones no deberían fusionarse desde esta fuente.

## Gating sugerido
Publicar brújula solo cuando se cumpla alguna condición de uso:

- ventana inicial de arranque, por ejemplo los primeros 5 a 15 segundos;
- robot estático durante una ventana larga, por ejemplo más de 30 a 60 segundos;
- velocidad lineal menor a un umbral bajo, por ejemplo `0.05 m/s`;
- yaw rate menor a un umbral bajo;
- heading de brújula fresco;
- salto angular menor a un máximo configurable respecto de la última muestra
  aceptada;
- `/gps/course_heading` no está válido o no tiene autoridad en ese instante.

Si `/gps/course_heading` está válido y el robot se mueve con RTK sano, la salida
de brújula debería dejar de publicarse o publicarse con una covarianza tan alta
que no compita con el heading GPS.

## Covarianza inicial
Usar covarianza alta al principio. Valores razonables para empezar:

- `0.5 rad^2` para una brújula calibrada pero no validada contra trayectoria;
- `1.0 rad^2` si solo se quiere una pista débil de arranque/reposo;
- subir aún más o no publicar si hay saltos o inconsistencia.

La covarianza debería reducirse solo después de validar con bags reales:

- error contra `/gps/course_heading` en tramos rectos;
- estabilidad en reposo largo;
- comportamiento cerca del controlador y actuadores;
- ausencia de saltos de `map -> odom`.

## Integración conceptual con robot_localization
La salida yaw-only podría entrar como una fuente IMU adicional del EKF global:

```yaml
imu_compass: /imu/compass_heading
imu_compass_config: [false, false, false,
                     false, false, true,
                     false, false, false,
                     false, false, false,
                     false, false, false]
imu_compass_differential: false
imu_compass_relative: false
```

La intención es fusionar solo yaw absoluto. El nombre final de la fuente debe
adaptarse al YAML real de `robot_localization` usado en el launch.

## Relación con heading GPS
La jerarquía deseada sería:

1. `/gps/course_heading`: autoridad principal cuando el robot se mueve y el
   gating RTK/cinemático lo considera válido.
2. `/imu/compass_heading`: referencia débil para arranque y reposo largo.
3. `/odometry/local` yaw hold: continuidad local cuando no hay heading absoluto
   confiable.

Esta jerarquía evita que la brújula contamine curvas o trayectoria, pero permite
que el sistema tenga una pista global inicial y una corrección lenta cuando el
robot queda detenido.

## Validación antes de activar en operación
Antes de usarlo en navegación real, grabar bags con:

- `/mavros_node/compass_hdg`;
- `/mavros_node/mag`;
- `/imu/data`;
- `/gps/course_heading`;
- `/gps/course_heading/debug`;
- `/odometry/local`;
- `/odometry/global`;
- `/tf`;
- `/controller/drive_telemetry`.

Revisar:

- diferencia angular brújula vs heading GPS en rectas;
- deriva en reposo;
- saltos del heading de brújula;
- impacto sobre `map -> odom`;
- comportamiento al encender actuadores o mover dirección;
- sensibilidad a posición del vehículo en el entorno.

## Decisión recomendada
Implementar primero como experimento desactivado por defecto.

La opción segura es lanzar el nodo de brújula en modo diagnóstico, publicar
debug y bags, y recién después permitir que alimente el EKF global con una
covarianza alta y ventanas de uso muy restringidas.
