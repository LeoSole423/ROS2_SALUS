# Brujula Gateada Para Heading Inicial

Estado: actual, desactivado por defecto
Alcance: integracion de brujula/magnetometro MAVROS como ayuda debil de
heading inicial y reposo largo en Global V2
Fuente de verdad: `compass_heading_gate.py`, `sim_compass_hdg.py`,
`localization_global_v2.launch.py`, `sim_global_v2.launch.py`,
`real_global_v2.launch.py` y tests de launch de `navegacion_gps`

## Resumen
La brujula se integra como fuente debil y gateada de yaw absoluto. Su objetivo
es ayudar al arranque o a reposos largos, no reemplazar el heading por avance
GPS ni corregir navegacion en movimiento.

Por defecto no cambia el comportamiento de navegacion:

- `enable_compass_heading:=false`
- `enable_compass_heading_fusion:=false`
- `enable_sim_compass:=false` en simulacion

La salida principal es `/imu/compass_heading`, un `sensor_msgs/msg/Imu`
yaw-only con covarianza conservadora. La salida de diagnostico es
`/imu/compass_heading/debug`, un JSON en `std_msgs/msg/String`.

## Fuentes
En robot real, MAVROS expone:

- `/mavros_node/compass_hdg` (`std_msgs/msg/Float64`): heading de brujula en
  grados, tipo compass.
- `/mavros_node/mag` (`sensor_msgs/msg/MagneticField`): campo magnetico crudo.
- `/imu/data` (`sensor_msgs/msg/Imu`): IMU del Pixhawk, usada por el gate para
  yaw-rate.

La implementacion usa `/mavros_node/compass_hdg`. No se fusiona
`/mavros_node/mag` crudo porque es muy sensible a hard iron, soft iron,
cableado, corriente de actuadores, inclinacion, calibracion y estructuras
metalicas cercanas.

## Conversion
MAVROS publica `compass_hdg` como rumbo de brujula en grados:

- norte: `0`
- este: `90`
- sur: `180`
- oeste: `270`

Nav2 y `robot_localization` trabajan con yaw ENU. La conversion usada es:

```text
yaw_enu_deg = normalize(90.0 - compass_hdg_deg)
```

Casos de referencia:

| Compass | Yaw ENU |
| --- | --- |
| `0` norte | `90` |
| `90` este | `0` |
| `180` sur | `-90` |
| `270` oeste | `180` o `-180` |

## Nodo `compass_heading_gate`
Executable:

```bash
ros2 run navegacion_gps compass_heading_gate
```

Entradas por defecto:

- `/mavros_node/compass_hdg`
- `/imu/data`
- `/controller/drive_telemetry`
- `/gps/course_heading/debug`

Salidas por defecto:

- `/imu/compass_heading`
- `/imu/compass_heading/debug`

El nodo publica solo cuando el robot esta quieto y la muestra es confiable
dentro del criterio conservador. Bloquea por:

- heading de brujula viejo;
- IMU vieja;
- velocidad medida mayor al umbral de reposo;
- yaw-rate alto;
- salto angular grande contra el ultimo yaw aceptado;
- `/gps/course_heading/debug` valido, si `block_when_gps_heading_valid` esta
  activo.

La covarianza inicial de yaw es `1.0 rad^2`. Roll, pitch, velocidad angular y
aceleracion lineal se marcan como no usados o con covarianza alta.

## Nodo `sim_compass_hdg`
Executable:

```bash
ros2 run navegacion_gps sim_compass_hdg
```

En simulacion genera un equivalente de `/mavros_node/compass_hdg` a partir de
`/imu/data`.

Entrada por defecto:

- `/imu/data`

Salida por defecto:

- `/sim/compass_hdg`

Conversion:

```text
compass_deg = normalize360(
  90.0 - (yaw_enu_deg + initial_yaw_offset_deg) + bias_deg + noise_deg
)
```

Parametros principales:

- `input_imu_topic`
- `output_topic`
- `publish_hz`
- `noise_stddev_deg`
- `bias_deg`
- `initial_yaw_offset_deg`
- `seed`

En los launches `sim_global_v2*`, `initial_yaw_offset_deg` se calcula desde
`spawn_yaw`. Esto compensa que la IMU simulada publica yaw relativo al arranque
del modelo, no yaw absoluto del mundo Gazebo.

## Flags de launch
`real_global_v2.launch.py` y `real_global_v2_wifi.launch.py`:

| Flag | Default | Uso |
| --- | --- | --- |
| `enable_compass_heading` | `false` | lanza `compass_heading_gate` |
| `compass_hdg_topic` | `/mavros_node/compass_hdg` | entrada real MAVROS |
| `compass_heading_topic` | `/imu/compass_heading` | salida yaw-only |
| `compass_heading_debug_topic` | `/imu/compass_heading/debug` | debug JSON |
| `compass_heading_yaw_variance_rad2` | `1.0` | covarianza yaw |
| `enable_compass_heading_fusion` | `false` | agrega `imu2` al EKF global |

`sim_global_v2.launch.py` y `sim_global_v2_wifi.launch.py` agregan ademas:

| Flag | Default | Uso |
| --- | --- | --- |
| `enable_sim_compass` | `false` | lanza `sim_compass_hdg` |
| `sim_compass_hdg_topic` | `/sim/compass_hdg` | salida compass simulada |
| `sim_compass_noise_stddev_deg` | `0.0` | ruido gaussiano |
| `sim_compass_bias_deg` | `0.0` | sesgo fijo |
| `sim_compass_publish_hz` | `5.0` | frecuencia de publicacion |
| `sim_compass_seed` | `1` | semilla deterministica |

Cuando `enable_sim_compass:=true`, `compass_heading_gate` lee
`sim_compass_hdg_topic`. Cuando esta apagado, lee `compass_hdg_topic`.

## Fusion en EKF global
La fusion esta implementada pero apagada por defecto.

Con `enable_compass_heading_fusion:=true`,
`localization_global_v2.launch.py` agrega una fuente `imu2` al EKF global:

```yaml
imu2: /imu/compass_heading
imu2_config: [false, false, false,
              false, false, true,
              false, false, false,
              false, false, false,
              false, false, false]
imu2_differential: false
imu2_relative: false
imu2_remove_gravitational_acceleration: false
```

Usar este flag solo en pruebas controladas. La primera etapa recomendada es
habilitar el nodo y mirar debug/bags sin fusionarlo.

## Prueba recomendada en simulacion
Para probar con el URDF realista V2:

```bash
./tools/launch_sim_global_v2_wifi_cuatri_real_v2.sh \
  enable_sim_compass:=true \
  enable_compass_heading:=true
```

Verificar:

```bash
ros2 topic echo /sim/compass_hdg
ros2 topic echo /imu/compass_heading
ros2 topic echo /imu/compass_heading/debug
```

En reposo inicial deberia publicar yaw aproximado. Al moverse, o si el heading
GPS queda valido, la brujula deberia quedar bloqueada o subordinada.

## Estado del plan
Implementado y validado en esta etapa:

- rama `feature/compass-heading-gate`;
- `compass_heading_gate` como nodo diagnostico/gateado;
- `sim_compass_hdg` para simulacion;
- entry points en `setup.py`;
- flags en `sim_global_v2*`, `real_global_v2*` y
  `localization_global_v2.launch.py`;
- fusion EKF disponible detras de `enable_compass_heading_fusion:=false`;
- documentacion operativa;
- tests unitarios y contratos de launch;
- prueba manual en sim con `cuatri_real_v2`;
- prueba manual en sim con `spawn_yaw` distinto de cero.

La prueba con `spawn_yaw` mostro que Gazebo crea el modelo rotado, pero la IMU
simulada publica yaw relativo al arranque del modelo. Por eso
`sim_compass_hdg` compensa el yaw inicial declarado mediante
`initial_yaw_offset_deg`. Esta prueba valida el wiring, conversiones, gates,
covarianzas y EKF opcional, pero no reemplaza la validacion del magnetometro
real.

## Pendiente antes de robot real operativo
La implementacion esta lista para probar en el robot en modo diagnostico. Lo
pendiente es validar el sensor real y su entorno magnetico antes de permitirle
influir en navegacion.

1. Confirmar topics reales de MAVROS:

```bash
ros2 topic list | grep -E 'compass|mag|imu'
ros2 topic echo --once /mavros_node/compass_hdg
ros2 topic echo --once /mavros_node/mag
```

Confirmar que `/mavros_node/compass_hdg` existe, publica en grados y responde
con valores razonables para la orientacion fisica del robot.

2. Probar solo diagnostico, sin fusion EKF:

```bash
./tools/launch_real_global_v2_wifi.sh enable_compass_heading:=true
```

No activar todavia:

```bash
enable_compass_heading_fusion:=true
```

3. Verificar debug en reposo:

```bash
ros2 topic echo /imu/compass_heading/debug
ros2 topic echo /imu/compass_heading
```

Esperado:

- `valid: true` en reposo inicial o reposo largo;
- `reason: startup_stationary` o `long_stationary`;
- yaw coherente con la orientacion fisica;
- `valid: false` al moverse, girar fuerte o cuando `/gps/course_heading` sea
  valido.

4. Test fisico con orientaciones conocidas:

| Orientacion fisica | Compass esperado | Yaw ENU esperado |
| --- | --- | --- |
| Norte | `0` | `90` |
| Este | `90` | `0` |
| Sur | `180` | `-90` |
| Oeste | `270` | `180` o `-180` |

Registrar `/mavros_node/compass_hdg` y `/imu/compass_heading/debug` en cada
orientacion. Aceptar solo como referencia aproximada: el magnetometro puede
tener bias local.

5. Grabar bag real minimo:

- `/mavros_node/compass_hdg`
- `/mavros_node/mag`
- `/imu/data`
- `/gps/course_heading`
- `/gps/course_heading/debug`
- `/imu/compass_heading`
- `/imu/compass_heading/debug`
- `/odometry/local`
- `/odometry/global`
- `/tf`
- `/controller/drive_telemetry`

6. Probar interferencia de actuadores:

- robot quieto con controlador y motores energizados;
- direccion quieta;
- direccion moviendose;
- cambios de carga electrica cercanos al Pixhawk/cableado.

Revisar saltos de `/mavros_node/compass_hdg`, cambios bruscos en
`/imu/compass_heading/debug` y cualquier discontinuidad en `map -> odom`.

7. Comparar contra heading GPS en rectas:

- circular en tramos rectos con RTK sano;
- comparar yaw convertido desde `/mavros_node/compass_hdg` contra
  `/gps/course_heading`;
- estimar si hay bias sistematico;
- decidir si hace falta calibracion fisica/Pixhawk o un offset configurable
  para el compass real.

8. Recién despues probar fusion EKF:

```bash
./tools/launch_real_global_v2_wifi.sh \
  enable_compass_heading:=true \
  enable_compass_heading_fusion:=true
```

Primero sin goal Nav2, luego con movimiento corto y controlado. Observar:

- saltos de `map -> odom`;
- yaw de `/odometry/global`;
- razones del debug de brujula;
- interaccion con `/gps/course_heading`;
- comportamiento de Nav2 durante arranque, reposo y primer avance.

## Jerarquia de heading
La jerarquia operativa deseada es:

1. `/gps/course_heading`: autoridad principal cuando el robot avanza y el gate
   RTK/cinematico lo considera valido.
2. `/imu/compass_heading`: referencia debil para arranque y reposo largo.
3. `/odometry/local`: continuidad local cuando no hay heading absoluto
   confiable.

Esta jerarquia evita que la brujula contamine curvas o trayectoria, pero da una
pista inicial cuando el robot todavia no pudo generar heading por avance GPS.

## Criterio de seguridad
- No usar `/mavros_node/mag` crudo en el EKF.
- No activar `enable_compass_heading_fusion` en operacion normal hasta validar
  bags reales.
- No usar la brujula para reemplazar `datum_yaw_deg` ni para auto-setear datum.
- Si hay saltos o interferencia magnetica, dejar solo diagnostico o apagar
  `enable_compass_heading`.

## Herramienta de calibracion para agentes
Para medir el bias entre magnetometro y heading por movimiento RTK/GPS existe
una herramienta pasiva:

```bash
./tools/record_compass_calibration.sh east_run_01 60
```

Ese wrapper guarda el JSON en:

```text
/ros2_ws/artifacts/compass_calibration/<label>_<timestamp>.json
```

Si hace falta pasar parametros extra al recorder, se agregan despues de la
duracion:

```bash
./tools/record_compass_calibration.sh east_run_01 60 \
  --max-abs-steer-deg 4.0 \
  --include-samples
```

Comando ROS equivalente:

```bash
ros2 run navegacion_gps compass_calibration_recorder \
  --duration-s 60 \
  --label east_run_01 \
  --output /ros2_ws/artifacts/compass_calibration/east_run_01.json
```

No publica comandos, no mueve el robot y no lanza navegacion. Solo escucha:

- `/mavros_node/compass_hdg`
- `/mavros_node/mag`
- `/gps/course_heading`
- `/gps/course_heading/debug`
- `/controller/drive_telemetry`
- `/odometry/global`
- `/odometry/local`
- `/imu/data`

Salida:

- JSON completo en stdout, siempre pensado para parseo automatico.
- Si `--output` esta presente, guarda el mismo JSON en archivo.

Ejemplo desde el robot con el launch real ya corriendo:

```bash
./tools/record_compass_calibration.sh east_run_01 60
```

Durante la medicion conviene mover el robot en linea recta y sin curvas
pronunciadas, con RTK sano. La herramienta acepta muestras solo cuando:

- `/gps/course_heading/debug.valid == true`;
- la velocidad supera `--min-speed-mps`;
- la direccion esta dentro de `--max-abs-steer-deg`;
- el yaw-rate esta dentro de `--max-abs-yaw-rate-rps`;
- el compass y el heading GPS estan cercanos en tiempo.

Campos principales del JSON:

- `sample_counts.valid_comparison`: cantidad de muestras usadas para bias.
- `summaries.delta_compass_minus_gps_yaw_deg`: diferencia angular medida.
- `summaries.mag_norm_uT`: intensidad magnetica cruda en microteslas.
- `recommendation.recommended_yaw_bias_deg`: offset sugerido para sumar al yaw
  ENU derivado del compass.
- `recommendation.recommended_compass_hdg_bias_deg`: offset equivalente para
  sumar al heading tipo brujula antes de convertir.
- `recommendation.enough_data`: indica si la medicion alcanzo muestras y
  dispersion suficientes.

Definiciones:

```text
compass_yaw_enu = normalize(90 - compass_hdg)
delta_yaw_compass_minus_gps = compass_yaw_enu - gps_course_yaw
recommended_yaw_bias_deg = -mean(delta_yaw_compass_minus_gps)
recommended_compass_hdg_bias_deg = mean(delta_yaw_compass_minus_gps)
```

Si `invalid_reasons` muestra muchos `speed_low`, falta una recta con mas
movimiento. Si muestra `steer_high` o `yaw_rate_high`, la pasada tuvo curva o
giro. Si `mag_norm_uT` cambia fuerte cerca del robot, priorizar reubicacion o
calibracion fisica del magnetometro antes de fusionar.
