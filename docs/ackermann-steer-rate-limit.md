# Limitador de Rate de Direccion (Ackermann)

Estado: actual, desactivado por defecto
Alcance: suavizado del comando de direccion en `controller_server` para reducir
la oscilacion/zigzag del vehiculo Ackermann en navegacion real
Fuente de verdad: `controller_server/control_logic.py`,
`controller_server/controller_server_node.py`,
`controller_server/test/test_control_logic.py`,
`navegacion_gps/launch/sim_global_v2.launch.py`

## Resumen
El comando de direccion que sale del controlador (Nav2 -> `/cmd_vel_final` ->
Ackermann) se convierte hoy a angulo de direccion de forma instantanea: el
`applied_steer_rad` sigue al instante lo que pide el controlador. Si ese comando
oscila (por heading ruidoso con un solo GPS o por lookahead corto en RPP), la
direccion lo persigue al instante, pero el actuador real tiene inercia y
dead-time, satura y sobrepasa -> zigzag.

Este mecanismo agrega un **limitador de rate (slew-rate) + low-pass opcional**
sobre el comando de direccion, modelando el actuador real para que las
correcciones de alta frecuencia no se traduzcan en reversiones instantaneas de
direccion. No cambia el seguimiento de path de baja frecuencia.

Por defecto no cambia el comportamiento: arranca **desactivado**
(`steer_rate_limit_rad_s = 0.0`, `steer_lpf_alpha = 1.0` = pass-through).

## Motivacion
- Causa de oscilacion analizada: lazo entre heading ruidoso (GPS unico, course
  diverge a baja velocidad) y un control que convierte cualquier error angular
  en curvatura. Ver [docs/nav-benchmarks.md](/home/leo/codigo/ROS2_SALUS/docs/nav-benchmarks.md)
  y [docs/compass-heading-gate.md](/home/leo/codigo/ROS2_SALUS/docs/compass-heading-gate.md).
- El servo de direccion fisico no puede saltar instantaneamente. Si Nav2 cree
  que la direccion ya esta donde la pidio pero el actuador todavia esta llegando,
  aparece sobrepaso y serpenteo (dead-time del actuador).
- El limitador acota cuanto puede moverse la direccion por tick, alineando el
  comando con lo que el hardware puede ejecutar.

## Como funciona
La logica vive en una funcion pura testeable, `rate_limit_steer_pct`, y se
aplica en el lazo de control (`_control_tick`, frecuencia fija `control_hz`):

```text
# slew-rate: limitar el cambio por tick
step       = clamp(target - previous, -max_step_pct, +max_step_pct)
limited    = previous + step
# low-pass opcional sobre el valor ya limitado
out        = alpha * limited + (1 - alpha) * previous
```

- Opera sobre `steer_pct` (entero -100..100 que se manda al actuador). Como
  `steer_pct` es un mapeo lineal del angulo (`steer_pct = steer_rad / steering_limit_rad * 100`),
  un limite en `rad/s` se traduce linealmente a `%/tick`.
- Conversion de la tasa fisica a paso por tick:

```text
max_step_pct = (steer_rate_limit_rad_s / steering_limit_rad) * 100 / control_hz
```

- Se aplica **solo al comando auto fresco**. En `estop` o timeout del
  controlador, el comando snapea (la seguridad sigue siendo inmediata) y el
  estado interno se resetea al steer realmente enviado, para que el
  re-enganche no arrastre una rampa vieja.
- No muta el comando auto guardado: se publica una copia con el `steer_pct`
  limitado, asi la telemetria `requested_auto_command` sigue mostrando el
  pedido original y el estado del limitador no se corrompe entre ticks.

## Parametros del nodo
Declarados en `controller_server_node.py`:

| Parametro | Default | Uso |
| --- | --- | --- |
| `steer_rate_limit_rad_s` | `0.0` | Tasa maxima de cambio de la direccion en rad/s. `<= 0` desactiva el slew-rate. |
| `steer_lpf_alpha` | `1.0` | Factor del low-pass en `(0, 1]`. `1.0` desactiva el filtro; mas chico = mas suave. |

El limitador queda activo si `steer_rate_limit_rad_s > 0` o `steer_lpf_alpha < 1`.

## Flags de launch
`sim_global_v2.launch.py` ya expone los dos parametros como launch args:

| Flag | Default | Uso |
| --- | --- | --- |
| `steer_rate_limit_rad_s` | `0.0` | pasa al `vehicle_controller_server` |
| `steer_lpf_alpha` | `1.0` | pasa al `vehicle_controller_server` |

Pendiente: cablear los mismos args en `real_global_v2.launch.py` /
`real_global_v2_wifi.launch.py` antes de usarlo en el robot (ver seccion
Pendiente).

## Como activarlo
En simulacion:

```bash
./tools/launch_sim_global_v2.sh   # editar o lanzar directo con el arg
# o lanzar directo:
ros2 launch navegacion_gps sim_global_v2.launch.py \
  gps_profile:=f9p_rtk steer_rate_limit_rad_s:=0.7
```

Verificar que quedo activo:

```bash
ros2 param get /vehicle_controller_server steer_rate_limit_rad_s
```

Referencia inicial sugerida: `steer_rate_limit_rad_s:=0.7` (~40 deg/s), valor
recomendado para servos de direccion tipicos. Ajustar al rate real del actuador
si se conoce.

## Resultados A/B en simulacion
Medido con `./tools/run_nav_benchmark.sh heading_core` (perfil base, n=1 por
escenario), baseline (desactivado) vs `steer_rate_limit_rad_s:=0.7`:

| Escenario | metrica | baseline | rate 0.7 |
| --- | --- | --- | --- |
| recta 6m | salto map->odom max | 1.46 deg | 0.70 deg |
| recta 12m | salto map->odom max | 0.88 deg | 0.96 deg |
| giro izq | valid_ratio | 0.42 | 0.47 |
| giro izq | goal_err | 13.5 m | 13.2 m |
| agregado | salto map->odom medio | 1.85 deg | 1.76 deg |
| agregado | goal_err medio | 9.79 m | 9.72 m |

Lectura honesta:
- Efecto **neto marginalmente positivo pero dentro del ruido** (n=1 por
  escenario). El agregado baja un poco; por escenario el resultado es mixto.
- El modelo de heading de la sim es demasiado limpio (baseline con 0 saltos
  sobre el umbral de 12 deg), por lo que hay poca oscilacion de alta frecuencia
  que suprimir. El valor real del limitador es en el robot fisico, con heading
  ruidoso.
- El `goal_err` de ~13 m persiste igual en ambos: es artefacto del frame global
  (map->odom corrido), no lo toca el limitador de direccion.

## Pendiente antes de robot real operativo
1. Cablear `steer_rate_limit_rad_s` y `steer_lpf_alpha` como launch args en
   `real_global_v2.launch.py` y `real_global_v2_wifi.launch.py`, pasandolos al
   `controller_server`.
2. Confirmar el rate real del actuador de direccion del `salus` y usarlo como
   default fisico en vez del valor de referencia.
3. Validar en el robot en recta y curva suave con RTK sano, comparando la
   oscilacion de `/cmd_vel_final.angular.z` y el `steer_deg_measured` con y sin
   limitador.
4. Mantener la paridad sim/real del valor elegido (ver
   [docs/sim-real-parity.md](/home/leo/codigo/ROS2_SALUS/docs/sim-real-parity.md)).

## Relacion con otros mecanismos
- Es independiente del guard de baja velocidad de `ackermann` (`min_effective`
  contra el blowup de `kappa = omega/v`) y del gating de heading
  (`gps_course_heading`, `compass_heading_gate`,
  `global_yaw_stationary_hold`). Ataca la oscilacion desde el actuador, no desde
  la estimacion.
- Complementa el tuning de RPP (lookahead, `use_fixed_curvature_lookahead`,
  `max_angular_accel`); no lo reemplaza.

## Tests
`controller_server/test/test_control_logic.py` cubre la funcion pura:
- pass-through cuando esta desactivado;
- clamp del paso por tick en ambas direcciones;
- no-overshoot cuando el gap es menor al paso;
- convergencia al target en N ticks;
- blend del low-pass.

Correr:

```bash
./tools/compile-ros.sh controller_server
# dentro del contenedor:
python3 -m pytest src/controller_server/test/test_control_logic.py -q
```
