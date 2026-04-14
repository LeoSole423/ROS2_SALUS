# Checklist de Validacion Antes de Usar `real_global_v2`

## Preparacion
- Confirmar que no haya otra navegacion corriendo en el robot.
- Confirmar que Pixhawk, RS16 y controlador esten conectados.
- Confirmar que el datum elegido corresponda al sitio operativo actual.
- Confirmar que `keepout_mask.yaml` este alineado con ese mismo sitio.
- Mantener datum fijo por sitio operativo. No usar `datum_setter` ni auto-set de datum; esa ruta es LEGACY.
- Antes de arrancar `real_global_v2`, alinear fisicamente el robot al Este o pasar `datum_yaw_deg:=...`.
- Para la primera validacion, preferir ruedas levantadas o un area amplia y controlada.

## Arranque Basico
Lanzar:

```bash
./tools/exec.sh "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && ros2 launch navegacion_gps real_global_v2.launch.py"
```

Si hace falta cambiar el datum del despliegue:

```bash
./tools/exec.sh "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && ros2 launch navegacion_gps real_global_v2.launch.py datum_lat:=<lat> datum_lon:=<lon> datum_yaw_deg:=<yaw_deg>"
```

En la PC local, abrir RViz global con:

```bash
ros2 launch navegacion_gps rviz_real_global_v2.launch.py
```

## Localizacion Global
Verificar TF:

- `map -> odom`
- `odom -> base_footprint`
- `base_footprint -> base_link`

Confirmar publicacion en:

- `/odometry/local`
- `/odometry/gps`
- `/gps/odometry_map`
- `/gps/course_heading`
- `/odometry/global`

Confirmar que:

- `/gps/odometry_map` sale en frame `map` y sigue al sitio operativo esperado
- `map -> odom` aparece y evoluciona sin saltos absurdos
- `fromLL` devuelve puntos coherentes con el mapa operativo
- `/gps/course_heading` aparece solo cuando el avance GPS es valido y ayuda a cerrar el yaw global
- un goal LL cae en la zona correcta de `map`
- el `global_costmap` queda en frame `map` con ventana rolling de `300 x 300 m`, centrada sobre el robot y sin depender de `origin_x/y` fijos

## Cadena de Navegacion
Verificar flujo:

- `planner_server -> controller_server -> /cmd_vel -> /cmd_vel_safe -> /cmd_vel_final`

Confirmar que:

- `nav_command_server` opera en `map`
- la web usa `map` como frame global
- `collision_monitor` y `stop_zone` siguen activos
- `vehicle_controller_server` recibe comandos coherentes con `/cmd_vel_final`

## RViz y Keepout
Verificar en RViz:

- `Fixed Frame = map`
- visualizacion de `/odometry/global`
- `Global Costmap`
- `Local Costmap`

Si el keepout no coincide con el sitio:

- relanzar con el datum correcto y validar de nuevo
- en el perfil real vigente, `use_keepout` arranca en `False` por default
- si hace falta reactivarlo para pruebas controladas, lanzar con `use_keepout:=True`
