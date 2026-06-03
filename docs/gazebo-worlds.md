# Mundos Gazebo Para Simulación

Estado: actual
Alcance: procedimiento para lanzar mundos Gazebo distintos al mundo plano por defecto
Fuente de verdad: `tools/launch_sim_global_v2_wifi.sh`, `tools/launch_sim_global_v2_wifi_sonoma.sh`, `sim_global_v2.launch.py`, `sim_global_v2_wifi.launch.py` y `sim_v2_base.launch.py`

## Objetivo
Este documento explica cómo abrir mundos nuevos en la simulación Global V2 sin cambiar el perfil de navegación del robot. La idea es probar percepción, localización y Nav2 en entornos más ricos que `vacio.world`, manteniendo la mayor paridad posible con el robot real.

El perfil recomendado para estas pruebas es:

```bash
./tools/launch_sim_global_v2_wifi.sh
```

Ese wrapper ahora acepta argumentos ROS 2 launch adicionales. Esto permite cambiar el mundo, el nombre del mundo y la pose inicial del robot desde el comando.

## Regla De Paridad
No se debe cambiar el tamaño del rolling window, la resolución del costmap, los parámetros de planner/controller ni los parámetros LiDAR solo para que un mundo sea más cómodo.

Para comparar contra el robot real, mantener:

```yaml
global_costmap:
  rolling_window: true
  width: 300
  height: 300
  resolution: 0.25
```

Con esta ventana, un goal debe estar dentro de aproximadamente 150 m del robot. Para rutas largas en mapas grandes, usar waypoints/tramos cercanos en vez de agrandar el costmap.

## Forma General
Para abrir un mundo local:

```bash
./tools/launch_sim_global_v2_wifi.sh \
  world:=/ros2_ws/src/navegacion_gps/worlds/mi_mundo.world \
  world_name:=mi_mundo \
  spawn_x:=0.0 \
  spawn_y:=0.0 \
  spawn_z:=0.2 \
  spawn_yaw:=0.0
```

Argumentos principales:

- `world`: path del archivo SDF/world dentro del contenedor, o URL de Fuel si Gazebo la soporta.
- `world_name`: nombre real del `<world name="...">` usado por Gazebo.
- `spawn_x`, `spawn_y`, `spawn_z`: posición inicial del robot en coordenadas del mundo Gazebo.
- `spawn_roll`, `spawn_pitch`, `spawn_yaw`: orientación inicial del robot.
- `datum_lat`, `datum_lon`, `datum_yaw_deg`: datum GPS usado por `map`. En mundos con NavSat/GPS, conviene alinearlo con el mundo simulado.

## Sonoma Limpio
Se agregó un mundo local basado en Sonoma Raceway sin el Prius ni controles de la demo original:

```text
src/navegacion_gps/worlds/sonoma_salus.world
```

Comando recomendado:

```bash
./tools/launch_sim_global_v2_wifi_sonoma.sh
```

Ese wrapper llama al launch WiFi normal con:

```bash
world:=/ros2_ws/src/navegacion_gps/worlds/sonoma_salus.world
world_name:=sonoma_salus
spawn_x:=278.08
spawn_y:=-134.22
spawn_z:=3.1
spawn_yaw:=0.97
datum_lat:=38.1606
datum_lon:=-122.4540
datum_yaw_deg:=0.0
```

El datum Sonoma evita que `map -> base_footprint` aparezca con coordenadas enormes por usar el datum real de Córdoba. Esto no cambia Nav2; solo hace que la localización simulada quede en coordenadas razonables para ese mundo.

## Probar Otro Mundo De Fuel
Hay dos formas:

1. Lanzar directamente una URL de mundo Fuel:

```bash
./tools/launch_sim_global_v2_wifi.sh \
  world:=https://fuel.gazebosim.org/1.0/openrobotics/worlds/prius%20on%20sonoma%20raceway \
  world_name:=sonoma
```

2. Crear un `.world` local que incluya modelos de Fuel:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="mi_mundo">
    <plugin filename="ignition-gazebo-physics-system"
            name="ignition::gazebo::systems::Physics"/>
    <plugin filename="ignition-gazebo-sensors-system"
            name="ignition::gazebo::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="ignition-gazebo-user-commands-system"
            name="ignition::gazebo::systems::UserCommands"/>
    <plugin filename="ignition-gazebo-scene-broadcaster-system"
            name="ignition::gazebo::systems::SceneBroadcaster"/>
    <plugin filename="ignition-gazebo-imu-system"
            name="ignition::gazebo::systems::Imu"/>
    <plugin filename="ignition-gazebo-navsat-system"
            name="ignition::gazebo::systems::NavSat"/>

    <include>
      <uri>https://fuel.ignitionrobotics.org/1.0/openrobotics/models/Sonoma Raceway</uri>
    </include>
  </world>
</sdf>
```

La segunda opción es preferible cuando se quiere limpiar una demo de Fuel que trae vehículos, GUI, teleop o cámaras que no se necesitan.

## Errores Frecuentes
### Mundo vacío
Si el mundo abre pero aparece vacío, revisar:

- Que `world_name` coincida con el `<world name="...">`.
- Que el archivo local tenga `<include>` de modelos válidos, no una URL de mundo usada como si fuera modelo.
- Que Gazebo haya podido descargar o encontrar el modelo en cache.

Ejemplo de error observado:

```text
does not contain model.config
```

Esto ocurre cuando se intenta incluir un mundo Fuel dentro de `<include>` como si fuera un modelo.

### El robot no aparece
Revisar si Gazebo publicó el mundo:

```bash
docker exec ros2_salus bash -lc 'ign topic -l | grep "^/world"'
```

Revisar si `ros_gz_sim create` creó la entidad:

```bash
docker exec ros2_salus bash -lc \
  'grep -E "Requested creation|OK creation|Requesting list of world names" /ros2_ws/logs/sim_global_v2_wifi.log | tail -40'
```

Si solo aparece `Requesting list of world names`, Gazebo todavía no publicó el mundo o falló al cargarlo.

### Goals rechazados o rutas abortadas
Revisar logs del planner:

```bash
docker exec ros2_salus bash -lc \
  'grep -E "Goal pose is out of costmap|Goal failed|Begin navigating" /ros2_ws/logs/sim_global_v2_wifi.log | tail -80'
```

Si aparece:

```text
Goal pose is out of costmap!
```

no significa necesariamente que Nav2 no acepte goals. Significa que el goal quedó fuera de la ventana rolling actual. Con el perfil comparable al real, probar un goal más cercano o dividir la ruta en tramos.

## Checklist De Validación
Después de lanzar un mundo nuevo:

```bash
docker exec ros2_salus bash -lc 'ign topic -l | grep "^/world"'
```

```bash
docker exec ros2_salus bash -lc \
  'source /opt/ros/humble/setup.bash; source /ros2_ws/install/setup.bash; ros2 lifecycle get /planner_server; ros2 lifecycle get /controller_server; ros2 lifecycle get /bt_navigator'
```

```bash
docker exec ros2_salus bash -lc \
  'source /opt/ros/humble/setup.bash; source /ros2_ws/install/setup.bash; timeout 5 ros2 run tf2_ros tf2_echo map base_footprint'
```

Valores esperados:

- `/planner_server`, `/controller_server` y `/bt_navigator` en `active`.
- TF `map -> base_footprint` disponible.
- Coordenadas `map` razonables para el mundo elegido.
- Goals cercanos al robot generan plan.

## Cuándo Crear Un Wrapper
Crear un wrapper en `tools/` cuando un mundo requiere siempre los mismos argumentos de:

- `world`
- `world_name`
- pose inicial
- datum

Ejemplo actual:

```text
tools/launch_sim_global_v2_wifi_sonoma.sh
```

Evitar que el wrapper cambie parámetros de Nav2 si el objetivo es comparar contra el robot real.
