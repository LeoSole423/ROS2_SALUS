# ROS2_SALUS

Estado: actual
Alcance: visión general del monorepo y puntos de entrada
Fuente de verdad: código bajo `src/`, launches y scripts bajo `tools/`

`ROS2_SALUS` es el workspace ROS 2 Humble del robot Salus. El repositorio agrupa navegación, sensores, control de actuadores, herramientas web y dependencias vendorizadas del LiDAR RoboSense.

## Paquetes del workspace
- Propios:
  - `interfaces`
  - `controller_server`
  - `map_tools`
  - `navegacion_gps`
  - `navegacion_gps_bt`
  - `sensores`
  - `vision_pipeline`
- Vendorizados:
  - `rslidar_msg`
  - `rslidar_sdk`

Todos los paquetes bajo `src/` viven dentro de este mismo repositorio git.

## Documentación
- Referencia integral del código actual: [docs/CODEBASE_REFERENCE.md](docs/CODEBASE_REFERENCE.md)
- Catálogo archivo por archivo del código propio: [docs/CODE_CATALOG_OWN.md](docs/CODE_CATALOG_OWN.md)
- Runtime, Docker y herramientas: [docs/CODE_CATALOG_RUNTIME_TOOLS.md](docs/CODE_CATALOG_RUNTIME_TOOLS.md)
- Código vendorizado RoboSense: [docs/CODE_CATALOG_VENDOR.md](docs/CODE_CATALOG_VENDOR.md)
- Catálogo archivo por archivo de Cockpit: [cockpit/CODE_CATALOG.md](cockpit/CODE_CATALOG.md)
- Índice general: [docs/INDEX.md](/home/leo/codigo/ROS2_SALUS/docs/INDEX.md)
- Matriz de launches y perfiles: [docs/launch-matrix.md](/home/leo/codigo/ROS2_SALUS/docs/launch-matrix.md)
- Politica de paridad sim/real: [docs/sim-real-parity.md](/home/leo/codigo/ROS2_SALUS/docs/sim-real-parity.md)
- Arquitectura runtime y flujo de tópicos: [docs/runtime-architecture.md](/home/leo/codigo/ROS2_SALUS/docs/runtime-architecture.md)
- Brujula gateada para heading inicial/reposo: [docs/compass-heading-gate.md](/home/leo/codigo/ROS2_SALUS/docs/compass-heading-gate.md)
- Integración con `cockpit`: [docs/cockpit-integration.md](/home/leo/codigo/ROS2_SALUS/docs/cockpit-integration.md)
- Cámara WebRTC + PTZ: [docs/camera-webrtc-ptz.md](/home/leosole/Desktop/AEye/ROS2_SALUS/docs/camera-webrtc-ptz.md)
- Históricos, transiciones y third-party: [docs/archive/README.md](/home/leo/codigo/ROS2_SALUS/docs/archive/README.md)

## Launches operativos
- Navegacion vigente:
  - `ros2 launch navegacion_gps sim_global_v2.launch.py`
  - `ros2 launch navegacion_gps sim_global_v2_wifi.launch.py`
  - `ros2 launch navegacion_gps real_global_v2.launch.py`
  - `ros2 launch navegacion_gps real_global_v2_wifi.launch.py` (recomendado para WiFi)
- Infraestructura:
  - `ros2 launch sensores mavros.launch.py`
  - `ros2 launch sensores rs16.launch.py`
  - `ros2 launch map_tools no_go_editor.launch.py`
  - `ros2 launch controller_server controller_server.launch.py`

`pixhawk_driver` y `ros2 launch sensores pixhawk.launch.py` quedan como codigo
legacy/de referencia. La navegacion real vigente (`real_global_v2` y
`real_global_v2_wifi`) usa MAVROS para la telemetria Pixhawk/GNSS.

Los perfiles `simulacion.launch.py`, `real.launch.py`, `sim_local_v2.launch.py` y `real_local_v2.launch.py` quedan como LEGACY o referencia tecnica. No son la navegacion operativa actual.

## Arquitectura operativa
- Nav2 publica `/cmd_vel`.
- `nav2_collision_monitor` publica `/cmd_vel_safe`.
- `nav_command_server` arbitra `/cmd_vel_safe` y control manual web en `/cmd_vel_teleop`.
- `nav_command_server` publica `/cmd_vel_final` (`interfaces/msg/CmdVelFinal`).
- `controller_server` consume `/cmd_vel_final`.
- El RS16 publica `/scan_3d` y `pointcloud_to_laserscan` publica `/scan`.
- Localización:
  - entradas: `/imu/data`, `/global_position/raw/fix`, `/controller/drive_telemetry` y compatibilidad `/gps/fix` según perfil
  - salidas: `/odometry/local`, `/odometry/gps`, `/odometry/global` según perfil
  - TF esperada: `map -> odom -> base_footprint`
- Heading auxiliar:
  - `/gps/course_heading` es la referencia principal cuando el robot avanza con gate valido.
  - `/imu/compass_heading` puede aportar brujula gateada en arranque/reposo, apagada por defecto.

## Docker y Portabilidad

### Portabilidad del contenedor

Los archivos de compose quedaron ajustados para que el workspace se monte desde el checkout actual del repo, sin depender de rutas absolutas de una maquina particular.

Esto implica que:

- `./src`, `./tools`, `./build`, `./install` y `./log` se resuelven relativo a la raiz de este repo
- `GZ_SIM_RESOURCE_PATH` e `IGN_GAZEBO_RESOURCE_PATH` apuntan a `/ros2_ws/install` y `/ros2_ws/src`, que son las rutas reales dentro del contenedor
- si una maquina o robot necesita mounts host especificos fuera de este repo, lo correcto es usar un archivo override propio y no editar el compose compartido

### Jetson ARM64 y Gazebo

El `Dockerfile` ahora usa `INSTALL_GAZEBO_SIM=auto`:

- en `amd64/x86_64`, instala Gazebo y `ros_gz` como parte del flujo de simulacion
- en `arm64`, los omite por defecto para evitar conflictos de dependencias observados en la Jetson Orin

Con eso buscamos mantener paridad razonable para simulacion en PC sin bloquear el build real en la Jetson.

## Flujo Docker recomendado
1. Levantar el contenedor:
```bash
docker compose up -d --build
```
   Este workspace fija `CycloneDDS` por default (`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`) para evitar timeouts de servicios y acciones observados con Fast DDS en la stack de navegación.
2. Compilar el workspace o paquetes puntuales:
```bash
./tools/compile-ros.sh
./tools/compile-ros.sh interfaces controller_server map_tools navegacion_gps sensores
```
3. Entrar al contenedor:
```bash
./tools/exec.sh
```

## Scripts útiles
- `./tools/exec.sh`
- `./tools/root-exec.sh`
- `./tools/compile-ros.sh`
- `./tools/launch_controller.sh`
- `./tools/launch_no_go_editor.sh`
- `./tools/launch_sim_global_v2.sh`
- `./tools/launch_sim_global_v2_wifi.sh`
- `./tools/launch_real_global_v2.sh`
- `./tools/launch_real_global_v2_wifi.sh`
- `./tools/launch_real_global_v2_rviz.sh`
- `./tools/launch_real_global_v2_wifi_rviz.sh`
- `./tools/record_nav_debug_bag.sh`
- `./tools/record_compass_calibration.sh`
- `./tools/healthcheck-lidar.sh`

Scripts legacy o de referencia:
- `./tools/launch_real_nav.sh`
- `./tools/launch_real_rviz.sh`
- `./tools/launch_sim_local_v2.sh`
- `./tools/launch_real_local_v2.sh`

## Notas
- `rslidar_sdk` y `rslidar_msg` son dependencias vendorizadas. Su documentación upstream no es la fuente de verdad del proyecto Salus.
- Algunos scripts en `tools/` siguen existiendo por compatibilidad operativa. La clasificación actual de perfiles y launches está en [docs/launch-matrix.md](/home/leo/codigo/ROS2_SALUS/docs/launch-matrix.md).
- No usar `vcstool` para reconstruir `src/` desde múltiples remotos. El flujo esperado para este checkout es `git clone` del monorepo y trabajo directo sobre la raíz.
