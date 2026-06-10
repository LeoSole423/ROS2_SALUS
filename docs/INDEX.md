# Documentación de ROS2_SALUS

Estado: actual
Alcance: índice y clasificación de la documentación del monorepo
Fuente de verdad: launches, scripts, nodos y tests del checkout actual

## Cómo leer este repo
- Empezar por [README.md](/home/leo/codigo/ROS2_SALUS/README.md) para contexto general.
- Usar [docs/launch-matrix.md](/home/leo/codigo/ROS2_SALUS/docs/launch-matrix.md) para decidir qué perfil ejecutar.
- Usar [docs/sim-real-parity.md](/home/leo/codigo/ROS2_SALUS/docs/sim-real-parity.md) como regla de mantenimiento para mantener sincronizados simulacion y robot real.
- Usar [docs/runtime-architecture.md](/home/leo/codigo/ROS2_SALUS/docs/runtime-architecture.md) para entender el wiring runtime.
- Usar [docs/gazebo-worlds.md](/home/leo/codigo/ROS2_SALUS/docs/gazebo-worlds.md) para abrir mundos Gazebo/Fuel nuevos manteniendo paridad con el perfil real.
- Usar [docs/lidar-perception.md](/home/leo/codigo/ROS2_SALUS/docs/lidar-perception.md) para el estado, diagnostico y plan de mejora de percepcion LiDAR.
- Usar [docs/lidar-noise-reduction-plan.md](/home/leo/codigo/ROS2_SALUS/docs/lidar-noise-reduction-plan.md) para el plan inmediato de reduccion de ruido sobre el pipeline legacy.
- Usar [docs/lidar-perception-v2-plan.md](/home/leo/codigo/ROS2_SALUS/docs/lidar-perception-v2-plan.md) como plan futuro de rediseño LiDAR 3D inspirado en pipelines tipo Autoware/Nav2/PCL.
- Usar [docs/cockpit-integration.md](/home/leo/codigo/ROS2_SALUS/docs/cockpit-integration.md) para el contrato con la UI `cockpit`.

## Documentación vigente
- Raíz del monorepo:
  - [README.md](/home/leo/codigo/ROS2_SALUS/README.md)
  - [docs/launch-matrix.md](/home/leo/codigo/ROS2_SALUS/docs/launch-matrix.md)
  - [docs/sim-real-parity.md](/home/leo/codigo/ROS2_SALUS/docs/sim-real-parity.md)
  - [docs/nav-benchmarks.md](/home/leo/codigo/ROS2_SALUS/docs/nav-benchmarks.md)
  - [docs/heading-compass-integration-ideas.md](/home/leo/codigo/ROS2_SALUS/docs/heading-compass-integration-ideas.md)
  - [docs/runtime-architecture.md](/home/leo/codigo/ROS2_SALUS/docs/runtime-architecture.md)
  - [docs/gazebo-worlds.md](/home/leo/codigo/ROS2_SALUS/docs/gazebo-worlds.md)
  - [docs/lidar-perception.md](/home/leo/codigo/ROS2_SALUS/docs/lidar-perception.md)
  - [docs/lidar-noise-reduction-plan.md](/home/leo/codigo/ROS2_SALUS/docs/lidar-noise-reduction-plan.md)
  - [docs/lidar-perception-v2-plan.md](/home/leo/codigo/ROS2_SALUS/docs/lidar-perception-v2-plan.md)
  - [docs/cockpit-integration.md](/home/leo/codigo/ROS2_SALUS/docs/cockpit-integration.md)
- Paquetes:
  - [src/interfaces/README.md](/home/leo/codigo/ROS2_SALUS/src/interfaces/README.md)
  - [src/controller_server/README.md](/home/leo/codigo/ROS2_SALUS/src/controller_server/README.md)
  - [src/map_tools/README.md](/home/leo/codigo/ROS2_SALUS/src/map_tools/README.md)
  - [src/navegacion_gps/README.md](/home/leo/codigo/ROS2_SALUS/src/navegacion_gps/README.md)
  - [src/sensores/README.md](/home/leo/codigo/ROS2_SALUS/src/sensores/README.md)

## Documentación de navegación vigente
- Navegación global V2:
  - [docs/sim-real-parity.md](/home/leo/codigo/ROS2_SALUS/docs/sim-real-parity.md)
  - [docs/gazebo-worlds.md](/home/leo/codigo/ROS2_SALUS/docs/gazebo-worlds.md)
  - [docs/lidar-perception.md](/home/leo/codigo/ROS2_SALUS/docs/lidar-perception.md)
  - [docs/lidar-noise-reduction-plan.md](/home/leo/codigo/ROS2_SALUS/docs/lidar-noise-reduction-plan.md)
  - [docs/lidar-perception-v2-plan.md](/home/leo/codigo/ROS2_SALUS/docs/lidar-perception-v2-plan.md)
  - [src/navegacion_gps/REAL_GLOBAL_V2_CHECKLIST.md](/home/leo/codigo/ROS2_SALUS/src/navegacion_gps/REAL_GLOBAL_V2_CHECKLIST.md)
  - [docs/nav-benchmarks.md](/home/leo/codigo/ROS2_SALUS/docs/nav-benchmarks.md)
  - [docs/heading-compass-integration-ideas.md](/home/leo/codigo/ROS2_SALUS/docs/heading-compass-integration-ideas.md)
- Base local V2 usada por Global V2:
  - [src/navegacion_gps/LOCAL_NAV_V2.md](/home/leo/codigo/ROS2_SALUS/src/navegacion_gps/LOCAL_NAV_V2.md)

## Legacy / referencia
- Mainline de navegación:
  - [src/navegacion_gps/README.md](/home/leo/codigo/ROS2_SALUS/src/navegacion_gps/README.md)
  - `simulacion.launch.py`
  - `real.launch.py`
  - `rviz_real.launch.py`
- Launches locales standalone:
  - [src/navegacion_gps/SIM_LOCAL_V2_FIDELITY.md](/home/leo/codigo/ROS2_SALUS/src/navegacion_gps/SIM_LOCAL_V2_FIDELITY.md)
  - [src/navegacion_gps/REAL_LOCAL_V2_CHECKLIST.md](/home/leo/codigo/ROS2_SALUS/src/navegacion_gps/REAL_LOCAL_V2_CHECKLIST.md)
  - `sim_local_v2.launch.py`
  - `real_local_v2.launch.py`
- Sensores, MAVROS y codigo Pixhawk legacy:
  - [src/sensores/README.md](/home/leo/codigo/ROS2_SALUS/src/sensores/README.md)

## Histórica o de transición
- Índice de históricos y third-party:
  - [docs/archive/README.md](/home/leo/codigo/ROS2_SALUS/docs/archive/README.md)

## Regla de mantenimiento
- Cada documento nuevo o actualizado debe indicar `Estado`, `Alcance` y `Fuente de verdad`.
- Los README de paquete deben explicar operación y contratos públicos, no historial de branches.
- Los documentos de diseño o experimentación deben quedar clasificados como `histórico`, `transición` o `archivo`.
