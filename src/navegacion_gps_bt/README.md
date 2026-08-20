# navegacion_gps_bt

Estado: actual.

Alcance: plugins C++ de Behavior Tree usados por la navegación global V2.

Fuente de verdad: `CMakeLists.txt`, `include/`, `src/` y XML BT bajo `../navegacion_gps/config/`.

## Plugins

- `nav2_is_path_clearance_valid_condition_bt_node`
  - condición Nav2 que valida si el path vigente mantiene clearance utilizable
  - permite replanificar por evento cuando el path deja de ser seguro
- `nav2_trace_replan_decorator_bt_node`
  - decorador que publica trazas y detalles de replanificación
  - alimenta observabilidad/diagnóstico de por qué se pidió un path nuevo

## Integración

Las shared libraries se instalan en `lib` y se cargan desde los Behavior Trees XML de `navegacion_gps`. El package depende de `behaviortree_cpp_v3`, `nav2_behavior_tree`, `nav2_msgs`, `nav_msgs`, `interfaces`, `diagnostic_msgs`, `geometry_msgs` y `rclcpp`.

## Build

```bash
./tools/compile-ros.sh interfaces navegacion_gps_bt navegacion_gps
```

Al cambiar un puerto BT, nombre de plugin o contrato, actualizar juntos C++, XML, dependencias y tests de launch/contrato.
