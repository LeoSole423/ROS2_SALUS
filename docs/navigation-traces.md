# Trazas de navegación para diagnóstico

Estado: actual
Alcance: diagnóstico automático de rutas multi-waypoint en simulación
Fuente de verdad: `nav_trace_recorder`, `TraceReplan`, `sim_global_v2.launch.py`

## Objetivo

`sim_global_v2` registra una traza por misión de `route_executor` para poder
entregar evidencia compacta a un agente de IA. La sesión abarca toda la ruta y
no se corta cuando Nav2 termina un chunk intermedio.

La instrumentación está desactivada en los perfiles reales. En simulación puede
desactivarse con:

```bash
ros2 launch navegacion_gps sim_global_v2.launch.py enable_nav_trace:=False
```

## Salida

Cada misión crea `/ros2_ws/artifacts/nav_traces/<fecha>_<mission_id>/`:

- `summary.md`: resumen listo para Claude/Codex.
- `timeline.jsonl`: eventos, progreso, BT, safety y resultados ordenados.
- `metadata.json`: commit, branch, estado dirty y configuración activa.
- `plans/plan_NNNN.json`: puntos y métricas de cada `/plan` publicado.
- `mission_path.json` y `chunks/`: geometría completa enviada por `route_executor`.
- `context/`: copia del YAML Nav2 y XML BT activos, con hash en metadata.

Para mostrar la última traza desde el host:

```bash
./tools/show_latest_nav_trace.sh
```

Para regenerar el resumen después de cambiar el analizador:

```bash
./tools/regenerate_nav_trace_report.sh artifacts/nav_traces/<traza>
```

## Causalidad

El BT trazado publica una pareja `REPLAN_STARTED`/`REPLAN_FINISHED` o
`REPLAN_FAILED` cada vez que ejecuta realmente `ComputePathThroughPoses`. El
campo `reason` no es inferido y toma uno de estos valores:

- `goal_updated`: cambió la lista de metas que consume Nav2.
- `path_invalid`: `IsPathValid` rechazó el path vigente.
- `clearance_invalid`: el path seguía válido, pero perdió margen lateral.

Los clears de costmap y recoveries se conservan como transiciones del BT en la
misma línea temporal.

El `path_clearance_validator` publica eventos propios cuando encuentra un path
inválido o un check lento:

- `CLEARANCE_INVALID`: incluye `reason`, `max_cost`, muestras revisadas e
  índices inválidos.
- `CLEARANCE_SLOW`: incluye duración, muestras y antigüedad del costmap.

## Heurísticas

El reporte marca, sin tratarlas como causa confirmada:

- tres o más replans dentro de diez segundos;
- retroceso del índice expandido sin cruce válido del loop;
- distancia mayor a `2m` entre el robot y el inicio del nuevo path;
- diferencia inicial de heading mayor a `60deg`;
- posible path en O por auto-intersección o giro acumulado mayor a `270deg` en
  los primeros `20m`.

Para analizar un incidente, entregar primero `summary.md` y luego
`timeline.jsonl` y el `plans/plan_NNNN.json` señalado como sospechoso.
