# interfaces

Estado: actual
Alcance: contratos ROS comunes entre navegación, web y control
Fuente de verdad: `msg/`, `srv/` y paquetes consumidores

`interfaces` centraliza los mensajes y servicios compartidos por el stack SALUS. No contiene nodos ni launches.

## Mensajes principales
- `CmdVelFinal.msg`
  - comando final de velocidad y freno usado entre `nav_command_server`, web manual y `controller_server`
  - `source` distingue comandos auto/manual/safety para aplicar limites de control correctos
- `DriveTelemetry.msg`
  - telemetría de conducción publicada por `controller_server`
- `BatteryMissionGuard.msg`
  - guardia tipada de batería para lógica de misión
  - separa la recomendación de `return_home` del `%` mostrado al operador
- `NavTelemetry.msg`
  - snapshot resumido del estado de navegación
- `NavEvent.msg`
  - eventos discretos de navegación para observabilidad
- `NavSnapshotLayers.msg`
  - metadatos asociados a snapshots de navegación
- `NoGoPoint.msg`
- `NoGoZone.msg`

## Servicios principales
- Navegación:
  - `SetNavGoalLL.srv`
  - `CancelNavGoal.srv`
  - `BrakeNav.srv`
    - `duration_s > 0` solicita freno sostenido; `duration_s <= 0` conserva el freno inmediato por ráfaga.
  - `SetManualMode.srv`
  - `GetNavState.srv`
  - `GetNavSnapshot.srv`
- Zonas y keepout:
  - `SetZonesGeoJson.srv`
  - `GetZonesState.srv`
  - `SetKeepoutZones.srv`
  - `GetKeepoutState.srv`
- Datum:
  - `SetDatum.srv`
  - `GetDatum.srv`
  - LEGACY: estos servicios pertenecen al flujo viejo de datum dinamico. La navegacion global vigente usa datum fijo por sitio operativo.
- Cámara:
  - `CameraPan.srv`
  - `CameraStatus.srv`
- Manual:
  - `SetManualCmd.srv`

## Consumidores principales
- `navegacion_gps`
- `map_tools`
- `controller_server`
- `sensores`

## Build
```bash
./tools/compile-ros.sh interfaces
```
