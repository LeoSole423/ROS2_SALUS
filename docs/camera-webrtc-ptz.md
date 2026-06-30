# Cámara WebRTC y PTZ

Estado: actual
Alcance: operación de la cámara IP con control PTZ desde ROS 2 y consumo WebRTC desde `cockpit`
Fuente de verdad: `src/sensores/sensores/camara.py`, `src/sensores/launch/camara.launch.py`, `src/navegacion_gps/launch/real_global_v2.launch.py` y el despliegue operativo de MediaMTX en el robot

## Resumen
- `ROS2_SALUS` expone el control PTZ de la cámara como servicios ROS 2 bajo `/camara/*`.
- `real_global_v2.launch.py` puede levantar ese nodo automáticamente con `launch_camera:=True`.
- El video para `cockpit` no sale desde ROS 2: el camino operativo recomendado es `cámara RTSP -> MediaMTX -> WHEP/WebRTC`.
- Para ahorrar datos móviles, MediaMTX debe configurarse en modo on-demand, de manera que el RTSP de la cámara solo se abra cuando exista un viewer activo.

## Qué vive dentro de ROS2_SALUS
- Nodo: `sensores/camara`
- Launch: `ros2 launch sensores camara.launch.py`
- Integración automática en navegación real:
  - `ros2 launch navegacion_gps real_global_v2.launch.py`
  - `ros2 launch navegacion_gps real_global_v2_wifi.launch.py`

Servicios publicados por el nodo:
- `/camara/camera_pan`
- `/camara/camera_zoom_toggle`
- `/camara/camera_status`
- `/camara/camera_ptz`
- `/camara/camera_preset`
- `/camara/camera_save_preset`
- `/camara/camera_ptz_state`

## Configuración del nodo `camara`
El nodo lee primero un archivo `.env` y luego variables de entorno del proceso. Si no encuentra valores, cae a defaults inseguros pensados solo para desarrollo.

Ubicaciones que intenta resolver:
- `install/sensores/share/sensores/.env`
- `src/sensores/.env`

Variables soportadas:
- `CAMERA_HOST`
- `CAMERA_PORT`
- `CAMERA_USER`
- `CAMERA_PASS`
- `CAMERA_CHANNEL`

Ejemplo:

```env
CAMERA_HOST=192.168.1.64
CAMERA_PORT=80
CAMERA_USER=admin
CAMERA_PASS=CAMBIAR_ESTA_CLAVE
CAMERA_CHANNEL=1
```

Si faltan `CAMERA_HOST`, `CAMERA_USER` o `CAMERA_PASS`, el nodo queda no listo y responde error en los servicios de PTZ/status.

Overrides persistidos de presets PTZ:
- el nodo guarda presets editables en un JSON separado del `.env`
- ubicación default: archivo hermano del `.env`
- ejemplos típicos:
  - `install/sensores/share/sensores/.camera_presets.json`
  - `src/sensores/.camera_presets.json`
- este archivo es local del robot y no debe versionarse en git

## Parámetros ROS relevantes
Además del `.env`, el nodo declara parámetros para adaptar límites y presets de la cámara:
- `camera_az_min`, `camera_az_max`
- `camera_el_min`, `camera_el_max`
- `camera_zoom_min`, `camera_zoom_max`
- `camera_zoom_fixed_level`
- `camera_zoom_zero_level`
- `camera_zoom_initial_in`
- `camera_preset_front_azimuth_deg`
- `camera_preset_left_azimuth_deg`
- `camera_preset_right_azimuth_deg`
- `camera_preset_rear_azimuth_deg`
- `camera_preset_neutral_elevation_deg`
- `camera_preset_home_zoom_level`

Presets lógicos expuestos hoy:
- `home`
- `front`
- `left`
- `right`
- `rear`

Alias aceptados:
- `center -> home`
- `back -> rear`

## Integración con `real_global_v2`
`real_global_v2.launch.py` incluye `sensores/camara.launch.py` cuando:

```bash
ros2 launch navegacion_gps real_global_v2.launch.py launch_camera:=True
```

El default actual de `launch_camera` en ese perfil es `True`, así que en operación normal el nodo sube junto con navegación, MAVROS, LiDAR y `web_zone_server`.

## Contrato con `cockpit`
El backend WebSocket de SALUS (`map_tools/web_zone_server.py`) usa esos servicios ROS para atender operaciones de UI:
- `camera_pan`
- `camera_zoom_toggle`
- `get_camera_status`
- `camera_ptz_move`
- `camera_ptz_preset`
- `camera_ptz_set_preset`
- `get_camera_ptz_state`

Esto cubre control PTZ y telemetría básica de la cámara, pero no el transporte de video.

## Presets editables desde `cockpit`
`cockpit` puede regrabar tres presets operativos desde la posición actual de la cámara:
- `HOME`
- `LEFT`
- `RIGHT`

Política de guardado:
- `HOME` guarda `pan + tilt + zoom` actuales
- `LEFT` guarda `pan + tilt` actuales y conserva el zoom amplio ya configurado para `LEFT`
- `RIGHT` guarda `pan + tilt` actuales y conserva el zoom amplio ya configurado para `RIGHT`
- `FRONT` y `REAR` no se editan desde la UI en esta iteración

Persistencia:
- al guardar, el nodo actualiza el archivo `.camera_presets.json`
- los nuevos presets sobreviven reinicio del nodo y de la Jetson
- si el archivo no existe, se crea automáticamente

## Transporte de video recomendado
El video en `cockpit` debe salir por MediaMTX usando WHEP/WebRTC. La parte ROS de este repo no proxyfica ni retransmite el stream.

Topología recomendada:

```text
Camara IP (RTSP/ISAPI)
  -> MediaMTX en el robot
  -> endpoint WHEP/WebRTC
  -> cockpit
```

Ventajas:
- menor latencia que MJPEG
- mejor compatibilidad con la UI actual
- separación clara entre control PTZ y transporte multimedia

## Recomendación para no gastar 4G
Para evitar consumo continuo de datos móviles, configurar MediaMTX con fuente on-demand:
- abrir el RTSP hacia la cámara solo cuando haya al menos un viewer
- cerrar el source cuando el último viewer se desconecta

Operativamente, esto implica:
- `cockpit` puede quedar abierto sin stream activo
- el robot no sostiene tráfico de video si nadie está mirando
- PTZ sigue disponible aunque el viewer no esté consumiendo video

La configuración exacta de MediaMTX depende del host del robot y queda fuera de este monorepo, pero el comportamiento esperado para SALUS es on-demand.

## Checklist operativo
1. Verificar conectividad IP entre robot y cámara.
2. Crear `src/sensores/.env` o instalar `.env` equivalente en `share/sensores/`.
3. Levantar navegación real con `launch_camera:=True`.
4. Confirmar que existan los servicios `/camara/*`.
5. Verificar PTZ con `camera_ptz` o `camera_preset`.
6. Si hace falta recalibrar vista, usar en `cockpit` los botones `SET HOME`, `SET LEFT` o `SET RIGHT`.
7. Levantar MediaMTX en el robot con path WHEP/WebRTC y source on-demand.
8. Configurar `cockpit` para consumir la URL WHEP publicada por MediaMTX.

## Diagnóstico rápido
Listar servicios:

```bash
ros2 service list | grep camara
```

Consultar estado:

```bash
ros2 service call /camara/camera_status interfaces/srv/CameraStatus "{}"
```

Mover a preset:

```bash
ros2 service call /camara/camera_preset interfaces/srv/CameraPreset "{preset: front}"
```

Guardar `HOME` desde la posición actual:

```bash
ros2 service call /camara/camera_save_preset interfaces/srv/CameraSavePreset "{preset: home, save_zoom: true}"
```

Guardar `LEFT` preservando su zoom actual:

```bash
ros2 service call /camara/camera_save_preset interfaces/srv/CameraSavePreset "{preset: left, save_zoom: false}"
```

Leer estado PTZ:

```bash
ros2 service call /camara/camera_ptz_state interfaces/srv/CameraPtzState "{}"
```

## Límites conocidos
- Este repo no versiona la config de MediaMTX del robot.
- El nodo `camara` controla PTZ por ISAPI/HTTP digest, no publica frames ROS ni hace transcoding.
- Si la cámara cambia credenciales, IP o canal, el `.env` debe actualizarse en el entorno donde corre ROS 2.
- Si el archivo `.camera_presets.json` está corrupto o no se puede escribir, el nodo mantiene los presets base y reporta warning/error en la operación de guardado.
