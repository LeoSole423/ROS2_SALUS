# RTK/NTRIP en el robot histórico

## Cambio activado el 2026-08-26

Propietario: `ROS2_SALUS` (main), desplegado en
`/home/admin/Desktop/SALUS/ROS2_SALUS` de la Jetson. No modifica la migración
`salus_robot`, el control de motores ni la estimación de orientación.

El servicio `salus-real-global-v2-wifi.service` iniciaba el bridge RTCM, pero no
el cliente NTRIP. El bridge intentaba leer `127.0.0.1:2102`, sin servidor allí.
Los perfiles `real_global_v2` y `real_global_v2_wifi` ahora habilitan
`enable_rtk_source_manager=true`: cliente NTRIP → `/rtcm` → `rtk_bridge` →
`/mavros_node/send_rtcm` → Pixhawk/GNSS. En esta modalidad se deshabilita la
entrada TCP del bridge para evitar duplicación. Desactivar el manager conserva
la opción de un relay TCP externo; no usar ambas entradas para la misma base.

## Configuración privada

Base elegida para la ubicación actual en Córdoba: `IGN UCOR`, caster
`ntrip.ign.gob.ar:2101`, mountpoint `UCOR-v3.3`. No es una elección universal:
revisar distancia y disponibilidad al trasladar el robot.

`sensores/mavros.launch.py` prefiere
`/ros2_ws/src/sensores/config/rtk_sources.local.yaml` cuando existe. Si no,
conserva el archivo histórico. `rtk_sources_config` permite elegir otro archivo;
`active_rtk_source_id` sobrescribe la selección inicial. Vacío usa el
`active_source_id` guardado y, en archivos históricos sin selección, la primera base.

Ejecutar `python3 tools/configure-ign-rtk.py` en el checkout del robot para
introducir credenciales sin mostrarlas ni pasarlas como argumentos. Conserva
las otras bases y guarda IGN como activa en un YAML local, ignorado por Git,
con permisos 0600. No inicia procesos. Hay una plantilla sin secretos en
`src/sensores/config/rtk_sources.example.yaml`.

No incluir credenciales en documentación, logs, capturas o commits. El YAML
histórico todavía contiene credenciales heredadas: su saneamiento y rotación
son deuda separada. NTRIP en este endpoint usa HTTP/Basic, **no TLS**; los
permisos del archivo protegen el almacenamiento, no el tránsito de red.

## Estado y recuperación

El cliente valida encabezados HTTP completos, respuestas ICY, transferencia
chunked y CRC24Q de cada trama RTCM. Rechaza tablas de fuentes, errores de
autenticación y contenido de texto. Si no hay RTCM válido durante 10 s,
reconecta con espera progresiva de 2 a 60 s. Esto no evita cuotas ni bloqueos
del proveedor. La selección/edición se persiste atómicamente antes del cambio
en vivo; no se publican tramas de una generación anterior de la fuente.

`/gps/rtk_source/status_json` separa:

- `connected`: handshake NTRIP aceptado.
- `receiving_rtcm`: tramas con CRC válido recibidas recientemente.
- `rtcm_age_s`, `received_count`, `crc_errors`, `last_error`: diagnóstico.
- `status_sequence`: latido para detectar telemetría congelada.

Cockpit main muestra estos estados en el módulo de navegación. No inventa una
base a partir de la etiqueta GPS; el modal retira el indicador activo si pierde
la conexión con el backend o el latido durante 5 s. Recibir RTCM **no prueba**
que el receptor haya alcanzado RTK float/fixed. Eso se comprueba aparte en
`/gps/rtk_status_mavros` (salida del bridge en el perfil real), `/gps/rtk_status`
cuando esté publicada y la telemetría GNSS, preferentemente a cielo abierto.

## Activación y validación

El cambio de archivos no reinicia el proceso Python activo. El operador confirmó
propulsión físicamente inhibida y E-stop accesible y autorizó la activación el
2026-08-26. Se reinició el servicio y se verificó la cadena operativa completa.
Mantener esa precaución en reinicios futuros: las correcciones pueden cambiar
la posición estimada.
No ejecutar un segundo cliente conectado al mismo `/rtcm`.

Antes de activar, comprobar build de `sensores` y `navegacion_gps`, argumentos
de los perfiles real/WiFi con `--show-args`, tests del cliente y estado del
servicio. Con autorización del operador, reiniciar el servicio existente y
verificar una única instancia del manager, contador RTCM creciente,
`receiving_rtcm=true`, ausencia de TCP 2102 y recepción del bridge/MAVROS.
No modificar datum/calibraciones para forzar un fix.

La inspección autenticada aislada contra IGN recibió 67 tramas RTCM válidas en
15 s; cerró la conexión sin publicar al robot. Las pruebas automáticas usan
servidores locales falsos, sin hardware ni credenciales reales.

Validación final del código desplegado (2026-08-26):

- Build aislado de `sensores` y `navegacion_gps` correcto, sin sustituir los
  directorios build/install del proceso activo. Ambos perfiles exponen manager
  habilitado y el archivo privado correcto en `--show-args`.
- Suite `src/sensores/test`: 48 pruebas correctas en ROS Humble.
  Se deshabilitó el autoload de plugins de pytest para evitar la incompatibilidad
  del plugin histórico `launch_testing` con pytest 9; los tests de condiciones
  de launch sí se ejecutaron.
- Instancia del cliente corregido en dominio ROS 73, sólo loopback, tópico
  `/salus_rtk_probe/rtcm`: 11 tramas válidas, 0 errores CRC, conexión IGN correcta.
  Se cerró al finalizar; no publicó en `/rtcm` ni en MAVROS del robot.
- Cockpit main: build correcto y 153 pruebas correctas, 1 omitida, con Node 24.
  Node 26 presenta un fallo ajeno de `localStorage` en la prueba del AppShell.
- Primera comprobación sin reiniciar el servicio (MainPID 3624); después de la
  autorización se activó el servicio (MainPID 60687 en la comprobación final).

### Evidencia operativa y arranque automático

`salus-real-global-v2-wifi.service` está `enabled` y `active (running)`, con
`Restart=always`, espera de red/Docker y Docker habilitado al arranque. Su
`ExecStartPre` levanta el contenedor y su `ExecStart` ejecuta el perfil WiFi.
Las credenciales y selección IGN están en el directorio persistente montado en
Docker, no en `/tmp`. El cliente reintenta si aún no hay Internet al arrancar.
No se reinició la Jetson completa; se verificaron la configuración de arranque
y la puesta en marcha real del servicio.

Muestra de 20 s después de activar:

- 90 tramas en `/rtcm` y 90 reenviadas a `/mavros_node/send_rtcm`.
- 169 mensajes MAVLink `GPS_RTCM_DATA` (id 233) observados en
  `/uas1/mavlink_sink`; puede haber varios fragmentos por trama RTCM.
- Una instancia de `rtk_source_manager`, una de `rtk_bridge` y un publicador
  de `/rtcm`. Fuente `ign_ucor`, recepción activa, 0 errores CRC y edad 0,27 s.
- GNSS: `fix_type=5` (**RTK Float**), 25 satélites. No confundir con RTK Fixed.
- Pixhawk conectado, desarmado, modo HOLD; tracción deshabilitada y velocidad 0.
- Comprobación posterior: `/gps/rtk_status_mavros=rtk_float`; WebSocket de
  Cockpit en 8766 anuncia IGN UCOR, recepción activa, contador 507 y sin error.

Se detectó que detener sólo el cliente `docker exec` de systemd dejaba vivos
los nodos ROS del contenedor. Se instaló
`tools/systemd/salus-real-global-v2-wifi-lifecycle.conf` como
`/etc/systemd/system/salus-real-global-v2-wifi.service.d/lifecycle.conf`:

- Antes de iniciar, comprueba que no haya otro launch real Global V2.
- Al detener, `tools/stop_real_global_v2.py` envía SIGINT únicamente al supervisor
  de ese launch y espera la salida de sus descendientes. Si falla, detiene el
  contenedor dedicado `ros2_salus` para evitar stacks duplicados.
- La parada ordenada del stack anterior se verificó en hardware; las dos pruebas
  de alcance del selector pasan. No afecta otros contenedores de la Jetson.

No iniciar un cliente NTRIP adicional manualmente. Para intervención autorizada,
usar `sudo systemctl restart salus-real-global-v2-wifi.service`; comprobar con
`systemctl is-enabled` / `systemctl is-active` y la telemetría RTCM.

Respaldo de archivos rastreados previos en la Jetson:
`/tmp/salus-rtk-review.5OYt7H/pre-change-tracked.tar` (temporal, no durable).
Para volver atrás, detener/reiniciar sólo con autorización, revisar cambios
posteriores antes de restaurar archivos concretos y conservar el YAML local
privado. No usar un reset global del checkout.
