# SALUS — Eventos del Sistema

Referencia completa de todos los eventos críticos del stack SALUS.
Usada para diagnóstico de misiones, análisis post-mortem y diseño del sistema de grabación.

**Leyenda de severidad:**
- 🟢 **INFO** — Operación normal. Se registra para contexto y replay.
- 🟡 **WARNING** — Estado degradado. El sistema sigue funcionando pero algo no está bien.
- 🔴 **ERROR / CRITICAL** — Fallo real. Requiere atención. Puede haber detenido la misión.

---

## 1. Ciclo de vida de navegación

Estos eventos cubren el flujo completo de una misión, desde que el operador envía un destino hasta que el robot llega o falla.

| Evento | Severidad | Por qué ocurre | Qué significa operacionalmente |
|--------|-----------|----------------|-------------------------------|
| `GOAL_REQUESTED` | 🟢 INFO | El operador clickeó "Ir" o envió waypoints. Se llama al servicio `/nav_command_server/set_goal_ll`. | Inicio de misión. Punto de entrada en el timeline de cualquier grabación. |
| `GOAL_ACCEPTED` | 🟢 INFO | Nav2 recibió el goal y lo aceptó internamente. El action server devolvió `accepted = True`. | El robot empezó a calcular el camino. Si no aparece después de GOAL_REQUESTED, Nav2 puede estar ocupado o con un goal anterior activo. |
| `GOAL_RESULT_SUCCEEDED` | 🟢 INFO | El robot llegó al destino. Nav2 devolvió `STATUS_SUCCEEDED`. | Misión exitosa. Referencia para comparar con misiones fallidas. |
| `GOAL_CANCELLED` | 🟡 WARNING | El operador canceló manualmente, o el sistema canceló porque se activó modo manual. | No es un error, pero si ocurre frecuentemente puede indicar problemas de confianza en la navegación autónoma. |
| `GOAL_RESULT_ABORTED` | 🔴 ERROR | Nav2 no pudo completar el goal. Causas comunes: obstáculo bloqueando el camino, timeout del planner, costmap inválido, pérdida de localización. | El motivo real está en `nav_result_text`. Siempre revisar este campo en el análisis post-mortem. |
| `GOAL_REJECTED` | 🟡 WARNING | Nav2 rechazó el goal antes de empezar. Causas: goal dentro de una zona prohibida, goal fuera del mapa, o ya hay un goal activo. | Si ocurre al inicio de la misión, verificar que el punto de destino sea alcanzable y que no haya un goal anterior colgado. |
| `LOOP_RESTART_FAILED` | 🔴 ERROR | La patrulla en loop completó un segmento pero no pudo enviar el siguiente. Puede ser un timeout del servicio o que el archivo de waypoints se corrompió en memoria. | La patrulla se detiene. El robot queda parado en el último waypoint. Requiere reinicio manual de la patrulla. |
| `ACTION_SERVER_UNAVAILABLE` | 🔴 ERROR | El action server de Nav2 (`FollowWaypoints` o `NavigateThroughPoses`) no responde al `wait_for_server()`. Timeout configurado en `request_timeout_s`. | Nav2 no está corriendo o crasheó. Sin este servicio, no hay navegación autónoma posible. Verificar que todos los nodos de Nav2 estén levantados. |

---

## 2. Seguridad y emergencia

Eventos que indican que el sistema de seguridad intervino. Son los más importantes para el análisis de incidentes.

| Evento | Severidad | Por qué ocurre | Qué significa operacionalmente |
|--------|-----------|----------------|-------------------------------|
| `BRAKE_APPLIED` | 🔴 CRITICAL | Se llamó al servicio de freno (`/nav_command_server/brake`). Causas: detección de colisión, visión (YOLO), comando explícito del operador, o resultado abortado de goal. | El robot se detuvo completamente. En el análisis, siempre buscar qué evento ocurrió 0-500ms antes para entender la causa raíz. |
| `COLLISION_STOP_ACTIVE` | 🔴 CRITICAL | El nodo `collision_monitor` publicó estado `STOP` en su topic. Esto ocurre cuando el LIDAR detecta un obstáculo dentro del radio de seguridad configurado. | Nav2 solicitó parada de emergencia por obstáculo físico detectado por LIDAR. El robot no retomará la navegación solo hasta que el obstáculo desaparezca. |
| `VISION_BRAKE_TRIGGERED` | 🔴 CRITICAL | `vision_brake_guard.py` detectó el mismo objeto (persona, obstáculo) en al menos 3 frames consecutivos con confianza suficiente. Llama directamente al servicio de freno. | El sistema de visión paró el robot por seguridad. Ver el frame guardado para confirmar si fue un falso positivo. El umbral es configurable (`required_consecutive_hits`, default: 3). |
| `VISION_BRAKE_SERVICE_FAILED` | 🔴 ERROR | `vision_brake_guard.py` intentó llamar al freno pero el servicio no respondió o lanzó una excepción. | Doble falla: el sistema detectó un peligro pero no pudo frenar. Estado muy peligroso. Indica que el nodo de freno no está disponible. |
| `MANUAL_TAKEOVER` | 🟡 WARNING | El operador activó el modo manual mientras había un goal activo. El goal activo fue cancelado automáticamente. | El operador decidió tomar el control. Si ocurre frecuentemente, puede indicar que la navegación autónoma no es confiable en ese entorno. |
| `MANUAL_WATCHDOG_STOP` | 🟡 WARNING | En modo manual, el backend dejó de recibir comandos por más de `manual_cmd_timeout_s` (default: 0.4 segundos). El robot se detiene automáticamente como medida de seguridad. | El cockpit perdió conexión o el operador no mandó comandos a tiempo. El robot queda parado pero en modo manual, esperando el siguiente comando. |
| `CONTROL_LOCK_ENGAGED` | 🟡 WARNING | Los controles fueron bloqueados. Causas: timeout de heartbeat de la UI, solicitud explícita del operador, o evento de seguridad del backend. | Ningún comando de movimiento es aceptado hasta que se libere el lock. El robot queda inmovilizado por seguridad. |
| `CONTROL_LOCK_RELEASED` | 🟢 INFO | El lock de controles fue liberado. El operador puede volver a enviar comandos. | Estado normal restaurado. Si aparece inmediatamente después de CONTROL_LOCK_ENGAGED con poco tiempo entre ambos, el lock fue momentáneo. |
| `UI_HEARTBEAT_TIMEOUT` | 🔴 ERROR | El cockpit dejó de enviar el heartbeat periódico al backend. El backend activa automáticamente el control lock. Causa más común: pérdida de WebSocket. | El backend asume que el operador perdió conexión y bloquea los controles por seguridad. El robot queda inmovilizado. Revisar eventos de WS_DISCONNECTED cerca en el tiempo. |
| `ESTOP_ACTIVE` | 🔴 CRITICAL | El controlador de motores reportó que su E-stop interno está activo. Viene de la telemetría del controlador (`estop_active: true` en `/controller/telemetry`). | Parada de emergencia a nivel de hardware. Puede ser física (botón de emergencia presionado) o lógica. El robot no puede moverse hasta que se desactive el E-stop. |

---

## 3. GPS y localización

La navegación GPS del cuatri depende completamente de la calidad de la señal. Estos eventos afectan directamente la precisión y confiabilidad de la trayectoria.

| Evento | Severidad | Por qué ocurre | Qué significa operacionalmente |
|--------|-----------|----------------|-------------------------------|
| `FROMLL_FAILED` | 🔴 ERROR | El servicio `fromLL` (conversión de coordenadas geográficas lat/lon a coordenadas locales del mapa) falló después de 4 reintentos con 150ms de espera entre cada uno. | El robot no puede convertir el destino enviado por el operador a coordenadas que Nav2 entiende. La misión no puede empezar. Causa más común: el nodo de transformación de coordenadas (`robot_localization` o similar) no está activo. |
| `GPS_STALE` | 🟡 WARNING | No se recibió una nueva lectura GPS en más de `gps_stale_warn_s` (default: 1.5 segundos). | El GPS sigue funcionando pero hay latencia. En movimiento, esto puede causar pequeñas desviaciones de ruta. Si persiste más de 3 segundos, escala a GPS_MISSING. |
| `GPS_MISSING` | 🔴 ERROR | No se recibió GPS en más de `gps_stale_error_s` (default: 4.0 segundos). | El GPS se perdió completamente. La localización del robot es incierta. Nav2 puede abortar el goal si no puede mantener la posición estimada. |
| `LOCALIZATION_STALE` | 🟡 WARNING | La odometría local (`/odometry/local`) no se actualizó en más de `odom_stale_warn_s` (default: 1.0 segundo). | El sistema de fusión de sensores (EKF) está tardando. La posición estimada del robot puede no ser precisa. |
| `LOCALIZATION_MISSING` | 🔴 ERROR | La odometría local no se actualizó en más de `odom_stale_error_s` (default: 3.0 segundos). | La localización del robot se perdió. Sin posición confiable, Nav2 no puede planificar ni navegar. Puede provocar GOAL_RESULT_ABORTED. |
| `RTK_FIXED` | 🟢 INFO | El receptor GPS tiene corrección RTK fija. Precisión típica: 1-3 cm. | Estado ideal para navegación. La trayectoria será precisa. |
| `RTK_FLOAT` | 🟡 WARNING | El receptor GPS tiene corrección RTK pero en modo flotante. Precisión típica: 20-50 cm. | Navegación degradada. El robot puede desviarse de la ruta planificada. Causas: señal RTK débil, pocas antenas visibles, interferencia. |
| `RTCM_STALE` | 🔴 ERROR | La corrección diferencial RTCM del servidor RTK dejó de llegar. El receptor GPS ya no tiene datos frescos para la corrección. | El GPS va a degradarse de RTK a 3D FIX en pocos segundos. La precisión se pierde. Verificar conectividad con el servidor NTRIP/RTK. |
| `NO_FIX` | 🔴 CRITICAL | El receptor GPS no tiene fix. No hay posición disponible. Causas: antena desconectada, cielo tapado, receptor reiniciando. | Sin GPS no hay navegación posible. El robot no puede estimar su posición. |

---

## 4. Conexión y red

La comunicación entre el cockpit y el backend es el canal por donde fluyen todos los comandos y datos. Una pérdida de conexión durante una misión es siempre un evento crítico.

| Evento | Severidad | Por qué ocurre | Qué significa operacionalmente |
|--------|-----------|----------------|-------------------------------|
| `WS_CONNECTED` | 🟢 INFO | El WebSocket entre el cockpit y el backend se estableció correctamente. | El operador puede enviar comandos y recibir telemetría. Punto de referencia en el timeline. |
| `WS_DISCONNECTED` | 🔴 ERROR | El WebSocket se cerró inesperadamente. Causas: red inestable, el backend crasheó, el robot salió del rango WiFi, timeout de inactividad. | El cockpit perdió contacto con el robot. Si había una misión activa, el backend activa el `UI_HEARTBEAT_TIMEOUT` y bloquea los controles. |
| `WS_RECONNECT_SCHEDULED` | 🟡 WARNING | El cockpit detectó la desconexión y programó un intento de reconexión con backoff exponencial (2s → 4s → 6s → ... → 15s máximo). | El cockpit está intentando reconectarse. Si en el análisis ves muchos de estos eventos seguidos, hubo inestabilidad de red prolongada. |
| `CMD_VEL_FLOW_STALE` | 🟡 WARNING | Durante una misión activa, el topic `/cmd_vel_safe` no recibió mensajes en más de `cmd_stale_warn_s` (default: 1.0 segundo). | Nav2 dejó de enviar comandos de velocidad. El robot va a detenerse pronto. Puede ser que el planner esté recalculando o que haya un obstáculo. |
| `CMD_VEL_FLOW_BROKEN` | 🔴 ERROR | Durante una misión activa, `/cmd_vel_safe` no recibió mensajes en más de `cmd_stale_error_s` (default: 3.0 segundos). | Nav2 dejó de funcionar con goal activo. El robot está parado sin razón aparente. Muy probablemente el planner crasheó o está en un loop infinito. |
| `CONTROLLER_TELEMETRY_MISSING` | 🔴 ERROR | El controlador de motores no envió telemetría en más de `controller_stale_error_s` (default: 3.0 segundos). | El controlador puede estar desconectado o caído. Sin telemetría del controlador, no se puede saber si los comandos de movimiento están llegando al hardware. |
| `FAILSAFE_ACTIVE` | 🔴 CRITICAL | El controlador de motores reportó `failsafe_active: true` en su telemetría. El failsafe se activa cuando el controlador pierde comunicación con su fuente de comandos. | El controlador entró en modo seguro por su cuenta. Los motores están siendo controlados por la lógica de failsafe del firmware (típicamente: detenerse). |

---

## 5. Cámara y visión

| Evento | Severidad | Por qué ocurre | Qué significa operacionalmente |
|--------|-----------|----------------|-------------------------------|
| `CAMERA_CONNECTED` | 🟢 INFO | El primer frame de `/camera/image_raw` llegó al cockpit después de un período sin frames. | La cámara está transmitiendo. Si aparece después de un CAMERA_TIMEOUT, la cámara se recuperó. |
| `CAMERA_TIMEOUT` | 🔴 ERROR | No se recibió ningún frame de cámara en 3000ms. Calculado en el cliente por `CameraVisionService`. | El stream de cámara se cortó. Causas: el nodo de cámara crasheó, pérdida de ancho de banda, la cámara física se desconectó. Si hay misión activa y `vision_brake_guard` está activo, el sistema de seguridad por visión queda ciego. |
| `DETECTIONS_ACTIVE` | 🟢 INFO | Llegó un batch de detecciones YOLO desde `/detections` y son más recientes que 2000ms. | El sistema de visión está funcionando y detectando objetos activamente. |
| `DETECTIONS_STALE` | 🟡 WARNING | Las últimas detecciones YOLO tienen más de 2000ms de antigüedad. El cliente las descarta. | El modelo de visión dejó de inferir o el nodo de detecciones está lento. La seguridad por visión puede no estar funcionando. |
| `VISION_BRAKE_TRIGGERED` | 🔴 CRITICAL | `vision_brake_guard.py` detectó un objeto peligroso (persona u obstáculo) en 3 frames consecutivos. Llamó directamente al servicio de freno. | Ver entrada detallada en sección de Seguridad. El frame de cámara del momento del trigger es el dato más valioso para el análisis. |
| `VISION_BRAKE_SERVICE_FAILED` | 🔴 ERROR | `vision_brake_guard.py` no pudo ejecutar el freno porque el servicio ROS2 no respondió. | Ver entrada detallada en sección de Seguridad. |

---

## 6. Patrulla y misiones de ruta

| Evento | Severidad | Por qué ocurre | Qué significa operacionalmente |
|--------|-----------|----------------|-------------------------------|
| `PATROL_STARTED` | 🟢 INFO | El servicio `/patrol/start` fue llamado con `data=True`. El nodo `loop_patrol_runner` cargó el archivo de waypoints y empezó el primer segmento. | Inicio de misión de patrulla. El archivo de waypoints y la cantidad de puntos quedan registrados. |
| `PATROL_STOPPED` | 🟢 INFO | La patrulla fue detenida por el operador (`data=False`) o completó todos sus ciclos. | Fin de misión de patrulla. Si no fue solicitado por el operador, revisar si fue por un error. |
| `PATROL_WAYPOINT_REACHED` | 🟢 INFO | El robot llegó a un waypoint del loop. Nav2 devolvió `STATUS_SUCCEEDED` para ese segmento. | Progreso normal de la patrulla. Útil para calcular velocidad promedio y eficiencia de la ruta. |
| `PATROL_WAYPOINT_FAILED` | 🟡 WARNING | El robot no pudo llegar al waypoint actual. Nav2 devolvió ABORTED o CANCELLED para ese segmento. El `loop_patrol_runner` puede reintentar según configuración. | Un punto de la ruta falló. El sistema puede reintentarlo. Si falla repetidamente el mismo waypoint, hay un obstáculo permanente o un problema de localización en esa zona. |
| `PATROL_GOAL_TIMEOUT` | 🔴 ERROR | Un goal de patrulla lleva más de `goal_timeout_s` (default: 120 segundos) en vuelo sin resolverse. El `loop_patrol_runner` lo cancela forzosamente. | El robot lleva 2 minutos intentando llegar a un waypoint sin resultado. Puede estar atascado, dando vueltas, o con el planner en un estado inválido. |

---

## 7. Sensores

| Evento | Severidad | Por qué ocurre | Qué significa operacionalmente |
|--------|-----------|----------------|-------------------------------|
| `SCAN_STALE` | 🟡 WARNING | El LIDAR no publicó datos en `/scan` en más de `scan_stale_warn_s` (default: 1.0 segundo). | El LIDAR está tardando. La detección de obstáculos puede estar usando datos viejos. |
| `SCAN_MISSING` | 🔴 ERROR | El LIDAR no publicó datos en más de `scan_stale_error_s` (default: 3.0 segundos). | El LIDAR se desconectó o el nodo driver crasheó. Sin LIDAR, el collision_monitor no funciona y la detección de obstáculos está ciega. Nav2 puede seguir navegando pero sin seguridad local. |
| `NO_COLLISION_MONITOR_STATE` | 🔴 ERROR | `collision_monitor` no publicó su estado en su topic mientras había un goal activo. Detectado por `nav_observability.py`. | El sistema de detección de colisiones no está activo durante una misión. El robot puede chocar sin que el sistema de seguridad intervenga. |
| `IMU_DATA_STALE` | 🟡 WARNING | Los datos de la IMU (`/imu/data`) dejaron de llegar. La IMU es usada por el EKF para la fusión de sensores y estimación de orientación. | La estimación de orientación (yaw, pitch, roll) puede degradarse. Si persiste, el EKF puede divergir y la localización se vuelve inestable. |

---

## 8. Procesos y archivos

| Evento | Severidad | Por qué ocurre | Qué significa operacionalmente |
|--------|-----------|----------------|-------------------------------|
| `ROSBAG_STARTED` | 🟢 INFO | El proceso `ros2 bag record` fue lanzado exitosamente como subprocess por `web_zone_server.py`. | La grabación ROS2 está activa. El PID y el directorio de salida quedan registrados. |
| `ROSBAG_STOPPED` | 🟢 INFO | El proceso `ros2 bag record` terminó limpiamente (exit code 0) después de recibir SIGINT. | La grabación se cerró correctamente. El archivo `.db3` está completo y es legible. |
| `ROSBAG_FAILED` | 🔴 ERROR | El proceso `ros2 bag record` terminó con exit code distinto de 0. Causas: disco lleno, permisos insuficientes, error al abrir topics. | La grabación se corrompió o nunca comenzó realmente. El archivo resultante puede estar incompleto o ser ilegible. |
| `WAYPOINTS_PARSE_ERROR` | 🔴 ERROR | El `loop_patrol_runner` intentó cargar el archivo YAML de waypoints pero el formato es inválido o el archivo está corrupto. | La patrulla no puede iniciar porque no tiene waypoints válidos. Revisar el archivo con un parser YAML antes de volver a intentar. |
| `RECORDING_WAYPOINTS_SAVE_FAILED` | 🔴 ERROR | El `manual_waypoint_recorder` no pudo escribir el archivo YAML al disco al detener la grabación. Causas: disco lleno, permisos, ruta inexistente. | Los waypoints grabados manualmente se perdieron. No se guardó nada al disco a pesar de que el operador grabó una ruta. |
| `ZONES_FILE_IO_ERROR` | 🔴 ERROR | El `zones_manager` no pudo leer o escribir el archivo GeoJSON de zonas prohibidas. | Las zonas de navegación no están cargadas o no se pueden persistir. El robot puede navegar por áreas que deberían estar bloqueadas. |

---

## Referencia rápida — por frecuencia en misiones reales

### Al inicio de misión (siempre deben aparecer en orden)
```
GOAL_REQUESTED → GOAL_ACCEPTED → [navegación] → GOAL_RESULT_SUCCEEDED
```
Si falta alguno de estos, la misión no comenzó o falló silenciosamente.

### Cadena de fallo típica
```
RTK_FLOAT → LOCALIZATION_STALE → CMD_VEL_FLOW_STALE → GOAL_RESULT_ABORTED
```
El GPS se degradó → la localización se volvió imprecisa → Nav2 dejó de mandar comandos → el goal abortó.

### Intervención de seguridad
```
COLLISION_STOP_ACTIVE o VISION_BRAKE_TRIGGERED → BRAKE_APPLIED → GOAL_RESULT_ABORTED
```

### Pérdida de conexión durante misión
```
WS_DISCONNECTED → UI_HEARTBEAT_TIMEOUT → CONTROL_LOCK_ENGAGED → WS_RECONNECT_SCHEDULED → WS_CONNECTED → CONTROL_LOCK_RELEASED
```

---

*Generado para el proyecto SALUS — 2026-05-16*
*Fuentes: nav_command_server.py, nav_observability.py, vision_brake_guard.py, loop_patrol_runner.py, web_zone_server.py, WebSocketTransport.ts, CameraVisionService.ts, TelemetryService.ts*
