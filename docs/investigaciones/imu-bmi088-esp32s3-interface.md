# Sustitución de la IMU del Pixhawk por BMI088 + ESP32-S3

Estado: investigación y especificación de integración para la siguiente revisión de hardware  
Alcance: IMU exclusivamente; GNSS, RTK y heading externo quedan fuera de este documento  
Fuente de verdad: perfil `real_global_v2`, configuración de `robot_localization`, implementación MAVROS y datasheet BMI088  
Última revisión: 2026-08-06

## 1. Objetivo y decisión

El Pixhawk 6X puede reemplazarse, para la función inercial que usa actualmente
ROS2_SALUS, por una PCB con:

- un BMI088;
- un ESP32-S3;
- una conexión de datos entre la ESP32-S3 y la Jetson Orin o Raspberry Pi;
- un nodo ROS 2 o micro-ROS que publique `sensor_msgs/msg/Imu`.

El dato estrictamente necesario para los EKF actuales es la velocidad angular
alrededor del eje vertical del vehículo:

```text
angular_velocity.z, en rad/s, con signo ROS FLU
```

Los EKF no usan actualmente la orientación del Pixhawk ni la aceleración lineal.
Por tanto, la ESP32-S3 no necesita ejecutar un EKF, AHRS, Madgwick o Mahony para
reemplazar esta función. El heading absoluto de la siguiente arquitectura se
obtendrá fuera de esta PCB y no se trata aquí.

## 2. Arquitectura actual

El perfil real vigente arranca MAVROS desde
[`real_global_v2.launch.py`](../../src/navegacion_gps/launch/real_global_v2.launch.py).
La cadena IMU es:

```text
IMU interna del Pixhawk
    -> firmware ArduPilot
    -> mensajes MAVLink
    -> plugin IMU de MAVROS
    -> conversión de unidades y frames
    -> /imu/data (sensor_msgs/msg/Imu)
    -> EKF local y EKF global de robot_localization
```

El launch de sensores habilita el plugin `imu` y remapea el tópico privado
`mavros_node/data` a `/imu/data`:

- [`mavros_sensor_only_pluginlists.yaml`](../../src/sensores/config/mavros_sensor_only_pluginlists.yaml)
- [`mavros.launch.py`](../../src/sensores/launch/mavros.launch.py)
- [`mavros_apm_overrides.yaml`](../../src/sensores/config/mavros_apm_overrides.yaml)

El driver propio
[`pixhawk_driver.py`](../../src/sensores/sensores/pixhawk_driver.py) es código
legacy/de referencia y no es la fuente activa de `real_global_v2`.

## 3. Qué contiene actualmente `/imu/data`

`/imu/data` no es una copia directa de los registros de una IMU física. Es un
mensaje compuesto por MAVROS usando varios mensajes MAVLink.

| Campo ROS 2 | Procedencia actual | Procesamiento previo | ¿Registro crudo? |
|---|---|---|---|
| `header.stamp` | tiempo MAVLink sincronizado por MAVROS | sincronización con el reloj ROS | No |
| `header.frame_id` | parámetro MAVROS | fijado a `imu_link` | No aplica |
| `orientation` | `ATTITUDE` o `ATTITUDE_QUATERNION` del autopiloto | estimación de actitud del Pixhawk y conversión NED/FRD a ENU/FLU | No; es una orientación estimada |
| `angular_velocity` | tasas corporales incluidas en el mensaje de actitud | firmware del autopiloto, escalado MAVLink y conversión FRD a FLU | No es una lectura directa garantizada de registros |
| `linear_acceleration` | último `HIGHRES_IMU`, `RAW_IMU` o `SCALED_IMU` aceptado por MAVROS | selección del flujo, escalado de unidades y conversión FRD a FLU | Es una medición inercial, pero no un valor ADC/registro sin procesar |
| covarianzas | parámetros estáticos de MAVROS | desviaciones configuradas elevadas al cuadrado | No son calculadas muestra a muestra |

La implementación oficial del plugin IMU de MAVROS publica dos contratos:

- `~/data_raw`: acelerómetro y giróscopo, sin orientación válida;
- `~/data`: orientación del autopiloto, tasas angulares y la última aceleración
  disponible.

El proyecto remapea `~/data`, no `~/data_raw`. Véase el
[plugin IMU oficial de MAVROS](https://raw.githubusercontent.com/mavlink/mavros/ros2/mavros/src/plugins/imu.cpp).

### 3.1 Significado de "crudo"

Conviene separar tres niveles:

1. **Registro crudo**: entero con signo leído de los registros del BMI088.
2. **Medición física calibrada**: registro convertido a rad/s o m/s², con
   corrección de bias, escala y ejes.
3. **Estimación fusionada**: orientación calculada combinando sensores y un
   estimador, como el cuaternión entregado por el Pixhawk.

El EKF de ROS2_SALUS necesita el nivel 2 para `angular_velocity.z`. No necesita
el entero de registro ni la orientación fusionada del nivel 3.

### 3.2 Referencia del driver legacy

Aunque no se usa en el perfil real actual, el driver legacy confirma la misma
separación conceptual:

- aceleración y giróscopo desde `SCALED_IMU`/`SCALED_IMU2`;
- orientación desde `ATTITUDE_QUATERNION`;
- conversión de mg a m/s²;
- conversión de mrad/s a rad/s;
- conversión de ejes FRD a FLU.

La lógica está en
[`pixhawk_driver.py`](../../src/sensores/sensores/pixhawk_driver.py), métodos
`_handle_scaled_imu()`, `_handle_attitude_quaternion()` y
`_publish_imu_if_ready()`.

## 4. Qué usa realmente ROS2_SALUS

### 4.1 EKF local

El EKF local recibe `/imu/data`, pero su vector `imu0_config` sólo habilita el
índice 11, correspondiente a `vyaw` o velocidad angular Z:

```yaml
imu0_config: [false, false, false,
              false, false, false,
              false, false, false,
              false, false, true,
              false, false, false]
```

Fuente:
[`localization_v2.yaml`](../../src/navegacion_gps/config/localization_v2.yaml).

No se fusionan desde la IMU:

- roll, pitch o yaw absolutos;
- velocidad angular X o Y;
- aceleración X, Y o Z.

Aunque el YAML conserva `imu0_remove_gravitational_acceleration: true`, esa
opción no actúa sobre el estado actual porque los tres índices de aceleración
están deshabilitados en `imu0_config`.

### 4.2 EKF global

El EKF global tiene la misma selección: solamente `angular_velocity.z`.

Fuente:
[`localization_global_v2.yaml`](../../src/navegacion_gps/config/localization_global_v2.yaml).

Antes de entrar al EKF global, el nodo
[`global_imu_stationary_gate.py`](../../src/navegacion_gps/navegacion_gps/global_imu_stationary_gate.py)
genera `/imu/data_global`. Si la telemetría de ruedas es reciente y la
velocidad medida es menor o igual que `0.03 m/s`, reemplaza las tres velocidades
angulares por cero. Fuera de esa condición, reenvía el mensaje sin modificarlo.

La ESP32-S3 no debe duplicar este gate usando la velocidad del vehículo. Sí debe
corregir el bias propio del giróscopo.

### 4.3 Parámetros temporales relevantes

Los EKF trabajan a 30 Hz y declaran:

```text
sensor_timeout: 0.2 s
imu0_queue_size: 20
```

Un flujo de 100 Hz cumple holgadamente el contrato actual. Una publicación
inferior a 10 Hz no se recomienda, aunque todavía pudiera no superar siempre el
timeout.

### 4.4 Otros consumidores actuales

Hay consumidores de `/imu/data` que leen el cuaternión aunque los EKF no lo
usen, entre ellos dashboards, diagnósticos y el filtro LiDAR opcional. En la
nueva arquitectura deben recibir la orientación desde la fuente correspondiente
o quedar deshabilitados/remapeados. En particular, no deben interpretar como
real el cuaternión placeholder de la BMI088.

Este punto no cambia el requisito del EKF: la PCB BMI088 sólo debe proporcionar
las mediciones inerciales.

## 5. Contrato ROS 2 que debe producir la nueva PCB

### 5.1 Tópico y tipo

Para reemplazar el Pixhawk sin cambiar inicialmente los launches:

```text
Tópico: /imu/data
Tipo:   sensor_msgs/msg/Imu
Frame:  imu_link
Tasa:   100 Hz recomendada
QoS:    SensorDataQoS, best effort, volatile
```

Una migración más limpia según REP-145 sería publicar la BMI088 como
`/imu/data_raw` y pasar ese nombre mediante el argumento `imu_topic`. Mientras
se mantenga el nombre `/imu/data`, el contenido debe documentarse como
"acelerómetro + giróscopo, orientación no disponible".

### 5.2 Campos obligatorios

| Campo | Requisito |
|---|---|
| `header.stamp` | instante de adquisición de la muestra, monotónico y convertido al reloj ROS |
| `header.frame_id` | `imu_link` |
| `orientation` | placeholder numéricamente válido; se recomienda identidad `(0,0,0,1)` |
| `orientation_covariance[0]` | `-1.0`, indicando que no existe orientación |
| `angular_velocity` | XYZ en rad/s, sistema ROS FLU y regla de la mano derecha |
| `angular_velocity_covariance` | varianzas en `(rad/s)²`; no copiar valores arbitrarios del driver legacy |
| `linear_acceleration` | XYZ en m/s², sistema ROS FLU, incluyendo la fuerza específica de gravedad |
| `linear_acceleration_covariance` | varianzas en `(m/s²)²` |

El contrato oficial de
[`sensor_msgs/msg/Imu`](https://docs.ros2.org/latest/api/sensor_msgs/msg/Imu.html)
establece rad/s, m/s² y el valor `-1` en la primera posición de la covarianza
cuando una magnitud no está disponible.

No se debe publicar una identidad con covarianza de orientación positiva: eso
afirmaría incorrectamente que el vehículo siempre tiene orientación cero.

### 5.3 Convención de ejes

ROS usa un frame derecho FLU para el cuerpo del vehículo:

```text
+X: adelante
+Y: izquierda
+Z: arriba
```

Las velocidades angulares siguen la regla de la mano derecha. Visto desde
arriba, una rotación antihoraria debe producir `angular_velocity.z > 0`.

La transformación general es:

```text
omega_ros = R_sensor_a_imu_link * omega_sensor
accel_ros = R_sensor_a_imu_link * accel_sensor
```

`R_sensor_a_imu_link` debe derivarse de la orientación del encapsulado en la PCB
y de la orientación física de la PCB en el vehículo. No se deben fijar signos
por ensayo sin documentar la matriz.

El URDF actual ubica `imu_link` respecto de `base_link` en:

```xml
<origin xyz="0.66 0 0.63" rpy="0 0 0"/>
```

Fuente:
[`cuatri_real_v2.urdf`](../../src/navegacion_gps/models/cuatri_real_v2.urdf).
Si la PCB nueva se monta en otra posición u orientación, se debe actualizar ese
joint. Para la velocidad angular importa principalmente la rotación; la posición
será relevante si en el futuro se fusiona aceleración y se compensa brazo de
palanca.

### 5.4 Convención del acelerómetro

No se debe quitar la gravedad en la ESP32-S3. Según
[REP-145](https://ros.org/reps/rep-0145.html), una IMU quieta, nivelada y con
`+Z` hacia arriba debe publicar aproximadamente:

```text
linear_acceleration.x ~= 0
linear_acceleration.y ~= 0
linear_acceleration.z ~= +9.80665 m/s²
```

Aunque el EKF actual no selecciona la aceleración, conservar esta convención
evita una incompatibilidad futura.

### 5.5 Timestamp

El timestamp debe corresponder a la adquisición, no al momento arbitrario en
que una tarea de baja prioridad consigue transmitir.

Opciones aceptables:

1. **micro-ROS en la ESP32-S3**: sincronizar el reloj con el agente y construir
   `header.stamp` a partir del tiempo sincronizado.
2. **Protocolo binario + nodo bridge en Linux**: transmitir `sensor_time_us` y
   mantener en el bridge una relación entre el reloj monotónico de la ESP32 y el
   reloj ROS.

Para un primer prototipo, el bridge puede sellar la muestra al recibirla, pero
se debe medir el jitter. No debe publicarse repetidamente una muestra antigua
con timestamps nuevos.

## 6. Configuración inicial propuesta del BMI088

Esta es una base de validación para un vehículo terrestre Ackermann; no sustituye
las pruebas de vibración y saturación.

| Función | Configuración inicial | Motivo |
|---|---|---|
| Interfaz | SPI | lectura determinista y margen de tasa |
| Tasa de publicación ROS | 100 Hz | coincide con el modelo de simulación y supera el EKF de 30 Hz |
| ODR giróscopo | 200 Hz | permite filtrar y decimar a 100 Hz |
| BW giróscopo | 23 Hz (`GYRO_BANDWIDTH = 0x04`) | limita vibración de alta frecuencia manteniendo dinámica del vehículo |
| Rango giróscopo | ±250 °/s (`GYRO_RANGE = 0x03`) | buena resolución para un vehículo lento; subir a ±500 °/s si hay saturaciones |
| ODR acelerómetro | 200 Hz | tasa coherente con el giróscopo |
| Filtro acelerómetro | OSR4, 20 Hz (`ACC_CONF = 0x89`) | reduce vibración; validar respuesta real |
| Rango acelerómetro | ±6 g (`ACC_RANGE = 0x01`) | margen inicial razonable para terreno; registrar saturaciones |

El BMI088 soporta rangos programables, ODR y filtros digitales. Los valores y
registros anteriores proceden del
[datasheet oficial BMI088](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi088-ds001.pdf).

No se debe asumir que estos filtros reproducen exactamente el procesamiento de
ArduPilot. La aceptación debe basarse en comparación de bags y comportamiento
del EKF.

## 7. Conversión de registros BMI088

El acelerómetro y el giróscopo entregan enteros de 16 bits en complemento a dos.
Las lecturas XYZ deben hacerse en burst, empezando por el byte LSB, para evitar
combinar bytes de muestras diferentes.

### 7.1 Giróscopo

Para cada eje:

```text
omega_deg_s = raw_int16 / sensitivity_lsb_per_deg_s
omega_rad_s = omega_deg_s * pi / 180
```

Sensibilidades relevantes:

| Rango | Sensibilidad |
|---|---:|
| ±125 °/s | 262.144 LSB/(°/s) |
| ±250 °/s | 131.072 LSB/(°/s) |
| ±500 °/s | 65.536 LSB/(°/s) |
| ±1000 °/s | 32.768 LSB/(°/s) |
| ±2000 °/s | 16.384 LSB/(°/s) |

Después de convertir unidades:

```text
omega_cal = scale_matrix * (omega - bias)
omega_ros = R_sensor_a_imu_link * omega_cal
```

### 7.2 Acelerómetro

Para cada eje:

```text
accel_g = raw_int16 / sensitivity_lsb_per_g
accel_m_s2 = accel_g * 9.80665
```

Sensibilidades:

| Rango | Sensibilidad |
|---|---:|
| ±3 g | 10920 LSB/g |
| ±6 g | 5460 LSB/g |
| ±12 g | 2730 LSB/g |
| ±24 g | 1365 LSB/g |

Después deben aplicarse calibración y rotación de frame, pero no eliminación de
gravedad.

## 8. Inicialización y adquisición en la ESP32-S3

### 8.1 Conexión SPI mínima

El BMI088 contiene interfaces separadas para acelerómetro y giróscopo:

- SCK compartido;
- SDI compartido;
- salida SDO del acelerómetro;
- salida SDO del giróscopo;
- `CSB1` para acelerómetro;
- `CSB2` para giróscopo;
- `PS` a GND para seleccionar SPI;
- al menos una interrupción `data-ready`, preferentemente una por subsensor.

El datasheet admite SPI modo 0 o modo 3 y hasta 10 MHz. Se recomienda comenzar
con 5 MHz y aumentar sólo después de validar integridad de señal.

La alimentación operativa es 2.4–3.6 V para VDD y 1.2–3.6 V para VDDIO; una
implementación ESP32-S3 normalmente usará 3.3 V en ambos. El desacoplo, retorno
de masa, huella y restricciones mecánicas deben seguir el datasheet y la guía de
montaje de Bosch, no valores inferidos de este documento.

### 8.2 Particularidades obligatorias de SPI

- El acelerómetro arranca en modo I²C incluso con `PS` en nivel SPI. Se debe
  generar un flanco ascendente en `CSB1`; Bosch propone una lectura dummy de
  `ACC_CHIP_ID` cuyo resultado se descarta.
- En cada lectura SPI del acelerómetro aparece un byte dummy antes del dato
  válido. Esto no ocurre en el giróscopo.
- Deben respetarse al menos 2 µs entre escrituras en modo normal.
- Después de POR, reset o cambio de modo del giróscopo se deben respetar los
  tiempos de espera del datasheet; después de soft reset son 30 ms.
- Verificar `ACC_CHIP_ID = 0x1E` y `GYRO_CHIP_ID = 0x0F` antes de publicar.

Se recomienda integrar la
[API C oficial BMI08x de Bosch](https://github.com/boschsensortec/BMI08x_SensorAPI)
en ESP-IDF y proporcionar los callbacks SPI/delay de la plataforma, en lugar de
reescribir inicialmente toda la secuencia de registros.

### 8.3 Ciclo de firmware recomendado

```text
Arranque
  -> inicializar alimentación, SPI y GPIO
  -> ejecutar secuencia SPI especial del acelerómetro
  -> leer y validar ambos CHIP_ID
  -> reset/configuración de ambos subsensores
  -> leer de vuelta rango, ODR y filtros
  -> ejecutar self-test en modo de fabricación/servicio
  -> estimar bias de gyro si se confirma reposo
  -> habilitar interrupciones data-ready

Adquisición
  -> ISR data-ready captura tiempo monotónico y despierta una tarea
  -> tarea hace burst read
  -> valida comunicación y saturación
  -> convierte/calibra en la ESP32 o conserva raw para el bridge, según transporte
  -> arma muestra con secuencia y timestamp
  -> transmite o publica

Supervisión
  -> contador de muestras
  -> contador de interrupciones perdidas
  -> errores SPI
  -> saturaciones por eje
  -> temperatura
  -> edad de última muestra
```

Para el requisito actual, el giróscopo marca el tiempo principal de la muestra.
El acelerómetro y el giróscopo del BMI088 tienen caminos independientes. Si en
el futuro se fusiona aceleración a alta precisión, implementar el mecanismo de
[sincronización recomendado por Bosch](https://www.bosch-sensortec.com/media/boschsensortec/downloads/application_notes_1/bst-mis-an006.pdf).

## 9. Calibración

### 9.1 Bias del giróscopo

El bias de Z es el error más importante para este proyecto porque se integra en
yaw. Procedimiento mínimo:

1. Confirmar reposo mediante una condición externa o por varianza baja durante
   varios segundos.
2. Descartar el período de calentamiento inicial.
3. Promediar varios cientos o miles de muestras.
4. Guardar el bias por eje y la temperatura de calibración.
5. Restarlo antes de publicar.
6. No recalibrar durante movimiento sólo porque la velocidad angular sea baja.

Para producción se recomienda caracterizar bias contra temperatura y aplicar
una compensación interpolada. El datasheet especifica variación térmica del
offset del giróscopo; no debe asumirse que una calibración a temperatura ambiente
cubre todo el rango operativo.

### 9.2 Acelerómetro

Aunque hoy no se fusiona, conviene realizar calibración de seis posiciones para
estimar bias y escala por eje. El vector publicado en reposo debe conservar
módulo cercano a `g`.

### 9.3 Covarianzas

Las covarianzas deben medirse con la PCB montada en el vehículo, no copiarse del
Pixhawk ni derivarse únicamente de la resolución de 16 bits.

Procedimiento inicial:

1. Registrar al menos varios minutos en reposo, a temperatura estable.
2. Convertir todas las muestras a SI y al frame `imu_link`.
3. Calcular la varianza por eje después de eliminar el bias medio.
4. Colocar esas varianzas en la diagonal de las matrices ROS.
5. Repetir con motores/controlador energizados para capturar ruido eléctrico y
   vibración real.
6. Usar el escenario peor representativo o parametrizar por plataforma.

Los valores cero significan "covarianza desconocida" en `sensor_msgs/Imu`; son
válidos para el primer bring-up, pero no son el objetivo de producción.

## 10. Transporte ESP32-S3 -> computador ROS 2

Hay dos implementaciones válidas.

### Opción A: micro-ROS

La ESP32-S3 publica directamente `sensor_msgs/msg/Imu` mediante un agente
micro-ROS en la Jetson/Raspberry.

Ventajas:

- contrato ROS directo;
- menos código bridge propio;
- QoS y tipos definidos por ROS.

Requisitos:

- reconexión automática con el agente;
- sincronización del reloj micro-ROS;
- watchdog si el transporte queda bloqueado;
- memoria estática o preasignada en el camino de alta frecuencia.

### Opción B: protocolo binario y bridge ROS 2

La ESP32-S3 envía muestras por USB CDC, UART o Ethernet y un nodo en Linux
publica `/imu/data`.

Paquete binario mínimo recomendado:

```text
magic
protocol_version
payload_length
sequence
sensor_time_us
gyro_raw[3]
accel_raw[3]
temperature_raw
status_flags
crc32
```

Se recomienda transmitir enteros crudos más metadatos y hacer la conversión a
ROS en un único lugar claramente versionado. Alternativamente la ESP32 puede
transmitir valores SI `float32`, pero el protocolo debe declarar unidades y
endianness.

La conversión, calibración y matriz de montaje deben tener un único dueño. Si
se envían enteros crudos, pertenecen al bridge; si micro-ROS publica el mensaje
final, pertenecen al firmware. No deben aplicarse una vez en cada extremo.

El bridge debe:

- rechazar CRC incorrectos;
- detectar saltos de secuencia;
- no republicar la última muestra como si fuera nueva;
- convertir timestamp, unidades y ejes;
- publicar diagnósticos de tasa, latencia, saturación y errores;
- exponer rango, ODR, firmware y matriz de montaje como parámetros o
  diagnóstico.

Para el primer prototipo cableado, USB CDC + bridge ROS 2 suele ser la opción
más simple de depurar y no agrega una dependencia micro-ROS al firmware.

## 11. Ejemplo conceptual de mensaje

Con la PCB quieta y nivelada:

```yaml
header:
  stamp: <instante de adquisición>
  frame_id: imu_link
orientation:
  x: 0.0
  y: 0.0
  z: 0.0
  w: 1.0
orientation_covariance: [-1.0, 0.0, 0.0,
                          0.0, 0.0, 0.0,
                          0.0, 0.0, 0.0]
angular_velocity:
  x: aproximadamente 0.0
  y: aproximadamente 0.0
  z: aproximadamente 0.0
linear_acceleration:
  x: aproximadamente 0.0
  y: aproximadamente 0.0
  z: aproximadamente 9.80665
```

Las matrices de covarianza de gyro y acelerómetro deben completarse con valores
medidos.

## 12. Criterios de aceptación

### 12.1 Pruebas de driver y transporte

- Ambos CHIP_ID coinciden en 100 arranques consecutivos.
- No hay errores CRC/SPI durante una prueba prolongada.
- `/imu/data` mantiene 100 Hz con jitter y latencia medidos.
- Los timestamps son estrictamente crecientes.
- Los saltos de secuencia se detectan y contabilizan.
- Desconectar la PCB detiene las muestras; no aparecen datos congelados con
  timestamps nuevos.
- Temperatura, rango y configuración leída de vuelta quedan disponibles en
  diagnóstico.

### 12.2 Pruebas de unidades y ejes

- En reposo y nivelado, `az` es aproximadamente `+g`.
- Al inclinar cada eje, la dirección del vector de gravedad coincide con FLU.
- Un giro antihorario visto desde arriba produce `wz > 0`.
- Una vuelta manual conocida produce una integral de `wz` con signo y magnitud
  coherentes, descontando bias.
- No hay saturaciones durante las maniobras máximas del vehículo.

### 12.3 Pruebas con ROS 2

Comandos de bring-up:

```bash
ros2 topic info -v /imu/data
ros2 topic hz /imu/data
ros2 topic bw /imu/data
ros2 topic echo --once /imu/data
ros2 run tf2_ros tf2_echo base_link imu_link
```

Validaciones:

- tipo exacto `sensor_msgs/msg/Imu`;
- `frame_id=imu_link` y TF existente;
- orientación marcada como no disponible;
- `angular_velocity` finita y en rad/s;
- EKF local recibe la muestra sin errores de timestamp o transform;
- `/odometry/local` gira en la dirección correcta;
- el stationary gate lleva `/imu/data_global.angular_velocity` a cero sólo con
  telemetría válida de vehículo detenido.

### 12.4 Comparación contra Pixhawk

Antes de retirar definitivamente el Pixhawk, registrar ambos sensores en la
misma prueba:

```text
/imu/pixhawk_reference
/imu/bmi088_candidate
/controller/drive_telemetry
/odometry/local
```

Comparar:

- media y desviación de `wz` en reposo;
- ruido espectral con electrónica energizada;
- respuesta y retardo durante giros;
- integral de yaw en maniobras repetibles;
- temperatura y deriva de bias;
- pérdidas de muestra y latencia.

## 13. Cambios de integración necesarios al retirar el Pixhawk

1. Dejar de iniciar MAVROS como publicador de `/imu/data`, o deshabilitar su
   plugin/remap IMU.
2. Iniciar el agente micro-ROS o el bridge de la PCB antes de los EKF.
3. Garantizar un único publicador efectivo de `/imu/data`.
4. Actualizar el joint `imu_link` del URDF con el montaje real.
5. Configurar los consumidores de orientación para no leer el placeholder de la
   BMI088.
6. Mantener `imu0_config` con sólo `angular_velocity.z` hasta completar una
   validación explícita de cualquier nuevo campo a fusionar.
7. Registrar la configuración BMI088 y versión de firmware en cada bag de
   validación.

## 14. Checklist de implementación

### PCB

- [ ] VDD/VDDIO dentro de especificación y desacopladas según Bosch.
- [ ] `PS` definido por hardware para SPI.
- [ ] CS independientes para acelerómetro y giróscopo.
- [ ] Líneas SDO independientes conectadas correctamente.
- [ ] Interrupciones data-ready accesibles a la ESP32-S3.
- [ ] Ejes del encapsulado y orientación de montaje documentados.
- [ ] Ubicación alejada de fuentes térmicas y vibraciones evitables.

### Firmware ESP32-S3

- [ ] API oficial o secuencia equivalente validada contra datasheet.
- [ ] CHIP_ID, reset, configuración y read-back.
- [ ] Burst reads y byte dummy del acelerómetro implementados.
- [ ] Conversión a SI verificada con tests unitarios.
- [ ] Matriz de ejes documentada y testeada.
- [ ] Bias, temperatura, saturación y errores supervisados.
- [ ] Timestamps de adquisición y contador de secuencia.
- [ ] CRC y reconexión del transporte.
- [ ] Watchdog y estado de fallo seguro.

### ROS 2

- [ ] Un solo publicador de `/imu/data`.
- [ ] `sensor_msgs/msg/Imu`, `imu_link`, 100 Hz.
- [ ] Orientación con `orientation_covariance[0] = -1`.
- [ ] Gyro en rad/s, aceleración en m/s², convención FLU.
- [ ] Covarianzas obtenidas experimentalmente.
- [ ] TF de montaje actualizado.
- [ ] Bag comparativo aprobado antes de eliminar el Pixhawk.

## 15. Referencias

- [Bosch Sensortec, BMI088 Datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi088-ds001.pdf)
- [Bosch Sensortec, BMI08x Sensor API](https://github.com/boschsensortec/BMI08x_SensorAPI)
- [Bosch Sensortec, Data Synchronization for BMI085/BMI088](https://www.bosch-sensortec.com/media/boschsensortec/downloads/application_notes_1/bst-mis-an006.pdf)
- [ROS REP-145, Conventions for IMU Sensor Drivers](https://ros.org/reps/rep-0145.html)
- [ROS 2 `sensor_msgs/msg/Imu`](https://docs.ros2.org/latest/api/sensor_msgs/msg/Imu.html)
- [MAVROS IMU plugin](https://raw.githubusercontent.com/mavlink/mavros/ros2/mavros/src/plugins/imu.cpp)
- [`mavros.launch.py`](../../src/sensores/launch/mavros.launch.py)
- [`localization_v2.yaml`](../../src/navegacion_gps/config/localization_v2.yaml)
- [`localization_global_v2.yaml`](../../src/navegacion_gps/config/localization_global_v2.yaml)
