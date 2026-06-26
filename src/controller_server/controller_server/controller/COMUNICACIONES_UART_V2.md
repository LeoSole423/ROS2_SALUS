# Comunicaciones Raspberry <-> ESP32 (UART v2)

Este documento describe como funciona la comunicacion entre la Raspberry Pi y la ESP32 en el proyecto `RAPY_ESP32_COMMS`.

## 1. Resumen de arquitectura

- Transporte fisico: UART (`/dev/serial0` en Raspberry).
- Velocidad: `115200` baudios.
- Direccion Pi -> ESP32: comandos de manejo (drive, estop, velocidad, direccion, freno).
- Direccion ESP32 -> Pi: telemetria de control, estado de seguridad y batería.
- Envio continuo Pi -> ESP32: `50 Hz` por defecto (para mantener frames frescos).

## 2. Formato de frames

## 2.1 Pi -> ESP32 (7 bytes)

Formato:

`0xAA | ver_flags | steer_i8 | speed_cmd_u16_le | brake_u8 | crc8`

Campos:

- `0xAA`: header de comando.
- `ver_flags`:
  - nibble alto: version (`2`).
  - bit0: `ESTOP`.
  - bit1: `DRIVE_EN`.
  - bit2: `REV_REQ`.
- `steer_i8`: direccion en `-100..100`.
- `speed_cmd_u16_le`: magnitud de velocidad objetivo en `m/s x100` (little-endian).
  - Ejemplo: `1.60 m/s -> 160`.
  - Comando efectivo: `signed_speed = (REV_REQ ? -1 : +1) * speed_cmd_u16`.
- `brake_u8`: freno `0..100`.
- `crc8`: CRC-8 Dallas/Maxim del frame sin el ultimo byte.

## 2.2 ESP32 -> Pi control (8 bytes)

Formato:

`0x55 | status_flags | speed_meas_u16_le | steer_meas_i16_le | brake_applied_u8 | crc8`

Campos:

- `0x55`: header de telemetria.
- `status_flags`: bits de estado del control.
- `speed_meas_u16_le`: velocidad Hall en `m/s x100`.
- `steer_meas_i16_le`: angulo de direccion centrado en `deg x100`.
- `brake_applied_u8`: freno realmente aplicado (`0..100`).
- `crc8`: CRC-8 Dallas/Maxim.

Sentinels:

- `speed_meas_u16 == 0xFFFF` -> `speed_mps = None`.
- `steer_meas_i16 == -32768` -> `steer_deg = None`.

## 2.3 ESP32 -> Pi batería (8 bytes)

Formato:

`0x56 | battery_flags | battery_cv_u16_le | adc_pin_mv_u16_le | sample_age_ds_u8 | crc8`

Campos:

- `0x56`: header de telemetría de batería.
- `battery_flags`:
  - bit0: `READY`
  - bit1: `FRESH`
  - bit2: `SUSPECT`
  - bit3: `CALIBRATED`
- `battery_cv_u16_le`: voltaje de batería ya calibrado en `V x100`.
- `adc_pin_mv_u16_le`: voltaje real del pin ADC en `mV`.
- `sample_age_ds_u8`: antigüedad de la muestra en decisegundos.
- `crc8`: CRC-8 Dallas/Maxim.

Notas:

- La calibración ADC/divisor vive del lado ESP32, para que ROS2 reciba un voltaje útil directamente.
- ROS2 usa `battery_cv` como dato principal, y `adc_pin_mv` para diagnóstico/recalibración.
- La trama `0x55` anterior no cambia y sigue siendo válida.

## 2.4 status_flags

Bits:

- bit0: `READY`
- bit1: `ESTOP_ACTIVE`
- bit2: `FAILSAFE_ACTIVE`
- bit3: `PI_FRESH`
- bit4-5: `CONTROL_SOURCE`
  - `00 NONE`
  - `01 PI`
  - `10 RC`
  - `11 TEL`
- bit6: `OVERSPEED_ACTIVE`
- bit7: reservado

## 3. Comportamiento de seguridad

- Al iniciar el cliente en Raspberry, el estado arranca seguro:
  - `drive=off`, `estop=off`, `speed=0`, `steer=0`, `brake=0`.
- Al cerrar, se envian 3 frames finales en estado seguro.
- `estop on` fuerza frenado fuerte en ESP32 (telemetria reporta `ESTOP_ACTIVE` y freno alto).
- Si `drive off`, la fuente efectiva deja de ser `PI` y pasa a `NONE` (o la que arbitre firmware).

## 4. Parser y robustez RX en Raspberry

El parser de telemetria en Raspberry:

- usa buffer incremental,
- busca headers `0x55` y `0x56`,
- valida CRC antes de aceptar un frame,
- resincroniza descartando bytes solo cuando hay desalineacion/CRC invalido.

Metricas expuestas en `stats`:

- `rx_frames_ok`
- `rx_control_frames_ok`
- `rx_battery_frames_ok`
- `rx_crc_errors`
- `rx_parse_drops`

Nota: `rx_parse_drops=0` y `rx_crc_errors=0` en pruebas E2E recientes.

## 5. Comandos de la CLI interactiva

Comando de arranque:

```bash
cd /home/salus/codigo/RAPY_ESP32_COMMS
source .venv/bin/activate
python -m controller_server.rpy_esp32_comms
```

Comandos disponibles dentro del prompt `rpy>`:

- `help`: muestra ayuda.
- `status`: estado deseado + ultima telemetria de control + última batería + estadísticas.
- `drive on|off`: habilita/deshabilita mando de traccion desde Pi.
- `estop on|off`: activa/desactiva parada de emergencia.
- `speed <mps>`: setpoint firmado de velocidad (ej: `speed 1.2` o `speed -0.8`).
- `steer <int -100..100>`: direccion en porcentaje.
- `brake <0..100>`: freno deseado.
- `watch on|off`: imprime telemetria periódica de control y batería.
- `log on|off`: imprime frames TX/RX en consola.
- `quit`: salida segura.

## 6. Flags de arranque de la app

```bash
python -m controller_server.rpy_esp32_comms \
  --port /dev/serial0 \
  --baud 115200 \
  --tx-hz 50 \
  --telemetry-print-hz 10 \
  --max-reverse-mps 1.30 \
  --log-file /home/salus/codigo/RAPY_ESP32_COMMS/session.jsonl
```

Descripcion:

- `--port`: puerto serie.
- `--baud`: baudios UART.
- `--tx-hz`: frecuencia de envio Pi->ESP32.
- `--telemetry-print-hz`: frecuencia de impresion al usar `watch on`.
- `--max-reverse-mps`: magnitud maxima permitida para comando de reversa (`speed<0`).
- `--log-file`: guarda eventos/telemetria en JSONL.

## 7. Ejemplo de secuencia de manejo

En la CLI:

```text
rpy> drive on
rpy> speed 1.0
rpy> steer 25
rpy> brake 30
rpy> brake 0
rpy> estop on
rpy> estop off
rpy> drive off
rpy> quit
```

## 8. Diagnostico rapido

1. Verificar puerto y permisos:

```bash
ls -l /dev/serial0
groups
```

2. Ver estado en vivo:

```text
rpy> status
rpy> watch on
```

3. Si no hay telemetria:

- revisar cableado TX/RX/GND,
- confirmar firmware ESP32 en protocolo v2,
- confirmar baudios iguales (`115200`),
- revisar que no haya otro proceso usando el puerto.

## 9. Referencias de codigo

- Protocolo: `controller_server/rpy_esp32_comms/protocol.py`
- Transporte serial e hilos TX/RX: `controller_server/rpy_esp32_comms/transport.py`
- CLI: `controller_server/rpy_esp32_comms/cli.py`
- Modelo de telemetria: `controller_server/rpy_esp32_comms/telemetry.py`
- Estado de comando: `controller_server/rpy_esp32_comms/controller.py`
