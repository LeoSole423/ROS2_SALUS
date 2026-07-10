# Jetson Power Monitor V2

`jetson_power_monitor.py` corre fuera de Docker y registra la alimentacion propia de la Jetson usando `ina3221`, mas contexto termico y de potencia desde `tegrastats`.

## Captura

Perfiles soportados:

- `aggressive`
- `moderate`

Default operativo recomendado:

- fast INA/hwmon: `100 Hz`
- contexto `tegrastats`: `2 Hz`
- segmentos de `10 s`
- `fdatasync` rapido cada `100 ms`

## Archivos

- `log/jetson_power_monitor/fast-<boot_id>-000001.jsonl`
- `log/jetson_power_monitor/context-<boot_id>-000001.jsonl`
- `log/jetson_power_monitor/events-<boot_id>.jsonl`
- `log/jetson_power_monitor/analysis-<boot_id>.json`

## Eventos

- `monitor_profile_aggressive`
- `monitor_started`
- `vdd_in_low`
- `vdd_in_critical`
- `fast_voltage_drop`
- `tegrastats_missing`
- `operator_marker`
- `corrupt_tail_detected`
- `abrupt_session_end_detected`
- `post_reboot_analysis`
- `monitor_stopped_cleanly`

## Uso manual

Desde el host Jetson:

```bash
cd /home/admin/Desktop/SALUS/ROS2_SALUS
python3 tools/jetson_power_monitor.py --profile aggressive
```

Prueba corta:

```bash
python3 tools/jetson_power_monitor.py --profile aggressive --max-samples 200
python3 tools/jetson_power_report.py
```

## Markers operativos

```bash
python3 tools/jetson_power_mark.py --label steering_test_start
python3 tools/jetson_power_mark.py --label steering_left
python3 tools/jetson_power_mark.py --label steering_right
```

## Ver diagnostico

```bash
python3 tools/jetson_power_report.py
python3 tools/jetson_power_report.py --list-boots
python3 tools/jetson_power_report.py --boot-id <boot_id>
```

## Instalar como servicio

```bash
cd /home/admin/Desktop/SALUS/ROS2_SALUS
./tools/install_jetson_power_monitor_service.sh
```

La unidad `systemd` usa:

```bash
/home/admin/Desktop/SALUS/ROS2_SALUS/tools/run_jetson_power_monitor.sh --profile aggressive
```

## Nota de interpretacion

Aunque el robot alimente la Jetson por jack DC a `19 V`, este monitor ve el rail interno exportado por la placa, no la linea de `19 V` directamente.

Si el reporte indica:

- `internal_rail_drop_suspected`: hubo evidencia visible en el rail interno monitoreado
- `abrupt_reset_internal_rail_stable`: la Jetson reinicio abruptamente pero el rail interno visible se mantuvo estable en las ultimas muestras validas
- `monitor_gap_unknown`: faltaron muestras o el arranque/cierre no fue suficiente para concluir
