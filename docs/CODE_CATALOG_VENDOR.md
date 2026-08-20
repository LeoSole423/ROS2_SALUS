# Catálogo del código vendorizado RoboSense

Estado: auditado línea por línea contra `main` local el 2026-08-16.

Alcance: 138 archivos de texto: 8 de `rslidar_msg` y 130 de `rslidar_sdk`. Incluye código, configuración, build ROS/CMake/Visual Studio, tests, licencias/changelogs relevantes, el README superior, el helper DDS y dos CMake generados presentes en source. El manifiesto de procedencia apunta a `https://github.com/RoboSense-LiDAR/rslidar_msg` y `https://github.com/RoboSense-LiDAR/rslidar_sdk.git` sin commit/tag fijado; por eso la copia local es la fuente reproducible real.

Regla: este es código de tercero. SALUS lo configura para RS16, pero no debe modificarse para resolver wiring, QoS o filtros propios salvo una tarea explícita con prueba upstream-compatible.

## 1. `rslidar_msg` — 8 archivos

| Archivo | Responsabilidad |
|---|---|
| `src/rslidar_msg/msg/RslidarPacket.msg` | Paquete UDP LiDAR serializado como bytes más timestamp. |
| `src/rslidar_msg/LICENSE` | Licencia BSD de la copia vendorizada. |
| `src/rslidar_msg/CMakeLists.txt` | Selector/build superior compatible con ROS. |
| `src/rslidar_msg/package.xml` | Manifiesto superior. |
| `src/rslidar_msg/ros1/CMakeLists.txt` | Generación del mensaje en catkin. |
| `src/rslidar_msg/ros1/package.xml` | Dependencias ROS 1. |
| `src/rslidar_msg/ros2/CMakeLists.txt` | Generación rosidl ROS 2. |
| `src/rslidar_msg/ros2/package.xml` | Dependencias ROS 2. |

## 2. Integración ROS del SDK

| Archivo | Responsabilidad |
|---|---|
| `src/rslidar_sdk/CMakeLists.txt` | Build del nodo SDK según ROS 1/2 y librería driver embebida. |
| `src/rslidar_sdk/package.xml` | Dependencias ROS del paquete. |
| `src/rslidar_sdk/config/config.yaml` | Config upstream de source/destinations y uno o más lidars. SALUS normalmente usa `sensores/config/rs16.yaml`. |
| `src/rslidar_sdk/node/rslidar_sdk_node.cpp` | `main`: init ROS, carga YAML, inicia/detiene `NodeManager` y maneja señales. |
| `src/rslidar_sdk/src/manager/node_manager.hpp` | Contrato lifecycle del manager. |
| `src/rslidar_sdk/src/manager/node_manager.cpp` | Construye sources/destinations según YAML y registra packet/cloud callbacks. |
| `src/rslidar_sdk/src/source/source.hpp` | Interfaces base Source, DestinationPacket y DestinationPointCloud. |
| `src/rslidar_sdk/src/source/source_driver.hpp` | Adaptador del `LidarDriver`: parámetros, colas, callbacks de packet/cloud/IMU y lifecycle. |
| `src/rslidar_sdk/src/source/source_packet_ros.hpp` | Entrada/salida de packets ROS 1/ROS 2 y compatibilidad legacy. |
| `src/rslidar_sdk/src/source/source_pointcloud_ros.hpp` | Conversión de cloud/IMU interna a mensajes ROS y publisher destination. |
| `src/rslidar_sdk/src/utility/common.hpp` | Macros/tipos comunes por versión de ROS. |
| `src/rslidar_sdk/src/utility/yaml_reader.hpp` | Lectura tipada de configuración YAML. |
| `src/rslidar_sdk/launch/elequent_start.py` | Launch para ROS 2 Eloquent. |
| `src/rslidar_sdk/launch/humble_start.py` | Launch Humble del nodo/params/RViz. |
| `src/rslidar_sdk/launch/start.py` | Launch genérico compatible con la detección upstream. |
| `src/rslidar_sdk/launch/start.launch` | Launch XML legacy ROS 1 del nodo y RViz. |
| `src/rslidar_sdk/rviz/rviz.rviz` | Viewer ROS 1. |
| `src/rslidar_sdk/rviz/rviz2.rviz` | Viewer ROS 2. |
| `src/rslidar_sdk/create_debian.sh` | Script upstream de empaquetado Debian. |
| `src/rslidar_sdk/doc/howto/script/dds_mod.sh` | Ajuste upstream de configuración DDS. |

## 3. Tipos de mensajes internos/ROS

| Archivo | Responsabilidad |
|---|---|
| `src/rslidar_sdk/src/msg/ros_msg/RsCompressedImage.msg` | Imagen comprimida específica upstream. |
| `src/rslidar_sdk/src/msg/ros_msg/RslidarPacket.msg` | Definición packet interna del SDK. |
| `src/rslidar_sdk/src/msg/ros_msg/rs_compressed_image.hpp` | Adaptadores de imagen comprimida ROS. |
| `src/rslidar_sdk/src/msg/ros_msg/rslidar_packet.hpp` | Adaptador packet moderno. |
| `src/rslidar_sdk/src/msg/ros_msg/rslidar_packet_legacy.hpp` | Adaptador packet legacy. |
| `src/rslidar_sdk/src/msg/ros_msg/rslidar_scan_legacy.hpp` | Adaptador scan legacy. |
| `src/rslidar_sdk/src/msg/rs_msg/lidar_point_cloud_msg.hpp` | Alias/configuración del tipo de point cloud usado por el SDK. |

## 4. Núcleo `rs_driver`

### API, estado y parámetros

| Archivo | Responsabilidad |
|---|---|
| `src/rslidar_sdk/src/rs_driver/CMakeLists.txt` | Build raíz de la librería driver. |
| `src/rslidar_sdk/src/rs_driver/src/rs_driver/api/lidar_driver.hpp` | API pública del driver. |
| `src/rslidar_sdk/src/rs_driver/src/rs_driver/driver/lidar_driver_impl.hpp` | Implementación lifecycle, input→decoder, callbacks y split de frames. |
| `src/rslidar_sdk/src/rs_driver/src/rs_driver/driver/driver_param.hpp` | Parámetros de input/decoder/lidar. |
| `src/rslidar_sdk/src/rs_driver/src/rs_driver/common/error_code.hpp` | Códigos/categorías de error. |
| `src/rslidar_sdk/src/rs_driver/src/rs_driver/common/rs_common.hpp` | Endianness, tiempo y helpers comunes. |
| `src/rslidar_sdk/src/rs_driver/src/rs_driver/common/rs_log.hpp` | Macros de logging. |
| `src/rslidar_sdk/src/rs_driver/src/rs_driver/macro/version.hpp` | Versión compilada del driver. |

### Decode común

| Archivo | Responsabilidad |
|---|---|
| `src/rslidar_sdk/src/rs_driver/src/rs_driver/driver/decoder/basic_attr.hpp` | Atributos básicos de modelos/packets. |
| `src/rslidar_sdk/src/rs_driver/src/rs_driver/driver/decoder/block_iterator.hpp` | Iteradores de bloques/retornos. |
| `src/rslidar_sdk/src/rs_driver/src/rs_driver/driver/decoder/chan_angles.hpp` | Calibración/interpolación de ángulos por canal. |
| `src/rslidar_sdk/src/rs_driver/src/rs_driver/driver/decoder/decoder.hpp` | Base de decoder, thresholds y callbacks. |
| `src/rslidar_sdk/src/rs_driver/src/rs_driver/driver/decoder/decoder_factory.hpp` | Selecciona decoder por `LidarType`. |
| `src/rslidar_sdk/src/rs_driver/src/rs_driver/driver/decoder/decoder_mech.hpp` | Base para lidars mecánicos. |
| `src/rslidar_sdk/src/rs_driver/src/rs_driver/driver/decoder/member_checker.hpp` | Detección compile-time de miembros opcionales del point type. |
| `src/rslidar_sdk/src/rs_driver/src/rs_driver/driver/decoder/section.hpp` | Ventanas angulares/rangos de sección. |
| `src/rslidar_sdk/src/rs_driver/src/rs_driver/driver/decoder/split_strategy.hpp` | Estrategias de corte de frame. |
| `src/rslidar_sdk/src/rs_driver/src/rs_driver/driver/decoder/trigon.hpp` | Tablas trigonométricas rápidas. |

### Decoders por modelo

Cada archivo implementa layout de packet, calibración, distancia, timestamp y retornos del modelo indicado:

- `src/rslidar_sdk/src/rs_driver/src/rs_driver/driver/decoder/decoder_RS16.hpp`
- `src/rslidar_sdk/src/rs_driver/src/rs_driver/driver/decoder/decoder_RS32.hpp`
- `src/rslidar_sdk/src/rs_driver/src/rs_driver/driver/decoder/decoder_RS48.hpp`
- `src/rslidar_sdk/src/rs_driver/src/rs_driver/driver/decoder/decoder_RS80.hpp`
- `src/rslidar_sdk/src/rs_driver/src/rs_driver/driver/decoder/decoder_RS128.hpp`
- `src/rslidar_sdk/src/rs_driver/src/rs_driver/driver/decoder/decoder_RSBP.hpp`
- `src/rslidar_sdk/src/rs_driver/src/rs_driver/driver/decoder/decoder_RSHELIOS.hpp`
- `src/rslidar_sdk/src/rs_driver/src/rs_driver/driver/decoder/decoder_RSHELIOS_16P.hpp`
- `src/rslidar_sdk/src/rs_driver/src/rs_driver/driver/decoder/decoder_RSAIRY.hpp`
- `src/rslidar_sdk/src/rs_driver/src/rs_driver/driver/decoder/decoder_RSE1.hpp`
- `src/rslidar_sdk/src/rs_driver/src/rs_driver/driver/decoder/decoder_RSM1.hpp`
- `src/rslidar_sdk/src/rs_driver/src/rs_driver/driver/decoder/decoder_RSM1_Jumbo.hpp`
- `src/rslidar_sdk/src/rs_driver/src/rs_driver/driver/decoder/decoder_RSM2.hpp`
- `src/rslidar_sdk/src/rs_driver/src/rs_driver/driver/decoder/decoder_RSM3.hpp`
- `src/rslidar_sdk/src/rs_driver/src/rs_driver/driver/decoder/decoder_RSMX.hpp`
- `src/rslidar_sdk/src/rs_driver/src/rs_driver/driver/decoder/decoder_RSP48.hpp`
- `src/rslidar_sdk/src/rs_driver/src/rs_driver/driver/decoder/decoder_RSP80.hpp`
- `src/rslidar_sdk/src/rs_driver/src/rs_driver/driver/decoder/decoder_RSP128.hpp`

SALUS selecciona RS16. Los demás decoders están presentes porque el SDK es multiproducto, no porque el robot use esos sensores.

### Inputs

| Archivo | Responsabilidad |
|---|---|
| `src/rslidar_sdk/src/rs_driver/src/rs_driver/driver/input/input.hpp` | Base común de input y callbacks. |
| `src/rslidar_sdk/src/rs_driver/src/rs_driver/driver/input/input_factory.hpp` | Selecciona socket/pcap/raw y variante jumbo. |
| `src/rslidar_sdk/src/rs_driver/src/rs_driver/driver/input/input_pcap.hpp` | Replay PCAP. |
| `src/rslidar_sdk/src/rs_driver/src/rs_driver/driver/input/input_pcap_jumbo.hpp` | Replay PCAP jumbo. |
| `src/rslidar_sdk/src/rs_driver/src/rs_driver/driver/input/input_raw.hpp` | Inyección de packets raw. |
| `src/rslidar_sdk/src/rs_driver/src/rs_driver/driver/input/input_raw_jumbo.hpp` | Raw jumbo. |
| `src/rslidar_sdk/src/rs_driver/src/rs_driver/driver/input/input_sock.hpp` | Input UDP normal. |
| `src/rslidar_sdk/src/rs_driver/src/rs_driver/driver/input/input_sock_jumbo.hpp` | UDP jumbo. |
| `src/rslidar_sdk/src/rs_driver/src/rs_driver/driver/input/jumbo.hpp` | Ensamblado/fragmentación jumbo. |
| `src/rslidar_sdk/src/rs_driver/src/rs_driver/driver/input/unix/input_sock_epoll.hpp` | Backend Linux epoll. |
| `src/rslidar_sdk/src/rs_driver/src/rs_driver/driver/input/unix/input_sock_select.hpp` | Backend Unix select. |
| `src/rslidar_sdk/src/rs_driver/src/rs_driver/driver/input/win/input_sock_select.hpp` | Backend Windows select. |

### Mensajes y utilidades del driver

| Archivo | Responsabilidad |
|---|---|
| `src/rslidar_sdk/src/rs_driver/src/rs_driver/msg/imu_data_msg.hpp` | Tipo IMU interno. |
| `src/rslidar_sdk/src/rs_driver/src/rs_driver/msg/packet.hpp` | Buffer/tipo de packet y metadatos. |
| `src/rslidar_sdk/src/rs_driver/src/rs_driver/msg/pcl_point_cloud_msg.hpp` | Adaptación opcional PCL. |
| `src/rslidar_sdk/src/rs_driver/src/rs_driver/msg/point_cloud_msg.hpp` | Point cloud genérica del driver. |
| `src/rslidar_sdk/src/rs_driver/src/rs_driver/utility/buffer.hpp` | Buffer reutilizable para input/decode. |
| `src/rslidar_sdk/src/rs_driver/src/rs_driver/utility/dbg.hpp` | Instrumentación/debug condicional. |
| `src/rslidar_sdk/src/rs_driver/src/rs_driver/utility/sync_queue.hpp` | Cola sincronizada entre threads. |

## 5. Demos y herramientas upstream

| Archivo | Responsabilidad |
|---|---|
| `src/rslidar_sdk/src/rs_driver/demo/CMakeLists.txt` | Build de demos. |
| `src/rslidar_sdk/src/rs_driver/demo/demo_online.cpp` | Captura online de un LiDAR. |
| `src/rslidar_sdk/src/rs_driver/demo/demo_online_multi_lidars.cpp` | Captura concurrente de múltiples LiDAR. |
| `src/rslidar_sdk/src/rs_driver/demo/demo_pcap.cpp` | Decode desde PCAP. |
| `src/rslidar_sdk/src/rs_driver/tool/CMakeLists.txt` | Build de utilidades. |
| `src/rslidar_sdk/src/rs_driver/tool/rs_driver_pcdsaver.cpp` | Guarda clouds como PCD. |
| `src/rslidar_sdk/src/rs_driver/tool/rs_driver_viewer.cpp` | Viewer PCL del driver. |

## 6. Tests upstream

`src/rslidar_sdk/src/rs_driver/test/CMakeLists.txt` compila los siguientes tests:

- Iteradores/retornos: `src/rslidar_sdk/src/rs_driver/test/ab_dual_return_block_iterator_test.cpp`, `src/rslidar_sdk/src/rs_driver/test/dual_return_block_iterator_test.cpp`, `src/rslidar_sdk/src/rs_driver/test/single_return_block_iterator_test.cpp`, `src/rslidar_sdk/src/rs_driver/test/rs16_dual_return_block_iterator_test.cpp` y `src/rslidar_sdk/src/rs_driver/test/rs16_single_return_block_iterator_test.cpp`.
- Decode: `src/rslidar_sdk/src/rs_driver/test/decoder_test.cpp`, `src/rslidar_sdk/src/rs_driver/test/decoder_rs16_test.cpp`, `src/rslidar_sdk/src/rs_driver/test/decoder_rs32_test.cpp` y `src/rslidar_sdk/src/rs_driver/test/decoder_rsbp_test.cpp`.
- Atributos/calibración: `src/rslidar_sdk/src/rs_driver/test/basic_attr_test.cpp` y `src/rslidar_sdk/src/rs_driver/test/chan_angles_test.cpp`.
- Infraestructura: `src/rslidar_sdk/src/rs_driver/test/buffer_test.cpp`, `src/rslidar_sdk/src/rs_driver/test/dbg_test.cpp`, `src/rslidar_sdk/src/rs_driver/test/rs_common_test.cpp`, `src/rslidar_sdk/src/rs_driver/test/rs_driver_test.cpp`, `src/rslidar_sdk/src/rs_driver/test/section_test.cpp`, `src/rslidar_sdk/src/rs_driver/test/split_strategy_test.cpp`, `src/rslidar_sdk/src/rs_driver/test/sync_queue_test.cpp` y `src/rslidar_sdk/src/rs_driver/test/trigon_test.cpp`.

## 7. Build, metadata y plataformas auxiliares

| Archivo o grupo | Responsabilidad |
|---|---|
| `src/rslidar_sdk/.clang-format` | Estilo C++ del SDK superior. |
| `src/rslidar_sdk/.gitignore` | Exclusiones de build/generados del SDK superior. |
| `src/rslidar_sdk/.gitmodules` | Referencia upstream del submódulo `rs_driver`. |
| `src/rslidar_sdk/README.md` | Contrato de uso e integración superior. |
| `src/rslidar_sdk/CHANGELOG.md` | Historial/versionado del SDK superior. |
| `src/rslidar_sdk/LICENSE` | Licencia BSD del SDK superior. |
| `src/rslidar_sdk/src/rs_driver/.clang-format` | Estilo C++ de la librería driver embebida. |
| `src/rslidar_sdk/src/rs_driver/.gitignore` | Exclusiones de build/generados del driver. |
| `src/rslidar_sdk/src/rs_driver/CHANGELOG.md` | Historial y versión de la librería driver. |
| `src/rslidar_sdk/src/rs_driver/LICENSE` | Licencia BSD de la librería driver. |
| `src/rslidar_sdk/src/rs_driver/cmake/FindPCAP.cmake` | Descubrimiento de libpcap. |
| `src/rslidar_sdk/src/rs_driver/cmake/cmake_uninstall.cmake.in` | Plantilla de desinstalación CMake. |
| `src/rslidar_sdk/src/rs_driver/cmake/rs_driverConfig.cmake.in` | Plantilla exportable de package config. |
| `src/rslidar_sdk/src/rs_driver/cmake/rs_driverConfigVersion.cmake.in` | Plantilla exportable de package version. |
| `src/rslidar_sdk/src/rs_driver/cmake/rs_driverConfig.cmake` | Copia generada presente en source; conserva rutas absolutas del entorno que la produjo. |
| `src/rslidar_sdk/src/rs_driver/cmake/rs_driverConfigVersion.cmake` | Copia generada de compatibilidad de versión. |
| `src/rslidar_sdk/src/rs_driver/src/rs_driver/macro/version.hpp.in` | Plantilla del header de versión. |
| `src/rslidar_sdk/src/rs_driver/win/rs_driver.sln` | Solución Visual Studio. |
| `src/rslidar_sdk/src/rs_driver/win/demo_online/demo_online.vcxproj` | Proyecto Windows del demo UDP online. |
| `src/rslidar_sdk/src/rs_driver/win/demo_pcap/demo_pcap.vcxproj` | Proyecto Windows del demo PCAP. |
| `src/rslidar_sdk/src/rs_driver/win/rs_driver_viewer/rs_driver_viewer.vcxproj` | Proyecto Windows del viewer PCL. |

## 8. Cobertura

Los 138 archivos vendor del alcance están fijados individualmente en `BACKEND_LINE_MANIFEST.tsv`; este catálogo describe su responsabilidad por ruta o grupo homogéneo. Se excluyeron manuales Markdown upstream, README en chino e imágenes de documentación porque no son lógica ejecutable ni configuración del backend. No se confundieron los hallazgos vendor con código propio SALUS y no se aplicaron parches al tercero.
