# Proyecto CSI - Segmentacion Dinamica en el Borde

Este repositorio contiene el avance de un proyecto de reconocimiento continuo de
actividades humanas usando CSI Wi-Fi (Channel State Information) y un ESP32 como
dispositivo Edge.

La idea principal es capturar variaciones del canal Wi-Fi, procesarlas en el
ESP32 y detectar segmentos donde probablemente existe actividad humana. En lugar
de enviar todo el flujo CSI crudo a un servidor, el ESP32 realiza una primera
segmentacion local para reducir datos y preparar una futura fase de
clasificacion.

## Estado Actual

El proyecto se encuentra en el milestone:

```text
M1 - Segmentador Validado
```

Hasta este punto se implemento y valido:

- Captura CSI en ESP32 con ESP-IDF.
- Procesamiento de fase a partir de pares I/Q.
- Varianza deslizante por subportadora.
- Ventana temporal circular de 50 frames.
- Maquina de estados `STATIC` / `DYNAMIC`.
- Histeresis para cierre estable de eventos.
- Timeout maximo de segmento.
- Evaluacion sintetica reproducible.
- Prueba real en ESP32 con monitor serial.

## Hardware y Entorno

Hardware usado en la prueba:

```text
ESP32-D0WD-V3 revision v3.1
```

Entorno:

```text
ESP-IDF v6.0.1
Target: esp32
Sistema probado: Windows / PowerShell
```

## Arquitectura General

```text
Tramas Wi-Fi
    |
    v
Callback CSI del ESP32
    |
    v
Extraccion I/Q
    |
    v
Calculo de fase con atan2f
    |
    v
Varianza deslizante por subportadora
    |
    v
Varianza promedio del entorno
    |
    v
FSM STATIC / DYNAMIC
    |
    v
Segmentos de actividad
```

El firmware trabaja en modo `WIFI_MODE_APSTA`:

- `STA`: el ESP32 puede conectarse a un router local.
- `AP`: el ESP32 crea una red propia llamada `ESP32_CSI_AP`.
- `CSI`: se activa para capturar informacion del canal Wi-Fi.

## Parametros Finales M1

| Parametro | Valor |
|-----------|-------|
| `NUM_SUBCARRIERS` | 64 |
| `WINDOW_SIZE` | 50 |
| `VARIANCE_T_START` | 2.60 |
| `VARIANCE_T_STOP` | 2.50 |
| `HYSTERESIS_SAMPLES` | 10 |
| `MAX_SEGMENT_LENGTH` | 100 |

## Resultados de Evaluacion

La evaluacion sintetica usa flujos CSI continuos y metricas de precision,
recall y F1 basadas en rangos.

| Escenario | Precision | Recall | F1 |
|-----------|-----------|--------|----|
| Flujo normal | 0.7529 | 0.9600 | 0.8440 |
| Actividades cortas con ruido abundante | 0.5187 | 0.9250 | 0.6647 |
| Actividades sostenidas | 0.9000 | 0.9000 | 0.9000 |
| Promedio | 0.7239 | 0.9283 | 0.8029 |

El F1 promedio supera el umbral de validacion de 0.70.

## Evidencia en Hardware

El firmware fue compilado, flasheado y ejecutado en un ESP32 real. El monitor
serial confirmo:

```text
CSI ACTIVO
[TELEM] total=45 (+45/2s)
[TELEM] total=64 (+19/2s) buffer=LISTO
EVENTO DETECTADO
FIN POR TIMEOUT
```

Interpretacion:

- `CSI ACTIVO`: la configuracion CSI fue aceptada por ESP-IDF.
- `total` subiendo: el callback esta recibiendo frames CSI reales.
- `buffer=LISTO`: la ventana de 50 frames se lleno correctamente.
- `EVENTO DETECTADO`: la FSM detecto un segmento dinamico.
- `FIN POR TIMEOUT`: el cierre maximo de segmento funciona.

## Estructura del Repositorio

```text
.
|-- CMakeLists.txt
|-- main/
|   |-- csi_baremetal_edge_segmentation.cpp
|   |-- edge_segmenter.hpp
|   |-- segmentation_metrics.hpp
|   |-- segmentation_test_harness.cpp
|   |-- synthetic_csi_generator.hpp
|   |-- wifi_credentials.example.h
|   `-- CMakeLists.txt
|-- evaluation/
|   |-- segmentation_test.py
|   |-- parameter_search.py
|   `-- debug_variance.py
|-- M1_Results.json
|-- segmentation_evaluation_results.json
|-- idea_proyecto.pdf
|-- sdkconfig
`-- README.md
```

## Archivos Principales

| Archivo | Descripcion |
|---------|-------------|
| `main/csi_baremetal_edge_segmentation.cpp` | Firmware principal para ESP32 |
| `main/edge_segmenter.hpp` | Segmentador portable en C++ |
| `main/segmentation_metrics.hpp` | Metricas range-based |
| `main/synthetic_csi_generator.hpp` | Generador CSI sintetico |
| `evaluation/segmentation_test.py` | Evaluacion principal de M1 |
| `evaluation/parameter_search.py` | Busqueda de parametros |
| `M1_Results.json` | Resultados finales estructurados |

## Configuracion de Credenciales Wi-Fi

Las credenciales reales no se suben al repositorio.

Para compilar localmente con conexion STA, copiar:

```text
main/wifi_credentials.example.h
```

a:

```text
main/wifi_credentials.h
```

y completar:

```cpp
#define STA_SSID "NOMBRE_DE_TU_WIFI"
#define STA_PASS "CLAVE_DE_TU_WIFI"
```

`main/wifi_credentials.h` esta ignorado por Git.

## Compilacion

Desde una terminal con ESP-IDF configurado:

```powershell
idf.py build
```

Si se usa Windows y el entorno ESP-IDF no esta cargado, abrir una terminal de
ESP-IDF o ejecutar el script de exportacion correspondiente antes de compilar.

## Flasheo

Para placas con auto-reset:

```powershell
idf.py -p COM5 flash
```

Cambiar `COM5` por el puerto correspondiente.

Para placas que requieren boton `BOOT`:

```text
1. Mantener BOOT presionado.
2. Pulsar y soltar EN/RST.
3. Seguir manteniendo BOOT.
4. Flashear con --before no-reset.
```

Ejemplo:

```powershell
cd build
python -m esptool --chip esp32 -p COM5 -b 115200 --before no-reset --after hard-reset write-flash --flash-mode dio --flash-freq 40m --flash-size 2MB 0x1000 bootloader\bootloader.bin 0x8000 partition_table\partition-table.bin 0x10000 csi_segmentation.bin
```

## Monitor Serial

```powershell
idf.py -p COM5 monitor
```

o:

```powershell
python -m serial.tools.miniterm COM5 115200 --raw
```

Logs esperados:

```text
CSI ACTIVO
Sistema OPERACIONAL
[TELEM] total=... (+.../2s) var=... buffer=...
```

## Evaluacion Sintetica

Ejecutar:

```powershell
python evaluation\segmentation_test.py
```

Busqueda de parametros:

```powershell
python evaluation\parameter_search.py
```

Diagnostico de varianza:

```powershell
python evaluation\debug_variance.py
```

Los scripts usan solo biblioteca estandar de Python.

## Proximo Paso

El siguiente hito es la integracion Edge-Server:

- Empaquetar segmentos detectados.
- Enviar segmentos mediante MQTT o WebSockets.
- Medir reduccion de ancho de banda frente a envio CSI crudo.
- Recolectar CSI real etiquetado.
- Entrenar o integrar un clasificador de actividades.

## Nota

Los documentos privados de estudio y defensa de la exposicion no forman parte de
este repositorio publico. Este `README.md` resume el estado tecnico del proyecto
para cualquier persona que visite el repositorio.
