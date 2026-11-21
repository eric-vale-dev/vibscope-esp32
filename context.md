# Analizador de vibraciones portátil (ESP32 + MPU9250 + Dashboard web)

> Documento de contexto y requerimientos 
> Idioma del proyecto: español. Comentarios de código, README y textos de la interfaz en español.

---

## 1. Resumen del proyecto

Se debe construir la **emulación de un prototipo de analizador de vibraciones** similar a los equipos portátiles de monitoreo de condición (referencia: SKF), que se visualiza desde un **dashboard HTML en un celular**.

- **Hardware:** ESP32 DevKit de 38 pines + acelerómetro MPU9250.
- **Firmware:** Arduino IDE (C++ para ESP32).
- **Dashboard:** solo **HTML + CSS + JavaScript puro** (sin frameworks, sin bundlers, sin CDN).
- **Análisis posterior:** exportar datos a CSV para analizarlos con **Matlab**.
- **Validación:** sobre un prototipo de **marco rígido tipo VFACA**.
- **Fecha de presentación indicada por el profesor:** 18/11/2026 (confirmar si hubo cambio de fecha).
- **Evidencias:** entrega individual por Teams (capturas, fotos, video corto).

### Referencias del profesor
- https://www.skf.com/mx/products/condition-monitoring-systems/portable-systems
- https://www.youtube.com/watch?v=1xVvRJSIPAQ
- https://www.youtube.com/watch?v=wd3xawxXJuE

### Objetivo técnico central
1. Leer aceleración en 3 ejes del MPU9250 desde el ESP32 y registrarla.
2. Guardar los datos para analizarlos en Matlab, **o bien** graficar el espectro de la señal en vivo mientras se registran los valores (se implementan ambos).

---

## 2. Restricciones y decisiones ya tomadas

> **Actualización (implementación actual):** el transporte es **USB serie con Web Serial API** (Chrome/Edge),
> no WiFi/WebSocket. Se descartan por ahora el modo AP, `dashboard_html.h`, la vista móvil y el semáforo.
> El protocolo JSON de la sección 4.3 se reemplazó por el protocolo binario descrito abajo, porque a 1 kHz el
> texto (~30 KB/s) no cabe en 115200 baudios. Estructura real: `arduino/analizador_vibraciones/`, `web/`, `matlab/`.
> Firmware sin librerías (acceso directo a registros del MPU). El CSV se exporta sin encabezado: `t_s,ax_g,ay_g,az_g`.
>
> **Protocolo binario ESP32 → PC** (921600 baudios, little-endian):
> `0xAA 0x55 | tipo(u8) | largo(u16) | carga | checksum(u8 = XOR de tipo..carga)`
> - tipo 1 (datos): `fs(u16) | fs_real×10(u16) | rango_g(u8) | reservado(u8) | perdidas(u16) | t0_ms(u32) | N×[ax,ay,az int16]`; g = valor / (32768 / rango_g).
> - tipo 2 (texto ASCII): mensajes de estado (`LISTO`, `OK ...`, `ERROR ...`, `INFO ...`).
>
> **Comandos PC → ESP32** (texto + `\n`): `START`, `STOP`, `CAL` (conserva gravedad), `CAL DC`, `SET fs=1000 range=4`, `INFO`.
> fs válidas: 250/500/1000 Hz. Rangos: 2/4/8/16 g.

| Tema | Decisión |
|---|---|
| Entorno de desarrollo | **Fedora Linux** (Arduino IDE 2.x) |
| Entorno de prueba del profesor | **Windows** → todo debe ser **multiplataforma** |
| Dashboard | Solo HTML/CSS/JS, fácil de abrir y leer. Sin npm, sin frameworks, sin CDN |
| Conectividad | El **ESP32 crea su propia red WiFi (modo AP)** y sirve el dashboard. El celular se conecta a esa red y abre la IP del ESP32. No requiere internet |
| Tiempo real | **WebSocket** ESP32 → navegador |
| Cálculo de FFT | **En el navegador (JS)**, no en el ESP32, para no cargar el microcontrolador |
| Gráficas | `<canvas>` dibujado a mano (no Chart.js ni librerías externas, porque no hay internet en la red del ESP32) |
| Exportación | CSV generado en el navegador (botón de descarga) |
| Entrega del HTML al ESP32 | Embebido en el firmware como cadena en PROGMEM (`dashboard_html.h`), para no depender del plugin de subida de LittleFS, que cambia entre sistemas operativos |

---

## 3. Hardware

- ESP32 DevKit 38 pines.
- MPU9250 por I2C, **3.3 V** (nunca 5 V).
- Conexiones:

| MPU9250 | ESP32 |
|---|---|
| VCC | 3V3 |
| GND | GND |
| SDA | GPIO21 |
| SCL | GPIO22 |

- I2C a 400 kHz (`Wire.setClock(400000)`).
- Alimentación por USB o power bank.
- Sensor fijado rígidamente al marco (tornillo, base magnética o adhesivo duro). Documentar la orientación de los ejes respecto al marco.

---

## 4. Requerimientos funcionales

### 4.1 Firmware (Arduino IDE)

| ID | Requerimiento |
|---|---|
| RF-01 | Inicializar el MPU9250 con rango configurable (±2 / ±4 / ±8 / ±16 g; por defecto ±4 g) y filtro digital pasa-bajas (DLPF) acorde a la frecuencia de muestreo |
| RF-02 | Muestrear X, Y, Z a frecuencia fija. Meta: **1000 Hz** (ancho de banda útil ≈ 500 Hz). Frecuencia configurable (por ejemplo 250 / 500 / 1000 Hz) |
| RF-03 | Temporización estable con timer de hardware o `micros()`; **prohibido `delay()`** en el ciclo de muestreo. Reportar la frecuencia real medida |
| RF-04 | Acumular muestras en bloques (512 o 1024 por eje) y enviarlas en lote; **no** enviar muestra por muestra |
| RF-05 | Modo WiFi AP con SSID y contraseña definidos al inicio del código (constantes fáciles de cambiar). IP típica `192.168.4.1` |
| RF-06 | Servidor HTTP que entregue el dashboard en `/` y servidor WebSocket para datos y comandos |
| RF-07 | Calibración: promediar N muestras en reposo y restar el offset; la gravedad en el eje vertical se maneja como opción (quitar DC o conservarla) |
| RF-08 | Salida serie opcional en CSV (`t_ms,ax,ay,az`) a 115200 baudios para depuración y respaldo |
| RF-09 | Separar tareas con FreeRTOS si es necesario: muestreo en un núcleo, red en el otro, para no perder muestras |

### 4.2 Dashboard (HTML + CSS + JS)

| ID | Requerimiento |
|---|---|
| RD-01 | Archivos fuente separados en `web/`: `index.html`, `style.css`, `app.js` (luego se empaquetan en `dashboard_html.h`) |
| RD-02 | Diseño responsive, **mobile-first**; usable en vertical y horizontal. Modo oscuro recomendado |
| RD-03 | Gráfica de aceleración vs. tiempo, 3 ejes superpuestos con colores distintos y leyenda |
| RD-04 | Gráfica de **espectro FFT** (amplitud vs. frecuencia, eje X en Hz), con selector de eje (X / Y / Z) y ventana (Hann por defecto) |
| RD-05 | Indicadores numéricos por eje: **RMS, pico, pico a pico, factor de cresta, frecuencia dominante** |
| RD-06 | Controles: Iniciar/Pausar, Calibrar, frecuencia de muestreo, rango (g), eje a mostrar |
| RD-07 | Estado de conexión (conectado / reconectando), frecuencia de muestreo real, tamaño del bloque y tiempo de captura |
| RD-08 | **Exportar CSV** del registro (`t_s,ax_g,ay_g,az_g`) mediante descarga desde el navegador |
| RD-09 | Reconexión automática del WebSocket si se pierde la conexión |
| RD-10 | Opcional: semáforo de severidad (verde/amarillo/rojo) con umbrales configurables sobre el RMS |
| RD-11 | Implementar la FFT en JS propio (radix-2, tamaño potencia de 2), sin librerías externas |

### 4.3 Protocolo ESP32 ↔ Dashboard (propuesto)

WebSocket en `ws://<ip>/ws`.

**ESP32 → navegador** (JSON, un mensaje por bloque):
```json
{
  "type": "data",
  "fs": 1000,
  "t0_ms": 123456,
  "range_g": 4,
  "ax": [0.012, 0.015],
  "ay": [-0.003, -0.001],
  "az": [0.998, 1.001]
}
```
- Unidades en **g**. Las muestras de cada bloque son contiguas en el tiempo.
- Si el JSON resulta demasiado pesado a 1 kHz, cambiar a **binario** (Int16 por eje + cabecera) y documentarlo en este archivo.

**Navegador → ESP32** (comandos JSON):
```json
{"cmd": "start"}
{"cmd": "stop"}
{"cmd": "calibrate"}
{"cmd": "set", "fs": 1000, "range_g": 4}
```

---

## 5. Requerimientos multiplataforma (Fedora ↔ Windows)

- **Arduino IDE 2.x** con el paquete de placas *esp32 by Espressif Systems* (documentar la versión usada).
- Librerías instaladas desde el Gestor de Librerías (documentar nombre y versión exacta en el README):
  - Una librería para MPU9250 (por ejemplo *MPU9250* de hideakitai, o *Bolder Flight Systems MPU9250*), o acceso directo por I2C con `Wire`.
  - Servidor web + WebSocket (por ejemplo *ESPAsyncWebServer* + *AsyncTCP*, o *WebServer* + *WebSockets* de Markus Sattler).
- **Linux (Fedora):** agregar el usuario al grupo `dialout` para acceder a `/dev/ttyUSB0` o `/dev/ttyACM0` (`sudo usermod -aG dialout $USER`, luego cerrar sesión y volver a entrar).
- **Windows:** instalar el driver del chip USB-serie de la placa (CP210x o CH340, según el modelo).
- Sin rutas absolutas ni comandos específicos del sistema operativo en el código ni en los scripts.
- Si se incluye un script para generar `dashboard_html.h`, debe ser multiplataforma (Python 3 con rutas relativas) y también documentar el procedimiento manual.
- Dashboard probado en Chrome y Firefox de escritorio y en el navegador del celular (Android y/o iOS).

---

## 6. Análisis en Matlab

- El CSV exportado debe abrirse con `readmatrix` sin preprocesamiento.
- Script `analisis_vibraciones.m` que:
  1. Cargue el CSV.
  2. Calcule la FFT y/o la densidad espectral de potencia (`pwelch`).
  3. Grafique señal en el tiempo y espectro por eje.
  4. Reporte RMS, pico y frecuencia dominante para compararlos con los valores del dashboard (**validación cruzada**).

---

## 7. Plan de validación (marco rígido VFACA)

1. **Reposo:** medir ruido base del sensor (RMS ≈ ruido de fondo).
2. **Impulso (golpe):** identificar la frecuencia natural del marco en el espectro.
3. **Excitación sostenida** (si hay motor o vibrador disponible): verificar que el pico espectral coincida con la frecuencia de excitación.
4. **Repetibilidad:** repetir cada prueba 3 veces y comparar resultados.
5. **Comparación dashboard vs. Matlab:** diferencia de frecuencia dominante y RMS dentro de una tolerancia razonable (≤ 5 %).

### Criterios de aceptación
- [ ] El celular abre el dashboard conectándose solo a la red WiFi del ESP32.
- [ ] Se ven los 3 ejes en tiempo real y el espectro actualizado.
- [ ] La frecuencia de muestreo real se muestra y es estable (±2 %).
- [ ] El CSV exportado se abre en Matlab y reproduce el espectro del dashboard.
- [ ] El proyecto compila y corre igual en Fedora y en Windows siguiendo el README.
- [ ] Se identifica la frecuencia natural del marco en la prueba de impulso.

---

## 8. Estructura de archivos propuesta

```
analizador-vibraciones/
├── README.md                     # instalación para Fedora y Windows, conexiones, uso
├── firmware/
│   └── analizador_vibraciones/
│       ├── analizador_vibraciones.ino
│       └── dashboard_html.h      # HTML/CSS/JS embebido (generado desde web/)
├── web/
│   ├── index.html
│   ├── style.css
│   └── app.js
├── matlab/
│   └── analisis_vibraciones.m
├── tools/
│   └── generar_dashboard_h.py    # opcional, multiplataforma
└── docs/
    └── evidencias/               # fotos, capturas, video
```

---

## 9. Consideraciones técnicas importantes

- Muestreo de 1 kHz con 3 ejes por I2C a 400 kHz es viable pero justo: leer los 6 bytes de aceleración en una sola transacción (o usar el FIFO del MPU9250) para no perder muestras.
- La FFT requiere muestreo uniforme; si el ESP32 pierde muestras, el espectro se distorsiona. Medir y reportar pérdidas.
- Frecuencia máxima observable = fs / 2 (Nyquist). Ajustar el DLPF del sensor para evitar aliasing.
- Resolución en frecuencia = fs / N (por ejemplo 1000 / 1024 ≈ 0.98 Hz).
- El WiFi del ESP32 puede introducir jitter: enviar bloques con marca de tiempo `t0_ms` y no depender del momento de llegada del mensaje.
- Al embeber el HTML en PROGMEM, usar raw string literal (`R"rawliteral( ... )rawliteral"`) y vigilar que el contenido no contenga la secuencia `)rawliteral"`.
- Los MPU9250 clonados o los que traen el chip MPU6500 (sin magnetómetro AK8963) son comunes: detectar con `WHO_AM_I` y no depender del magnetómetro, ya que solo se necesita el acelerómetro.

---

## 10. Instrucciones para el agente de IA

- Responder y comentar el código **en español**.
- Priorizar **simplicidad y legibilidad**: el HTML/JS debe ser fácil de leer y explicar en una exposición.
- No introducir dependencias de frontend (npm, CDN, frameworks). Todo en los tres archivos de `web/`.
- No usar `delay()` en el ciclo de muestreo.
- Entregar los archivos completos y funcionales; indicar siempre qué librerías de Arduino instalar y con qué versión.
- Cuando haya una decisión de diseño con alternativas (JSON vs. binario, librería A vs. B), explicar brevemente el porqué.
- Mantener este documento actualizado si cambia el protocolo, los pines o las decisiones técnicas.

---

## 11. Preguntas abiertas (por confirmar)

- [ ] Modelo exacto de la placa ESP32 y del chip USB-serie (CP210x / CH340).
- [ ] ¿El MPU9250 es original o clon (MPU6500)?
- [ ] Frecuencia de muestreo objetivo definitiva (¿1 kHz es suficiente para el marco?).
- [ ] ¿Se requiere un rango de frecuencias específico para el análisis (por ejemplo 10–1000 Hz, como en ISO 10816)?
- [ ] ¿Se pedirá velocidad (mm/s) además de aceleración? Implicaría integrar la señal en el dashboard.
- [ ] Fecha real de presentación (la indicada, 18/11/2026, ya pasó).
