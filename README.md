# vibscope-esp32

Analizador de vibraciones con ESP32 y acelerómetro MPU9250. La ESP32 envía la aceleración por USB a un dashboard web local que muestra la señal, la FFT y los indicadores, y exporta un CSV para Matlab.

## Estructura

```
arduino/arduino.ino              firmware de la ESP32
web/                             dashboard (index.html, style.css, app.js)
matlab/analisis_vibraciones.m    análisis del CSV
context.md                       descripción del proyecto
```

## Conexiones

El MPU9250 va a **3.3 V** (nunca a 5 V).

| MPU9250 | ESP32  |
|---------|--------|
| VCC     | 3V3    |
| GND     | GND    |
| SDA     | GPIO21 |
| SCL     | GPIO22 |

## Requisitos

- Arduino IDE 2.x con el paquete de placas **esp32 by Espressif Systems** (probado con 4.0.0). No se necesitan librerías.
- Chrome o Edge (Web Serial no existe en Firefox).
- **Linux:** agrega tu usuario al grupo `dialout`, cierra sesión y vuelve a entrar: `sudo usermod -aG dialout $USER`
- **Windows:** instala el driver USB-serie de la placa (CP210x o CH340).

## Cómo usarlo

1. Abre `arduino/arduino.ino` en Arduino IDE, elige la placa **ESP32 Dev Module** y el puerto, y sube el programa.
2. Cierra el Monitor Serie (el puerto no se puede compartir con el navegador).
3. Abre `web/index.html` en Chrome o Edge. Si no funciona desde archivo, ejecuta `python3 -m http.server 8000` dentro de `web/` y abre `http://localhost:8000`.
4. Pulsa **Conectar** y elige el puerto. Espera unos 2 segundos a que la placa se reinicie.
5. Con el sensor quieto pulsa **Calibrar**, y luego **Iniciar**.
6. Para guardar datos: **Grabar**, luego **Detener grabación** y **Descargar CSV**.

Sin hardware, el botón **Simulador** genera una señal de prueba.

## Matlab

Abre `matlab/analisis_vibraciones.m`, escribe el nombre del CSV en `archivo` (o déjalo vacío para elegirlo) y ejecútalo. Muestra la señal, el espectro y una tabla con RMS, pico y frecuencia dominante para compararla con el dashboard. `pwelch` requiere el Signal Processing Toolbox.

El CSV no tiene encabezado. Columnas: `t_s, ax_g, ay_g, az_g`.

## Protocolo serie

- ESP32 → PC (binario, 921600 baudios): `0xAA 0x55 | tipo | largo(u16) | carga | checksum XOR`. Tipo 1 = datos (`fs`, `fs_real×10`, `rango_g`, reservado, `perdidas`, `t0_ms`, muestras `int16` X, Y, Z). Tipo 2 = texto de estado. Conversión a g: `valor / (32768 / rango_g)`.
- PC → ESP32 (texto + `\n`): `START`, `STOP`, `CAL`, `CAL DC`, `SET fs=1000 range=4`, `INFO`. Valores válidos: fs 250, 500 o 1000 Hz; rango 2, 4, 8 o 16 g.
