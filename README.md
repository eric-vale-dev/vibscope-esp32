# vibscope-esp32

Analizador de vibraciones portátil (emulación de prototipo) con **ESP32 + acelerómetro MPU9250**.
El firmware muestrea la aceleración en 3 ejes y la envía por **USB serie** a un dashboard
local (HTML + CSS + JavaScript puro) que muestra la señal, la FFT en vivo, indicadores y exporta
un **CSV** para analizarlo en **Matlab**.

> Alcance actual: conexión por cable USB (Web Serial). La vista móvil, el WiFi/WebSocket y el
> semáforo de severidad quedan fuera por ahora. Más contexto y decisiones en [`context.md`](context.md).

```
MPU9250 --I2C 400 kHz--> ESP32 --USB serie 921600 baud (binario)--> Dashboard (Chrome/Edge)
                                                                     ├─ Gráfica en el tiempo
                                                                     ├─ FFT / espectro
                                                                     ├─ RMS, pico, cresta, frec. dominante
                                                                     └─ CSV --> Matlab
```

## Estructura

```
sismografo/
├── arduino/arduino.ino   # firmware
├── web/index.html, style.css, app.js                           # dashboard
├── matlab/analisis_vibraciones.m                               # análisis del CSV
├── context.md                                                  # requerimientos y decisiones
└── README.md
```

## 1. Conexiones

El MPU9250 se alimenta a **3.3 V** (nunca a 5 V).

| MPU9250 | ESP32 (38 pines, ESP-32S) |
|---------|---------------------------|
| VCC     | 3V3                       |
| GND     | GND                       |
| SDA     | GPIO21                    |
| SCL     | GPIO22                    |

Fija el sensor de forma rígida al marco (tornillo, base magnética o adhesivo duro) y anota la
orientación de los ejes X, Y, Z respecto al marco.

## 2. Firmware (Arduino IDE 2.x)

1. Instala el paquete de placas **esp32 by Espressif Systems** (Gestor de placas). Probado con la versión **4.0.0**.
2. Selecciona la placa **ESP32 Dev Module**.
3. **No hace falta instalar librerías**: el MPU9250 se lee por registros con `Wire`.
4. Abre `arduino/arduino.ino`, selecciona el puerto y sube el programa.
5. **No abras el Monitor Serie** mientras uses el dashboard: el puerto solo puede usarlo un programa a la vez.

### Permisos del puerto serie

- **Fedora / Linux:** agrega tu usuario al grupo `dialout`, cierra sesión y vuelve a entrar:
  ```bash
  sudo usermod -aG dialout $USER
  ```
  El puerto aparece como `/dev/ttyUSB0` o `/dev/ttyACM0`.
- **Windows:** instala el driver del chip USB-serie de la placa (**CP210x** o **CH340**, según el modelo). Aparece como `COMx`.

## 3. Dashboard

Requiere **Chrome o Edge** (Web Serial no existe en Firefox ni Safari).

1. Abre `web/index.html` con doble clic o arrastrándolo a Chrome.
2. Pulsa **Conectar** y elige el puerto de la ESP32.
3. Pulsa **Iniciar**. Verás los tres ejes, el espectro y los indicadores.

Si tu Chrome no permite Web Serial desde `file://`, sirve la carpeta en local y abre `http://localhost:8000`:

```bash
cd web
python3 -m http.server 8000      # en Windows: python -m http.server 8000
```

**Sin hardware:** el botón **Simulador** genera una señal sintética (X = 50 Hz, Y = 120 Hz, Z = 1 g + 80 Hz) para probar la interfaz.

### Controles

| Control | Función |
|---|---|
| Iniciar / Pausar | Envía `START` / `STOP` a la ESP32 |
| Calibrar | Promedia 500 muestras en reposo y resta el offset. Con *Conservar gravedad* marcado, el eje vertical sigue midiendo 1 g |
| Frecuencia de muestreo | 250, 500 o 1000 Hz |
| Rango | ±2, ±4, ±8 o ±16 g |
| Eje, puntos FFT, ventana | Configuración del espectro (Hann por defecto, 1024 puntos) |
| Grabar / Descargar CSV | Guarda las muestras recibidas mientras la grabación está activa |

Los indicadores (RMS, pico, pico a pico, factor de cresta y frecuencia dominante) se calculan sobre la
última ventana de la FFT, sin componente DC.

### Formato del CSV

Sin encabezado, para abrirlo con `readmatrix` sin preprocesar. Columnas: `t_s, ax_g, ay_g, az_g`.

```
0.000000,0.012345,-0.003210,0.998700
0.001000,0.011800,-0.002900,0.999100
```

El tiempo se calcula como `índice / fs_real_promedio`. Si el dashboard informa **muestras perdidas**, el eje de tiempo de esa grabación no es exacto: repite la captura.

## 4. Análisis en Matlab

1. Descarga el CSV desde el dashboard.
2. Abre `matlab/analisis_vibraciones.m`, escribe el nombre del archivo en `archivo` (o déjalo vacío para elegirlo) y ejecútalo.
3. El script grafica la señal y el espectro por eje (FFT y `pwelch`) y muestra una tabla con RMS, pico y frecuencia dominante.
4. Compara la tabla de la **última ventana** con los indicadores del dashboard (tolerancia esperada ≤ 5 %).

`pwelch` requiere el Signal Processing Toolbox. Sin él, el script omite esa parte.

## 5. Protocolo serie

Detalle en el encabezado del `.ino` y en `context.md`. Resumen:

- ESP32 → PC (binario, 921600 baudios): `0xAA 0x55 | tipo | largo(u16) | carga | checksum XOR`. Tipo 1 = datos (bloques de 128 a 512 muestras, int16 por eje), tipo 2 = texto de estado.
- PC → ESP32 (texto con `\n`): `START`, `STOP`, `CAL`, `CAL DC`, `SET fs=1000 range=4`, `INFO`.

## 6. Plan de validación (marco rígido)

1. **Reposo:** medir el ruido base (RMS ≈ ruido de fondo del sensor).
2. **Impulso (golpe):** localizar la frecuencia natural del marco en el espectro.
3. **Excitación sostenida** (si hay motor o vibrador): el pico debe coincidir con la frecuencia de excitación.
4. **Repetibilidad:** repetir cada prueba 3 veces.
5. **Dashboard vs. Matlab:** diferencia de frecuencia dominante y RMS ≤ 5 %.

## 7. Notas técnicas y limitaciones

- La fs real se mide en la ESP32 y se muestra en el dashboard. Debe quedar dentro de ±2 % de la nominal.
- El MPU9250 entrega datos a 1 kHz internamente, y el muestreo del ESP32 no está sincronizado con ese reloj. A 1000 Hz puede haber alguna muestra repetida u omitida. Para el análisis de un marco rígido el efecto es pequeño, pero conviene validarlo en las pruebas.
- Filtro digital del sensor (DLPF): 420 Hz a 1000 Hz, 218 Hz a 500 Hz y 99 Hz a 250 Hz. A 1000 Hz el filtro no atenúa del todo cerca de Nyquist (500 Hz).
- Los MPU9250 "clon" con chip MPU6500 funcionan igual, porque solo se usa el acelerómetro. El firmware informa el chip detectado en el panel de mensajes.
- Al abrir el puerto, la ESP32 suele reiniciarse. El dashboard espera unos segundos antes de configurarla.
