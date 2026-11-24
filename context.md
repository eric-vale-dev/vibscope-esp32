# vibscope-esp32

Emulación de un prototipo de analizador de vibraciones, similar a los equipos portátiles de monitoreo de condición (por ejemplo SKF).

- **Hardware:** ESP32 (38 pines) y acelerómetro MPU9250.
- **Firmware:** Arduino IDE. Lee la aceleración en 3 ejes a 250, 500 o 1000 Hz y la envía por USB serie.
- **Dashboard:** HTML, CSS y JavaScript puro, abierto en Chrome o Edge. Muestra la señal, el espectro FFT y los indicadores (RMS, pico, pico a pico, factor de cresta y frecuencia dominante), y exporta CSV.
- **Análisis:** el CSV se analiza en Matlab para comparar con el dashboard.
- **Validación:** sobre un marco rígido tipo VFACA (reposo, impulso para hallar la frecuencia natural, repetibilidad).

Para instalar y usar el proyecto, ver [README.md](README.md).

## Alcance actual

- Conexión por cable USB (Web Serial).
- Fuera de alcance por ahora: WiFi, vista para celular y semáforo de severidad.

## Referencias

- https://www.skf.com/mx/products/condition-monitoring-systems/portable-systems
- https://www.youtube.com/watch?v=1xVvRJSIPAQ
- https://www.youtube.com/watch?v=wd3xawxXJuE
