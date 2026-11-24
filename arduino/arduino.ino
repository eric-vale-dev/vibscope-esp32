#include <Arduino.h>
#include <Wire.h>
#include <stdarg.h>

// Configuración
static const uint8_t  PIN_SDA        = 21;
static const uint8_t  PIN_SCL        = 22;
static const uint32_t I2C_HZ         = 400000;
static const uint32_t BAUDIOS        = 921600;
static const uint16_t FS_INICIAL     = 1000;   // Hz: 250, 500 o 1000
static const uint8_t  RANGO_INICIAL  = 4;      // g: 2, 4, 8 o 16
static const uint16_t MUESTRAS_CAL   = 500;    // muestras promediadas al calibrar

// Registros del MPU
static const uint8_t REG_SMPLRT_DIV   = 0x19;
static const uint8_t REG_ACCEL_CONFIG = 0x1C;
static const uint8_t REG_ACCEL_CONF2  = 0x1D;
static const uint8_t REG_ACCEL_XOUT_H = 0x3B;
static const uint8_t REG_PWR_MGMT_1   = 0x6B;
static const uint8_t REG_PWR_MGMT_2   = 0x6C;
static const uint8_t REG_WHO_AM_I     = 0x75;

// Bloques de muestras
static const uint16_t MAX_N       = 512;   // muestras máximas por bloque
static const uint8_t  NUM_BUFFERS = 6;     // bloques en circulación

struct Bloque {
  uint16_t n;
  uint16_t fs;
  uint16_t fsReal10;     // frecuencia real medida x10 (0.1 Hz)
  uint8_t  rango;
  uint16_t perdidas;     // muestras perdidas desde el bloque anterior
  uint32_t t0ms;
  int16_t  datos[MAX_N * 3];
};

struct Mensaje { char t[96]; };

static Bloque       bloques[NUM_BUFFERS];
static QueueHandle_t colaLibres;   // Bloque*
static QueueHandle_t colaLlenos;   // Bloque*
static QueueHandle_t colaMsg;      // Mensaje

// Estado compartido
static volatile bool     corriendo     = false;
static volatile uint8_t  calSolicitada = 0;      // 0 = nada, 1 = conservar gravedad, 2 = quitar todo (DC)
static volatile uint16_t pendFs        = 0;      // 0 = sin cambio pendiente
static volatile uint8_t  pendRango     = 0;
static volatile uint16_t fsActual      = FS_INICIAL;
static volatile uint8_t  rangoActual   = RANGO_INICIAL;
static volatile int32_t  offsets[3]    = {0, 0, 0};

static uint8_t dirMPU   = 0x68;
static uint8_t chipId   = 0;
static bool    sensorOk = false;

// Utilidades

// Encola un mensaje de texto para el PC
static void avisar(const char* fmt, ...) {
  Mensaje m;
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(m.t, sizeof(m.t), fmt, ap);
  va_end(ap);
  xQueueSend(colaMsg, &m, 0);
}

static uint16_t tamanoBloque(uint16_t fs) {
  // Bloque de ~0.5 s
  if (fs >= 1000) return 512;
  if (fs >= 500)  return 256;
  return 128;
}

// Filtro pasa-bajas del sensor según fs
static uint8_t dlpfPara(uint16_t fs) {
  if (fs >= 1000) return 7;   // 420 Hz de ancho de banda
  if (fs >= 500)  return 1;   // 218 Hz
  return 2;                   // 99 Hz
}

static uint8_t indiceRango(uint8_t g) {
  switch (g) { case 2: return 0; case 4: return 1; case 8: return 2; default: return 3; }
}

// Acceso al MPU

static bool escribirReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(dirMPU);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

static int leerReg(uint8_t dir, uint8_t reg) {
  Wire.beginTransmission(dir);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return -1;
  if (Wire.requestFrom((int)dir, 1) != 1) return -1;
  return Wire.read();
}

// Lee X, Y, Z (6 bytes en una transacción)
static bool leerAceleracion(int16_t* v) {
  Wire.beginTransmission(dirMPU);
  Wire.write(REG_ACCEL_XOUT_H);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)dirMPU, 6) != 6) return false;
  for (int i = 0; i < 3; i++) {
    uint8_t h = Wire.read();
    uint8_t l = Wire.read();
    v[i] = (int16_t)((h << 8) | l);
  }
  return true;
}

// Aplica rango y filtro
static void aplicarConfigMPU() {
  escribirReg(REG_ACCEL_CONFIG, indiceRango(rangoActual) << 3);
  escribirReg(REG_ACCEL_CONF2, dlpfPara(fsActual) & 0x07);   // DLPF activo
}

static bool iniciarMPU() {
  // Buscar el sensor en 0x68 o 0x69
  const uint8_t candidatas[2] = {0x68, 0x69};
  sensorOk = false;
  for (uint8_t i = 0; i < 2; i++) {
    int id = leerReg(candidatas[i], REG_WHO_AM_I);
    if (id < 0) continue;
    // MPU9250, MPU9255, MPU6500 (clon) o MPU6050
    if (id == 0x71 || id == 0x73 || id == 0x70 || id == 0x68) {
      dirMPU = candidatas[i];
      chipId = (uint8_t)id;
      sensorOk = true;
      break;
    }
    chipId = (uint8_t)id;   // chip desconocido: se informa
  }
  if (!sensorOk) return false;

  escribirReg(REG_PWR_MGMT_1, 0x80);   // reset
  delay(100);
  escribirReg(REG_PWR_MGMT_1, 0x01);   // despierta, reloj PLL
  delay(10);
  escribirReg(REG_PWR_MGMT_2, 0x00);   // acelerómetro y giróscopo activos
  escribirReg(REG_SMPLRT_DIV, 0x00);
  aplicarConfigMPU();
  return true;
}

static const char* nombreChip(uint8_t id) {
  switch (id) {
    case 0x71: return "MPU9250";
    case 0x73: return "MPU9255";
    case 0x70: return "MPU6500 (clon, sin magnetometro)";
    case 0x68: return "MPU6050/9150";
    default:   return "desconocido";
  }
}

// Tarea de muestreo 
static void tareaMuestreo(void*) {
  uint32_t periodo = 1000000UL / fsActual;   // microsegundos entre muestras
  uint32_t proximo = 0;
  bool     enCurso = false;

  Bloque*  b = nullptr;
  uint16_t k = 0;
  uint32_t tPrimera = 0, tUltima = 0;
  uint16_t perdidasAcum = 0;

  bool    calActiva = false;
  uint8_t calModo = 0;
  int32_t sumas[3] = {0, 0, 0};
  uint16_t calN = 0;

  for (;;) {
    // Cambiar configuración
    if (pendFs || pendRango) {
      uint16_t nf = pendFs ? pendFs : fsActual;
      uint8_t  nr = pendRango ? pendRango : rangoActual;
      if (nr != rangoActual) { offsets[0] = offsets[1] = offsets[2] = 0; }
      fsActual = nf;
      rangoActual = nr;
      pendFs = 0;
      pendRango = 0;
      aplicarConfigMPU();
      periodo = 1000000UL / fsActual;
      enCurso = false;
      avisar("OK SET fs=%u range=%u", (unsigned)fsActual, (unsigned)rangoActual);
    }

    // Iniciar calibración
    if (calSolicitada) {
      calModo = calSolicitada;
      calSolicitada = 0;
      calActiva = true;
      calN = 0;
      sumas[0] = sumas[1] = sumas[2] = 0;
      enCurso = false;
    }

    // En pausa: ceder el procesador
    if (!corriendo && !calActiva) {
      enCurso = false;
      k = 0;
      vTaskDelay(pdMS_TO_TICKS(5));
      continue;
    }

    if (!enCurso) {
      proximo = micros() + periodo;
      enCurso = true;
      k = 0;
      perdidasAcum = 0;
    }

    // Esperar a la próxima muestra
    int32_t falta = (int32_t)(proximo - micros());
    if (falta > 2000) { vTaskDelay(1); continue; }
    while ((int32_t)(proximo - micros()) > 0) { /* espera activa corta */ }
    uint32_t tMuestra = micros();
    proximo += periodo;

    // Si hay atraso, contar muestras perdidas
    int32_t atraso = (int32_t)(micros() - proximo);
    if (atraso > (int32_t)(3 * periodo)) {
      perdidasAcum += (uint16_t)(atraso / (int32_t)periodo);
      proximo = micros() + periodo;
    }

    int16_t raw[3];
    if (!leerAceleracion(raw)) { perdidasAcum++; continue; }

    // Calibración
    if (calActiva) {
      for (int i = 0; i < 3; i++) sumas[i] += raw[i];
      calN++;
      if (calN >= MUESTRAS_CAL) {
        float media[3];
        int eje = 0;
        for (int i = 0; i < 3; i++) {
          media[i] = (float)sumas[i] / (float)calN;
          if (fabsf(media[i]) > fabsf(media[eje])) eje = i;
        }
        for (int i = 0; i < 3; i++) offsets[i] = (int32_t)media[i];
        if (calModo == 1) {
          // Conservar 1 g en el eje vertical
          float g = 32768.0f / (float)rangoActual;
          offsets[eje] = (int32_t)(media[eje] - (media[eje] > 0 ? g : -g));
        }
        calActiva = false;
        avisar("OK CAL modo=%s offsets=%ld,%ld,%ld", calModo == 1 ? "gravedad" : "DC",
               (long)offsets[0], (long)offsets[1], (long)offsets[2]);
      }
      if (!corriendo) continue;
    }

    // Llenar bloque
    if (b == nullptr) {
      if (xQueueReceive(colaLibres, &b, 0) != pdTRUE) {
        b = nullptr;
        perdidasAcum++;       // sin bloque libre
        continue;
      }
      k = 0;
    }
    if (k == 0) {
      b->t0ms = tMuestra / 1000;
      tPrimera = tMuestra;
    }
    for (int i = 0; i < 3; i++) {
      int32_t v = (int32_t)raw[i] - offsets[i];
      if (v > 32767) v = 32767;
      if (v < -32768) v = -32768;
      b->datos[k * 3 + i] = (int16_t)v;
    }
    tUltima = tMuestra;
    k++;

    uint16_t N = tamanoBloque(fsActual);
    if (k >= N) {
      b->n = N;
      b->fs = fsActual;
      b->rango = rangoActual;
      uint32_t dur = tUltima - tPrimera;
      b->fsReal10 = dur ? (uint16_t)(((uint64_t)(N - 1) * 10000000ULL) / dur) : 0;
      b->perdidas = perdidasAcum;
      perdidasAcum = 0;
      xQueueSend(colaLlenos, &b, 0);
      b = nullptr;
      k = 0;
    }
  }
}

// Envío y comandos (núcleo 0)
static uint8_t trama[3200];

static void enviarTrama(uint8_t tipo, uint16_t len) {
  trama[0] = 0xAA;
  trama[1] = 0x55;
  trama[2] = tipo;
  trama[3] = len & 0xFF;
  trama[4] = len >> 8;
  uint8_t cs = 0;
  for (uint16_t i = 2; i < 5 + len; i++) cs ^= trama[i];
  trama[5 + len] = cs;
  Serial.write(trama, len + 6);
}

static void enviarTexto(const char* s) {
  uint16_t len = strlen(s);
  if (len > 120) len = 120;
  memcpy(trama + 5, s, len);
  enviarTrama(2, len);
}

static void enviarBloque(const Bloque* b) {
  uint8_t* p = trama + 5;
  p[0] = b->fs & 0xFF;        p[1] = b->fs >> 8;
  p[2] = b->fsReal10 & 0xFF;  p[3] = b->fsReal10 >> 8;
  p[4] = b->rango;
  p[5] = 0;
  p[6] = b->perdidas & 0xFF;  p[7] = b->perdidas >> 8;
  p[8]  = b->t0ms & 0xFF;
  p[9]  = (b->t0ms >> 8) & 0xFF;
  p[10] = (b->t0ms >> 16) & 0xFF;
  p[11] = (b->t0ms >> 24) & 0xFF;
  memcpy(p + 12, b->datos, (size_t)b->n * 6);   // el ESP32 es little-endian
  enviarTrama(1, 12 + b->n * 6);
}

static void procesarComando(char* linea) {
  for (char* c = linea; *c; c++) *c = toupper((unsigned char)*c);

  if (strcmp(linea, "START") == 0) {
    if (!sensorOk) { avisar("ERROR sensor no detectado"); return; }
    corriendo = true;
    avisar("OK START");
  } else if (strcmp(linea, "STOP") == 0) {
    corriendo = false;
    avisar("OK STOP");
  } else if (strncmp(linea, "CAL", 3) == 0) {
    if (!sensorOk) { avisar("ERROR sensor no detectado"); return; }
    calSolicitada = (strstr(linea, "DC") != nullptr) ? 2 : 1;
    avisar("OK CAL en curso, no mover el sensor");
  } else if (strncmp(linea, "SET", 3) == 0) {
    char* pf = strstr(linea, "FS=");
    char* pr = strstr(linea, "RANGE=");
    uint16_t nf = pf ? atoi(pf + 3) : 0;
    uint8_t  nr = pr ? atoi(pr + 6) : 0;
    bool fsValida = (nf == 0) || nf == 250 || nf == 500 || nf == 1000;
    bool rValido  = (nr == 0) || nr == 2 || nr == 4 || nr == 8 || nr == 16;
    if (!fsValida || !rValido || (nf == 0 && nr == 0)) {
      avisar("ERROR SET: fs=250|500|1000 range=2|4|8|16");
      return;
    }
    pendFs = nf;
    pendRango = nr;
  } else if (strcmp(linea, "INFO") == 0) {
    avisar("INFO chip=%s id=0x%02X dir=0x%02X fs=%u range=%u corriendo=%d",
           nombreChip(chipId), chipId, dirMPU, (unsigned)fsActual, (unsigned)rangoActual, (int)corriendo);
  } else if (linea[0] != 0) {
    avisar("ERROR comando desconocido");
  }
}

static void leerComandos() {
  static char linea[64];
  static uint8_t n = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      linea[n] = 0;
      if (n > 0) procesarComando(linea);
      n = 0;
    } else if (n < sizeof(linea) - 1) {
      linea[n++] = c;
    }
  }
}

static void tareaEnvio(void*) {
  for (;;) {
    leerComandos();

    Mensaje m;
    while (xQueueReceive(colaMsg, &m, 0) == pdTRUE) enviarTexto(m.t);

    Bloque* b;
    if (xQueueReceive(colaLlenos, &b, pdMS_TO_TICKS(5)) == pdTRUE) {
      enviarBloque(b);
      xQueueSend(colaLibres, &b, 0);
    }
  }
}

// Arduino

void setup() {
  Serial.setTxBufferSize(8192);
  Serial.begin(BAUDIOS);

  colaLibres = xQueueCreate(NUM_BUFFERS, sizeof(Bloque*));
  colaLlenos = xQueueCreate(NUM_BUFFERS, sizeof(Bloque*));
  colaMsg    = xQueueCreate(8, sizeof(Mensaje));
  for (uint8_t i = 0; i < NUM_BUFFERS; i++) {
    Bloque* p = &bloques[i];
    xQueueSend(colaLibres, &p, 0);
  }

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(I2C_HZ);
  delay(100);

  if (iniciarMPU()) {
    avisar("LISTO chip=%s fs=%u range=%u", nombreChip(chipId), (unsigned)fsActual, (unsigned)rangoActual);
  } else {
    avisar("ERROR MPU no detectado (WHO_AM_I=0x%02X). Revisar SDA=21, SCL=22, 3V3, GND", chipId);
  }

  xTaskCreatePinnedToCore(tareaEnvio, "envio", 6144, nullptr, 1, nullptr, 0);
  // Muestreo en el núcleo 1 con prioridad alta
  xTaskCreatePinnedToCore(tareaMuestreo, "muestreo", 4096, nullptr, 3, nullptr, 1);
}

void loop() {
  // Todo corre en las tareas
  vTaskDelete(nullptr);
}
