/*
  vibscope-esp32 - Dashboard del analizador de vibraciones
  HTML + CSS + JavaScript puro, sin librerías.

  Flujo:  ESP32 --USB serie (Web Serial, 921600 baud, binario)--> procesarBytes()
          --> procesarBloque() --> buffers circulares --> FFT / indicadores / gráficas / CSV

  Formato de trama (ver arduino/arduino.ino y context.md):
    0xAA 0x55 | tipo | largo(u16) | carga útil | checksum(XOR de tipo..carga)
*/

'use strict';

// =====================================================================
// 1. FFT radix-2 (tamaño potencia de 2) y utilidades de señal
// =====================================================================

const cacheFFT = {};   // tablas precalculadas por tamaño

function prepararFFT(n) {
  if (cacheFFT[n]) return cacheFFT[n];
  const cos = new Float64Array(n / 2);
  const sin = new Float64Array(n / 2);
  for (let i = 0; i < n / 2; i++) {
    cos[i] = Math.cos((2 * Math.PI * i) / n);
    sin[i] = -Math.sin((2 * Math.PI * i) / n);
  }
  // Tabla de inversión de bits
  const rev = new Uint32Array(n);
  const bits = Math.log2(n);
  for (let i = 0; i < n; i++) {
    let r = 0;
    for (let b = 0; b < bits; b++) if (i & (1 << b)) r |= 1 << (bits - 1 - b);
    rev[i] = r;
  }
  cacheFFT[n] = { cos, sin, rev };
  return cacheFFT[n];
}

// FFT in situ: re e im son Float64Array de largo n (potencia de 2).
function fft(re, im) {
  const n = re.length;
  const { cos, sin, rev } = prepararFFT(n);
  for (let i = 0; i < n; i++) {
    const j = rev[i];
    if (j > i) {
      let t = re[i]; re[i] = re[j]; re[j] = t;
      t = im[i]; im[i] = im[j]; im[j] = t;
    }
  }
  for (let tam = 2; tam <= n; tam <<= 1) {
    const mitad = tam >> 1;
    const paso = n / tam;
    for (let ini = 0; ini < n; ini += tam) {
      for (let k = 0; k < mitad; k++) {
        const wr = cos[k * paso], wi = sin[k * paso];
        const a = ini + k, b = a + mitad;
        const tr = re[b] * wr - im[b] * wi;
        const ti = re[b] * wi + im[b] * wr;
        re[b] = re[a] - tr; im[b] = im[a] - ti;
        re[a] += tr;        im[a] += ti;
      }
    }
  }
}

// Ventanas (simétricas, igual que en el script de Matlab)
function crearVentana(tipo, n) {
  const w = new Float64Array(n);
  for (let i = 0; i < n; i++) {
    if (tipo === 'hann') w[i] = 0.5 - 0.5 * Math.cos((2 * Math.PI * i) / (n - 1));
    else if (tipo === 'hamming') w[i] = 0.54 - 0.46 * Math.cos((2 * Math.PI * i) / (n - 1));
    else w[i] = 1;
  }
  return w;
}

// Espectro de amplitud de un solo lado, en las mismas unidades de la señal.
// Una senoide de amplitud A produce un pico de altura A.
// Devuelve { amp: Float64Array(n/2+1), df, fPico, aPico }.
function calcularEspectro(senal, fs, tipoVentana) {
  const n = senal.length;
  let media = 0;
  for (let i = 0; i < n; i++) media += senal[i];
  media /= n;

  const w = crearVentana(tipoVentana, n);
  let sumaW = 0;
  const re = new Float64Array(n), im = new Float64Array(n);
  for (let i = 0; i < n; i++) {
    re[i] = (senal[i] - media) * w[i];   // se quita la componente DC antes de la FFT
    sumaW += w[i];
  }
  fft(re, im);

  const mitad = n / 2;
  const amp = new Float64Array(mitad + 1);
  for (let k = 0; k <= mitad; k++) {
    const mag = Math.hypot(re[k], im[k]);
    amp[k] = (k === 0 || k === mitad ? 1 : 2) * mag / sumaW;
  }
  const df = fs / n;

  // Pico (ignorando DC) con interpolación parabólica para afinar la frecuencia
  let kMax = 1;
  for (let k = 2; k < mitad; k++) if (amp[k] > amp[kMax]) kMax = k;
  let fPico = kMax * df, aPico = amp[kMax];
  if (kMax > 1 && kMax < mitad) {
    const a = amp[kMax - 1], b = amp[kMax], c = amp[kMax + 1];
    const den = a - 2 * b + c;
    if (den !== 0) {
      const d = 0.5 * (a - c) / den;
      fPico = (kMax + d) * df;
      aPico = b - 0.25 * (a - c) * d;
    }
  }
  return { amp, df, fPico, aPico };
}

// Indicadores de una señal (sin componente DC)
function calcularIndicadores(senal) {
  const n = senal.length;
  let media = 0;
  for (let i = 0; i < n; i++) media += senal[i];
  media /= n;
  let suma2 = 0, max = -Infinity, min = Infinity, pico = 0;
  for (let i = 0; i < n; i++) {
    const v = senal[i] - media;
    suma2 += v * v;
    if (v > max) max = v;
    if (v < min) min = v;
    if (Math.abs(v) > pico) pico = Math.abs(v);
  }
  const rms = Math.sqrt(suma2 / n);
  return { rms, pico, picoAPico: max - min, cresta: rms > 0 ? pico / rms : 0 };
}

// =====================================================================
// 2. Decodificación del protocolo binario
// =====================================================================

class DecodificadorTramas {
  // alRecibirDatos(bloque), alRecibirTexto(texto), alError()
  constructor(alRecibirDatos, alRecibirTexto, alError) {
    this.alRecibirDatos = alRecibirDatos;
    this.alRecibirTexto = alRecibirTexto;
    this.alError = alError;
    this.buf = new Uint8Array(0);
  }

  agregar(chunk) {
    const nuevo = new Uint8Array(this.buf.length + chunk.length);
    nuevo.set(this.buf, 0);
    nuevo.set(chunk, this.buf.length);
    this.buf = nuevo;

    const b = this.buf;
    let i = 0;
    while (true) {
      // Buscar la cabecera 0xAA 0x55 (resincronización)
      while (i + 1 < b.length && !(b[i] === 0xAA && b[i + 1] === 0x55)) i++;
      if (i + 1 >= b.length) break;
      if (b.length - i < 5) break;                 // falta el encabezado completo

      const tipo = b[i + 2];
      const largo = b[i + 3] | (b[i + 4] << 8);
      if (largo > 4096) { i += 2; continue; }      // largo imposible: falsa cabecera
      const total = 6 + largo;
      if (b.length - i < total) break;             // trama incompleta, esperar más bytes

      let cs = 0;
      for (let j = i + 2; j < i + 5 + largo; j++) cs ^= b[j];
      if (cs !== b[i + 5 + largo]) {
        this.alError();
        i += 2;                                    // checksum malo: buscar la siguiente cabecera
        continue;
      }

      const carga = b.subarray(i + 5, i + 5 + largo);
      if (tipo === 1) this.alRecibirDatos(this.decodificarDatos(carga));
      else if (tipo === 2) this.alRecibirTexto(new TextDecoder().decode(carga));
      i += total;
    }
    // Conservar solo lo no procesado (y un posible 0xAA al final)
    this.buf = b.slice(i);
  }

  decodificarDatos(carga) {
    const dv = new DataView(carga.buffer, carga.byteOffset, carga.byteLength);
    const fs = dv.getUint16(0, true);
    const fsReal = dv.getUint16(2, true) / 10;
    const rango = dv.getUint8(4);
    const perdidas = dv.getUint16(6, true);
    const t0ms = dv.getUint32(8, true);
    const n = Math.floor((carga.byteLength - 12) / 6);
    const escala = rango / 32768;                  // int16 -> g
    const x = new Float32Array(n), y = new Float32Array(n), z = new Float32Array(n);
    for (let k = 0; k < n; k++) {
      const o = 12 + k * 6;
      x[k] = dv.getInt16(o, true) * escala;
      y[k] = dv.getInt16(o + 2, true) * escala;
      z[k] = dv.getInt16(o + 4, true) * escala;
    }
    return { fs, fsReal, rango, perdidas, t0ms, x, y, z };
  }
}

// =====================================================================
// 3. Estado de la aplicación y buffers circulares
// =====================================================================

const CAPACIDAD = 8192;   // muestras por eje en el buffer circular (8 s a 1 kHz)

const estado = {
  fs: 1000,              // frecuencia de muestreo nominal
  fsReal: 0,
  rango: 4,
  tamBloque: 0,
  bloques: 0,
  perdidas: 0,
  errores: 0,
  total: 0,              // muestras recibidas desde que se conectó
  ejes: [new Float32Array(CAPACIDAD), new Float32Array(CAPACIDAD), new Float32Array(CAPACIDAD)],
  nuevo: false,          // hay datos sin dibujar
  inicioCaptura: 0,      // ms (performance.now) del primer bloque
  grabando: false,
  grabacion: null,       // { x:[], y:[], z:[], n, sumaFs, bloques }
  csv: null,             // último CSV generado
};

function reiniciarDatos() {
  estado.total = 0;
  estado.bloques = 0;
  estado.perdidas = 0;
  estado.errores = 0;
  estado.nuevo = true;
  estado.inicioCaptura = 0;
  for (const e of estado.ejes) e.fill(0);
}

// Últimas m muestras de un eje, en orden cronológico (copia)
function ultimas(eje, m) {
  const salida = new Float32Array(m);
  const inicio = estado.total - m;
  for (let i = 0; i < m; i++) salida[i] = estado.ejes[eje][(inicio + i) % CAPACIDAD];
  return salida;
}

// Punto de entrada de los datos (viene del puerto serie o del simulador)
function procesarBloque(bq) {
  const n = bq.x.length;
  for (let k = 0; k < n; k++) {
    const idx = (estado.total + k) % CAPACIDAD;
    estado.ejes[0][idx] = bq.x[k];
    estado.ejes[1][idx] = bq.y[k];
    estado.ejes[2][idx] = bq.z[k];
  }
  estado.total += n;
  estado.fs = bq.fs;
  estado.fsReal = bq.fsReal;
  estado.rango = bq.rango;
  estado.tamBloque = n;
  estado.bloques++;
  estado.perdidas += bq.perdidas;
  if (!estado.inicioCaptura) estado.inicioCaptura = performance.now();
  estado.nuevo = true;

  if (estado.grabando && estado.grabacion) {
    const g = estado.grabacion;
    g.x.push(bq.x); g.y.push(bq.y); g.z.push(bq.z);
    g.n += n;
    g.sumaFs += bq.fsReal > 0 ? bq.fsReal : bq.fs;
    g.bloques++;
    g.perdidas += bq.perdidas;
  }
}

// =====================================================================
// 4. Exportación CSV  (columnas: t_s, ax_g, ay_g, az_g; sin encabezado)
// =====================================================================

function generarCSV(g) {
  const fsProm = g.sumaFs / g.bloques;       // frecuencia real promedio de la grabación
  const filas = [];
  let i = 0;
  for (let b = 0; b < g.x.length; b++) {
    const x = g.x[b], y = g.y[b], z = g.z[b];
    for (let k = 0; k < x.length; k++, i++) {
      filas.push((i / fsProm).toFixed(6) + ',' + x[k].toFixed(6) + ',' + y[k].toFixed(6) + ',' + z[k].toFixed(6));
    }
  }
  return filas.join('\n') + '\n';
}

function descargarArchivo(nombre, texto) {
  const blob = new Blob([texto], { type: 'text/csv' });
  const url = URL.createObjectURL(blob);
  const a = document.createElement('a');
  a.href = url;
  a.download = nombre;
  document.body.appendChild(a);
  a.click();
  a.remove();
  setTimeout(() => URL.revokeObjectURL(url), 1000);
}

function nombreArchivo() {
  const d = new Date();
  const p = (v) => String(v).padStart(2, '0');
  return `vibraciones_${d.getFullYear()}${p(d.getMonth() + 1)}${p(d.getDate())}_${p(d.getHours())}${p(d.getMinutes())}${p(d.getSeconds())}.csv`;
}

// =====================================================================
// 5. Dibujo en canvas (sin librerías)
// =====================================================================

const COLORES = ['#ff6b6b', '#4ade80', '#60a5fa'];   // X, Y, Z

function prepararCanvas(canvas) {
  const r = canvas.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  const w = Math.round(r.width * dpr), h = Math.round(r.height * dpr);
  if (canvas.width !== w || canvas.height !== h) { canvas.width = w; canvas.height = h; }
  const g = canvas.getContext('2d');
  g.setTransform(dpr, 0, 0, dpr, 0, 0);
  return { g, w: r.width, h: r.height };
}

// Marcas "bonitas" para un eje: 1, 2, 5 x 10^n
function marcas(min, max, cuantas) {
  if (!(max > min)) max = min + 1e-6;
  const paso0 = (max - min) / cuantas;
  const mag = Math.pow(10, Math.floor(Math.log10(paso0)));
  const norm = paso0 / mag;
  const paso = (norm < 1.5 ? 1 : norm < 3.5 ? 2 : norm < 7.5 ? 5 : 10) * mag;
  const lista = [];
  for (let k = Math.ceil(min / paso); k * paso <= max + paso * 1e-6; k++) lista.push(k * paso);
  const dec = Math.min(6, Math.max(0, Math.ceil(-Math.log10(paso))));
  return { lista, dec };
}

/*
  opciones: { series:[{datos, color, x0, dx}], xMin, xMax, yMin, yMax, xTitulo, yTitulo }
  Devuelve funciones para convertir datos -> píxeles (para marcar el pico).
*/
function dibujarGrafica(canvas, o) {
  const { g, w, h } = prepararCanvas(canvas);
  const mi = 52, md = 12, ms = 10, mb = 34;           // márgenes
  const ancho = w - mi - md, alto = h - ms - mb;
  g.clearRect(0, 0, w, h);
  g.fillStyle = '#11161d';
  g.fillRect(0, 0, w, h);

  const px = (x) => mi + ((x - o.xMin) / (o.xMax - o.xMin)) * ancho;
  const py = (y) => ms + (1 - (y - o.yMin) / (o.yMax - o.yMin)) * alto;

  g.font = '11px system-ui, sans-serif';
  g.lineWidth = 1;

  // Cuadrícula y etiquetas
  const mx = marcas(o.xMin, o.xMax, Math.max(2, Math.floor(ancho / 80)));
  const my = marcas(o.yMin, o.yMax, Math.max(2, Math.floor(alto / 40)));
  g.strokeStyle = '#26303c';
  g.fillStyle = '#8b98a8';
  g.textAlign = 'center';
  for (const v of mx.lista) {
    const x = Math.round(px(v)) + 0.5;
    g.beginPath(); g.moveTo(x, ms); g.lineTo(x, ms + alto); g.stroke();
    g.fillText(v.toFixed(mx.dec), x, ms + alto + 14);
  }
  g.textAlign = 'right';
  for (const v of my.lista) {
    const y = Math.round(py(v)) + 0.5;
    g.beginPath(); g.moveTo(mi, y); g.lineTo(mi + ancho, y); g.stroke();
    g.fillText(v.toFixed(my.dec), mi - 6, y + 4);
  }
  // Títulos de ejes
  g.textAlign = 'center';
  g.fillText(o.xTitulo, mi + ancho / 2, h - 5);
  g.save();
  g.translate(12, ms + alto / 2);
  g.rotate(-Math.PI / 2);
  g.fillText(o.yTitulo, 0, 0);
  g.restore();

  // Marco
  g.strokeStyle = '#3a4656';
  g.strokeRect(mi + 0.5, ms + 0.5, ancho, alto);

  // Series (recortadas al área de la gráfica)
  g.save();
  g.beginPath();
  g.rect(mi, ms, ancho, alto);
  g.clip();
  g.lineWidth = 1.5;
  for (const s of o.series) {
    g.strokeStyle = s.color;
    g.beginPath();
    for (let i = 0; i < s.datos.length; i++) {
      const x = px(s.x0 + i * s.dx), y = py(s.datos[i]);
      if (i === 0) g.moveTo(x, y); else g.lineTo(x, y);
    }
    g.stroke();
  }
  g.restore();

  return { px, py, g, mi, ms, ancho, alto };
}

function mensajeEnCanvas(canvas, texto) {
  const { g, w, h } = prepararCanvas(canvas);
  g.fillStyle = '#11161d';
  g.fillRect(0, 0, w, h);
  g.fillStyle = '#8b98a8';
  g.font = '14px system-ui, sans-serif';
  g.textAlign = 'center';
  g.fillText(texto, w / 2, h / 2);
}

// =====================================================================
// 6. Interfaz, Web Serial y simulador
// =====================================================================

function iniciarInterfaz() {
  const $ = (id) => document.getElementById(id);
  const el = {
    estado: $('estado'), aviso: $('avisoSoporte'),
    btnConectar: $('btnConectar'), btnIniciar: $('btnIniciar'), btnCalibrar: $('btnCalibrar'),
    chkGravedad: $('chkGravedad'), selFs: $('selFs'), selRango: $('selRango'),
    selEje: $('selEje'), selNfft: $('selNfft'), selVentana: $('selVentana'), selVisible: $('selVisible'),
    chkQuitarDC: $('chkQuitarDC'),
    btnGrabar: $('btnGrabar'), btnDescargar: $('btnDescargar'), infoGrabacion: $('infoGrabacion'),
    btnSimulador: $('btnSimulador'),
    vFs: $('vFs'), vFsReal: $('vFsReal'), vBloque: $('vBloque'), vBloques: $('vBloques'),
    vPerdidas: $('vPerdidas'), vErrores: $('vErrores'), vTiempo: $('vTiempo'), vResolucion: $('vResolucion'),
    tituloEje: $('tituloEje'), registro: $('registro'),
    cvTiempo: $('cvTiempo'), cvEspectro: $('cvEspectro'),
    filas: document.querySelectorAll('#tablaIndicadores tbody tr'),
  };

  const NOMBRES_EJE = ['X', 'Y', 'Z'];
  let puerto = null, lector = null, conectado = false, midiendo = false;
  let simTimer = null, simFase = 0;
  const codificador = new TextEncoder();

  function log(texto) {
    const hora = new Date().toLocaleTimeString();
    el.registro.textContent += `[${hora}] ${texto}\n`;
    el.registro.scrollTop = el.registro.scrollHeight;
  }

  const decodificador = new DecodificadorTramas(
    procesarBloque,
    (t) => log(t),
    () => { estado.errores++; estado.nuevo = true; }
  );

  // ---------- Estado visual ----------
  function actualizarBotones() {
    const hayFuente = conectado || simTimer !== null;
    el.btnIniciar.disabled = !conectado;
    el.btnCalibrar.disabled = !conectado;
    el.btnGrabar.disabled = !hayFuente;
    el.btnConectar.textContent = conectado ? 'Desconectar' : 'Conectar';
    el.btnConectar.classList.toggle('primario', !conectado);
    el.btnIniciar.textContent = midiendo ? 'Pausar' : 'Iniciar';
    el.btnSimulador.classList.toggle('activo', simTimer !== null);
    el.btnSimulador.textContent = simTimer !== null ? 'Detener simulador' : 'Simulador';
    el.estado.className = 'insignia ' + (simTimer !== null ? 'simulador' : conectado ? 'conectado' : 'desconectado');
    el.estado.textContent = simTimer !== null ? 'Simulador' : conectado ? 'Conectado' : 'Desconectado';
  }

  // ---------- Web Serial ----------
  async function enviar(comando) {
    if (!puerto || !puerto.writable) return;
    const escritor = puerto.writable.getWriter();
    try { await escritor.write(codificador.encode(comando + '\n')); }
    finally { escritor.releaseLock(); }
  }

  async function conectar() {
    try {
      puerto = await navigator.serial.requestPort();
      await puerto.open({ baudRate: 921600, bufferSize: 65536 });
    } catch (e) {
      log('No se pudo abrir el puerto: ' + e.message);
      puerto = null;
      return;
    }
    detenerSimulador();
    reiniciarDatos();
    conectado = true;
    midiendo = false;
    actualizarBotones();
    log('Puerto abierto a 921600 baudios');
    leerPuerto();
    // Al abrir el puerto la placa suele reiniciarse: esperar antes de preguntar.
    setTimeout(() => { if (conectado) { enviar('INFO'); enviar(`SET fs=${el.selFs.value} range=${el.selRango.value}`); } }, 1800);
  }

  async function leerPuerto() {
    while (puerto && puerto.readable && conectado) {
      lector = puerto.readable.getReader();
      try {
        while (true) {
          const { value, done } = await lector.read();
          if (done) break;
          if (value) decodificador.agregar(value);
        }
      } catch (e) {
        log('Error de lectura: ' + e.message);
        break;
      } finally {
        lector.releaseLock();
        lector = null;
      }
    }
  }

  async function desconectar() {
    if (!puerto) return;
    try { await enviar('STOP'); } catch (_) { /* ignorar */ }
    conectado = false;
    midiendo = false;
    try { if (lector) await lector.cancel(); } catch (_) { /* ignorar */ }
    try { await puerto.close(); } catch (_) { /* ignorar */ }
    puerto = null;
    actualizarBotones();
    log('Puerto cerrado');
  }

  // ---------- Simulador (señal sintética, sin hardware) ----------
  function iniciarSimulador() {
    if (conectado) return;
    reiniciarDatos();
    const fs = parseInt(el.selFs.value, 10);
    const rango = parseInt(el.selRango.value, 10);
    const n = fs / 2;                        // medio segundo por bloque
    simFase = 0;
    simTimer = setInterval(() => {
      const x = new Float32Array(n), y = new Float32Array(n), z = new Float32Array(n);
      for (let k = 0; k < n; k++) {
        const t = (simFase + k) / fs;
        x[k] = 0.50 * Math.sin(2 * Math.PI * 50 * t) + 0.01 * (Math.random() - 0.5);
        y[k] = 0.20 * Math.sin(2 * Math.PI * 120 * t) + 0.01 * (Math.random() - 0.5);
        z[k] = 1.0 + 0.10 * Math.sin(2 * Math.PI * 80 * t) + 0.01 * (Math.random() - 0.5);
      }
      simFase += n;
      procesarBloque({ fs, fsReal: fs, rango, perdidas: 0, t0ms: 0, x, y, z });
    }, 500);
    log(`Simulador: X=50 Hz (0.5 g), Y=120 Hz (0.2 g), Z=1 g + 80 Hz (0.1 g), fs=${fs} Hz`);
    actualizarBotones();
  }

  function detenerSimulador() {
    if (simTimer === null) return;
    clearInterval(simTimer);
    simTimer = null;
    actualizarBotones();
  }

  // ---------- Grabación y CSV ----------
  function alternarGrabacion() {
    if (!estado.grabando) {
      estado.grabacion = { x: [], y: [], z: [], n: 0, sumaFs: 0, bloques: 0, perdidas: 0 };
      estado.grabando = true;
      estado.csv = null;
      el.btnGrabar.textContent = 'Detener grabación';
      el.btnGrabar.classList.add('activo');
      el.btnDescargar.disabled = true;
    } else {
      estado.grabando = false;
      el.btnGrabar.textContent = 'Grabar';
      el.btnGrabar.classList.remove('activo');
      const g = estado.grabacion;
      if (g && g.n > 0) {
        estado.csv = generarCSV(g);
        el.btnDescargar.disabled = false;
        el.infoGrabacion.textContent = `Listo: ${g.n} muestras (${(g.n / (g.sumaFs / g.bloques)).toFixed(2)} s)` +
          (g.perdidas ? `, ${g.perdidas} perdidas` : '');
      } else {
        el.infoGrabacion.textContent = 'Grabación vacía (no llegaron datos)';
      }
    }
  }

  // ---------- Refresco periódico: indicadores, FFT y gráficas ----------
  function refrescar() {
    // Estado numérico (se actualiza siempre)
    el.vFs.textContent = estado.bloques ? estado.fs + ' Hz' : '–';
    el.vFsReal.textContent = estado.fsReal ? estado.fsReal.toFixed(1) + ' Hz' : '–';
    el.vBloque.textContent = estado.tamBloque ? estado.tamBloque + ' muestras' : '–';
    el.vBloques.textContent = estado.bloques;
    el.vPerdidas.textContent = estado.perdidas;
    el.vErrores.textContent = estado.errores;
    // Tiempo de captura = muestras recibidas / fs. Se detiene cuando dejan de llegar datos.
    const fsTiempo = estado.fsReal > 0 ? estado.fsReal : estado.fs;
    el.vTiempo.textContent = (estado.total / fsTiempo).toFixed(1) + ' s';
    if (estado.grabando && estado.grabacion) {
      const g = estado.grabacion;
      el.infoGrabacion.textContent = `Grabando: ${g.n} muestras`;
    }

    if (!estado.nuevo) return;
    estado.nuevo = false;

    const fs = estado.fsReal > 0 ? estado.fsReal : estado.fs;
    const nfft = parseInt(el.selNfft.value, 10);
    const eje = parseInt(el.selEje.value, 10);
    el.vResolucion.textContent = (fs / nfft).toFixed(2) + ' Hz';
    el.tituloEje.textContent = `(eje ${NOMBRES_EJE[eje]})`;

    // ----- Gráfica de aceleración vs. tiempo -----
    const visibles = Math.min(estado.total, Math.round(parseFloat(el.selVisible.value) * fs), CAPACIDAD);
    if (visibles < 2) {
      mensajeEnCanvas(el.cvTiempo, 'Sin datos. Conecta la ESP32 y pulsa Iniciar, o activa el simulador.');
      mensajeEnCanvas(el.cvEspectro, 'Sin datos');
      return;
    }
    const series = [];
    let yMin = Infinity, yMax = -Infinity;
    for (let e = 0; e < 3; e++) {
      const datos = ultimas(e, visibles);
      if (el.chkQuitarDC.checked) {
        let m = 0;
        for (let i = 0; i < visibles; i++) m += datos[i];
        m /= visibles;
        for (let i = 0; i < visibles; i++) datos[i] -= m;
      }
      for (let i = 0; i < visibles; i++) {
        if (datos[i] < yMin) yMin = datos[i];
        if (datos[i] > yMax) yMax = datos[i];
      }
      series.push({ datos, color: COLORES[e], x0: 0, dx: 1 / fs });
    }
    const margen = Math.max((yMax - yMin) * 0.1, 0.001);
    dibujarGrafica(el.cvTiempo, {
      series, xMin: 0, xMax: (visibles - 1) / fs, yMin: yMin - margen, yMax: yMax + margen,
      xTitulo: 'Tiempo (s)', yTitulo: 'Aceleración (g)',
    });

    // ----- Indicadores y espectro (ventana de análisis = últimas nfft muestras) -----
    if (estado.total < nfft) {
      mensajeEnCanvas(el.cvEspectro, `Acumulando muestras para la FFT (${estado.total}/${nfft})…`);
      return;
    }
    let espectroSel = null;
    for (let e = 0; e < 3; e++) {
      const datos = ultimas(e, nfft);
      const ind = calcularIndicadores(datos);
      const esp = calcularEspectro(datos, fs, el.selVentana.value);
      const celdas = el.filas[e].children;
      celdas[1].textContent = ind.rms.toFixed(4);
      celdas[2].textContent = ind.pico.toFixed(4);
      celdas[3].textContent = ind.picoAPico.toFixed(4);
      celdas[4].textContent = ind.cresta.toFixed(2);
      celdas[5].textContent = esp.fPico.toFixed(1);
      if (e === eje) espectroSel = esp;
    }

    // ----- Gráfica del espectro -----
    const esp = espectroSel;
    let aMax = 0;
    for (let k = 1; k < esp.amp.length; k++) if (esp.amp[k] > aMax) aMax = esp.amp[k];
    aMax = Math.max(aMax * 1.15, 1e-4);
    const vista = esp.amp.subarray(1);              // sin la barra de DC
    const r = dibujarGrafica(el.cvEspectro, {
      series: [{ datos: vista, color: COLORES[eje], x0: esp.df, dx: esp.df }],
      xMin: 0, xMax: fs / 2, yMin: 0, yMax: aMax,
      xTitulo: 'Frecuencia (Hz)', yTitulo: 'Amplitud (g)',
    });
    // Marcador del pico
    const xp = r.px(esp.fPico), yp = r.py(esp.aPico);
    r.g.fillStyle = '#fbbf24';
    r.g.beginPath(); r.g.arc(xp, yp, 4, 0, 2 * Math.PI); r.g.fill();
    r.g.font = '12px system-ui, sans-serif';
    r.g.textAlign = xp > r.mi + r.ancho - 120 ? 'right' : 'left';
    r.g.fillText(`${esp.fPico.toFixed(1)} Hz · ${esp.aPico.toFixed(4)} g`, xp + (r.g.textAlign === 'left' ? 8 : -8), Math.max(yp, r.ms + 14));
  }

  // ---------- Eventos ----------
  el.btnConectar.addEventListener('click', () => (conectado ? desconectar() : conectar()));

  el.btnIniciar.addEventListener('click', async () => {
    midiendo = !midiendo;
    if (midiendo) reiniciarDatos();
    await enviar(midiendo ? 'START' : 'STOP');
    actualizarBotones();
  });

  el.btnCalibrar.addEventListener('click', () => {
    log('Calibrando: mantén el sensor quieto…');
    enviar(el.chkGravedad.checked ? 'CAL' : 'CAL DC');
  });

  function enviarConfiguracion() {
    if (conectado) enviar(`SET fs=${el.selFs.value} range=${el.selRango.value}`);
    if (simTimer !== null) { detenerSimulador(); iniciarSimulador(); }
    estado.nuevo = true;
  }
  el.selFs.addEventListener('change', enviarConfiguracion);
  el.selRango.addEventListener('change', enviarConfiguracion);

  for (const s of [el.selEje, el.selNfft, el.selVentana, el.selVisible, el.chkQuitarDC]) {
    s.addEventListener('change', () => { estado.nuevo = true; });
  }

  el.btnGrabar.addEventListener('click', alternarGrabacion);
  el.btnDescargar.addEventListener('click', () => { if (estado.csv) descargarArchivo(nombreArchivo(), estado.csv); });
  el.btnSimulador.addEventListener('click', () => (simTimer !== null ? detenerSimulador() : iniciarSimulador()));

  window.addEventListener('resize', () => { estado.nuevo = true; });

  if ('serial' in navigator) {
    navigator.serial.addEventListener('disconnect', () => {
      if (conectado) { log('Cable desconectado'); desconectar(); }
    });
  } else {
    el.aviso.hidden = false;
    el.btnConectar.disabled = true;
  }

  actualizarBotones();
  estado.nuevo = true;
  setInterval(refrescar, 100);   // 10 actualizaciones por segundo
}

if (typeof document !== 'undefined') {
  iniciarInterfaz();
}

// Para pruebas en Node (no afecta al navegador)
if (typeof module !== 'undefined') {
  module.exports = { fft, calcularEspectro, calcularIndicadores, DecodificadorTramas, generarCSV };
}
