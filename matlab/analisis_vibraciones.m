%% analisis_vibraciones.m
% Analisis del CSV exportado por el dashboard (vibscope-esp32).
% Columnas del CSV (sin encabezado):  t_s, ax_g, ay_g, az_g
%
% Uso:
%   1) Pon el nombre del archivo en "archivo" (o dejalo vacio para elegirlo con una ventana).
%   2) Ejecuta el script. Muestra figuras y una tabla de resultados que se
%      comparan con los indicadores del dashboard (validacion cruzada).
%
% Requisitos: MATLAB. pwelch pertenece al Signal Processing Toolbox; si no lo
% tienes, el script omite esa parte y usa solo la FFT propia.

clear; close all; clc;

%% ---- Parametros ----
archivo   = '';        % ej. 'vibraciones_20261118_101500.csv'  ('' = elegir con ventana)
nfft      = 1024;      % puntos de la ventana de analisis (igual que el dashboard)
ventana   = 'hann';    % 'hann' | 'hamming' | 'rect'

%% ---- 1. Cargar el CSV ----
if isempty(archivo)
    [f, ruta] = uigetfile('*.csv', 'Selecciona el CSV del dashboard');
    if isequal(f, 0), error('No se selecciono ningun archivo.'); end
    archivo = fullfile(ruta, f);
end
datos = readmatrix(archivo);
t = datos(:, 1);
a = datos(:, 2:4);                       % [ax ay az] en g
ejes = {'X', 'Y', 'Z'};
N = size(a, 1);
fs = (N - 1) / (t(end) - t(1));          % frecuencia de muestreo real del registro
fprintf('Archivo: %s\n', archivo);
fprintf('Muestras: %d | Duracion: %.2f s | fs = %.2f Hz\n\n', N, t(end) - t(1), fs);

if N < nfft
    nfft = 2^floor(log2(N));
    warning('Registro corto: se usa nfft = %d', nfft);
end

%% ---- 2. Indicadores del registro completo (sin componente DC) ----
ac = a - mean(a, 1);                     % se quita la componente DC (gravedad / offset)
rmsReg  = sqrt(mean(ac.^2, 1));
picoReg = max(abs(ac), [], 1);
p2pReg  = max(ac, [], 1) - min(ac, [], 1);
crestaReg = picoReg ./ rmsReg;

%% ---- 3. FFT de la ultima ventana de nfft muestras (igual que el dashboard) ----
w = ventana_(ventana, nfft);
seg = ac(end - nfft + 1:end, :);         % ultimas nfft muestras
% Nota: el dashboard quita la media de la ventana, no la del registro completo
seg = seg - mean(seg, 1);
espectro = abs(fft(seg .* w, nfft)) * 2 / sum(w);
espectro = espectro(1:nfft/2 + 1, :);
f = (0:nfft/2)' * fs / nfft;

fDom = zeros(1, 3);
aDom = zeros(1, 3);
for e = 1:3
    [~, k] = max(espectro(2:end-1, e));
    k = k + 1;                           % indice dentro de espectro (se salta DC)
    % Interpolacion parabolica (igual que el dashboard)
    y1 = espectro(k-1, e); y2 = espectro(k, e); y3 = espectro(k+1, e);
    den = y1 - 2*y2 + y3;
    d = 0;
    if den ~= 0, d = 0.5 * (y1 - y3) / den; end
    fDom(e) = (k - 1 + d) * fs / nfft;
    aDom(e) = y2;
end

% Indicadores sobre la misma ventana (comparables con la tabla del dashboard)
rmsVen  = sqrt(mean(seg.^2, 1));
picoVen = max(abs(seg), [], 1);
p2pVen  = max(seg, [], 1) - min(seg, [], 1);
crestaVen = picoVen ./ rmsVen;

%% ---- 4. Densidad espectral de potencia (pwelch), si esta disponible ----
tienePwelch = exist('pwelch', 'file') == 2;
if tienePwelch
    nseg = min(N, 1024);
    fDomPsd = zeros(1, 3);
    psd = cell(1, 3); fPsd = cell(1, 3);
    for e = 1:3
        [psd{e}, fPsd{e}] = pwelch(ac(:, e), hann(nseg), floor(nseg/2), nseg, fs);
        [~, k] = max(psd{e}(2:end));
        fDomPsd(e) = fPsd{e}(k + 1);
    end
end

%% ---- 5. Figuras ----
colores = {[1 0.42 0.42], [0.29 0.87 0.5], [0.38 0.65 0.98]};

figure('Name', 'Senal en el tiempo', 'Color', 'w');
for e = 1:3
    subplot(3, 1, e);
    plot(t, a(:, e), 'Color', colores{e});
    grid on; ylabel(sprintf('a_%s (g)', ejes{e}));
    if e == 1, title('Aceleracion vs. tiempo'); end
end
xlabel('Tiempo (s)');

figure('Name', 'Espectro FFT', 'Color', 'w');
for e = 1:3
    subplot(3, 1, e);
    plot(f, espectro(:, e), 'Color', colores{e});
    grid on; ylabel(sprintf('|A_%s| (g)', ejes{e}));
    xline(fDom(e), '--k', sprintf('%.1f Hz', fDom(e)));
    if e == 1, title(sprintf('Espectro de amplitud (FFT %d puntos, ventana %s)', nfft, ventana)); end
end
xlabel('Frecuencia (Hz)');

if tienePwelch
    figure('Name', 'Densidad espectral (pwelch)', 'Color', 'w');
    for e = 1:3
        plot(fPsd{e}, 10*log10(psd{e}), 'Color', colores{e}); hold on;
    end
    grid on; legend(ejes{:});
    xlabel('Frecuencia (Hz)'); ylabel('PSD (dB/Hz)');
    title('Densidad espectral de potencia (Welch)');
end

%% ---- 6. Tabla de resultados para comparar con el dashboard ----
fprintf('=== Registro completo (sin DC) ===\n');
T1 = table(ejes', rmsReg', picoReg', p2pReg', crestaReg', ...
    'VariableNames', {'Eje', 'RMS_g', 'Pico_g', 'PicoAPico_g', 'Cresta'});
disp(T1);

fprintf('=== Ultima ventana de %d muestras (comparar con el dashboard) ===\n', nfft);
T2 = table(ejes', rmsVen', picoVen', p2pVen', crestaVen', fDom', aDom', ...
    'VariableNames', {'Eje', 'RMS_g', 'Pico_g', 'PicoAPico_g', 'Cresta', 'FrecDom_Hz', 'AmpDom_g'});
disp(T2);

if tienePwelch
    fprintf('Frecuencia dominante por pwelch (Hz): X=%.1f  Y=%.1f  Z=%.1f\n', fDomPsd);
end

%% ---- Funcion local: ventanas (simetricas, igual que el dashboard) ----
function w = ventana_(tipo, n)
    i = (0:n-1)';
    switch tipo
        case 'hann',    w = 0.5 - 0.5*cos(2*pi*i/(n-1));
        case 'hamming', w = 0.54 - 0.46*cos(2*pi*i/(n-1));
        otherwise,      w = ones(n, 1);
    end
end
