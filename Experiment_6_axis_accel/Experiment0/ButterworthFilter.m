% ---------------------------------------------------------------
% Butterworth LOW-PASS Filter + FFT in dB (0 to -100 dB)
% ---------------------------------------------------------------
% FFT power spectrum plotted from 0 dB down to -100 dB
% ---------------------------------------------------------------

clear; clc; close all;

%% 1. USER SETTINGS
csvFilename    = 'stable_hand_2G_scale_5ms_115200_baud_data.csv';
outputFilename = 'filtered_all_sensors_lowpass.csv';

filterOrder = 4;
cutoffFreq  = 3;           % Low-pass cutoff in Hz

%% 2. READ + PARSE DATA
opts = detectImportOptions(csvFilename);
opts = setvartype(opts, opts.VariableNames{1}, 'datetime');
opts = setvaropts(opts, opts.VariableNames{1}, 'DatetimeFormat', 'yyyy-MM-dd HH:mm:ss.SSS');
T = readtable(csvFilename, opts);

time_dt = T{:,1};
time    = seconds(time_dt - time_dt(1));

%% 3. FIND VALID SENSOR COLUMNS
varNames      = T.Properties.VariableNames;
sensorCols    = 2:numel(varNames);
validSensorCols = [];

for i = sensorCols
    col = T{:,i};
    if isnumeric(col) && any(isfinite(col))
        validSensorCols = [validSensorCols i];
    end
end

if isempty(validSensorCols)
    error('No valid sensor columns found.');
end

%% 4. SAMPLING FREQUENCY & FILTER DESIGN
dt = mean(diff(time));
Fs = 1/dt;
nyquist = Fs/2;

fprintf('Fs = %.2f Hz  (Nyquist = %.2f Hz)\n', Fs, nyquist);

Wn = cutoffFreq / nyquist;
[b, a] = butter(filterOrder, Wn, 'low');

%% 5. APPLY FILTER
nSensors = numel(validSensorCols);
filtered = nan(numel(time), nSensors);

for j = 1:nSensors
    colIdx = validSensorCols(j);
    raw = T{:,colIdx};
    ok  = isfinite(raw);
    filtered(ok,j) = filtfilt(b, a, raw(ok));
end

%% 6. SAVE FILTERED DATA
outNames = cellfun(@(x) ['filt_' x], varNames(validSensorCols), 'UniformOutput', false);
outTable = array2table([time filtered], 'VariableNames', [{'Time'} outNames]);
writetable(outTable, outputFilename);
fprintf('Filtered data saved: %s\n', outputFilename);

%% 7. PLOT: TIME DOMAIN + FFT in dB (0 to -100 dB)
example = 1;
colIdx  = validSensorCols(example);
origName = varNames{colIdx};
filtName = outNames{example};

rawSignal  = T{:,colIdx};
filtSignal = filtered(:,example);
ok         = isfinite(rawSignal);

rawClean   = rawSignal(ok);
filtClean  = filtSignal(ok);
N          = length(rawClean);

% Remove DC
rawClean  = rawClean  - mean(rawClean);
filtClean = filtClean - mean(filtClean);

% FFT
Y_orig = fft(rawClean);
Y_filt = fft(filtClean);

% Single-sided amplitude spectrum
P2_orig = abs(Y_orig/N);
P1_orig = P2_orig(1:N/2+1);
P1_orig(2:end-1) = 2*P1_orig(2:end-1);

P2_filt = abs(Y_filt/N);
P1_filt = P2_filt(1:N/2+1);
P1_filt(2:end-1) = 2*P1_filt(2:end-1);

f = Fs*(0:(N/2))/N;

% Convert to dB (20*log10)
PSD_orig_dB = 20*log10(P1_orig + eps);
PSD_filt_dB = 20*log10(P1_filt + eps);

%% PLOT: Time + Frequency (0 to -100 dB)
figure('Position', [100, 50, 1100, 850], 'Color', 'w');

% Time domain
subplot(2,1,1);
plot(time(ok), rawSignal(ok), 'b', 'LineWidth', 1, 'DisplayName', 'Original');
hold on;
plot(time(ok), filtSignal(ok), 'r', 'LineWidth', 1.8, 'DisplayName', 'Low-pass Filtered');
xlabel('Time (s)'); ylabel('Amplitude');
title(sprintf('Time Domain – %s', origName));
legend('Location', 'best'); grid on;

% Frequency domain: 0 dB to -100 dB
subplot(2,1,2);
plot(f, PSD_orig_dB, 'b', 'LineWidth', 1.2, 'DisplayName', 'Original');
hold on;
plot(f, PSD_filt_dB, 'r', 'LineWidth', 2, 'DisplayName', 'Low-pass Filtered');

% Cutoff line
xline(cutoffFreq, '--k', 'LineWidth', 1.5, ...
      'Label', sprintf('Cutoff = %.1f Hz', cutoffFreq), ...
      'LabelOrientation', 'horizontal', 'FontWeight', 'bold');

xlabel('Frequency (Hz)');
ylabel('Magnitude (dB)');
title(sprintf('Power Spectrum (dB) – %s', origName));
legend('Location', 'southwest'); grid on;

% CRITICAL: Set Y-axis from 0 dB to -100 dB
ylim([-100 0]);           % ← This is the only change you needed
xlim([0 20]);             % Optional: focus on 0–20 Hz

% Optional: save
% saveas(gcf, 'LowPass_Filter_FFT_0_to_-100dB.png');